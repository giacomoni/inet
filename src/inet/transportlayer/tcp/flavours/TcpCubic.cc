/*
 * TcpCubic.cc
 *
 *  Created on: Nov 9, 2022
 *      Author: Luca Giacomoni
 */

#include "inet/transportlayer/tcp/flavours/TcpCubic.h"

#include "inet/transportlayer/tcp/Tcp.h"

namespace inet {
namespace tcp {

Register_Class(TcpCubic);

simsignal_t TcpCubic::bicTargetSignal = cComponent::registerSignal("bicTarget");
simsignal_t TcpCubic::originPointSignal = cComponent::registerSignal(
        "originalPoint");
simsignal_t TcpCubic::cwndSegSignal = cComponent::registerSignal("cwndSeg");
simsignal_t TcpCubic::bicKSignal = cComponent::registerSignal("bicK");
simsignal_t TcpCubic::cntSignal = cComponent::registerSignal("cnt");
simsignal_t TcpCubic::lastMaxWindowSignal = cComponent::registerSignal(
        "lastMaxWindow");
simsignal_t TcpCubic::delayMinSignal = cComponent::registerSignal("delayMin");
simsignal_t TcpCubic::concaveSignal = cComponent::registerSignal("concave");
simsignal_t TcpCubic::convexSignal = cComponent::registerSignal("convex");
simsignal_t TcpCubic::friendlySignal = cComponent::registerSignal("friendly");

TcpCubic::TcpCubic() :
        TcpTahoeRenoFamily(), state(
                (TcpCubicStateVariables*&) TcpAlgorithm::state) {
}

std::string TcpCubicStateVariables::str() const {
    std::stringstream out;
    out << TcpTahoeRenoFamilyStateVariables::str();
    return out.str();
}

std::string TcpCubicStateVariables::detailedInfo() const {
    std::stringstream out;
    out << TcpTahoeRenoFamilyStateVariables::detailedInfo();
    return out.str();
}

void TcpCubic::reset() {
    state->cnt = 0;
    state->last_max_cwnd = 0;
    state->loss_cwnd = 0;
    state->last_cwnd = 0;
    state->last_time = 0;
    state->bic_origin_point = 0;
    state->bic_K = 0;
    state->delay_min = 0;
    state->epoch_start = 0;
    state->ack_cnt = 0;
    state->tcp_cwnd = 0;

    conn->emit(cntSignal, state->cnt);
    conn->emit(lastMaxWindowSignal, state->last_max_cwnd);
    conn->emit(originPointSignal, state->bic_origin_point);
    conn->emit(bicKSignal, state->bic_K);
}

void TcpCubic::initialize() {
    TcpTahoeRenoFamily::initialize();
    reset();
    /* Precompute a bunch of the scaling factors that are used per-packet
     * based on SRTT of 100ms
     */

    state->beta_scale = 8 * (BICTCP_BETA_SCALE + state->beta) / 3
            / (BICTCP_BETA_SCALE - state->beta);

    state->cube_rtt_scale = (state->bic_scale << 3) / 10; /* 1024*c/rtt */

    /* calculate the "K" for (wmax-cwnd) = c/rtt * K^3
     *  so K = cubic_root( (wmax-cwnd)*rtt/c )
     * the unit of K is bictcp_HZ=2^10, not HZ
     *
     *  c = bic_scale >> 10
     *  rtt = 100ms
     *
     * the following code has been designed and tested for
     * cwnd < 1 million packets
     * RTT < 100 seconds
     * HZ < 1,000,00  (corresponding to 10 nano-second)
     */

    /* 1/c * 2^2*bictcp_HZ * srtt */
    state->cube_factor = 1ull << (10 + 3 * BICTCP_HZ); /* 2^40 */

    /* divide by bic_scale and by constant Srtt (100ms) */
    state->cube_factor /= state->bic_scale * 10;

}

uint32_t TcpCubic::calculateCubicRoot(uint64_t number) {
    return std::cbrt(number);
}

void TcpCubic::updateCubicCwnd(uint32_t acked) {

    uint64_t offs;
    uint32_t delta, t, bic_target, min_cnt, max_cnt;

    uint32_t cwnd = state->snd_cwnd / state->snd_mss;

    //In the kernel code this is the number of jiffies.
    //The number of jiffies is incremented HZ times per second
    //tcp_time_stamp is in ms unit. The jiffy variable would match if HZ = 1000
    uint32_t tcp_time_stamp = simTime().inUnit(SIMTIME_MS);

    state->ack_cnt++; /* count the number of ACKs */

    if (state->last_cwnd == cwnd
            && (int32_t) (tcp_time_stamp - state->last_time) <= HZ / 32)
        return;

    state->last_cwnd = cwnd;
    state->last_time = tcp_time_stamp;

    if (state->epoch_start == 0) {
        state->epoch_start = tcp_time_stamp; /* record the beginning of an epoch */
        state->ack_cnt = 1; /* start counting */
        state->tcp_cwnd = cwnd; /* syn with cubic */

        if (state->last_max_cwnd <= cwnd) {
            state->bic_K = 0;
            state->bic_origin_point = cwnd;
        } else {
            /* Compute new K based on
             * (wmax-cwnd) * (srtt>>3 / HZ) / c * 2^(3*bictcp_HZ)
             */
            state->bic_K = calculateCubicRoot(
                    state->cube_factor * (state->last_max_cwnd - cwnd));
            state->bic_origin_point = state->last_max_cwnd;
        }
    }

    /* cubic function - calc*/
    /* calculate c * time^3 / rtt,
     *  while considering overflow in calculation of time^3
     * (so time^3 is done by using 64 bit)
     * and without the support of division of 64bit numbers
     * (so all divisions are done by using 32 bit)
     *  also NOTE the unit of those veriables
     *    time  = (t - K) / 2^bictcp_HZ
     *    c = bic_scale >> 10
     * rtt  = (srtt >> 3) / HZ
     * !!! The following code does not have overflow problems,
     * if the cwnd < 1 million packets !!!
     */

    /* change the unit from HZ to bictcp_HZ */
    t = ((tcp_time_stamp + state->delay_min - state->epoch_start) << BICTCP_HZ)
            / HZ;

    if (t < state->bic_K) /* t - K */
        offs = state->bic_K - t;
    else
        offs = t - state->bic_K;

    /* c/rtt * (t-K)^3 */
    delta = (state->cube_rtt_scale * offs * offs * offs)
            >> (10 + 3 * BICTCP_HZ);
    if (t < state->bic_K) /* below origin*/
        bic_target = state->bic_origin_point - delta;
    else
        /* above origin*/
        bic_target = state->bic_origin_point + delta;

    /* cubic function - calc bictcp_cnt*/
    if (bic_target > cwnd) {
        state->cnt = cwnd / (bic_target - cwnd);
    } else {
        state->cnt = 100 * cwnd; /* very small increment*/
    }

    if (state->delay_min > 0) {
        /* max increment = Smax * rtt / 0.1  */
        min_cnt = (cwnd * HZ * 8)
                / (10 * state->max_increment * state->delay_min);
        if (state->cnt < min_cnt)
            state->cnt = min_cnt;
    }

    /* slow start and low utilization  */
    if (state->loss_cwnd == 0) /* could be aggressive in slow start */
        state->cnt = 50;

    /* TCP Friendly */
    if (state->tcp_friendliness) {
        uint32_t scale = state->beta_scale;
        delta = (cwnd * scale) >> 3;
        while (state->ack_cnt > delta) { /* update tcp cwnd */
            state->ack_cnt -= delta;
            state->tcp_cwnd++;
        }

        if (state->tcp_cwnd > cwnd) { /* if bic is slower than tcp */
            delta = state->tcp_cwnd - cwnd;
            max_cnt = cwnd / delta;
            if (state->cnt > max_cnt)
                state->cnt = max_cnt;
        }
    }

    conn->emit(cntSignal, state->cnt);
}

void TcpCubic::recalculateSlowStartThreshold() {

    state->epoch_start = 0; /* end of epoch */

    uint32_t cwnd = state->snd_cwnd / state->snd_mss;

    /* Wmax and fast convergence */
    if (cwnd <= state->last_max_cwnd && state->fast_convergence)
        state->last_max_cwnd = (cwnd * (BICTCP_BETA_SCALE + state->beta))
                / (2 * BICTCP_BETA_SCALE);
    else
        state->last_max_cwnd = cwnd;

    state->loss_cwnd = cwnd;

    state->ssthresh = uint32_t(
            std::max((cwnd * state->beta) / BICTCP_BETA_SCALE, 2U))
            * state->snd_mss;

    conn->emit(cwndSignal, state->snd_cwnd);
    conn->emit(ssthreshSignal, state->ssthresh);
    conn->emit(lastMaxWindowSignal, state->last_max_cwnd);
    conn->emit(cwndSegSignal, cwnd);

}

void TcpCubic::processRexmitTimer(TcpEventCode &event) {
    TcpTahoeRenoFamily::processRexmitTimer(event);
//    RFC 5681:
//    On the other hand, when a TCP sender detects segment loss using the
//    retransmission timer and the given segment has already been
//    retransmitted by way of the retransmission timer at least once, the
//    value of ssthresh is held constant.
//    Furthermore, upon a timeout (as specified in [RFC2988]) cwnd MUST be
//    set to no more than the loss window, LW, which equals 1 full-sized
//    segment (regardless of the value of IW).

    std::cerr << "RTO at " << simTime() << std::endl;
    std::cerr << "cwnd=: " << state->snd_cwnd / state->snd_mss << ", in-flight="
            << (state->snd_max - state->snd_una) / state->snd_mss << std::endl;
    if (event == TCP_E_ABORT)
        return;

    reset();
    recalculateSlowStartThreshold();
    conn->emit(recoverSignal, state->recover);
    state->snd_cwnd = state->snd_mss;

    state->afterRto = true;
    conn->retransmitOneSegment(true);
    conn->emit(highRxtSignal, state->highRxt);

    conn->emit(cwndSignal, state->snd_cwnd);
    conn->emit(ssthreshSignal, state->ssthresh);
    conn->emit(cwndSegSignal, state->snd_cwnd / state->snd_mss);

}

void TcpCubic::receivedDataAck(uint32_t firstSeqAcked) {
    TcpTahoeRenoFamily::receivedDataAck(firstSeqAcked);

    if (state->delay_min == 0
            || state->delay_min > state->last_rtt.inUnit(SIMTIME_MS))
        state->delay_min = state->last_rtt.inUnit(SIMTIME_MS);

    if (state->snd_cwnd < state->ssthresh) {
        EV_INFO
                       << "cwnd <= ssthresh: Slow Start: increasing cwnd by one SMSS bytes to ";

        // perform Slow Start. RFC 2581: "During slow start, a TCP increments cwnd
        // by at most SMSS bytes for each ACK received that acknowledges new data."
        state->snd_cwnd += state->snd_mss;

        conn->emit(cwndSignal, state->snd_cwnd);
        conn->emit(ssthreshSignal, state->ssthresh);

        EV_INFO << "cwnd=" << state->snd_cwnd << "\n";
    } else {
        // perform Congestion Avoidance (RFC 2581)
        updateCubicCwnd(1);
        if (state->cwnd_cnt >= state->cnt) {
            state->snd_cwnd += state->snd_mss;
            state->cwnd_cnt = 0;
        } else {
            state->cwnd_cnt++;
        }

        conn->emit(cwndSignal, state->snd_cwnd);
        conn->emit(ssthreshSignal, state->ssthresh);

        //
        // Note: some implementations use extra additive constant mss / 8 here
        // which is known to be incorrect (RFC 2581 p5)
        //
        // Note 2: RFC 3465 (experimental) "Appropriate Byte Counting" (ABC)
        // would require maintaining a bytes_acked variable here which we don't do
        //

        EV_INFO
                       << "cwnd > ssthresh: Congestion Avoidance: increasing cwnd linearly, to "
                       << state->snd_cwnd << "\n";
    }

    // Check if recovery phase has ended
    if (state->lossRecovery && state->sack_enabled) {
        if (seqGE(state->snd_una, state->recoveryPoint)) {
            EV_INFO << "Loss Recovery terminated.\n";
            state->lossRecovery = false;
            conn->emit(lossRecoverySignal, 0);
        }
    }

    // Send data, either in the recovery mode or normal mode
    if (state->lossRecovery) {
        conn->setPipe();

        // RFC 3517, page 7: "(C) If cwnd - pipe >= 1 SMSS the sender SHOULD transmit one or more
        // segments as follows:"
        if (((int) (state->snd_cwnd / state->snd_mss)
                - (int) (state->pipe / (state->snd_mss - 12))) >= 1) // Note: Typecast needed to avoid prohibited transmissions
            conn->sendDataDuringLossRecoveryPhase(state->snd_cwnd);
    } else {
        sendData(false);
    }

    conn->emit(cwndSegSignal, state->snd_cwnd / state->snd_mss);

}

void TcpCubic::receivedDuplicateAck() {
    TcpTahoeRenoFamily::receivedDuplicateAck();

//      When a TCP sender receives the duplicate ACK corresponding to
//      DupThresh ACKs, the scoreboard MUST be updated with the new SACK
//      information (via Update ()).  If no previous loss event has occurred
//      on the connection or the cumulative acknowledgment point is beyond
//      the last value of RecoveryPoint, a loss recovery phase SHOULD be
//      initiated, per the fast retransmit algorithm outlined in [RFC2581].
    if (state->dupacks >= state->dupthresh) {
        if (!state->lossRecovery
                && (state->recoveryPoint == 0
                        || seqGE(state->snd_una, state->recoveryPoint))) {

            state->recoveryPoint = state->snd_max; // HighData = snd_max
            state->lossRecovery = true;
            EV_DETAIL << " recoveryPoint=" << state->recoveryPoint;
            conn->emit(lossRecoverySignal, 1);

            // enter Fast Recovery
            recalculateSlowStartThreshold();
            // "set cwnd to ssthresh plus 3 * SMSS." (RFC 2581)
            state->snd_cwnd = state->ssthresh;

            conn->emit(cwndSignal, state->snd_cwnd);

            EV_DETAIL << " set cwnd=" << state->snd_cwnd << ", ssthresh="
                             << state->ssthresh << "\n";

            // Fast Retransmission: retransmit missing segment without waiting
            // for the REXMIT timer to expire
            conn->retransmitOneSegment(false);
            conn->emit(highRxtSignal, state->highRxt);
        }
        // perform Congestion Avoidance (RFC 2581)
        updateCubicCwnd(1);
        if (state->cwnd_cnt >= state->cnt) {
            state->snd_cwnd += state->snd_mss;
            state->cwnd_cnt = 0;
        } else {
            state->cwnd_cnt++;
        }

        conn->emit(cwndSignal, state->snd_cwnd);
        conn->emit(ssthreshSignal, state->ssthresh);

    }

    if (state->lossRecovery) {
        conn->setPipe();

        if (((int) (state->snd_cwnd / state->snd_mss)
                - (int) (state->pipe / (state->snd_mss - 12))) >= 1) { // Note: Typecast needed to avoid prohibited transmissions
            conn->sendDataDuringLossRecoveryPhase(state->snd_cwnd);
            restartRexmitTimer();
        }
    }
}

} //tcp
} //inet
