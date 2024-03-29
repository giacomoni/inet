//
// Copyright (C) 2009-2010 Thomas Reschka
// Copyright (C) 2011 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#include "inet/transportlayer/tcp/TcpSackRexmitQueue.h"

namespace inet {

namespace tcp {

TcpSackRexmitQueue::TcpSackRexmitQueue()
{
    conn = nullptr;
    begin = end = 0;
}

TcpSackRexmitQueue::~TcpSackRexmitQueue()
{
    while (!rexmitQueue.empty())
        rexmitQueue.pop_front();
}

void TcpSackRexmitQueue::init(uint32_t seqNum)
{
    begin = seqNum;
    end = seqNum;
}

std::string TcpSackRexmitQueue::str() const
{
    std::stringstream out;

    out << "[" << begin << ".." << end << ")";
    return out.str();
}

std::string TcpSackRexmitQueue::detailedInfo() const
{
    std::stringstream out;
    out << str() << endl;

    uint j = 1;

    for (const auto& elem : rexmitQueue) {
        out << j << ". region: [" << elem.beginSeqNum << ".." << elem.endSeqNum
            << ") \t sacked=" << elem.sacked << "\t rexmitted=" << elem.rexmitted
            << endl;
        j++;
    }
    return out.str();
}

void TcpSackRexmitQueue::discardUpTo(uint32_t seqNum)
{
    ASSERT(seqLE(begin, seqNum) && seqLE(seqNum, end));

    if (!rexmitQueue.empty()) {
        auto i = rexmitQueue.begin();

        while ((i != rexmitQueue.end()) && seqLE(i->endSeqNum, seqNum)) // discard/delete regions from rexmit queue, which have been acked
            i = rexmitQueue.erase(i);

        if (i != rexmitQueue.end()) {
            ASSERT(seqLE(i->beginSeqNum, seqNum) && seqLess(seqNum, i->endSeqNum));
            i->beginSeqNum = seqNum;
        }
    }

    begin = seqNum;

    // TESTING queue:
    ASSERT(checkQueue());
}

void TcpSackRexmitQueue::enqueueSentData(uint32_t fromSeqNum, uint32_t toSeqNum)
{
    ASSERT(seqLE(begin, fromSeqNum) && seqLE(fromSeqNum, end));

    bool found = false;
    Region region;

    EV_INFO << "rexmitQ: " << str() << " enqueueSentData [" << fromSeqNum << ".." << toSeqNum << ")\n";

    ASSERT(seqLess(fromSeqNum, toSeqNum));


    if (rexmitQueue.empty()) { //If queue is empty, then just create new region and insert
        region.beginSeqNum = fromSeqNum;
        region.endSeqNum = toSeqNum;
        region.sacked = false;
        region.rexmitted = false;
        rexmitQueue.push_back(region);
        found = true;
        fromSeqNum = toSeqNum;
    }
    else {
        auto lastElem = std::prev(rexmitQueue.end());
        if(end == fromSeqNum){ //First time sending this data
            if(!lastElem->sacked && !lastElem->rexmitted){ // Last sent region has not been rexmitted or sacked: we can extend it
                lastElem->endSeqNum = toSeqNum;
                found = true;
                fromSeqNum = toSeqNum;
            }else{ // Last sent region has been rexmitter or sacked, we have to create a new region and insert it
                region.beginSeqNum = fromSeqNum;
                region.endSeqNum = toSeqNum;
                region.sacked = false;
                region.rexmitted = false;
                rexmitQueue.push_back(region);
                found = true;
                fromSeqNum = toSeqNum;
            }
        }else{

            auto i = rexmitQueue.begin();


            while (i != rexmitQueue.end() && seqLE(i->endSeqNum, fromSeqNum))
                i++;

            ASSERT(i != rexmitQueue.end());
            ASSERT(seqLE(i->beginSeqNum, fromSeqNum) && seqLess(fromSeqNum, i->endSeqNum));

            auto j = i; // We will need to compare each element's state with the previous elemnt's state. j will point to the previous element

            if (i->beginSeqNum != fromSeqNum) {
                // chunk item. i will point to j+1 because of the new insertion
                region = *i;
                region.endSeqNum = fromSeqNum;
                rexmitQueue.insert(i, region);
                i->beginSeqNum = fromSeqNum;
            }else{
                j = std::prev(j); // we have to decrease j since i has not moved
            }

            while (i != rexmitQueue.end() && seqLE(i->endSeqNum, toSeqNum)) {
                i->rexmitted = true;
                fromSeqNum = i->endSeqNum;
                if(j->rexmitted == i->rexmitted && j->sacked == i->sacked){ //if states of consecutive regions match, we can merge the regions and delete region i. i will point to the next region and j to the region before i
                    j->endSeqNum = i->endSeqNum;
                    i = rexmitQueue.erase(i);
                }else{ //We have to shift both i and j to point to their respective next regions.
                    i++;
                    j++;
                }
                found = true;
            }

            if (fromSeqNum != toSeqNum) {
                bool beforeEnd = (i != rexmitQueue.end());

                ASSERT(i == rexmitQueue.end() || seqLess(i->beginSeqNum, toSeqNum));

                region.beginSeqNum = fromSeqNum;
                region.endSeqNum = toSeqNum;
                region.sacked = beforeEnd ? i->sacked : false;
                region.rexmitted = beforeEnd;
                rexmitQueue.insert(i, region);
                found = true;
                fromSeqNum = toSeqNum;

                if (beforeEnd)
                    i->beginSeqNum = toSeqNum;
            }
        }
    }

    ASSERT(fromSeqNum == toSeqNum);

    if (!found) {
        EV_DEBUG << "Not found enqueueSentData(" << fromSeqNum << ", " << toSeqNum << ")\nThe Queue is:\n" << detailedInfo();
    }

    ASSERT(found);

    begin = rexmitQueue.front().beginSeqNum;
    end = rexmitQueue.back().endSeqNum;

    // TESTING queue:
    ASSERT(checkQueue());

//    tcpEV << "rexmitQ: rexmitQLength=" << getQueueLength() << "\n";
}

bool TcpSackRexmitQueue::checkQueue() const
{
    uint32_t b = begin;
    bool f = true;

    for (const auto& elem : rexmitQueue) {
        f = f && (b == elem.beginSeqNum);
        f = f && seqLess(elem.beginSeqNum, elem.endSeqNum);
        b = elem.endSeqNum;
    }

    f = f && (b == end);

    if (!f) {
        EV_DEBUG << "Invalid Queue\nThe Queue is:\n" << detailedInfo();
    }

    return f;
}

void TcpSackRexmitQueue::setSackedBit(uint32_t fromSeqNum, uint32_t toSeqNum)
{
    if (seqLess(fromSeqNum, begin))
        fromSeqNum = begin;

    ASSERT(seqLess(fromSeqNum, end));
    ASSERT(seqLess(begin, toSeqNum) && seqLE(toSeqNum, end));
    ASSERT(seqLess(fromSeqNum, toSeqNum));

    bool found = false;

    if (!rexmitQueue.empty()) {
        auto i = rexmitQueue.begin();

        while (i != rexmitQueue.end() && seqLE(i->endSeqNum, fromSeqNum))
            i++;

        ASSERT(i != rexmitQueue.end() && seqLE(i->beginSeqNum, fromSeqNum) && seqLess(fromSeqNum, i->endSeqNum));

        auto j = i; // We will need to compare each element's state with the previous elemnt's state. j will point to the previous element

        if (i->beginSeqNum != fromSeqNum) {
            Region region = *i;

            region.endSeqNum = fromSeqNum;
            rexmitQueue.insert(i, region); // i will point to j+1 because of the new insertion
            i->beginSeqNum = fromSeqNum;
        }else{
            j = std::prev(j); // we have to decrease j since i has not moved
        }

        while (i != rexmitQueue.end() && seqLE(i->endSeqNum, toSeqNum)) {
            if (seqGE(i->beginSeqNum, fromSeqNum)) { // Search region in queue!
                found = true;
                i->sacked = true; // set sacked bit
            }
            if(i->rexmitted == j->rexmitted && i->sacked == j->sacked){ //if states of consecutive regions match, we can merge the regions and delete region i. i will point to the next region and j to the region before i.
                j->endSeqNum = i->endSeqNum;
                i = rexmitQueue.erase(i);
            }else{ //We have to shift both i and j to point to their respective next regions.
            j++;
            i++;
            }
        }

        if (i != rexmitQueue.end() && seqLess(i->beginSeqNum, toSeqNum) && seqLess(toSeqNum, i->endSeqNum)) {
            Region region = *i;

            region.endSeqNum = toSeqNum;
            region.sacked = true;
            rexmitQueue.insert(i, region);
            i->beginSeqNum = toSeqNum;
        }
    }

    if (!found)
        EV_DETAIL << "FAILED to set sacked bit for region: [" << fromSeqNum << ".." << toSeqNum << "). Not found in retransmission queue.\n";

    ASSERT(checkQueue());
}

bool TcpSackRexmitQueue::getSackedBit(uint32_t seqNum) const
{
    ASSERT(seqLE(begin, seqNum) && seqLE(seqNum, end));

    RexmitQueue::const_iterator i = rexmitQueue.begin();

    if (end == seqNum)
        return false;

    while (i != rexmitQueue.end() && seqLE(i->endSeqNum, seqNum))
        i++;

    ASSERT((i != rexmitQueue.end()) && seqLE(i->beginSeqNum, seqNum) && seqLess(seqNum, i->endSeqNum));

    return i->sacked;
}

uint32_t TcpSackRexmitQueue::getHighestSackedSeqNum() const
{
    for (RexmitQueue::const_reverse_iterator i = rexmitQueue.rbegin(); i != rexmitQueue.rend(); i++) {
        if (i->sacked)
            return i->endSeqNum;
    }

    return begin;
}

uint32_t TcpSackRexmitQueue::getHighestRexmittedSeqNum() const
{
    for (RexmitQueue::const_reverse_iterator i = rexmitQueue.rbegin(); i != rexmitQueue.rend(); i++) {
        if (i->rexmitted)
            return i->endSeqNum;
    }

    return begin;
}

uint32_t TcpSackRexmitQueue::checkRexmitQueueForSackedOrRexmittedSegments(uint32_t fromSeqNum) const
{
    ASSERT(seqLE(begin, fromSeqNum) && seqLE(fromSeqNum, end));

    if (rexmitQueue.empty() || (end == fromSeqNum))
        return 0;

    RexmitQueue::const_iterator i = rexmitQueue.begin();
    uint32_t bytes = 0;

    while (i != rexmitQueue.end() && seqLE(i->endSeqNum, fromSeqNum))
        i++;

    while (i != rexmitQueue.end() && ((i->sacked || i->rexmitted))) {
        ASSERT(seqLE(i->beginSeqNum, fromSeqNum) && seqLess(fromSeqNum, i->endSeqNum));

        bytes += (i->endSeqNum - fromSeqNum);
        fromSeqNum = i->endSeqNum;
        i++;
    }

    return bytes;
}

void TcpSackRexmitQueue::resetSackedBit()
{
    for (auto& elem : rexmitQueue)
        elem.sacked = false; // reset sacked bit
}

void TcpSackRexmitQueue::resetRexmittedBit()
{
    for (auto& elem : rexmitQueue)
        elem.rexmitted = false; // reset rexmitted bit
}

uint32_t TcpSackRexmitQueue::getTotalAmountOfSackedBytes() const
{
    uint32_t bytes = 0;

    for (const auto& elem : rexmitQueue) {
        if (elem.sacked)
            bytes += (elem.endSeqNum - elem.beginSeqNum);
    }

    return bytes;
}

uint32_t TcpSackRexmitQueue::getAmountOfSackedBytes(uint32_t fromSeqNum) const
{
    ASSERT(seqLE(begin, fromSeqNum) && seqLE(fromSeqNum, end));

    uint32_t bytes = 0;
    RexmitQueue::const_reverse_iterator i = rexmitQueue.rbegin();

    for (; i != rexmitQueue.rend() && seqLE(fromSeqNum, i->beginSeqNum); i++) {
        if (i->sacked)
            bytes += (i->endSeqNum - i->beginSeqNum);
    }

    if (i != rexmitQueue.rend()
        && seqLess(i->beginSeqNum, fromSeqNum) && seqLess(fromSeqNum, i->endSeqNum) && i->sacked)
    {
        bytes += (i->endSeqNum - fromSeqNum);
    }

    return bytes;
}

uint32_t TcpSackRexmitQueue::getNumOfDiscontiguousSacks(uint32_t fromSeqNum) const
{
    ASSERT(seqLE(begin, fromSeqNum) && seqLE(fromSeqNum, end));

    if (rexmitQueue.empty() || (fromSeqNum == end))
        return 0;

    RexmitQueue::const_iterator i = rexmitQueue.begin();
    uint32_t counter = 0;

    while (i != rexmitQueue.end() && seqLE(i->endSeqNum, fromSeqNum)) // search for seqNum
        i++;

    // search for discontiguous sacked regions
    bool prevSacked = false;

    while (i != rexmitQueue.end()) {
        if (i->sacked && !prevSacked)
            counter++;

        prevSacked = i->sacked;
        i++;
    }

    return counter;
}

void TcpSackRexmitQueue::checkSackBlock(uint32_t fromSeqNum, uint32_t& length, bool& sacked, bool& rexmitted) const
{
    ASSERT(seqLE(begin, fromSeqNum) && seqLess(fromSeqNum, end));

    RexmitQueue::const_iterator i = rexmitQueue.begin();

    while (i != rexmitQueue.end() && seqLE(i->endSeqNum, fromSeqNum)) // search for seqNum
        i++;

    ASSERT(i != rexmitQueue.end());
    ASSERT(seqLE(i->beginSeqNum, fromSeqNum) && seqLess(fromSeqNum, i->endSeqNum));

    length = (i->endSeqNum - fromSeqNum);
    sacked = i->sacked;
    rexmitted = i->rexmitted;
}

} // namespace tcp

} // namespace inet
