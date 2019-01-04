//
// Copyright (C) OpenSim Ltd.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see http://www.gnu.org/licenses/.
//

#include "TestProtocolUsingAccessor.h"

namespace inet {
namespace queue {

Define_Module(TestProtocolUsingAccessor);

void TestProtocolUsingAccessor::initialize(int stage)
{
    if (stage == INITSTAGE_LOCAL)
        queueAccessor = new QueueAccessor(getSubmodule("queue"), gate("in"), this);
    else if (stage == INITSTAGE_LAST)
        queueAccessor->startDequeingPacket();
}

void TestProtocolUsingAccessor::handleMessage(cMessage *msg)
{
    if (msg == &timer) {
        send(packet, "out");
        packet = nullptr;
        queueAccessor->startDequeingPacket();
    }
    else if (msg->isPacket()) {
        auto packet = check_and_cast<Packet *>(msg);
        queueAccessor->handlePacket(packet);
    }
}

bool TestProtocolUsingAccessor::isDequeingPacketEnabled()
{
    return !timer.isScheduled();
}

void TestProtocolUsingAccessor::processDequedPacket(Packet *packet)
{
    this->packet = packet;
    scheduleAt(simTime() + par("interval"), &timer);
}

} // namespace queue
} // namespace inet

