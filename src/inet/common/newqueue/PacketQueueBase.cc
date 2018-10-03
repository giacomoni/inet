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

#include "inet/common/newqueue/PacketQueueBase.h"

namespace inet {
namespace queue {

void PacketQueueBase::handleMessage(cMessage *msg)
{
    pushPacket(check_and_cast<Packet *>(msg));
}

void PacketQueueBase::requestPacket()
{
    ASSERT(!hasPendingRequestPacket);
    hasPendingRequestPacket = true;
    handlePendingRequestPacket();
}

void PacketQueueBase::handlePendingRequestPacket()
{
    Enter_Method_Silent();
    if (!isEmpty()) {
        send(popPacket(), "out");
        hasPendingRequestPacket = false;
    }
}

void PacketQueueBase::animateSend(Packet *packet, cGate *gate)
{
    auto envir = getEnvir();
    if (envir->isGUI()) {
        envir->beginSend(packet);
        envir->messageSendHop(packet, gate);
        envir->endSend(packet);
    }
}

} // namespace queue
} // namespace inet
