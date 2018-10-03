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

#include "inet/common/newqueue/QueueingProtocolBase.h"

namespace inet {
namespace queue {

void QueueingProtocolBase::initializeQueue(cModule *submodule, cGate *gate)
{
    queue = check_and_cast_nullable<IPacketQueue *>(submodule);
    internalQueue = queue != nullptr;
    if (!internalQueue) {
        while (!dynamic_cast<IPacketQueue *>(gate->getOwnerModule()))
            gate = gate->getPreviousGate();
        queue = dynamic_cast<IPacketQueue *>(gate->getOwnerModule());
        queue->requestPacket();
    }
}

void QueueingProtocolBase::startDequeingPacket()
{
    if (internalQueue) {
        if (queue->getNumPackets() != 0)
            processDequedPacket(queue->popPacket());
    }
    else
        queue->requestPacket();
}

void QueueingProtocolBase::handlePacket(Packet *packet)
{
    if (internalQueue) {
        queue->pushPacket(packet);
        if (isDequeingPacketEnabled())
            processDequedPacket(queue->popPacket());
    }
    else
        processDequedPacket(packet);
}

} // namespace queue
} // namespace inet

