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

#include "inet/common/newqueue/DropTailQueue.h"

namespace inet {
namespace queue {

Define_Module(DropTailQueue);

void DropTailQueue::initialize()
{
    frameCapacity = par("frameCapacity");
}

int DropTailQueue::getNumPackets()
{
    return queue.getLength();
}

void DropTailQueue::pushPacket(Packet *packet)
{
    if (frameCapacity && queue.getLength() >= frameCapacity)
        delete packet;
    else {
        queue.insert(packet);
        if (hasPendingRequestPacket)
            handlePendingRequestPacket();
    }
}

Packet *DropTailQueue::popPacket()
{
    return check_and_cast<Packet *>(queue.pop());
}

} // namespace queue
} // namespace inet
