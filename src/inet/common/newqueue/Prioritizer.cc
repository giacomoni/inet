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

#include "inet/common/newqueue/Prioritizer.h"

namespace inet {
namespace queue {

Define_Module(Prioritizer);

void Prioritizer::initialize()
{
    for (int i = 0; i < gateSize("in"); i++) {
        auto inputGate = gate("in", i);
        auto inputQueue = check_and_cast<IPacketQueue *>(inputGate->getPathStartGate()->getOwnerModule());
        inputGates.push_back(inputGate);
        inputQueues.push_back(inputQueue);
    }
}

int Prioritizer::getNumPackets()
{
    int size = 0;
    for (int i = 0; i < gateSize("in"); i++)
        size += inputQueues[i]->getNumPackets();
    return size;
}

void Prioritizer::pushPacket(Packet *packet)
{
    throw cRuntimeError("Invalid operation");
}

Packet *Prioritizer::popPacket()
{
    for (int i = 0; i < gateSize("in"); i++) {
        auto queue = inputQueues[i];
        if (queue->getNumPackets() != 0) {
            auto packet = queue->popPacket();
            animateSend(packet, inputGates[i]->getPathStartGate());
            return packet;
        }
    }
    throw cRuntimeError("Invalid operation");
}

} // namespace queue
} // namespace inet
