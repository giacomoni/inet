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

#include "inet/common/newqueue/CompoundQueue.h"

namespace inet {
namespace queue {

Define_Module(CompoundQueue);

bool CompoundQueue::InputGate::deliver(cMessage *msg, simtime_t at)
{
    bool result = cGate::deliver(msg, at);
    auto compoundQueue = check_and_cast<CompoundQueue *>(getOwnerModule());
    compoundQueue->scheduleHandlePendingRequest();
    return result;
}

cGate *CompoundQueue::createGateObject(cGate::Type type)
{
    if (type == cGate::INPUT)
        return new InputGate();
    else
        return cModule::createGateObject(type);
}

void CompoundQueue::initialize()
{
    inputGate = gate("in");
    outputGate = gate("out");
    inputQueue = check_and_cast<IPacketQueue *>(gate("in")->getPathEndGate()->getOwnerModule());
    outputQueue = check_and_cast<IPacketQueue *>(gate("out")->getPathStartGate()->getOwnerModule());
}

void CompoundQueue::handleMessage(cMessage *msg)
{
    if (msg == &handle) {
        if (hasPendingRequestPacket)
            handlePendingRequestPacket();
    }
    else
        throw cRuntimeError("Unknown message");
}

void CompoundQueue::handlePendingRequestPacket()
{
    if (!isEmpty()) {
        outputQueue->requestPacket();
        hasPendingRequestPacket = false;
    }
}

void CompoundQueue::scheduleHandlePendingRequest()
{
    scheduleAt(simTime(), &handle);
}

int CompoundQueue::getNumPackets()
{
    return outputQueue->getNumPackets();
}

void CompoundQueue::pushPacket(Packet *packet)
{
    animateSend(packet, inputGate);
    inputQueue->pushPacket(packet);
    if (hasPendingRequestPacket)
        handlePendingRequestPacket();
}

Packet *CompoundQueue::popPacket()
{
    auto packet = outputQueue->popPacket();
    animateSend(packet, outputGate->getPathStartGate());
    return packet;
}

} // namespace queue
} // namespace inet
