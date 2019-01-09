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

#include "inet/common/ModuleAccess.h"
#include "inet/common/newqueue/Bypass.h"

namespace inet {
namespace queue {

Define_Module(Bypass);

void Bypass::initX(Output& output, cPar& gateName, cPar& synchronous)
{
    auto outGate = gate(gateName.stringValue());
    if (outGate->isConnected()) {
        if (synchronous.boolValue())
            output.sink = dynamic_cast<IPacketSink *>(outGate->getPathEndGate()->getOwnerModule());
        else
            output.gate = outGate;
    }
}

void Bypass::sendX(Packet *packet, Output& output)
{
    if (output.gate != nullptr)
        send(packet, output.gate);
    else if (output.sink != nullptr)
        output.sink->processPacket(packet);
    else
        throw cRuntimeError("Cannot send packet");
}

void Bypass::handleMessage(cMessage *message)
{
    auto packet = check_and_cast<Packet *>(message);
    sendX(packet, output);
//    sendDirect(packet, in);
//    out->processPacket(packet);
//    // connected or free
//    // synchronous or asynchronous
//    // push or pull
//
//    // 1. push connected async
//    send(packet, "out");
//    // 2. push connected sync
//    sink = dynamic_cast<IPacketSink *>(outGate->getPathEndGate()->getOwnerModule());
//    sink->processPacket(packet);
//    // 3. push free async
//    sink = getModuleFromPar<cModule>(par("outModule"), this);
//    sendDirect(packet, sink, "in");
//    // 4. push free sync
//    sink = getModuleFromPar<IPacketSink>(par("outModule"), this);
//    sink->processPacket(packet);
//    // 5. pull connected async
}

void Bypass::initialize()
{
    initX(output, par("out"), par("synchronous"));
}

//void Bypass::handleMessage(cMessage *message)
//{
//    processPacket(check_and_cast<Packet *>(message));
//}
//
//void Bypass::processPacket(Packet *packet)
//{
//    if (sink == nullptr)
//        send(packet, "out");
//    else
//        sink->processPacket(packet);
//}

} // namespace queue
} // namespace inet
