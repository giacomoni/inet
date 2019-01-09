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
#include "inet/common/newqueue/Delayer.h"

namespace inet {
namespace queue {

Define_Module(Delayer);

void Delayer::initialize()
{
    auto outGate = gate("out");
    if (outGate->isConnected())
        sink = dynamic_cast<IPacketSink *>(outGate->getPathEndGate()->getOwnerModule());
    else
        sink = getModuleFromPar<IPacketSink>(par("outModule"), this);
}

void Delayer::handleMessage(cMessage *message)
{
    auto packet = check_and_cast<Packet *>(message);
    if (packet->isSelfMessage()) {
        if (sink == nullptr)
            send(packet, "out");
        else
            sink->processPacket(packet);
    }
    else
        processPacket(packet);
}

void Delayer::processPacket(Packet *packet)
{
    scheduleAt(simTime() + par("delay"), packet);
}

} // namespace queue
} // namespace inet
