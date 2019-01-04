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

#include "inet/common/packet/chunk/BitCountChunk.h"
#include "inet/common/packet/Packet.h"
#include "TestSource.h"

namespace inet {
namespace queue {

Define_Module(TestSource);

void TestSource::initialize()
{
    scheduleAt(simTime() + par("interval"), &timer);
}

void TestSource::handleMessage(cMessage *msg)
{
    char name[32];
    sprintf(name, "%d", count++);
    auto length = b(intuniform(0, 1));
    auto content = makeShared<BitCountChunk>(length);
    auto packet = new Packet(name, content);
    send(packet, "out");
    std::cout << "Packet sent: " << packet << "\n";
    scheduleAt(simTime() + par("interval"), &timer);
}

} // namespace queue
} // namespace inet

