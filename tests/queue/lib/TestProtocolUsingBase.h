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

#ifndef __INET_TESTPROTOCOLUSINGBASE_H
#define __INET_TESTPROTOCOLUSINGBASE_H

#include "inet/common/newqueue/IPacketQueue.h"
#include "inet/common/newqueue/QueueingProtocolBase.h"

namespace inet {
namespace queue {

class INET_API TestProtocolUsingBase : public cSimpleModule, public QueueingProtocolBase
{
  protected:
    cMessage timer;
    Packet *packet = nullptr;

  protected:
    virtual ~TestProtocolUsingBase() { cancelEvent(&timer); delete packet; }
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual bool isDequeingPacketEnabled() override;
    virtual void processDequedPacket(Packet *packet) override;
};

} // namespace queue
} // namespace inet

#endif // ifndef __INET_TESTPROTOCOLUSINGBASE_H

