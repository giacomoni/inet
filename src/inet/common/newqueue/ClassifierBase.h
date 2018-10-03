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

#ifndef __INET_CLASSIFIERBASE_H
#define __INET_CLASSIFIERBASE_H

#include "inet/common/newqueue/PacketQueueBase.h"

namespace inet {
namespace queue {

class INET_API ClassifierBase : public PacketQueueBase
{
  protected:
    std::vector<cGate *> outputGates;
    std::vector<IPacketQueue *> outputQueues;

  protected:
    virtual void initialize() override;
    virtual int classifyPacket(Packet *packet) const = 0;

  public:
    virtual int getNumPackets() override;
    virtual void pushPacket(Packet *packet) override;
    virtual Packet *popPacket() override;
};

} // namespace queue
} // namespace inet

#endif // ifndef __INET_CLASSIFIERBASE_H

