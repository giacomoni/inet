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

#ifndef __INET_QUEUEACCESSOR_H
#define __INET_QUEUEACCESSOR_H

#include "inet/common/newqueue/IPacketQueue.h"

namespace inet {
namespace queue {

class INET_API QueueAccessor
{
  public:
    class INET_API ICallback
    {
      public:
        virtual bool isDequeingPacketEnabled() = 0;
        virtual void processDequedPacket(Packet *packet) = 0;
    };

  protected:
    bool internalQueue = false;
    IPacketQueue *queue = nullptr;
    ICallback *callback = nullptr;

  public:
    QueueAccessor(cModule *submodule, cGate *gate, ICallback *callback);
    virtual ~QueueAccessor() { }

    virtual IPacketQueue *getQueue() const { return queue; }
    virtual void startDequeingPacket();
    virtual void handlePacket(Packet *packet);
};

} // namespace queue
} // namespace inet

#endif // ifndef __INET_QUEUEACCESSOR_H

