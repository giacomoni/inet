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

#ifndef __INET_IPACKETQUEUE_H
#define __INET_IPACKETQUEUE_H

#include "inet/common/packet/Packet.h"

namespace inet {
namespace queue {

/**
 * This class defines the interface for packet queues.
 */
class INET_API IPacketQueue
{
  public:
    /**
     * Returns the number of available packets in the queue.
     */
    virtual int getNumPackets() = 0;

    /**
     * Returns true if there are no packets available in the queue.
     */
    virtual bool isEmpty() = 0;

    /**
     * Inserts a packet into the queue.
     */
    virtual void pushPacket(Packet *packet) = 0;

    /**
     * Removes a packet from the queue. The queue must not be empty.
     */
    virtual Packet *popPacket() = 0;

    /**
     * Requests the queue to send out a packet. If a packet is readily available,
     * then it is sent out immediately. Otherwise, as soon as a packet becomes
     * available, it is sent out.
     */
    virtual void requestPacket() = 0;
};

} // namespace queue
} // namespace inet

#endif // ifndef __INET_IPACKETQUEUE_H

