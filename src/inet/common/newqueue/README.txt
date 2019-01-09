The queue API should simulatenously support the following:

different external structure:
 - queues as submodules without being connected to the outside world via gates
   the queue is not in the packet path, it's rather part of the processing module
 - queues which are connected to generators and consumers
   the queue is in the packet path

different modes of operation:
 - queues must be able to operate synchronously without utilizing handleMessage
   a packet getting into a queue may immediately cause another packet to get out from the queue
 - queues must be able to operate asynchronously with utilizing handleMessage
   this is the old INET 4.0 behaviour

abstraction via composition:
 - queues can be simple modules
 - queues can be composed from simple modules into compound queue modules

easy access:
 - protocols should be able to use queues without knowing what is the queue's
   module structure, mode of operation, simple or compound, etc.

IPacketQueue
PacketQueueBase
TailDropQueue
Prioritizer -> PriorityQueue
InfiniteQueue/FifoQueue
InfiniteStack/LifoQueue
CompoundQueue

Join/Fork/Delay

output gate:
 - connected or free
 - synchronous or asynchronous
 - push or pull

input gate:
 - connected or free

what can be connected to where exactly? it's not clear what is possible with composition!

what doesn't work, for example:
 - Queue -> Queue getNumPackets()? popPacket()? requestPacket()?
 - Queue -> Sink no requestPacket()?
 - Queue -> Delayer ???
 - SchedulerBase overrides WrrScheduler behavior in pop()

primitive modules:
 - infinite queue (1 in gate, 1 out gate)
 - limited queue (1 in gate, 1 out gate, capacity parameter)
 - packet classifier (1 in gate, N out gate, classifier function parameter)
 - priority queue (N in gate, 1 out gate, priority function parameter)
 - wrr scheduler 
 - delay (1 in gate, 1 out gate, delay parameter)
   when pushPacket() is called, it calls pushPacket() on out after delay
   when requestPacket() is called, it calls requestPacket() on in gate
 - drop (1 in gate, 1 out gate, drop predicate parameter)
   when pushPacket() is called, it calls pushPacket() on out gate if drop predicate returns false
   when requestPacket() is called, it calls requestPacket() on in gate
 - source (1 out gate, interval parameter, packet generator function parameter)
   calls pushPacket() on out gate periodically with a newly generated packet
 - sink (1 in gate)
   calls requestPacket() on in gate initially and every time it receives a packet
   when pushPacket() is called, it deletes the packet

question: does it make sense to connect any out gate to any in gate? if not, then what are the constraints?

operations:
 - getNumPackets(): returns the number of packets of what exactly?
 - pushPacket()
 - popPacket()
 - requestPacket()
