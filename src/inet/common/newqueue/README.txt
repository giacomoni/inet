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