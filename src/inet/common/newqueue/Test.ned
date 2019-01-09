//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package inet.common.newqueue;

simple Source
{
    gates:
        output out;
}

simple Sink
{
    gates:
        input in;
}

module Test1
{
    submodules:
        source: Source {
            @display("p=39,78");
        }
        sink: Sink {
            @display("p=291,78");
        }
    connections:
        source.out --> sink.in;
}

module Test2a
{
    submodules:
        source: Source {
            @display("p=39,78");
        }
        queue: InfiniteQueue {
            @display("p=165,78");
        }
        sink: Sink {
            @display("p=291,78");
        }
    connections:
        source.out --> queue.in;
        queue.out --> sink.in;
}

module Test2b
{
    submodules:
        source: Source {
            @display("p=39,78");
        }
        queue: InfiniteQueue {
            @display("p=165,78");
        }
        delayer: Delayer {
            @display("p=265,78");
        }
        sink: Sink {
            @display("p=359,78");
        }
    connections:
        source.out --> queue.in;
        queue.out --> delayer.in;
        delayer.out --> sink.in;
}

module Test3
{
    submodules:
        source: Source {
            @display("p=39,78");
        }
        classifier: ClassifierBase {
            @display("p=135,78");
        }
        queue1: InfiniteQueue {
            @display("p=222,30");
        }
        queue2: InfiniteQueue {
            @display("p=222,129");
        }
        prioritizer: Prioritizer {
            @display("p=319,78");
        }
        sink: Sink {
            @display("p=403,78");
        }
    connections:
        source.out --> classifier.in;
        classifier.out++ --> queue1.in;
        classifier.out++ --> queue2.in;
        queue1.out --> prioritizer.in++;
        queue2.out --> prioritizer.in++;
        prioritizer.out --> sink.in;
}

module Test4
{
    submodules:
        source: Source {
            @display("p=39,78");
        }
        classifier: ClassifierBase {
            @display("p=135,78");
        }
        delayer1: Delayer {
            @display("p=211,32");
        }
        delayer2: Delayer {
            @display("p=211,127");
        }
        queue1: InfiniteQueue {
            @display("p=299,32");
        }
        queue2: InfiniteQueue {
            @display("p=299,127");
        }
        prioritizer: Prioritizer {
            @display("p=379,78");
        }
        sink: Sink {
            @display("p=465,78;is=l");
        }
    connections:
        source.out --> classifier.in;
        classifier.out++ --> delayer1.in;
        classifier.out++ --> delayer2.in;
        delayer1.out --> queue1.in;
        delayer2.out --> queue2.in;
        queue1.out --> prioritizer.in++;
        queue2.out --> prioritizer.in++;
        prioritizer.out --> sink.in;
}
