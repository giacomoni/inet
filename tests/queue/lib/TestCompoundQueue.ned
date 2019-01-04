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

package test.queue;

import inet.common.newqueue.CompoundQueue;
import inet.common.newqueue.Prioritizer;
import inet.common.newqueue.InfiniteQueue;
import test.queue.TestClassifier;

module TestCompoundQueue extends CompoundQueue
{
    submodules:
        classifier: TestClassifier {
            @display("p=100,100");
        }
        queue1: InfiniteQueue {
            @display("p=200,50");
        }
        queue2: InfiniteQueue {
            @display("p=200,150");
        }
        prioritizer: Prioritizer {
            @display("p=300,100");
        }
    connections:
        in --> { @display("m=w"); } --> classifier.in;
        classifier.out++ --> queue1.in;
        classifier.out++ --> queue2.in;
        queue1.out --> prioritizer.in++;
        queue2.out --> prioritizer.in++;
        prioritizer.out --> { @display("m=e"); } --> out;
}
