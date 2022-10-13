#!/usr/bin/python3
import math

from CS312Graph import *
import time


class NetworkRoutingSolver:
    def __init__(self):
        self.array = []
        self.heap = None
        pass

    def initializeNetwork(self, network):
        assert( type(network) == CS312Graph )
        self.network = network

    def getShortestPath( self, destIndex ):
        self.dest = destIndex
        # TODO: RETURN THE SHORTEST PATH FOR destIndex
        #       INSTEAD OF THE DUMMY SET OF EDGES BELOW
        #       IT'S JUST AN EXAMPLE OF THE FORMAT YOU'LL 
        #       NEED TO USE
        path_edges = []
        total_length = 0
        node = self.network.nodes[self.source]
        edges_left = 3
        while edges_left > 0:
            edge = node.neighbors[2]
            path_edges.append( (edge.src.loc, edge.dest.loc, '{:.0f}'.format(edge.length)) )
            total_length += edge.length
            node = edge.dest
            edges_left -= 1
        return {'cost':total_length, 'path':path_edges}

    def computeShortestPaths( self, srcIndex, use_heap=False ):
        self.source = srcIndex
        t1 = time.time()
        # TODO: RUN DIJKSTRA'S TO DETERMINE SHORTEST PATHS.
        #       ALSO, STORE THE RESULTS FOR THE SUBSEQUENT
        #       CALL TO getShortestPath(dest_index)

        dist = []
        i = 0
        for n in self.network.nodes:
            if i == srcIndex:
                dist.append(0)
            else:
                dist.append(float('inf'))
            i += 1
        print("dist: ", dist)
        prev = []
        for n in self.network.nodes:
            prev.append(None)
        print("prev: ", prev)

        if (use_heap):
            print("using Heap...\n")
            Queue = self.HeapQueue()
            Queue.makeQueue(self.network.nodes, self.heap)
        else:
            print("using Array...\n")
            Queue = self.ArrayQueue()
            Queue.makeQueue(self.network.nodes, self.array, srcIndex)

        edges_deleted = 0
        while edges_deleted != len(self.network.nodes):
            u = Queue.deleteMin(self.array)
            node = self.network.nodes[u]
            edges_deleted += 1
            for e in node.neighbors:
                if (dist[e.dest.node_id] > (dist[e.src.node_id] + e.length)):
                    print("old dist: ", dist[e.dest.node_id])
                    print("new dist: ", dist[e.src.node_id] + e.length)
                    dist[e.dest.node_id] = (dist[e.src.node_id] + e.length)
                    prev[e.dest.node_id] = e.src.node_id
                    Queue.decreaseKey(self.array)
            print("new dist[]: ", dist)
            print("new prev[]:", prev)

        t2 = time.time()
        return (t2-t1)

    class ArrayQueue:
        def makeQueue(self, nodes, array, index):
            print("array makeQueue\n")
            i = 0
            for n in nodes:
                if i == index:
                    self.insert(0, len(array), array)
                else:
                    self.insert(float('inf'), len(array), array)
                i += 1

            print("Array: ", array)

        def insert(self, n, index, array):
            #print("array insert\n")
            array.insert(index, n)

        def deleteMin(self, array):
            print("array deleteMin\n")
            changed = 1
            lowest = float('inf')
            while changed == 1:
                changed = 0
                for n in array:
                    if (n != float('inf')) & (float(n) < lowest) & (n >= 0):
                        lowest = n
                        changed = 1
            array[int(lowest)] = -1
            return lowest

        def decreaseKey(self, array):
            print("array decreaseKey\n")
            #does nothing



    class HeapQueue:
        def makeQueue(self, nodes, heap):
            print ("heap makeQueue\n")
        def insert(self):
            print("heap insert\n")

        def deleteMin(self):
            print("heap deleteMin\n")
            return None #return deleted node
        def decreaseKey(self):
            print("heap decreaseKey\n")
