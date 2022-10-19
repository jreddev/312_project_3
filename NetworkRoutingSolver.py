#!/usr/bin/python3
import math

from CS312Graph import *
import time


class NetworkRoutingSolver:
    def __init__(self):
        self.array = []
        self.prev_saved = []
        self.heap = None
        self.debug = 0
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
        n = self.dest
        if self.debug == 1:
            print("source: ", self.source)
            print("destination: ", self.dest)
        while n != self.source:
            node = self.network.nodes[n]
            prev_node = self.network.nodes[self.prev_saved[n]]
            found = 0
            for edge in prev_node.neighbors:
                if edge.dest.node_id == node.node_id:
                    path_edges.append((edge.src.loc, edge.dest.loc, '{:.0f}'.format(edge.length)))
                    found = 1
                    total_length += edge.length
            if found == 0:
                if self.debug == 1:
                    print("error! prev_edge not found!")
            n = prev_node.node_id

        #node = self.network.nodes[self.source]
        #edges_left = 3
        #while edges_left > 0:
            #edge = node.neighbors[2]
            #path_edges.append( (edge.src.loc, edge.dest.loc, '{:.0f}'.format(edge.length)) )
            #total_length += edge.length
            #node = edge.dest
            #edges_left -= 1
        return {'cost':total_length, 'path':path_edges}

    def computeShortestPaths( self, srcIndex, use_heap=False ):
        self.source = srcIndex
        t1 = time.time()
        # TODO: RUN DIJKSTRA'S TO DETERMINE SHORTEST PATHS.
        #       ALSO, STORE THE RESULTS FOR THE SUBSEQUENT
        #       CALL TO getShortestPath(dest_index)

        dist = []
        prev = []
        i = 0
        for n in self.network.nodes:
            prev.append(None)
            if i == srcIndex:
                dist.append(0)
            else:
                dist.append(float('inf'))
            i += 1
        print("dist: ", dist)
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
            currentID = Queue.deleteMin(self.array, dist)
            edges_deleted += 1
            if currentID == -1:
                continue
            node = self.network.nodes[currentID]
            neighbors = node.neighbors
            for edge in neighbors:
                sourceID = edge.src.node_id
                destID = edge.dest.node_id
                length = edge.length
                if (dist[destID] > (dist[sourceID] + length)):
                    if self.debug == 1:
                        print("old dist: ", dist[destID])
                        print("new dist: ", dist[sourceID] + length)
                    dist[destID] = (dist[sourceID] + length)
                    prev[destID] = sourceID
                    Queue.decreaseKey(self.array)
            if self.debug == 1:
                print("new dist[]: ", dist)
                print("new prev[]:", prev)

        self.prev_saved = prev

        t2 = time.time()
        return (t2-t1)

    class ArrayQueue:

        def __init__(self):
            self.debug = 0
        def makeQueue(self, nodes, array, index):
            if self.debug == 1:
                print("array makeQueue\n")
            i = 0
            for n in nodes:
                if i == index:
                    self.insert(0, len(array), array)
                else:
                    self.insert(float('inf'), len(array), array)
                i += 1

            if self.debug == 1:
                print("Array: ", array)

        def insert(self, n, index, array):
            #if self.debug == 1:
            #   print("array insert\n")
            array.append(n)

        def deleteMin(self, array, dist):
            if self.debug == 1:
                print("array deleteMin\n")
            i = 0
            length = len(dist)
            lowest = float('inf')
            for n in dist:
                if i == len(dist):
                    break
                if n != float('inf') and float(n) < lowest and array[i] >= 0 and i != len(dist):
                    lowest = i

                i += 1
            if lowest == float('inf'):
                if self.debug == 1:
                    print("lowest not changed, cannot reach any more nodes\n")
                    print("array at point: ", array)
                return -1
            array[lowest] = -1
            return lowest

        def decreaseKey(self, array):
            if self.debug == 1:
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
