#!/usr/bin/python3
import math

from CS312Graph import *
import time


class NetworkRoutingSolver:
    def __init__(self):
        self.array = []
        self.prev_saved = []
        self.dist_saved = []
        self.heap = None
        self.debug = 0
        pass

    def initializeNetwork(self, network):
        assert( type(network) == CS312Graph )
        self.network = network

    def getShortestPath( self, destIndex ):
        self.dest = destIndex

        path_edges = []
        total_length = 0
        n = self.dest
        if self.debug == 1:
            print("source: ", self.source)
            print("destination: ", self.dest)

        #print("prev: ", self.prev_saved)
        #print("dist: ", self.dist_saved)

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

        return {'cost':total_length, 'path':path_edges}

    def computeShortestPaths( self, srcIndex, use_heap=False ):
        self.source = srcIndex
        t1 = time.time()

        dist = [float('inf') for n in range(len(self.network.nodes))]
        prev = [None for n in range(len(self.network.nodes))]
        dist[self.source] = 0

        if (use_heap):
            Queue = self.HeapQueue()
            Queue.makeQueue(self.network.nodes, srcIndex)
        else:
            Queue = self.ArrayQueue()
            Queue.makeQueue(self.network.nodes, srcIndex)
        edges_deleted = 0
        while edges_deleted != len(self.network.nodes):
            currentID = Queue.deleteMin(dist)
            edges_deleted += 1
            if currentID == -1:
                break
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
                    Queue.decreaseKey(destID, (dist[sourceID] + length))
            if self.debug == 1:
                print("new dist[]: ", dist)
                print("new prev[]:", prev)

        self.prev_saved = prev
        self.dist_saved = dist

        t2 = time.time()
        return (t2-t1)

    class ArrayQueue:

        def __init__(self):
            self.debug = 0
            self.check = 0
            self.array = []
        def makeQueue(self, nodes, index):
            self.array = [float('inf') for n in range(len(nodes))]
            self.array[index] = 0

            if self.debug == 1:
                print("array makeQueue\n")
                print("Array: ", self.array)

            return self.array

        def insert(self, n, dist):
            self.array.append(n)

        def deleteMin(self, dist):
            if self.debug == 1:
                print("array deleteMin\n")

            lowest = float('inf')
            low_array = []
            index = 0

            for d in range(len(dist)):
                if dist[d] != float('inf'):
                    if dist[d] < lowest:
                        if int(index) < int(len(dist)):
                            if self.array[index] >= 0:
                                a = self.indexArray(index, dist[d])
                                low_array.append(a)
                index += 1

            index = 0
            low = float('inf')
            for val in low_array:
                if val.value < low:
                    low = val.value
                    index = val.index

            if low == float('inf'):
                return -1

            self.array[index] = -1

            return index

        def decreaseKey(self, node_id, len):
            if self.debug == 1:
                print("array decreaseKey\n")
            #does nothing

        class indexArray:
            def __init__(self, index, value):
                self.index = index
                self.value = value

    class HeapQueue:

        def __init__(self):
            self.map = {}
            self.array = list()

        class nodeWrap:
            def __init__(self, node, dist):
                self.node = node
                self.dist = dist

        def makeQueue(self, nodes, i):
            self.array = list()
            for n in nodes:
                self.array.append(self.nodeWrap(n, float('inf')))

            top = self.array[i]  # set start node to the top of the HeapArray
            top.dist = 0
            del self.array[i]
            self.array.insert(0, top)

            for i in range(0, len(self.array) - 1):
                self.map[self.array[i].node.node_id] = i

            return

        def insert(self, n, dist):
            self.array.append(self.nodeWrap(n, dist))
            index = len(self.array) - 1
            p_index = (index - 1) // 2
            while p_index >= 0 and self.array[p_index].dist > self.array[index].dist:
                self.bubbleUp(index, p_index)
                index = p_index
                p_index = (index - 1) // 2

        def decreaseKey(self, id, val):
            index = self.map[id]
            self.array[index].dist = val

            p_index = (index - 1) // 2
            while p_index >= 0 and self.array[p_index].dist > self.array[index].dist:
                self.bubbleUp(index, p_index)
                index = p_index
                p_index = (index - 1) // 2

        def deleteMin(self, dist):
            min_node = self.array[0]
            last_node = self.array[len(self.array) - 1]
            self.array[0] = last_node
            self.map[last_node.node.node_id] = 0
            self.array.pop()
            del self.map[min_node.node.node_id]

            self.siftDown()
            return min_node.node.node_id

        def bubbleUp(self, i, pi):
            p = self.array[pi]
            self.map[self.array[i].node.node_id] = pi
            self.array[pi] = self.array[i]
            self.map[p.node.node_id] = i
            self.array[i] = p

        def siftDown(self):
            index = 0
            while True:
                c1 = index * 2 + 1
                c2 = index * 2 + 2
                
                if c1 >= len(self.array) or c2 >= len(self.array):  # There is no child return
                    return
                
                elif self.array[index].dist > self.array[c1].dist and self.array[index].dist > self.array[c2].dist:  # Swap with the smaller child
                    if self.array[c2].dist < self.array[c1].dist:
                        curr = self.array[index]
                        self.map[self.array[c2].node.node_id] = index
                        self.array[index] = self.array[c2]
                        self.map[curr.node.node_id] = c2
                        self.array[c2] = curr
                        index = c2

                    else:
                        curr = self.array[index]
                        self.map[self.array[c1].node.node_id] = index
                        self.array[index] = self.array[c1]
                        self.map[curr.node.node_id] = c1
                        self.array[c1] = curr
                        index = c1

                elif self.array[index].dist > self.array[c2].dist:  # Swap with right child
                    curr = self.array[index]
                    self.map[self.array[c2].node.node_id] = index
                    self.array[index] = self.array[c2]
                    self.map[curr.node.node_id] = c2
                    self.array[c2] = curr
                    index = c2
                
                elif self.array[index].dist > self.array[c1].dist:  # Swap with left child
                    curr = self.array[index]
                    self.map[self.array[c1].node.node_id] = index
                    self.array[index] = self.array[c1]
                    self.map[curr.node.node_id] = c1
                    self.array[c1] = curr
                    index = c1

                else:  # Return correct location
                    return
