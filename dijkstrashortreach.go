package main

import (
	"container/heap"
	"fmt"
)

type weightedEdge struct {
	weight int
	y      int
}

type Graph2 struct {
	edges     map[int][]weightedEdge
	edgeCount int
	nodeCount int
	directed  bool
}

func NewGraph2() *Graph2 {
	graph := new(Graph2)
	graph.edges = make(map[int][]weightedEdge)
	graph.directed = false
	return graph
}

func (g *Graph2) addEdge(from int, to int, weight int, directed bool) {
	node := weightedEdge{
		weight: weight,
		y:      to,
	}
	x, in := g.edges[from]
	if !in {
		x = make([]weightedEdge, 0)
	}
	g.edges[from] = append(x, node)
	if !directed {
		g.addEdge(to, from, weight, true)
	}
}

type Graph2Bfs struct {
	distance map[int]int
	parent   map[int]int
}

func (g *Graph2Bfs) Distance(node int) int {
	dist, found := g.distance[node]
	if !found {
		return -1
	}
	return dist
}

func (g *Graph2) shortestPath(startPos int) *Graph2Bfs {
	bfs := new(Graph2Bfs)
	bfs.distance = make(map[int]int)
	pq := NewMinHeap()

	bfs.distance[startPos] = 0

	pq.PushVal(startPos, 0)

	for pq.Len() > 0 {
		x := pq.PopVal()
		for i := range g.edges[x] {
			n := g.edges[x][i]
			nDist := bfs.distance[x] + n.weight
			yDist, seen := bfs.distance[n.y]
			if !seen {
				pq.PushVal(n.y, nDist)
				bfs.distance[n.y] = nDist
			}
			if seen && nDist < yDist {
				// new shorter distance, update PQ.
				pq.Update(n.y, yDist, nDist)
				bfs.distance[n.y] = nDist
			}
		}
	}
	return bfs
}

func (g *Graph2) printShortestPath(startPos int) {
	graph := g.shortestPath(startPos)
	for i := 0; i < g.nodeCount; i++ {
		if i == startPos {
			continue
		}
		fmt.Print(graph.Distance(i))
		fmt.Print(" ")
	}
	fmt.Println()
}

func main() {
	var testCases int
	fmt.Scan(&testCases)
	for i := 0; i < testCases; i++ {
		graph := NewGraph2()
		fmt.Scan(&graph.nodeCount, &graph.edgeCount)
		for j := 0; j < graph.edgeCount; j++ {
			var from int
			var to int
			var weight int
			fmt.Scan(&from, &to, &weight)
			graph.addEdge(from-1, to-1, weight, false)
		}
		var startPos int
		fmt.Scan(&startPos)
		graph.printShortestPath(startPos - 1)
	}
}

// An Item is something we manage in a priority queue.
type Item struct {
	value    int // The value of the item; arbitrary.
	priority int // The priority of the item in the queue.
	// The index is needed by update and is maintained by the heap.Interface methods.
	index int // The index of the item in the heap.
}

// A MinHeap implements heap.Interface and holds Items.
type MinHeap []*Item

func (mh MinHeap) Len() int { return len(mh) }

func (mh MinHeap) Less(i, j int) bool {
	return mh[i].priority < mh[j].priority
}

func (mh MinHeap) Swap(i, j int) {
	mh[i], mh[j] = mh[j], mh[i]
	mh[i].index = i
	mh[j].index = j
}

func (mh *MinHeap) PushVal(value int, priority int) {
	item := &Item{
		value:    value,
		priority: priority,
	}
	heap.Push(mh, item)
}

func (mh *MinHeap) Push(x interface{}) {
	n := len(*mh)
	item := x.(*Item)
	item.index = n
	*mh = append(*mh, item)
}

func (mh *MinHeap) PopVal() int {
	item := heap.Pop(mh).(*Item)
	return item.value
}

func (mh *MinHeap) Pop() interface{} {
	old := *mh
	n := len(old)
	item := old[n-1]
	item.index = -1 // for safety
	*mh = old[0 : n-1]
	return item
}

// update modifies the priority and value of an Item in the queue.
func (mh *MinHeap) Update(value, oldPriority, newPriority int) {
	for i := 0; i < mh.Len(); i++ {
		x := (*mh)[i]
		if x.value == value && x.priority == oldPriority {
			x.priority = newPriority
			heap.Fix(mh, x.index)
			return
		}
	}
}

func (mh *MinHeap) Init() {
	heap.Init(mh)
}

func NewMinHeap() *MinHeap {
	mh := new(MinHeap)
	mh.Init()
	return mh
}
