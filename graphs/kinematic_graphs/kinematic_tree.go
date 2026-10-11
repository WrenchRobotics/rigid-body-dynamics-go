package kinematic_graphs

import "fmt"

type KinematicTree struct {
	underlyingGraph *KinematicGraph
	rootNode        *KinematicGraphNode
}

func (kt KinematicTree) UnderlyingGraph() *KinematicGraph {
	return kt.underlyingGraph
}

func (kt KinematicTree) RootNode() *KinematicGraphNode {
	return kt.rootNode
}

func (kt KinematicTree) ArcCount() int {
	if kt.underlyingGraph == nil {
		return 0
	}

	return len(kt.underlyingGraph.edges)
}

/*
Parents returns a map from the ID of each non-root node in the tree to the ID
of its parent in the tree (i.e., the parent array λ(i) from the textbook, but
indexed by node ID). The root node does not appear as a key.

The parent of a node is its neighbor that is closer to the root, regardless of
the direction of the joint connecting the two.
*/
func (kt KinematicTree) Parents() map[int64]int64 {
	parents, _ := kt.breadthFirstSearch()
	return parents
}

/*
Depths returns a map from the ID of each node reachable from the root to its
depth in the tree (i.e., its distance from the root, which has depth 0).
*/
func (kt KinematicTree) Depths() map[int64]int {
	_, depths := kt.breadthFirstSearch()
	return depths
}

/*
IsTreeEdge returns true if edge is one of the edges of the tree.
Edges of the original graph that are not tree edges are the loop-closing edges.
*/
func (kt KinematicTree) IsTreeEdge(edge *KinematicGraphEdge) bool {
	if kt.underlyingGraph == nil || edge == nil {
		return false
	}

	for _, treeEdge := range kt.underlyingGraph.edges {
		if treeEdge == edge {
			return true
		}
	}

	return false
}

/*
breadthFirstSearch traverses the tree's edges (ignoring their direction)
starting from the root, and returns the parent and depth of each node it reaches.
*/
func (kt KinematicTree) breadthFirstSearch() (map[int64]int64, map[int64]int) {
	parents := make(map[int64]int64)
	depths := make(map[int64]int)
	if kt.underlyingGraph == nil || kt.rootNode == nil {
		return parents, depths
	}

	// Collect the neighbors of each node along the tree's edges
	neighbors := make(map[int64][]int64, len(kt.underlyingGraph.nodeMap))
	for _, edge := range kt.underlyingGraph.edges {
		fromID, toID := edge.from.ID(), edge.to.ID()
		neighbors[fromID] = append(neighbors[fromID], toID)
		neighbors[toID] = append(neighbors[toID], fromID)
	}

	rootID := kt.rootNode.ID()
	depths[rootID] = 0
	queue := []int64{rootID}
	for len(queue) > 0 {
		currentID := queue[0]
		queue = queue[1:]
		for _, neighborID := range neighbors[currentID] {
			if _, seen := depths[neighborID]; seen {
				continue
			}
			parents[neighborID] = currentID
			depths[neighborID] = depths[currentID] + 1
			queue = append(queue, neighborID)
		}
	}

	return parents, depths
}

// func (kt KinematicTree) AssignLinksAndJointNumbers() (map[string]int, map[string]int) {
// 	// Create map for the links
// 	linkNameToID := make(map[string]int)

// 	// Fixed base gets number 0
// 	rootLinkName := kt.rootNode.asLink.Name
// 	linkNameToID[rootLinkName] = 0

// 	//
// }

func (kt KinematicTree) ComputeNumbering() (*Numbering, error) {
	if kt.underlyingGraph == nil {
		return nil, fmt.Errorf("underlying graph is nil")
	}

	// Create the numbering map if it doesn't exist
	numbering := NewNumbering()

	// 2. Assign the number 0 to the node representing the fixed base,
	// and define this node to be the root node of Gt.
	if kt.rootNode == nil {
		return nil, fmt.Errorf("root node is nil")
	}

	numbering.AssignNumber(kt.rootNode.ID(), 0)

	// 3. Number the remaining nodes from 1 to N_B in any order
	// such that each node has a higher number than its parent in Gt.
	NB := kt.underlyingGraph.Nodes().Len()

	visited := make([]*KinematicGraphNode, 0, NB)
	visited = append(visited, kt.rootNode)

	children, err := kt.underlyingGraph.GetChildren(kt.rootNode)
	if err != nil {
		return nil, fmt.Errorf("failed to get children of root node: %v", err)
	}

	nextNodeNumber := uint64(1)
	for len(children) > 0 {
		// Find the next node without a number among the children
		// Filter out children that have already been visited
		unvisitedChildren := make([]*KinematicGraphNode, 0, len(children))
		for _, child := range children {
			alreadyVisited := false
			for _, v := range visited {
				if v.ID() == child.ID() {
					alreadyVisited = true
					break
				}
			}
			if !alreadyVisited {
				unvisitedChildren = append(unvisitedChildren, child)
			}
		}

		// Assign the next available number to a node without a number
		for _, child := range unvisitedChildren {
			visited = append(visited, child)

			if _, ok := numbering.GetNodeNumber(child.ID()); !ok {
				numbering.AssignNumber(child.ID(), nextNodeNumber)
				nextNodeNumber++
			}
		}

		// Update the list of children to include
		// the children of the newly visited nodes
		newChildren := make([]*KinematicGraphNode, 0)
		for _, child := range unvisitedChildren {
			childrenOfChild, err := kt.underlyingGraph.GetChildren(child)
			if err != nil {
				return nil, fmt.Errorf("failed to get children of child: %v", err)
			}
			newChildren = append(newChildren, childrenOfChild...)
		}
		children = newChildren
	}

	finalNumberingCount, _ := numbering.MaxNodeNumber()
	if finalNumberingCount != uint64(NB-1) { // This should be unreachable
		return nil, fmt.Errorf(
			"not all nodes were numbered (%v out of %v), but the algorithm converged; there may be multiple disconnected components",
			finalNumberingCount+1,
			NB)
	}

	return numbering, nil
}
