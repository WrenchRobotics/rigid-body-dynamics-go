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
