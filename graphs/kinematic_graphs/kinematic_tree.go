package kinematic_graphs

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
