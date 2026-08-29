package connectivity_graphs

import (
	"fmt"

	"github.com/WrenchRobotics/rigid-body-dynamics-go/graphs/kinematic_graphs"
	"github.com/WrenchRobotics/urdf-go/urdf_model/link"
)

/*
ConnectivityGraph is a data structure that captures the connectivity of a kinematic graph.
It is designed to be used in the context of rigid body dynamics, where we often need to understand how different bodies are connected to each other through joints.
*/
type ConnectivityGraph struct {
	SourceGraph        *kinematic_graphs.KinematicGraph // The original kinematic graph from which this connectivity graph is derived
	Predecessors       []int64                          // Predecessors[i] is the body that is the predecessor of joint i
	Successors         []int64                          // Successors[i] is the body that is the successor of joint i
	bodyNumberToNodeID map[int64]uint64                 // Maps body numbers to their corresponding node IDs in the kinematic graph
}

func NewConnectivityGraph() *ConnectivityGraph {
	return &ConnectivityGraph{
		Predecessors: []int64{},
		Successors:   []int64{},
	}
}

func ExtractFromKinematicGraph(kinematicGraphIn *kinematic_graphs.KinematicGraph) (ConnectivityGraph, error) {
	if kinematicGraphIn == nil {
		return ConnectivityGraph{}, fmt.Errorf("no kinematic graph provided to function; i.e., received nil.")
	}

	// Initialize predecessor and successor arrays to be of the correct size
	g := &ConnectivityGraph{
		SourceGraph:  kinematicGraphIn,
		Predecessors: make([]int64, 0),
		Successors:   make([]int64, 0),
	}

	// Iterate through edges in kinematic graph and populate predecessor and successor arrays
	edges := kinematicGraphIn.Edges()
	for edges.Next() {
		edge := edges.Edge().(*kinematic_graphs.KinematicGraphEdge)
		fromID := edge.From().ID()
		toID := edge.To().ID()

		g.Predecessors = append(g.Predecessors, int64(fromID))
		g.Successors = append(g.Successors, int64(toID))
	}
	return *g, nil
}

func (g *ConnectivityGraph) GetAllPredecessors(nodeID int64) ([]int64, error) {
	predecessors := []int64{}
	for i, succ := range g.Successors {
		if succ == nodeID {
			predecessors = append(predecessors, g.Predecessors[i])
		}
	}
	return predecessors, nil
}

func (g *ConnectivityGraph) GetAllSuccessors(nodeID int64) ([]int64, error) {
	successors := []int64{}
	for i, pred := range g.Predecessors {
		if pred == nodeID {
			successors = append(successors, g.Successors[i])
		}
	}
	return successors, nil
}

func (g *ConnectivityGraph) GetLinkByNodeID(nodeID int64) (*link.Link, error) {
	node := g.SourceGraph.Node(nodeID)
	if node == nil {
		return nil, fmt.Errorf("node with ID %d not found in the source graph", nodeID)
	}

	kinematicNode, ok := node.(*kinematic_graphs.KinematicGraphNode)
	if !ok {
		return nil, fmt.Errorf("node with ID %d is not a KinematicGraphNode", nodeID)
	}

	link, err := kinematicNode.GetLink()
	if err != nil {
		return nil, fmt.Errorf("failed to get link for node with ID %d: %v", nodeID, err)
	}

	return link, nil
}
