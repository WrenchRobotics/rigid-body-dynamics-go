package kinematic_graphs

import (
	"fmt"
	"log"

	kinematic_graph_errors "github.com/WrenchRobotics/rigid-body-dynamics-go/errors/kinematic_graphs"
	link_errors "github.com/WrenchRobotics/rigid-body-dynamics-go/errors/link"
	"github.com/WrenchRobotics/rigid-body-dynamics-go/utils"
	urdfmodel "github.com/WrenchRobotics/urdf-go/urdf_model"
	"github.com/WrenchRobotics/urdf-go/urdf_model/link"

	"gonum.org/v1/gonum/graph"
	"gonum.org/v1/gonum/graph/iterator"
	"gonum.org/v1/gonum/graph/topo"
)

type KinematicGraph struct {
	nodeMap map[int64]*KinematicGraphNode
	edges   []*KinematicGraphEdge
}

func NewKinematicGraph() *KinematicGraph {
	return &KinematicGraph{
		nodeMap: make(map[int64]*KinematicGraphNode),
	}
}

func (g *KinematicGraph) AddNode(node graph.Node) {
	if node == nil {
		return
	}

	ktn, ok := node.(*KinematicGraphNode)
	if !ok {
		panic(
			fmt.Sprintf("Attempted to add node that is NOT a KinematicGraphNode to a KinematicGraph! Type: %v", node),
		)
	}

	log.Printf("adding the node with id %v...\n", node.ID())

	g.nodeMap[node.ID()] = ktn
}

// Returns the edge (if it exists) between node with id `uid`
// and the node `vid`.
// If it doesn't exist, then this should return nil.
func (g *KinematicGraph) Edge(uid, vid int64) graph.Edge {
	// Attempt to collect the edge in question
	kge, err := g.GetEdgeBetween(uid, vid)
	if err == nil {
		return kge
	}

	// The kinematic graph stores directed edges, but traversal in this type
	// uses undirected neighborhood semantics via From().
	kge, reverseErr := g.GetEdgeBetween(vid, uid)
	if reverseErr == nil {
		return kge
	}

	if _, ok := err.(*kinematic_graph_errors.EdgeNotFoundError); ok {
		return nil
	}

	panic(
		fmt.Errorf(
			"there was an issue collecting the edge between \"%v\" and \"%v\": %v",
			uid,
			vid,
			err,
		),
	)
}

// EdgeBetween returns the edge between nodes x and y
// with IDs xid and yid.
func (g *KinematicGraph) EdgeBetween(xid, yid int64) graph.Edge {
	// Attempt to find the edge between the two nodes
	edge, err := g.GetEdgeBetween(xid, yid)

	// Check the error value to decide what to do next
	if err == nil {
		return edge
	}

	edge, reverseErr := g.GetEdgeBetween(yid, xid)
	if reverseErr == nil {
		return edge
	}

	if _, ok := err.(*kinematic_graph_errors.EdgeNotFoundError); ok {
		return nil
	}

	panic(
		fmt.Errorf("there was an issue checking for an edge between the two graph nodes: %v", err),
	)
}

func (g *KinematicGraph) Edges() graph.Edges {
	edges := make([]graph.Edge, len(g.edges))
	for i, e := range g.edges {
		edges[i] = e
	}
	return iterator.NewOrderedEdges(edges)
}

func (g *KinematicGraph) ExtractFromModel(modelIn *urdfmodel.Model) error {
	if modelIn == nil {
		return fmt.Errorf("no model provided to function; i.e., received nil.")
	}

	// Create MultiWriter
	_, mw, err := utils.CreateIOMultiWriterForLogFile(
		fmt.Sprintf("%v-model.log", modelIn.Name),
	)
	if err != nil {
		return fmt.Errorf("failed to create log file: %v", err)
	}

	// Configure the logging library's output
	log.SetOutput(mw)

	log.SetPrefix("[ExtractFromModel] ")
	log.SetFlags(log.Ldate | log.Ltime | log.Lshortfile)

	// Iterate through links in model and:
	// - add each node to the graph
	for _, linkName := range modelIn.GetAllLinkNames() {
		// Get link from model
		linkIn, err := modelIn.GetLink(linkName)
		if err != nil {
			return err
		}

		// Create new kinematic tree node
		newNode := NewKinematicGraphNode(linkIn, g.Nodes().Len())

		log.Printf("adding node with the name \"%v\"\n", linkName)

		g.AddNode(&newNode)
	}

	// - add each edge to the graph now that all nodes exist
	for _, jointName := range modelIn.GetAllJointNames() {
		// Get joint from model
		jointIn, err := modelIn.GetJoint(jointName)
		if err != nil {
			return err
		}

		// Create new kinematic tree edge
		parentLink, _ := modelIn.GetLink(jointIn.ParentLinkName)
		parentNodeInGraph, err := g.GetNodeForLink(parentLink)
		if err != nil {
			return fmt.Errorf("there was an issue retrieving the link \"%v\" from the KinematicGraph.", parentLink.Name)
		}

		childLink, _ := modelIn.GetLink(jointIn.ChildLinkName)
		childNodeInGraph, err := g.GetNodeForLink(childLink)
		if err != nil {
			return fmt.Errorf("there was an issue retrieving the link \"%v\" from the KinematicGraph.", childLink.Name)
		}

		newEdge := NewKinematicGraphEdge(parentNodeInGraph, childNodeInGraph, jointIn)

		// Add edges to list
		g.edges = append(g.edges, &newEdge)
	}

	return nil
}

func (g *KinematicGraph) From(id int64) graph.Nodes {
	var out []graph.Node
	// Extract parent of node, if it exists
	parentNode, err := g.GetParent(g.nodeMap[id])
	if err != nil {
		return iterator.NewOrderedNodes(out)
	}
	if parentNode != nil {
		out = append(out, parentNode)
	}

	// Extract children of node, if they exist
	childrenNodes, err := g.GetChildren(g.nodeMap[id])
	if err != nil {
		return iterator.NewOrderedNodes(out)
	}
	for _, childNode := range childrenNodes {
		out = append(out, childNode)
	}
	return iterator.NewOrderedNodes(out)
}

func (g *KinematicGraph) GetChildren(n *KinematicGraphNode) ([]*KinematicGraphNode, error) {
	// Get children from link
	nAsLink, err := n.GetLink()
	if err != nil {
		return nil, err
	}

	children := make([]*KinematicGraphNode, len(nAsLink.ChildLinks))

	for i, childLink := range nAsLink.ChildLinks {
		childID, err := g.GetNodeIDForLink(childLink)
		if err != nil {
			return nil, err
		}

		nodeI, err := g.GetNode(childID)
		if err != nil {
			return nil, err
		}
		children[i] = nodeI
	}

	return children, nil
}

func (g *KinematicGraph) GetEdgeBetween(uid, vid int64) (*KinematicGraphEdge, error) {
	// Get the nodes with the ids given
	uAsNode, err := g.GetNode(uid)
	if err != nil {
		return nil, &kinematic_graph_errors.NodeDoesNotExistError{}
	}

	vAsNode, err := g.GetNode(vid)
	if err != nil {
		return nil, &kinematic_graph_errors.NodeDoesNotExistError{}
	}

	// Iterate through all edges in the internal data structure and
	// verify if one exists that has matching source and destination node values
	for _, edge := range g.edges {
		if (edge.from == uAsNode) && (edge.to == vAsNode) {
			return edge, nil
		}
	}

	// Otherwise edge was not found
	return nil, &kinematic_graph_errors.EdgeNotFoundError{
		FromID: uid,
		ToID:   vid,
	}
}

func (g *KinematicGraph) GetNode(id int64) (*KinematicGraphNode, error) {
	node, exists := g.nodeMap[id]
	if !exists {
		return nil, &kinematic_graph_errors.NodeDoesNotExistError{}
	}
	return node, nil
}

func (g *KinematicGraph) GetNodeIDForLink(link *link.Link) (int64, error) {
	// Return error if node does not exist
	if link == nil {
		return -1, &link_errors.LinkDoesNotExistError{}
	}

	// Iterate through map to find node with matching link
	var node *KinematicGraphNode
	for _, n := range g.nodeMap {
		if candidateLink, _ := n.GetLink(); candidateLink == link {
			node = n
			break
		}
	}

	// If node not found, return error
	if node == nil {
		return -1, &kinematic_graph_errors.NodeDoesNotExistError{}
	}

	return node.ID(), nil
}

func (g *KinematicGraph) GetNodeForLink(link *link.Link) (*KinematicGraphNode, error) {
	nodeID, err := g.GetNodeIDForLink(link)
	if err != nil {
		return nil, err
	}

	return g.GetNode(nodeID)
}

func (g *KinematicGraph) GetParent(n *KinematicGraphNode) (*KinematicGraphNode, error) {
	// Get Link from node
	nAsLink, err := n.GetLink()
	if err != nil {
		return nil, err
	}

	// If no parent, return nil
	if nAsLink.ParentLink == nil {
		return nil, nil
	}

	// Return the existing parent node already present in this graph.
	return g.GetNodeForLink(nAsLink.ParentLink)
}

func (g *KinematicGraph) connectedComponentsByStoredEdges() int {
	if len(g.nodeMap) == 0 {
		return 0
	}

	adjacency := make(map[int64][]int64, len(g.nodeMap))
	for nodeID := range g.nodeMap {
		adjacency[nodeID] = make([]int64, 0)
	}

	for _, edge := range g.edges {
		fromID := edge.from.ID()
		toID := edge.to.ID()
		adjacency[fromID] = append(adjacency[fromID], toID)
		adjacency[toID] = append(adjacency[toID], fromID)
	}

	visited := make(map[int64]bool, len(g.nodeMap))
	components := 0

	for startNodeID := range g.nodeMap {
		if visited[startNodeID] {
			continue
		}

		components++
		queue := []int64{startNodeID}
		visited[startNodeID] = true

		for len(queue) > 0 {
			current := queue[0]
			queue = queue[1:]

			for _, neighborID := range adjacency[current] {
				if visited[neighborID] {
					continue
				}

				visited[neighborID] = true
				queue = append(queue, neighborID)
			}
		}
	}

	return components
}

func (g *KinematicGraph) HasEdgeBetween(xid, yid int64) bool {
	// Try to collect the edge
	tempEdge, err := g.GetEdgeBetween(xid, yid)

	// If edge was found, then return nil
	if tempEdge != nil {
		return true
	}

	// If tempEdge was nil AND the error was "not found" error,
	// then return false
	edgeNotFoundError := kinematic_graph_errors.EdgeNotFoundError{FromID: xid, ToID: yid}
	if err.Error() == edgeNotFoundError.Error() {
		return false
	}

	// Otherwise, panic
	panic(fmt.Errorf(
		"unexpected error received while getting edge between %v and %v: %v",
		xid, yid,
		err,
	))
}

// IsTree returns true if the associated kinematic graph is also
// a tree; false if not.
func (g *KinematicGraph) IsTree() bool {
	if len(g.nodeMap) == 0 {
		return false
	}

	// For edge-defined tree semantics, the graph must be connected using
	// the stored edge set and must satisfy |E| = |V|-1.
	if g.connectedComponentsByStoredEdges() != 1 {
		return false
	}

	numNodes := len(g.nodeMap)
	numEdges := len(g.edges)
	return numNodes-1 == numEdges

}

// IsLinkTopologyTree returns whether the graph appears as a tree when
// traversed through link parent/child topology exposed by From().
func (g *KinematicGraph) IsLinkTopologyTree() bool {
	components := topo.ConnectedComponents(g)
	if len(components) != 1 {
		return false
	}

	numNodes := g.Nodes().Len()
	numEdges := len(g.edges)
	return numNodes-1 == numEdges
}

// Adds an edge in graph g between the nodes with ids uid and vid
func (g *KinematicGraph) NewEdge(from, to graph.Node) graph.Edge {
	// Check to see if edge already exists in the graph, then simply return it
	if g.HasEdgeBetween(from.ID(), to.ID()) {
		kge, err := g.GetEdgeBetween(from.ID(), to.ID())
		if err != nil {
			panic(
				kinematic_graph_errors.EdgeNotFoundError{
					FromID: from.ID(),
					ToID:   to.ID(),
				},
			)
		}

		return kge
	} else {
		// Identify the joint between the links in:
		// - from, and
		fromAsKTN, ok := from.(*KinematicGraphNode)
		if !ok {
			panic(
				fmt.Sprintf("The `from` to NewEdge is not a valid KinematicGraphNode."),
			)
		}
		fromLink, err := fromAsKTN.GetLink()
		if err != nil {
			panic(
				fmt.Errorf("There was an issue collecting the link from From: %v", err),
			)
		}

		// - to
		toAsKTN, ok := from.(*KinematicGraphNode)
		if !ok {
			panic(
				fmt.Sprintf("The `to` to NewEdge is not a valid KinematicGraphNode."),
			)
		}
		toLink, err := toAsKTN.GetLink()
		if err != nil {
			panic(
				fmt.Errorf("There was an issue collecting the link from From: %v", err),
			)
		}

		sharedJoints := utils.FindJointConnectingTheseLinks(fromLink, toLink)
		if len(sharedJoints) != 1 {
			panic(
				fmt.Sprintf(
					"There are multiple joints (%v) that are connected to the from and to links.",
					len(sharedJoints),
				),
			)
		}

		edge := NewKinematicGraphEdge(g.nodeMap[from.ID()], g.nodeMap[to.ID()], sharedJoints[0])

		g.edges = append(g.edges, &edge)
		return &edge
	}
}

func (g *KinematicGraph) NewNode() graph.Node {
	newNode := NewKinematicGraphNode(nil, g.Nodes().Len())

	g.nodeMap[newNode.ID()] = &newNode

	return &newNode
}

// Node returns the node with the given ID if it exists
// in the graph, and nil otherwise.
func (g *KinematicGraph) Node(id int64) graph.Node {
	n, _ := g.GetNode(id)
	return n
}

// Nodes returns all nodes in the Kinematic Graph.
//
// Nodes must not return nil.
func (g *KinematicGraph) Nodes() graph.Nodes {
	// Constants

	// Algorithm
	var out []graph.Node
	for _, n := range g.nodeMap {
		out = append(out, n)
	}

	return iterator.NewOrderedNodes(out)
}

// RemoveNode removes the node with the given ID
// from the graph, as well as any edges attached
// to it. If the node is not in the graph it is
// a no-op.
func (g *KinematicGraph) RemoveNode(id int64) {
	// Remove the node from the map
	delete(g.nodeMap, id)
}

// SetEdge adds the edge `edge` to the graph g.
// This expects that the edge `edge` refers to nodes in the current graph. If not,
// then this function will panic.
func (g *KinematicGraph) SetEdge(edge graph.Edge) {
	// Concretize the graph edge to a KinematicGraphEdge
	edgeAsKGE, ok := edge.(*KinematicGraphEdge)
	if !ok {
		panic(
			fmt.Errorf(
				"the edge given as input to SetEdge is not of type `*KinematicGraphEdge`; received %T",
				edge,
			),
		)
	}

	// Check that each node referred to in the edge are in the graph
	// - Source node
	sourceID := edgeAsKGE.from.ID()
	_, err := g.GetNode(sourceID)
	if err != nil {
		panic(
			fmt.Errorf("attempting to add an edge whose source node (ID %v) does not exist in graph! error while searching: %v", sourceID, err),
		)
	}

	// - Destination node
	destinationID := edgeAsKGE.to.ID()
	_, err = g.GetNode(destinationID)
	if err != nil {
		panic(
			fmt.Errorf("attempting to add an edge whose destination node (ID %v) does not exist in graph! error while searching: %v", destinationID, err),
		)
	}

	// Check to see if edge exists already in graph
	if g.HasEdgeBetween(sourceID, destinationID) {
		// Check to see if joint is the same as the current edge
		edgeToCompare, _ := g.GetEdgeBetween(sourceID, destinationID)
		if edgeToCompare.AsJoint == edgeAsKGE.AsJoint {
			return // Do nothing; the edge already exists in the graph
		} else {
			// Modify the edge that we already have.
			edgeToCompare.AsJoint = edgeAsKGE.AsJoint
		}
	} else {
		g.edges = append(g.edges, edgeAsKGE)
	}

}

// ToSpanningTree uses a greedy algorithm to construct a spanning tree
// for the current kinematic graph.
func (g *KinematicGraph) ToSpanningTree() (KinematicTree, error) {
	// A single spanning tree can only be formed from a connected graph.
	components := topo.ConnectedComponents(g)
	if len(components) != 1 {
		return KinematicTree{}, fmt.Errorf("graph has more than one connected component; cannot construct spanning tree")
	}

	nodes := g.Nodes()
	if !nodes.Next() {
		return KinematicTree{}, fmt.Errorf("graph has no nodes")
	}

	rootNode, ok := nodes.Node().(*KinematicGraphNode)
	if !ok {
		return KinematicTree{}, fmt.Errorf("first graph node is not a KinematicGraphNode")
	}

	return g.ToSpanningTreeWithDesiredRootNode(rootNode)
}

// ToSpanningTreeWithDesiredRootNode uses a greedy algorithm to construct a
// spanning tree for the current kinematic graph using rootNode as the root.
func (g *KinematicGraph) ToSpanningTreeWithDesiredRootNode(rootNode *KinematicGraphNode) (KinematicTree, error) {
	if rootNode == nil {
		return KinematicTree{}, fmt.Errorf("desired root node cannot be nil")
	}

	if _, err := g.GetNode(rootNode.ID()); err != nil {
		return KinematicTree{}, fmt.Errorf("desired root node with ID %d does not exist in graph", rootNode.ID())
	}

	// A single spanning tree can only be formed from a connected graph.
	components := topo.ConnectedComponents(g)
	if len(components) != 1 {
		return KinematicTree{}, fmt.Errorf("graph has more than one connected component; cannot construct spanning tree")
	}

	root := rootNode

	spanningGraph := NewKinematicGraph()
	for _, node := range g.nodeMap {
		spanningGraph.AddNode(node)
	}

	if g.IsTree() {
		for _, edge := range g.edges {
			spanningGraph.SetEdge(edge)
		}

		rootNode, err := spanningGraph.GetNode(root.ID())
		if err != nil {
			return KinematicTree{}, fmt.Errorf("there was an issue finding root node in spanning tree: %v", err)
		}

		return KinematicTree{
			underlyingGraph: spanningGraph,
			rootNode:        rootNode,
		}, nil
	}

	visited := make(map[int64]bool, len(g.nodeMap))
	parentByNodeID := make(map[int64]int64, len(g.nodeMap)-1)
	queue := []int64{root.ID()}
	visited[root.ID()] = true

	for len(queue) > 0 {
		currentID := queue[0]
		queue = queue[1:]

		neighbors := g.From(currentID)
		for neighbors.Next() {
			neighbor := neighbors.Node()
			neighborID := neighbor.ID()
			if visited[neighborID] {
				continue
			}

			visited[neighborID] = true
			parentByNodeID[neighborID] = currentID
			queue = append(queue, neighborID)
		}
	}

	for childID, parentID := range parentByNodeID {
		edge, err := g.GetEdgeBetween(parentID, childID)
		if err != nil {
			edge, err = g.GetEdgeBetween(childID, parentID)
			if err != nil {
				return KinematicTree{}, fmt.Errorf(
					"there was an issue finding BFS tree edge between nodes %d and %d: %v",
					parentID,
					childID,
					err,
				)
			}
		}

		spanningGraph.SetEdge(edge)
	}

	rootNode, err := spanningGraph.GetNode(root.ID())
	if err != nil {
		return KinematicTree{}, fmt.Errorf("there was an issue finding root node in spanning tree: %v", err)
	}

	return KinematicTree{
		underlyingGraph: spanningGraph,
		rootNode:        rootNode,
	}, nil
}
