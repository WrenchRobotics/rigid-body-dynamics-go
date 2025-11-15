package kinematic_graphs

import (
	kinematic_trees_errors "github.com/WrenchRobotics/rigid-body-dynamics-go/errors/kinematic_trees"
	link_errors "github.com/WrenchRobotics/rigid-body-dynamics-go/errors/link"
	urdfmodel "github.com/WrenchRobotics/urdf-go/urdf_model"
	"github.com/WrenchRobotics/urdf-go/urdf_model/link"

	"gonum.org/v1/gonum/graph"
	"gonum.org/v1/gonum/graph/iterator"
)

type KinematicGraph struct {
	NodeMap map[int64]*KinematicTreeNode
}

func NewKinematicGraph() *KinematicGraph {
	return &KinematicGraph{
		NodeMap: make(map[int64]*KinematicTreeNode),
	}
}

func (g *KinematicGraph) AddNode(node *KinematicTreeNode) {
	if node == nil {
		return
	}
	g.NodeMap[node.ID()] = node
}

func (g *KinematicGraph) Edge(uid, vid int64) graph.Edge {
	if g.HasEdge(uid, vid) {
		var edge KinematicGraphEdge
		edge.Create(g.NodeMap[uid], g.NodeMap[vid])
		return &edge
	}
	return nil
}

func (g *KinematicGraph) ExtractFromModel(modelIn *urdfmodel.Model) error {
	if modelIn == nil {
		return nil
	}

	// Iterate through links in model and add to graph
	for idx, linkName := range modelIn.GetAllLinkNames() {
		// Get link from model
		linkIn, err := modelIn.GetLink(linkName)
		if err != nil {
			return err
		}

		// Create new kinematic tree node
		var newNode *KinematicTreeNode
		newNode.Create(linkIn, int(idx))
		g.AddNode(newNode)
	}

	return nil
}

func (g *KinematicGraph) From(id int64) graph.Nodes {
	var out []graph.Node
	// Extract parent of node, if it exists
	parentNode, err := g.GetParent(g.NodeMap[id])
	if err != nil {
		return iterator.NewOrderedNodes(out)
	}
	if parentNode != nil {
		out = append(out, parentNode)
	}

	// Extract children of node, if they exist
	childrenNodes, err := g.GetChildren(g.NodeMap[id])
	if err != nil {
		return iterator.NewOrderedNodes(out)
	}
	for _, childNode := range childrenNodes {
		out = append(out, childNode)
	}
	return iterator.NewOrderedNodes(out)
}

func (g *KinematicGraph) GetChildren(n *KinematicTreeNode) ([]*KinematicTreeNode, error) {
	// Get children from link
	nAsLink, err := n.GetLink()
	if err != nil {
		return nil, err
	}

	children := make([]*KinematicTreeNode, len(nAsLink.ChildLinks))

	for i, childLink := range nAsLink.ChildLinks {
		childID, err := g.GetNodeID(childLink)
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

func (g *KinematicGraph) GetNode(id int64) (*KinematicTreeNode, error) {
	node, exists := g.NodeMap[id]
	if !exists {
		return nil, &kinematic_trees_errors.KinematicTreeNodeDoesNotExistError{}
	}
	return node, nil
}

func (g *KinematicGraph) GetNodeID(link *link.Link) (int64, error) {
	// Return error if node does not exist
	if link == nil {
		return -1, &link_errors.LinkDoesNotExistError{}
	}

	// Iterate through map to find node with matching link
	var node *KinematicTreeNode
	for _, n := range g.NodeMap {
		if candidateLink, _ := n.GetLink(); candidateLink == link {
			node = n
			break
		}
	}

	// If node not found, return error
	if node == nil {
		return -1, &kinematic_trees_errors.KinematicTreeNodeDoesNotExistError{}
	}

	return node.ID(), nil
}

func (g *KinematicGraph) GetParent(n *KinematicTreeNode) (*KinematicTreeNode, error) {
	// Get Link from node
	nAsLink, err := n.GetLink()
	if err != nil {
		return nil, err
	}

	// If no parent, return nil
	if nAsLink.ParentLink == nil {
		return nil, nil
	}

	// Return parent as kinematic tree node
	var parentAsKinematicNode *KinematicTreeNode
	parentAsKinematicNode.Create(nAsLink.ParentLink, len(g.NodeMap)+1)
	if err != nil {
		return nil, err
	}

	return parentAsKinematicNode, nil
}

func (g *KinematicGraph) HasEdge(xid, yid int64) bool {
	// Check if both nodes exist
	xNode, err := g.GetNode(xid)
	if err != nil {
		return false
	}
	yNode, err := g.GetNode(yid)
	if err != nil {
		return false
	}

	// Check if x is a child of y
	if parentNode, _ := g.GetParent(xNode); parentNode == yNode {
		return true
	}

	// Check if y is a child of x
	if parentNode, _ := g.GetParent(yNode); parentNode == xNode {
		return true
	}

	// Otherwise, no relationship exists
	return false
}

func (g *KinematicGraph) Nodes() graph.Nodes {
	// Constants

	// Algorithm
	var out []graph.Node
	for _, n := range g.NodeMap {
		out = append(out, n)
	}

	return iterator.NewOrderedNodes(out)
}
