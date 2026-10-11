package kinematic_graphs_test

import (
	"strings"
	"testing"

	"github.com/WrenchRobotics/rigid-body-dynamics-go/graphs/kinematic_graphs"
	"github.com/WrenchRobotics/rigid-body-dynamics-go/systems"
	"github.com/WrenchRobotics/urdf-go/urdf_model/link"
)

// TestKinematicGraph_IsTree1 verifies edge-structure-based tree semantics.
// Crank-slider has a kinematic loop, so it should not be a tree by stored edges.
func TestKinematicGraph_IsTree1(t *testing.T) {
	// Construct test graph
	kg, err := systems.CreateCrankSliderLinkageKinematicGraph()
	if err != nil {
		t.Fatalf(
			"there was an issue attempting to load CrankSliderLinkage system: %v", err,
		)
	}

	// This should not be a tree
	if kg.IsTree() {
		t.Errorf(
			"method believes that Crank-Slider Linkage system is a kinematic tree, but it is not!",
		)
	}
}

// TestKinematicGraph_IsLinkTopologyTree1 verifies link-topology-based tree semantics.
// Crank-slider should also fail this check because it contains a loop in link topology.
func TestKinematicGraph_IsLinkTopologyTree1(t *testing.T) {
	kg, err := systems.CreateCrankSliderLinkageKinematicGraph()
	if err != nil {
		t.Fatalf(
			"there was an issue attempting to load CrankSliderLinkage system: %v", err,
		)
	}

	if kg.IsLinkTopologyTree() {
		t.Errorf(
			"method believes that Crank-Slider Linkage system is a link-topology tree, but it is not!",
		)
	}
}

// TestKinematicGraph_ToSpanningTree_DisconnectedGraph verifies that
// attempting to construct a single spanning tree from a graph with more than
// one connected component returns an error.
func TestKinematicGraph_ToSpanningTree_DisconnectedGraph(t *testing.T) {
	kg := kinematic_graphs.NewKinematicGraph()

	// Build two isolated nodes so the graph has two connected components.
	link1 := &link.Link{Name: "link1"}
	link2 := &link.Link{Name: "link2"}
	node1 := kinematic_graphs.NewKinematicGraphNode(link1, 1)
	node2 := kinematic_graphs.NewKinematicGraphNode(link2, 2)

	kg.AddNode(&node1)
	kg.AddNode(&node2)

	_, err := kg.ToSpanningTree()
	if err == nil {
		t.Fatalf("expected error when constructing spanning tree from disconnected graph, but got nil")
	}
}

// TestKinematicGraph_ToSpanningTree_KinematicLoop verifies that a connected
// graph with a loop can still be reduced to a spanning tree.
func TestKinematicGraph_ToSpanningTree_KinematicLoop(t *testing.T) {
	kg, err := systems.CreateCrankSliderLinkageKinematicGraph()
	if err != nil {
		t.Fatalf("there was an issue attempting to load CrankSliderLinkage system: %v", err)
	}

	tree, err := kg.ToSpanningTree()
	if err != nil {
		t.Fatalf("expected ToSpanningTree to succeed for connected graph with kinematic loop, got error: %v", err)
	}

	spanningGraph := tree.UnderlyingGraph()
	if spanningGraph == nil {
		t.Fatalf("expected returned spanning tree to include a graph, got nil")
	}

	if tree.RootNode() == nil {
		t.Fatalf("expected returned spanning tree to have a root node, got nil")
	}

	if tree.ArcCount() != kg.Nodes().Len()-1 {
		t.Fatalf("expected spanning tree to have N-1 arcs; got %d for N=%d", tree.ArcCount(), kg.Nodes().Len())
	}

	// Edge-structure semantics on the returned spanning graph should be a tree.
	if !spanningGraph.IsTree() {
		t.Fatalf("expected returned spanning graph to satisfy edge-based tree semantics")
	}

	// The returned spanning graph should also satisfy link-topology tree semantics.
	if !spanningGraph.IsLinkTopologyTree() {
		t.Fatalf("expected returned spanning graph to satisfy link-topology tree semantics")
	}
}

// TestKinematicGraph_IsTreeDiffersFromIsLinkTopologyTree_EdgeTreeOnly creates
// a graph whose stored edges form a tree while link topology is disconnected.
func TestKinematicGraph_IsTreeDiffersFromIsLinkTopologyTree_EdgeTreeOnly(t *testing.T) {
	kg := kinematic_graphs.NewKinematicGraph()

	link1 := &link.Link{Name: "link1"}
	link2 := &link.Link{Name: "link2"}
	node1 := kinematic_graphs.NewKinematicGraphNode(link1, 1)
	node2 := kinematic_graphs.NewKinematicGraphNode(link2, 2)

	kg.AddNode(&node1)
	kg.AddNode(&node2)

	edge := kinematic_graphs.NewKinematicGraphEdge(&node1, &node2, nil)
	kg.SetEdge(&edge)

	if !kg.IsTree() {
		t.Fatalf("expected IsTree to be true when stored edges form a 2-node tree")
	}

	if kg.IsLinkTopologyTree() {
		t.Fatalf("expected IsLinkTopologyTree to be false when links have no parent/child connectivity")
	}
}

// TestKinematicGraph_IsTreeDiffersFromIsLinkTopologyTree_LinkTreeOnly creates
// a graph whose link topology forms a tree while stored edges are not connected.
func TestKinematicGraph_IsTreeDiffersFromIsLinkTopologyTree_LinkTreeOnly(t *testing.T) {
	kg := kinematic_graphs.NewKinematicGraph()

	link1 := &link.Link{Name: "link1"}
	link2 := &link.Link{Name: "link2"}
	link3 := &link.Link{Name: "link3"}

	// Build a link-topology chain: link1 -> link2 -> link3.
	link2.ParentLink = link1
	link3.ParentLink = link2
	link1.ChildLinks = []*link.Link{link2}
	link2.ChildLinks = []*link.Link{link3}

	node1 := kinematic_graphs.NewKinematicGraphNode(link1, 1)
	node2 := kinematic_graphs.NewKinematicGraphNode(link2, 2)
	node3 := kinematic_graphs.NewKinematicGraphNode(link3, 3)

	kg.AddNode(&node1)
	kg.AddNode(&node2)
	kg.AddNode(&node3)

	// Stored edges are not a connected tree:
	// - one edge between node1 and node2
	// - one self-loop on node3
	edge12 := kinematic_graphs.NewKinematicGraphEdge(&node1, &node2, nil)
	edge33 := kinematic_graphs.NewKinematicGraphEdge(&node3, &node3, nil)
	kg.SetEdge(&edge12)
	kg.SetEdge(&edge33)

	if kg.IsTree() {
		t.Fatalf("expected IsTree to be false when stored edges are disconnected")
	}

	if !kg.IsLinkTopologyTree() {
		t.Fatalf("expected IsLinkTopologyTree to be true when link parent/child topology is a tree and |E|=|V|-1")
	}
}

// TestKinematicGraph_ToSpanningTreeWithDesiredRootNode_DisconnectedByLinkTopology
// documents an important behavior of ToSpanningTreeWithDesiredRootNode.
//
// This fixture adds two nodes and one stored edge between them, which makes the
// graph connected under edge-based semantics. However, the spanning-tree method
// first checks connectivity using topo.ConnectedComponents over From(), which in
// this type is derived from link parent/child relationships. Because these links
// do not set ParentLink/ChildLinks, traversal sees two components and the method
// must return an error.
func TestKinematicGraph_ToSpanningTreeWithDesiredRootNode_DisconnectedByLinkTopology(t *testing.T) {
	kg := kinematic_graphs.NewKinematicGraph()

	link1 := &link.Link{Name: "link1"}
	link2 := &link.Link{Name: "link2"}

	node1 := kinematic_graphs.NewKinematicGraphNode(link1, 1)
	node2 := kinematic_graphs.NewKinematicGraphNode(link2, 2)

	kg.AddNode(&node1)
	kg.AddNode(&node2)

	edge := kinematic_graphs.NewKinematicGraphEdge(&node1, &node2, nil)
	kg.SetEdge(&edge)

	_, err := kg.ToSpanningTreeWithDesiredRootNode(&node2)
	if err == nil {
		t.Fatalf("expected ToSpanningTreeWithDesiredRootNode to fail for link-topology-disconnected graph, got nil error")
	}

	if !strings.Contains(err.Error(), "more than one connected component") {
		t.Fatalf("expected connected-component error, got: %v", err)
	}
}

func TestKinematicGraph_ToSpanningTreeWithDesiredRootNode_InvalidRootNode(t *testing.T) {
	kg := kinematic_graphs.NewKinematicGraph()

	link1 := &link.Link{Name: "link1"}
	link2 := &link.Link{Name: "link2"}
	link3 := &link.Link{Name: "link3"}

	node1 := kinematic_graphs.NewKinematicGraphNode(link1, 1)
	node2 := kinematic_graphs.NewKinematicGraphNode(link2, 2)
	invalidRoot := kinematic_graphs.NewKinematicGraphNode(link3, 3)

	kg.AddNode(&node1)
	kg.AddNode(&node2)

	edge := kinematic_graphs.NewKinematicGraphEdge(&node1, &node2, nil)
	kg.SetEdge(&edge)

	_, err := kg.ToSpanningTreeWithDesiredRootNode(&invalidRoot)
	if err == nil {
		t.Fatalf("expected error when desired root node does not exist in graph, but got nil")
	}
}

// TestKinematicGraph_ToSpanningTree_RootHasNoParent verifies that ToSpanningTree
// always roots the tree at the URDF link without a parent ("base" for the
// crank-slider). Map iteration order is randomized in Go, so the check is
// repeated to catch any non-deterministic root selection.
func TestKinematicGraph_ToSpanningTree_RootHasNoParent(t *testing.T) {
	for i := 0; i < 50; i++ {
		kg, err := systems.CreateCrankSliderLinkageKinematicGraph()
		if err != nil {
			t.Fatalf("there was an issue attempting to load CrankSliderLinkage system: %v", err)
		}

		tree, err := kg.ToSpanningTree()
		if err != nil {
			t.Fatalf("ToSpanningTree failed on iteration %d: %v", i, err)
		}

		rootLink, err := tree.RootNode().GetLink()
		if err != nil {
			t.Fatalf("failed to get link of root node: %v", err)
		}

		if rootLink.ParentLink != nil {
			t.Fatalf("expected root link to have no parent, but %q has parent %q", rootLink.Name, rootLink.ParentLink.Name)
		}

		if rootLink.Name != "base" {
			t.Fatalf("expected root link to be \"base\", got %q (iteration %d)", rootLink.Name, i)
		}
	}
}

// TestKinematicGraph_FindRootNode_NoNodes verifies that FindRootNode returns
// an error for an empty graph.
func TestKinematicGraph_FindRootNode_NoNodes(t *testing.T) {
	kg := kinematic_graphs.NewKinematicGraph()

	if _, err := kg.FindRootNode(); err == nil {
		t.Fatalf("expected error when finding root node of empty graph, got nil")
	}
}

// TestKinematicGraph_ComputeNumbering_CrankSlider verifies that every node of
// the crank-slider is numbered, that the root ("base") receives number 0, and
// that numbers are exactly 0..N_B-1. This guards against the regression where a
// random root (e.g. a leaf link) caused only a subset of nodes to be numbered.
func TestKinematicGraph_ComputeNumbering_CrankSlider(t *testing.T) {
	for i := 0; i < 50; i++ {
		kg, err := systems.CreateCrankSliderLinkageKinematicGraph()
		if err != nil {
			t.Fatalf("there was an issue attempting to load CrankSliderLinkage system: %v", err)
		}

		numbering, err := kg.ComputeNumbering()
		if err != nil {
			t.Fatalf("ComputeNumbering failed on iteration %d: %v", i, err)
		}

		nb := kg.Nodes().Len()
		seen := make(map[uint64]bool, nb)
		nodes := kg.Nodes()
		for nodes.Next() {
			node := nodes.Node().(*kinematic_graphs.KinematicGraphNode)
			number, ok := numbering.GetNodeNumber(node.ID())
			if !ok {
				t.Fatalf("node %d was not assigned a number (iteration %d)", node.ID(), i)
			}
			if number >= uint64(nb) {
				t.Fatalf("node %d has out-of-range number %d for N=%d", node.ID(), number, nb)
			}
			if seen[number] {
				t.Fatalf("number %d was assigned to more than one node", number)
			}
			seen[number] = true

			nodeLink, err := node.GetLink()
			if err != nil {
				t.Fatalf("failed to get link of node %d: %v", node.ID(), err)
			}
			if nodeLink.Name == "base" && number != 0 {
				t.Fatalf("expected \"base\" to be numbered 0, got %d", number)
			}
		}
	}
}
