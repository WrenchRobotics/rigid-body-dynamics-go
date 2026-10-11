package kinematic_graphs_test

import (
	"testing"

	"github.com/WrenchRobotics/rigid-body-dynamics-go/graphs/kinematic_graphs"
	"github.com/WrenchRobotics/rigid-body-dynamics-go/systems"
)

/*
crankSliderSpanningTree returns the crank-slider's kinematic graph, its spanning
tree, and a function that looks up the ID of a node by name.
*/
func crankSliderSpanningTree(t *testing.T) (*kinematic_graphs.KinematicGraph, kinematic_graphs.KinematicTree, func(string) int64) {
	t.Helper()

	kg, err := systems.CreateCrankSliderLinkageKinematicGraph()
	if err != nil {
		t.Fatalf("error creating crank-slider kinematic graph: %v", err)
	}

	tree, err := kg.ToSpanningTree()
	if err != nil {
		t.Fatalf("error computing spanning tree: %v", err)
	}

	idOf := func(name string) int64 {
		node, err := kg.GetNodeByName(name)
		if err != nil {
			t.Fatalf("error getting %v node: %v", name, err)
		}
		return node.ID()
	}

	return kg, tree, idOf
}

/*
TestKinematicTree_Parents_CrankSlider verifies that every non-root node of the
crank-slider's spanning tree has the expected parent, and the root has none.
*/
func TestKinematicTree_Parents_CrankSlider(t *testing.T) {
	_, tree, idOf := crankSliderSpanningTree(t)
	parents := tree.Parents()

	if _, ok := parents[idOf("base")]; ok {
		t.Fatalf("expected root node \"base\" to have no parent")
	}

	expected := map[string]string{"B1": "base", "B3": "base", "B2": "B1"}
	for child, parent := range expected {
		if got, ok := parents[idOf(child)]; !ok || got != idOf(parent) {
			t.Fatalf("expected parent of %v to be %v (ID %d), got ID %d (present: %v)", child, parent, idOf(parent), got, ok)
		}
	}
}

/*
TestKinematicTree_Depths_CrankSlider verifies the depth of each node in the
crank-slider's spanning tree.
*/
func TestKinematicTree_Depths_CrankSlider(t *testing.T) {
	_, tree, idOf := crankSliderSpanningTree(t)
	depths := tree.Depths()

	expected := map[string]int{"base": 0, "B1": 1, "B3": 1, "B2": 2}
	for name, depth := range expected {
		if got, ok := depths[idOf(name)]; !ok || got != depth {
			t.Fatalf("expected depth of %v to be %d, got %d (present: %v)", name, depth, got, ok)
		}
	}
}

/*
TestKinematicTree_IsTreeEdge_CrankSlider verifies that the crank-slider's one
loop-closing joint (J3) is the only edge that is not in the spanning tree.
*/
func TestKinematicTree_IsTreeEdge_CrankSlider(t *testing.T) {
	kg, tree, _ := crankSliderSpanningTree(t)

	edges := kg.Edges()
	for edges.Next() {
		edge := edges.Edge().(*kinematic_graphs.KinematicGraphEdge)
		isLoopJoint := edge.AsJoint.Name == "J3"
		if tree.IsTreeEdge(edge) == isLoopJoint {
			t.Fatalf("expected IsTreeEdge(%v) to be %v", edge.AsJoint.Name, !isLoopJoint)
		}
	}

	if tree.IsTreeEdge(nil) {
		t.Fatalf("expected IsTreeEdge(nil) to be false")
	}
}
