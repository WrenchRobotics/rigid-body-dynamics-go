package kinematic_graphs_test

import (
	"testing"

	"github.com/WrenchRobotics/rigid-body-dynamics-go/systems"
)

/*
TestNumbering_CrankSlider_BaseIsMinimumID tests the numbering of the crank-slider mechanism.
The ordering is listed in Figure 4.1 of the Rigid Body Dynamics book.

It verifies that the node with name "base" has the lowest ID in the kinematic graph.
*/
func TestNumbering_CrankSlider_BaseIsMinimumID(t *testing.T) {
	// Load the Crank-Slider system's model from a URDF file
	kg, err := systems.CreateCrankSliderLinkageKinematicGraph()
	if err != nil {
		// Handle the error appropriately
		panic(err)
	}

	// Verify that the node with name "base" has the lowest ID
	baseNode, err := kg.GetNode(0)
	if err != nil {
		t.Fatalf("error getting base node: %v", err)
	}

	if baseNode == nil {
		t.Fatalf("base node not found in kinematic graph")
	}
	if baseNode.ID() != 0 {
		t.Fatalf("expected base node to have ID 0, got %d", baseNode.ID())
	}
}

/*
TestNumbering_CrankSlider tests the numbering of the crank-slider mechanism.
The ordering is listed in Figure 4.1 of the Rigid Body Dynamics book.

It verifies that the node with name "B2" has the HIGHEST ID in the kinematic graph.
*/
func TestNumbering_CrankSlider_HighestID(t *testing.T) {
	kg, err := systems.CreateCrankSliderLinkageKinematicGraph()
	if err != nil {
		panic(err)
	}

	// Verify that the node with name "B2" has the highest ID
	b2Node, err := kg.GetNodeByName("B2")
	if err != nil {
		t.Fatalf("error getting B2 node: %v", err)
	}

	if b2Node == nil {
		t.Fatalf("B2 node not found in kinematic graph")
	}

	if int(b2Node.ID()) != kg.Nodes().Len()-1 {
		t.Fatalf("expected B2 node to have highest ID %d, got %d", kg.Nodes().Len()-1, b2Node.ID())
	}
}
