package kinematic_trees_test

import (
	"testing"

	"github.com/WrenchRobotics/rigid-body-dynamics-go/systems"
)

// TestKinematicGraph_IsTree1 is a test where the
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
