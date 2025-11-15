package kinematic_trees_test

import (
	"testing"

	"github.com/WrenchRobotics/rigid-body-dynamics-go/kinematic_trees"
	"github.com/WrenchRobotics/urdf-go/urdf_model/link"
)

func TestKinematicTreeNode_GetName1(t *testing.T) {
	// Setup
	link1 := &link.Link{
		Name: "link1",
	}

	// Create a new KinematicTreeNode
	var node kinematic_trees.KinematicTreeNode
	node.Create(link1, 1)

	// Get the name of the node
	name, err := node.GetName()
	if err != nil {
		t.Errorf("Error getting name: %v", err)
		return
	}

	// Check if the name is correct
	if name != "link1" {
		t.Errorf("Expected name 'link1', got '%s'", name)
	}
}
