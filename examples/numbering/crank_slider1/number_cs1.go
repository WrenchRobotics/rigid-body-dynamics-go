package main

import (
	"fmt"

	"github.com/WrenchRobotics/rigid-body-dynamics-go/systems"
)

func main() {
	// Load the Crank-Slider system's model from a URDF file
	kg, err := systems.CreateCrankSliderLinkageKinematicGraph()
	if err != nil {
		// Handle the error appropriately
		panic(err)
	}

	// Create numbering for the Crank-Slider system's kinematic graph
	numbering, err := kg.ComputeNumbering()
	if err != nil {
		// Handle the error appropriately
		panic(err)
	}

	// Print the numbering for each node
	nodes := kg.Nodes()
	for nodes.Next() {
		node := nodes.Node()
		nodeID := node.ID()
		number, ok := numbering.GetNodeNumber(nodeID)
		if !ok {
			panic("node number not found")
		}
		fmt.Printf("Node ID: %d, Number: %d\n", nodeID, number)
	}
}
