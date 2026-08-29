package main

import (
	"fmt"

	connectivity_graphs "github.com/WrenchRobotics/rigid-body-dynamics-go/graphs/connectivity"
	"github.com/WrenchRobotics/rigid-body-dynamics-go/graphs/kinematic_graphs"
	"github.com/WrenchRobotics/rigid-body-dynamics-go/systems"
)

func main() {
	// Load the Crank-Slider system's model from a URDF file
	kinematicGraph, err := systems.CreateCrankSliderLinkageKinematicGraph()
	if err != nil {
		// Handle the error appropriately
		panic(err)
	}

	// Extract the connectivity information from the kinematic graph
	connectivityGraph, err := connectivity_graphs.ExtractFromKinematicGraph(kinematicGraph)
	if err != nil {
		// Handle the error appropriately
		panic(err)
	}

	// Now, let's print some details about it
	fmt.Println("Connectivity Graph:")
	kinematicGraphNodes := connectivityGraph.SourceGraph.Nodes()
	for kinematicGraphNodes.Next() {
		node := kinematicGraphNodes.Node().ID()
		fmt.Printf("Node ID: %d\n", node)

		kinematicGraphNode := connectivityGraph.SourceGraph.Node(node).(*kinematic_graphs.KinematicGraphNode)
		asLink, err := kinematicGraphNode.GetLink()
		if err != nil {
			fmt.Printf("\tError retrieving link: %v\n", err)
			continue
		}
		fmt.Printf("  Name: %s\n", asLink.Name)

		// Print the predecessor and successor for each node
		// - Successors
		successors, err := connectivityGraph.GetAllSuccessors(node)
		if err != nil {
			fmt.Printf("\tError retrieving successors: %v\n", err)
			continue
		}
		for _, s := range successors {
			successorAsLink, err := connectivityGraph.GetLinkByNodeID(s)
			if err != nil {
				fmt.Printf("\tError retrieving successor link: %v\n", err)
				continue
			}
			fmt.Printf("  Successor: %d (%s)\n", s, successorAsLink.Name)
		}

		// - Predecessors
		predecessors, err := connectivityGraph.GetAllPredecessors(node)
		if err != nil {
			fmt.Printf("\tError retrieving predecessors: %v\n", err)
			continue
		}
		for _, p := range predecessors {
			predecessorAsLink, err := connectivityGraph.GetLinkByNodeID(p)
			if err != nil {
				fmt.Printf("\tError retrieving predecessor link: %v\n", err)
				continue
			}
			fmt.Printf("  Predecessor: %d (%s)\n", p, predecessorAsLink.Name)
		}
	}

}
