package kinematic_graphs_test

import (
	"strings"
	"testing"

	"github.com/WrenchRobotics/rigid-body-dynamics-go/graphs/kinematic_graphs"
	"github.com/WrenchRobotics/rigid-body-dynamics-go/systems"
)

/*
numberOfNodeNamed returns the number assigned by numbering to the node
in kg with the given name.
*/
func numberOfNodeNamed(t *testing.T, kg *kinematic_graphs.KinematicGraph, numbering *kinematic_graphs.Numbering, name string) uint64 {
	t.Helper()

	node, err := kg.GetNodeByName(name)
	if err != nil {
		t.Fatalf("error getting %v node: %v", name, err)
	}

	number, ok := numbering.GetNodeNumber(node.ID())
	if !ok {
		t.Fatalf("node %v was not assigned a number", name)
	}

	return number
}

/*
TestNumbering_CrankSlider_BaseIsMinimumNumber tests the numbering of the crank-slider mechanism.
The ordering is listed in Figure 4.1 of the Rigid Body Dynamics book.

It verifies that the node with name "base" is assigned the number 0.
*/
func TestNumbering_CrankSlider_BaseIsMinimumNumber(t *testing.T) {
	// Load the Crank-Slider system's model from a URDF file
	kg, err := systems.CreateCrankSliderLinkageKinematicGraph()
	if err != nil {
		t.Fatalf("error creating crank-slider kinematic graph: %v", err)
	}

	numbering, err := kg.ComputeNumbering()
	if err != nil {
		t.Fatalf("error computing numbering: %v", err)
	}

	// Verify that the node with name "base" has the number 0
	if number := numberOfNodeNamed(t, kg, numbering, "base"); number != 0 {
		t.Fatalf("expected base node to have number 0, got %d", number)
	}
}

/*
TestNumbering_CrankSlider_HighestNumber tests the numbering of the crank-slider mechanism.
The ordering is listed in Figure 4.1 of the Rigid Body Dynamics book.

It verifies that the node with name "B2" has the HIGHEST number in the kinematic graph.
*/
func TestNumbering_CrankSlider_HighestNumber(t *testing.T) {
	kg, err := systems.CreateCrankSliderLinkageKinematicGraph()
	if err != nil {
		t.Fatalf("error creating crank-slider kinematic graph: %v", err)
	}

	numbering, err := kg.ComputeNumbering()
	if err != nil {
		t.Fatalf("error computing numbering: %v", err)
	}

	// Verify that the node with name "B2" has the highest number
	expected := uint64(kg.Nodes().Len() - 1)
	if number := numberOfNodeNamed(t, kg, numbering, "B2"); number != expected {
		t.Fatalf("expected B2 node to have highest number %d, got %d", expected, number)
	}
}

/*
TestNumbering_CrankSlider_Deterministic verifies that the crank-slider is
numbered the same way every time it is loaded.
*/
func TestNumbering_CrankSlider_Deterministic(t *testing.T) {
	expected := map[string]uint64{"base": 0, "B1": 1, "B3": 2, "B2": 3}

	for i := 0; i < 50; i++ {
		kg, err := systems.CreateCrankSliderLinkageKinematicGraph()
		if err != nil {
			t.Fatalf("error creating crank-slider kinematic graph: %v", err)
		}

		numbering, err := kg.ComputeNumbering()
		if err != nil {
			t.Fatalf("error computing numbering: %v", err)
		}

		for name, number := range expected {
			if got := numberOfNodeNamed(t, kg, numbering, name); got != number {
				t.Fatalf("expected %v to have number %d, got %d (iteration %d)", name, number, got, i)
			}
		}
	}
}

/*
TestNumbering_CrankSlider_WriteNumberingSVG verifies that the SVG drawing of the
crank-slider's numbering contains every link name and joint name, and draws
exactly one (dashed) loop-closing joint.
*/
func TestNumbering_CrankSlider_WriteNumberingSVG(t *testing.T) {
	kg, err := systems.CreateCrankSliderLinkageKinematicGraph()
	if err != nil {
		t.Fatalf("error creating crank-slider kinematic graph: %v", err)
	}

	var b strings.Builder
	if err := kg.WriteNumberingSVG(&b); err != nil {
		t.Fatalf("error writing SVG: %v", err)
	}
	svg := b.String()

	if !strings.HasPrefix(svg, "<svg") || !strings.HasSuffix(svg, "</svg>\n") {
		t.Fatalf("expected output to be a single <svg> element, got:\n%v", svg)
	}

	for _, name := range []string{"base", "B1", "B2", "B3", "J1", "J2", "J3", "J4"} {
		if !strings.Contains(svg, ">"+name+"</text>") {
			t.Fatalf("expected SVG to contain a label for %v", name)
		}
	}

	if count := strings.Count(svg, "stroke-dasharray"); count != 1 {
		t.Fatalf("expected exactly 1 dashed (loop-closing) edge, got %d", count)
	}
}
