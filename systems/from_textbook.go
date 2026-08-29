package systems

import (
	"fmt"

	_ "embed"

	"github.com/WrenchRobotics/rigid-body-dynamics-go/graphs/kinematic_graphs"
	"github.com/WrenchRobotics/urdf-go/loaders"
)

//go:embed crank_slider_linkage.urdf
var crankSliderLinkageUrdf string // Contents of the above file are loaded into this string

func CreateCrankSliderLinkageKinematicGraph() (*kinematic_graphs.KinematicGraph, error) {
	// Load using our loading library
	urdfModel, err := loaders.FromURDFContents([]byte(crankSliderLinkageUrdf))
	if err != nil {
		return nil, fmt.Errorf("there was an issue deriving the urdf model from the urdf model elements: %v", err)
	}

	// Create kinematic graph using urdfmodel
	testGraph := kinematic_graphs.NewKinematicGraph()
	err = testGraph.ExtractFromModel(urdfModel)
	return testGraph, err

}
