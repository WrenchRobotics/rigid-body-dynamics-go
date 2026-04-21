package systems

import (
	"encoding/xml"
	"fmt"

	_ "embed"

	"github.com/WrenchRobotics/rigid-body-dynamics-go/graphs/kinematic_graphs"
	"github.com/WrenchRobotics/urdf-go/decoding"
	model_errors "github.com/WrenchRobotics/urdf-go/errors"
	urdfmodel "github.com/WrenchRobotics/urdf-go/urdf_model"
)

//go:embed crank_slider_linkage.urdf
var crankSliderLinkageUrdf string // Contents of the above file are loaded into this string

func CreateCrankSliderLinkageKinematicGraph() (*kinematic_graphs.KinematicGraph, error) {
	// Decode the xml
	var robotElts []decoding.RobotElement
	err := xml.Unmarshal([]byte(crankSliderLinkageUrdf), &robotElts)
	if err != nil {
		return nil, fmt.Errorf("there was an issue decoding the urdf: %v", err)
	}

	// Check that at least one robot element was found
	if len(robotElts) == 0 {
		return nil, model_errors.NoRobotsFoundError{FilePath: "crank_slider_linkage.urdf"}
	}

	// Load using our loading library
	urdfModel, err := urdfmodel.DeriveModelFrom(&robotElts[0])
	if err != nil {
		return nil, fmt.Errorf("there was an issue deriving the urdf model from the urdf model elements: %v", err)
	}

	// Create kinematic graph using urdfmodel
	testGraph := kinematic_graphs.NewKinematicGraph()
	err = testGraph.ExtractFromModel(urdfModel)
	return testGraph, err

}
