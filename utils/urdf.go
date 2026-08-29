package utils

import (
	"github.com/WrenchRobotics/urdf-go/urdf_model/joint"
	"github.com/WrenchRobotics/urdf-go/urdf_model/link"
)

func FindJointConnectingTheseLinks(l1, l2 *link.Link) []*joint.Joint {
	// Extract all joints connected to each link
	// - Link1 Connected Joints
	var jointsConnectedToL1 []*joint.Joint
	if l1.ParentJoint == nil {
		jointsConnectedToL1 = append(jointsConnectedToL1, l1.ParentJoint)
	}
	jointsConnectedToL1 = append(jointsConnectedToL1, l1.ChildJoints...)

	// - Link 2 Connected joints
	var jointsConnectedToL2 []*joint.Joint
	if l2.ParentJoint == nil {
		jointsConnectedToL2 = append(jointsConnectedToL2, l2.ParentJoint)
	}
	jointsConnectedToL2 = append(jointsConnectedToL2, l2.ChildJoints...)

	// Check to see which joints are shared
	var sharedJoints []*joint.Joint
	for _, j1 := range jointsConnectedToL1 {
		for _, j2 := range jointsConnectedToL2 {
			if j1 == j2 {
				sharedJoints = append(sharedJoints, j1)
			}
		}
	}

	return sharedJoints

}
