package kinematic_graphs

import (
	"github.com/WrenchRobotics/urdf-go/urdf_model/joint"
	"gonum.org/v1/gonum/graph"
)

type KinematicGraphEdge struct {
	from    *KinematicGraphNode
	to      *KinematicGraphNode
	AsJoint *joint.Joint
}

func NewKinematicGraphEdge(fromIn *KinematicGraphNode, toIn *KinematicGraphNode, joint *joint.Joint) KinematicGraphEdge {
	return KinematicGraphEdge{
		from:    fromIn,
		to:      toIn,
		AsJoint: joint,
	}
}

func (e *KinematicGraphEdge) From() graph.Node {
	return e.from
}

func (e *KinematicGraphEdge) To() graph.Node {
	return e.to
}

func (e *KinematicGraphEdge) ReversedEdge() graph.Edge {
	return &KinematicGraphEdge{
		from:    e.to,
		to:      e.from,
		AsJoint: e.AsJoint,
	}
}
