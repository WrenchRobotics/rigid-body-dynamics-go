package kinematic_graphs

import "gonum.org/v1/gonum/graph"

type KinematicGraphEdge struct {
	from *KinematicTreeNode
	to   *KinematicTreeNode
}

func (e *KinematicGraphEdge) Create(fromIn *KinematicTreeNode, toIn *KinematicTreeNode) {
	e.from = fromIn
	e.to = toIn
}

func (e *KinematicGraphEdge) From() graph.Node {
	return e.from
}

func (e *KinematicGraphEdge) To() graph.Node {
	return e.to
}

func (e *KinematicGraphEdge) ReversedEdge() graph.Edge {
	return &KinematicGraphEdge{
		from: e.to,
		to:   e.from,
	}
}
