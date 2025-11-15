package kinematic_trees

import (
	kinematic_trees_errors "github.com/WrenchRobotics/rigid-body-dynamics-go/errors/kinematic_trees"
	"github.com/WrenchRobotics/urdf-go/urdf_model/link"
)

type KinematicTreeNode struct {
	asLink *link.Link
	id     int64
}

func (n *KinematicTreeNode) Create(linkIn *link.Link, idIn int) {
	n.asLink = linkIn
	n.id = int64(idIn)
}

func (n *KinematicTreeNode) GetLink() (*link.Link, error) {
	if n == nil {
		return nil, &kinematic_trees_errors.KinematicTreeNodeDoesNotExistError{}
	}
	return n.asLink, nil
}

func (n *KinematicTreeNode) GetName() (string, error) {
	nAsLink, err := n.GetLink()
	if err != nil {
		return "", err
	}

	// Return name of the link
	return nAsLink.Name, nil
}

func (n *KinematicTreeNode) ID() int64 {
	return n.id
}
