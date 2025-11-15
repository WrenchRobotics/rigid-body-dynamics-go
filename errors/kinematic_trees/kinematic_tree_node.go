package kinematic_trees_errors

type KinematicTreeNodeDoesNotExistError struct{}

func (e *KinematicTreeNodeDoesNotExistError) Error() string {
	return "Kinematic tree node does not exist"
}
