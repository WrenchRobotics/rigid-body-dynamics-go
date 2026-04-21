package kinematic_graph_errors

import "fmt"

type NodeDoesNotExistError struct {
	id   int64
	name string
}

func MakeNodeDoesNotExistErrorForID(id int64) NodeDoesNotExistError {
	return NodeDoesNotExistError{
		id:   id,
		name: "",
	}
}

func MakeNodeDoesNotExistErrorForName(name string) NodeDoesNotExistError {
	return NodeDoesNotExistError{
		id:   -1,
		name: name,
	}
}

func (e *NodeDoesNotExistError) Error() string {
	nameIsGiven := e.name != ""
	if nameIsGiven {
		return fmt.Sprintf(
			"kinematic graph node with name \"%v\" does not exist",
			e.name,
		)
	} else {
		return fmt.Sprintf(
			"kinematic graph node with id \"%v\" does not exist",
			e.id,
		)
	}
}
