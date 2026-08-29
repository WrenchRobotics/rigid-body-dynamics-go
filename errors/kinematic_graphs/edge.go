package kinematic_graph_errors

import "fmt"

type EdgeNotFoundError struct {
	FromID int64
	ToID   int64
}

func (e *EdgeNotFoundError) Error() string {
	return fmt.Sprintf(
		"no edge was found between graph node \"%v\" and \"%v\".",
		e.FromID,
		e.ToID,
	)
}
