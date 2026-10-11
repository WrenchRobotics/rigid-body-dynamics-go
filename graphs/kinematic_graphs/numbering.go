package kinematic_graphs

type EdgeKey struct {
	FromID int64
	ToID   int64
}

type Numbering struct {
	nodeIDToNumbering  map[int64]uint64
	edgeKeyToNumbering map[EdgeKey]uint64
}

func NewNumbering() *Numbering {
	return &Numbering{
		nodeIDToNumbering:  make(map[int64]uint64),
		edgeKeyToNumbering: make(map[EdgeKey]uint64),
	}
}

func (n *Numbering) AssignNumber(nodeID int64, number uint64) {
	n.nodeIDToNumbering[nodeID] = number
}

func (n *Numbering) GetNodeNumber(nodeID int64) (uint64, bool) {
	num, ok := n.nodeIDToNumbering[nodeID]
	return num, ok
}

func (n *Numbering) GetEdgeNumber(fromID, toID int64) (uint64, bool) {
	num, ok := n.edgeKeyToNumbering[EdgeKey{FromID: fromID, ToID: toID}]
	return num, ok
}

// MaxNodeNumber returns the maximum node number assigned, and a boolean indicating if any nodes have been numbered.
func (n *Numbering) MaxNodeNumber() (uint64, bool) {
	var max uint64
	found := false
	for _, num := range n.nodeIDToNumbering {
		if !found || num > max {
			max = num
			found = true
		}
	}
	return max, found
}
