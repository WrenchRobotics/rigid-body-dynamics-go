package utils

import (
	"fmt"

	"gonum.org/v1/gonum/mat"
)

// CrossOperator returns the skew-symmetric matrix (cross-product operator) of a 3-dimensional vector.
// This matrix also represents the operator that performs the cross product of a vector r
// to another vector (given as input via multiplication on the right).
func CrossOperator(r mat.Vector) (*mat.Dense, error) {
	if r.Len() != 3 {
		return nil, fmt.Errorf("vector must be 3-dimensional; got %d", r.Len())
	}

	return mat.NewDense(3, 3, []float64{
		0, -r.AtVec(2), r.AtVec(1),
		r.AtVec(2), 0, -r.AtVec(0),
		-r.AtVec(1), r.AtVec(0), 0,
	}), nil
}

func Cross(r mat.Vector) (*mat.Dense, error) {
	return CrossOperator(r)
}

func XLT(r mat.Vector) (*mat.Dense, error) {
	// Create ingredients
	eye3 := mat.NewDense(3, 3, []float64{
		1, 0, 0,
		0, 1, 0,
		0, 0, 1,
	})
	zero3 := mat.NewDense(3, 3, []float64{
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
	})
	rCross, err := CrossOperator(r)
	if err != nil {
		return nil, err
	}

	top := mat.NewDense(3, 6, nil)
	top.Augment(eye3, zero3)

	bottom := mat.NewDense(3, 6, nil)
	bottom.Augment(rCross, eye3)

	// Create result
	result := mat.NewDense(6, 6, nil)
	result.Stack(top, bottom)

	return result, nil
}
