package kinematic_graphs

import (
	"fmt"
	"html"
	"io"
	"math"
	"os"
	"sort"
	"strings"
)

// Dimensions (in pixels) used when drawing a numbering diagram.
const (
	diagramColumnWidth  = 180.0
	diagramLayerHeight  = 140.0
	diagramMargin       = 60.0
	diagramNodeRadius   = 22.0
	diagramLabelPadding = 18.0
	diagramLoopBend     = 0.25 // How far loop-closing edges bend, relative to their length
)

type diagramPoint struct{ X, Y float64 }

// diagramNode is a link drawn as a circle containing its number, with its name underneath.
type diagramNode struct {
	Center diagramPoint
	Number uint64
	Name   string
	IsRoot bool
}

// diagramEdge is a joint drawn as a quadratic Bezier curve from parent to child.
type diagramEdge struct {
	Start, Control, End diagramPoint
	Label               string
	IsLoop              bool // true if the joint is not in the spanning tree
}

/*
numberingDiagram is a laid-out picture of a kinematic graph and its numbering.
It contains only geometry and labels, so it can be drawn without referring back
to the graph.
*/
type numberingDiagram struct {
	Width, Height float64
	Nodes         []diagramNode
	Edges         []diagramEdge
}

/*
SaveNumberingToFile draws the kinematic graph and its numbering as an SVG image
(see WriteNumberingSVG) and saves it to the file given by filePath.
*/
func (g *KinematicGraph) SaveNumberingToFile(filePath string) error {
	file, err := os.Create(filePath)
	if err != nil {
		return fmt.Errorf("failed to create file: %v", err)
	}
	defer file.Close()

	return g.WriteNumberingSVG(file)
}

/*
WriteNumberingSVG draws the kinematic graph as an SVG image and writes it to w.

Each node is drawn as a circle containing the number assigned to it by
ComputeNumbering, with the name of its link written underneath. Nodes are
arranged in layers by their depth in the spanning tree used for the numbering
(the root/fixed base is at the top). Each joint is drawn as an arrow from its
parent link to its child link; joints in the spanning tree are solid, while
loop-closing joints (those not in the spanning tree) are dashed.
*/
func (g *KinematicGraph) WriteNumberingSVG(w io.Writer) error {
	diagram, err := newNumberingDiagram(g)
	if err != nil {
		return err
	}

	return diagram.WriteSVG(w)
}

/*
newNumberingDiagram computes the spanning tree and numbering of g and lays out
the resulting diagram.
*/
func newNumberingDiagram(g *KinematicGraph) (numberingDiagram, error) {
	spanningTree, err := g.ToSpanningTree()
	if err != nil {
		return numberingDiagram{}, fmt.Errorf("failed to compute spanning tree: %v", err)
	}

	numbering, err := spanningTree.ComputeNumbering()
	if err != nil {
		return numberingDiagram{}, fmt.Errorf("failed to compute numbering: %v", err)
	}

	layers, err := groupNodesIntoLayers(g, spanningTree.Depths(), numbering)
	if err != nil {
		return numberingDiagram{}, err
	}

	diagram := numberingDiagram{}
	positions := diagram.layOut(layers)

	// Nodes (links)
	rootID := spanningTree.RootNode().ID()
	for _, layer := range layers {
		for _, id := range layer {
			name, err := g.nodeMap[id].GetName()
			if err != nil {
				return numberingDiagram{}, fmt.Errorf("failed to get node name: %v", err)
			}
			number, _ := numbering.GetNodeNumber(id) // Checked by groupNodesIntoLayers

			diagram.Nodes = append(diagram.Nodes, diagramNode{
				Center: positions[id],
				Number: number,
				Name:   name,
				IsRoot: id == rootID,
			})
		}
	}

	// Edges (joints)
	for _, edge := range g.edges {
		if edge == nil || edge.from == nil || edge.to == nil || edge.from == edge.to {
			continue
		}

		label := ""
		if edge.AsJoint != nil {
			label = edge.AsJoint.Name
		}
		if edgeNumber, ok := numbering.GetEdgeNumber(edge.from.ID(), edge.to.ID()); ok {
			label = fmt.Sprintf("%d: %s", edgeNumber, label)
		}

		diagram.Edges = append(diagram.Edges, newDiagramEdge(
			positions[edge.from.ID()],
			positions[edge.to.ID()],
			label,
			!spanningTree.IsTreeEdge(edge),
		))
	}

	return diagram, nil
}

/*
groupNodesIntoLayers returns the IDs of the nodes of g grouped by their depth,
with each layer sorted by node number.
*/
func groupNodesIntoLayers(g *KinematicGraph, depths map[int64]int, numbering *Numbering) ([][]int64, error) {
	var layers [][]int64
	for id := range g.nodeMap {
		depth, ok := depths[id]
		if !ok {
			return nil, fmt.Errorf("node with ID %d is not reachable from the root of the spanning tree", id)
		}
		if _, ok := numbering.GetNodeNumber(id); !ok {
			return nil, fmt.Errorf("node with ID %d was not assigned a number", id)
		}

		for len(layers) <= depth {
			layers = append(layers, nil)
		}
		layers[depth] = append(layers[depth], id)
	}

	for _, layer := range layers {
		sort.Slice(layer, func(i, j int) bool {
			ni, _ := numbering.GetNodeNumber(layer[i])
			nj, _ := numbering.GetNodeNumber(layer[j])
			return ni < nj
		})
	}

	return layers, nil
}

/*
layOut sets the size of the diagram to fit the given layers and returns the
position of each node, with each layer drawn as a centered row.
*/
func (d *numberingDiagram) layOut(layers [][]int64) map[int64]diagramPoint {
	widestLayer := 0
	for _, layer := range layers {
		widestLayer = max(widestLayer, len(layer))
	}
	d.Width = float64(widestLayer)*diagramColumnWidth + 2*diagramMargin
	d.Height = float64(len(layers)-1)*diagramLayerHeight + 2*diagramMargin + diagramNodeRadius + diagramLabelPadding

	positions := make(map[int64]diagramPoint)
	for depth, layer := range layers {
		offset := (d.Width - float64(len(layer))*diagramColumnWidth) / 2
		for i, id := range layer {
			positions[id] = diagramPoint{
				X: offset + (float64(i)+0.5)*diagramColumnWidth,
				Y: diagramMargin + float64(depth)*diagramLayerHeight,
			}
		}
	}

	return positions
}

/*
newDiagramEdge creates an edge between the nodes centered at from and to,
trimmed so that it starts and ends on the node circles. Loop-closing edges are
bent so that they are easy to tell apart from tree edges.
*/
func newDiagramEdge(from, to diagramPoint, label string, isLoop bool) diagramEdge {
	dx, dy := to.X-from.X, to.Y-from.Y
	length := math.Hypot(dx, dy)
	ux, uy := dx/length, dy/length

	start := diagramPoint{X: from.X + ux*diagramNodeRadius, Y: from.Y + uy*diagramNodeRadius}
	end := diagramPoint{X: to.X - ux*diagramNodeRadius, Y: to.Y - uy*diagramNodeRadius}
	control := diagramPoint{X: (start.X + end.X) / 2, Y: (start.Y + end.Y) / 2}
	if isLoop {
		bend := diagramLoopBend * length
		control.X, control.Y = control.X-uy*bend, control.Y+ux*bend
	}

	return diagramEdge{Start: start, Control: control, End: end, Label: label, IsLoop: isLoop}
}

// WriteSVG draws the diagram as an SVG image and writes it to w.
func (d numberingDiagram) WriteSVG(w io.Writer) error {
	var b strings.Builder
	fmt.Fprintf(&b, `<svg xmlns="http://www.w3.org/2000/svg" width="%.0f" height="%.0f" viewBox="0 0 %.0f %.0f" font-family="sans-serif">`+"\n", d.Width, d.Height, d.Width, d.Height)
	b.WriteString(`<defs><marker id="arrow" viewBox="0 0 10 10" refX="10" refY="5" markerWidth="8" markerHeight="8" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="#333"/></marker></defs>` + "\n")
	b.WriteString(`<rect width="100%" height="100%" fill="white"/>` + "\n")

	for _, edge := range d.Edges {
		dash := ""
		if edge.IsLoop {
			dash = ` stroke-dasharray="6,4"`
		}
		fmt.Fprintf(&b, `<path d="M%.1f,%.1f Q%.1f,%.1f %.1f,%.1f" fill="none" stroke="#333" stroke-width="1.5"%s marker-end="url(#arrow)"/>`+"\n",
			edge.Start.X, edge.Start.Y, edge.Control.X, edge.Control.Y, edge.End.X, edge.End.Y, dash)

		if edge.Label != "" {
			// The midpoint of a quadratic Bezier curve
			mx := 0.25*edge.Start.X + 0.5*edge.Control.X + 0.25*edge.End.X
			my := 0.25*edge.Start.Y + 0.5*edge.Control.Y + 0.25*edge.End.Y
			fmt.Fprintf(&b, `<text x="%.1f" y="%.1f" font-size="11" fill="#666" text-anchor="middle" dy="-4">%s</text>`+"\n",
				mx, my, html.EscapeString(edge.Label))
		}
	}

	for _, node := range d.Nodes {
		fill := "#e8f0fe"
		if node.IsRoot {
			fill = "#fde8c8"
		}
		fmt.Fprintf(&b, `<circle cx="%.1f" cy="%.1f" r="%.0f" fill="%s" stroke="#333" stroke-width="1.5"/>`+"\n",
			node.Center.X, node.Center.Y, diagramNodeRadius, fill)
		fmt.Fprintf(&b, `<text x="%.1f" y="%.1f" font-size="16" font-weight="bold" text-anchor="middle" dominant-baseline="central">%d</text>`+"\n",
			node.Center.X, node.Center.Y, node.Number)
		fmt.Fprintf(&b, `<text x="%.1f" y="%.1f" font-size="12" text-anchor="middle">%s</text>`+"\n",
			node.Center.X, node.Center.Y+diagramNodeRadius+diagramLabelPadding, html.EscapeString(node.Name))
	}

	b.WriteString("</svg>\n")

	if _, err := io.WriteString(w, b.String()); err != nil {
		return fmt.Errorf("failed to write SVG: %v", err)
	}

	return nil
}
