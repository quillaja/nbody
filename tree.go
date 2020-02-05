package main

import (
	"math"

	"github.com/go-gl/mathgl/mgl64"
)

/*

spacial tree acceleration structure.
point oct-tree based on Barnes-Hut.
https://en.wikipedia.org/wiki/Barnes%E2%80%93Hut_simulation

*/

type nodekind uint8

// node types
const (
	external nodekind = iota
	internal
)

type octant uint8

// child positions (octants)
// low bit is X axis, high bit is Z axis
// L (0) means < center, H (1) means >= center
const (
	LLL octant = 0b000
	LLH octant = 0b001
	LHL octant = 0b010
	LHH octant = 0b011
	HLL octant = 0b100
	HLH octant = 0b101
	HHL octant = 0b110
	HHH octant = 0b111
)

type nodebound struct {
	center, width mgl64.Vec3
}

// return 8 points corresponding to the corners of the bound.
func (n nodebound) corners() []mgl64.Vec3 {
	pts := make([]mgl64.Vec3, 8)
	halfwidth := n.width.Mul(0.5)
	for i := LLL; i <= HHH; i++ {
		pts[i] = mgl64.Vec3{
			n.center[0] + halfwidth[0]*(float64(((i&LLH)>>0)*2)-1.0),
			n.center[1] + halfwidth[1]*(float64(((i&LHL)>>1)*2)-1.0),
			n.center[2] + halfwidth[2]*(float64(((i&HLL)>>2)*2)-1.0),
		}
	}
	return pts
}

// returns the max width among the 3 dimensions.
func (n nodebound) max() float64 {
	return math.Max(n.width[0], math.Max(n.width[1], n.width[2]))
}

// does this bound contain point?
func (n nodebound) contains(point mgl64.Vec3) bool {
	// upper side is compared with < because when assigning octants (eg with octantBits())
	// a point exactly on the parent's midpoint is assigned to the HHH octant.
	halfwidth := n.width.Mul(0.5)
	return (n.center[0]-halfwidth[0] <= point[0] && point[0] < n.center[0]+halfwidth[0]) &&
		(n.center[1]-halfwidth[1] <= point[1] && point[1] < n.center[1]+halfwidth[1]) &&
		(n.center[2]-halfwidth[2] <= point[2] && point[2] < n.center[2]+halfwidth[2])
}

// does this bound completely contain a sphere at the point with radius r?
func (n nodebound) containsSphere(point mgl64.Vec3, r float64) bool {
	w := n.width.Sub(mgl64.Vec3{r, r, r})
	d := point.Sub(n.center)
	return math.Abs(d[0]) <= w[0] && math.Abs(d[1]) <= w[1] && math.Abs(d[2]) <= w[2]
}

// does this bound intersect a sphere at the point with radius r?
func (n nodebound) intersectSphere(point mgl64.Vec3, r float64) bool {
	w := n.width.Add(mgl64.Vec3{r, r, r})
	d := point.Sub(n.center)
	return math.Abs(d[0]) <= w[0] || math.Abs(d[1]) <= w[1] || math.Abs(d[2]) <= w[2]
}

// scale the width of the bounds.
func (n nodebound) scale(s float64) nodebound {
	n.width = n.width.Mul(s)
	return n
}

// move the center of the bound.
func (n nodebound) translate(tx mgl64.Vec3) nodebound {
	n.center = n.center.Add(tx)
	return n
}

// generate the bounds for and octant of the parent's bounds.
func octantBound(parent nodebound, oct octant) nodebound {
	// calculate a translation for the new octant using the fact that
	// each octant is ±1/4 of the parent's width from the parent's center.
	tx := mgl64.Vec3{
		parent.width[0] * 0.25 * (float64((oct&LLH)*2) - 1.0),
		parent.width[1] * 0.25 * (float64(((oct&LHL)>>1)*2) - 1.0),
		parent.width[2] * 0.25 * (float64(((oct&HLL)>>2)*2) - 1.0),
	}
	return parent.scale(0.5).translate(tx)
}

// determines which octant (relative to midpoint) in which point belongs.
func octantSimple(midpoint, point mgl64.Vec3) (oct octant) {
	if point.X() > midpoint.X() {
		oct |= LLH
	}
	if point.Y() > midpoint.Y() {
		oct |= LHL
	}
	if point.Z() > midpoint.Z() {
		oct |= HLL
	}
	return
}

// determines which octant (relative to midpoint) in which point belongs.
// a point on the midpoint goes to the (H)igh octant.
func octantBits(midpoint, point mgl64.Vec3) octant {
	return octant((^math.Float64bits(point[0]-midpoint[0]) >> 63) |
		(^math.Float64bits(point[1]-midpoint[1])>>63)<<1 |
		(^math.Float64bits(point[2]-midpoint[2])>>63)<<2)
}

type node struct {
	kind         nodekind
	children     []*node
	particle     **body
	totalMass    float64
	centerOfMass mgl64.Vec3 // com += pos * (mass/(mass+totalmass))
	bounds       nodebound
}

// place a body in the tree rooted at this node.
// returns false if the body doesn't belong in this node.
func (n *node) push(bp **body) bool {
	b := *bp
	point := mgl64.Vec3{b.X, b.Y, b.Z}
	if !n.bounds.contains(point) {
		return false
	}

	switch n.kind {
	case external:
		// simple case: this is an external (leaf) node
		// that is currently empty
		if n.particle == nil {
			n.particle = bp
			n.centerOfMass = point
			n.totalMass = b.Mass
			return true
		}

		// 'complex' case: this is an external (leaf) node
		// that already has a particle
		//
		// 1) convert this node into an internal node by splitting this node
		// into new subnodes (octants), and push
		// the existing body into appropriate child node.
		// actual child nodes are instantiated only when needed.
		n.children = make([]*node, 8)
		oct := octantBits(n.bounds.center, n.centerOfMass)
		if n.children[oct] == nil {
			n.children[oct] = &node{bounds: octantBound(n.bounds, oct)}
		}
		n.children[oct].push(n.particle)
		n.particle = nil  // "remove" the body from this node
		n.kind = internal // change kind to internal

		// 2) process the incoming body, which is (conveniently)
		// exactly the same as for an internal node
		fallthrough

	case internal:
		// push body to appropriate child node
		oct := octantBits(n.bounds.center, point)
		if n.children[oct] == nil {
			n.children[oct] = &node{bounds: octantBound(n.bounds, oct)}
		}
		if n.children[oct].push(bp) {
			// update group mass info
			n.totalMass += b.Mass
			n.centerOfMass = n.centerOfMass.Add(point.Mul(b.Mass / n.totalMass))
		}
	}

	return true
}

// walk the body through the tree, applying gravitational force to it
// from nearby bodies or distant "aggregate" bodies, using theta as the
// accuracy dial.
func (n *node) gravity(bp **body, theta float64) {
	if n.totalMass == 0 {
		return // this is an empty leaf
	}
	if n.particle == bp {
		return // prevent a body interacting with itself
	}

	b := *bp
	p := mgl64.Vec3{b.X, b.Y, b.Z}
	r := n.centerOfMass.Sub(p).Len()

	switch n.kind {
	case internal:
		criterion := n.bounds.max() / r
		if criterion > theta {
			// the body is too close to the node CoM to treat the node
			// as a single distant point.
			// recurse to children
			for i := LLL; i <= HHH; i++ {
				if n.children[i] != nil {
					n.children[i].gravity(bp, theta)
				}
			}
			return
		}
		// the body is far enough away from the node CoM.
		// proceed to calculate gravity on the body due to the node CoM.
		fallthrough

	case external:
		// calculate and apply gravitational force
		f := G * (n.totalMass * b.Mass) / (r * r) // magnitude of force

		b.fx += (n.centerOfMass.X() - b.X) / r * f
		b.fy += (n.centerOfMass.Y() - b.Y) / r * f
		b.fz += (n.centerOfMass.Z() - b.Z) / r * f
	}
}

// walks the body through the tree where the sphere of the body intersects
// a node's bounds. if the node is an leaf (external) AND has a non-nil body,
// check that for a collision and enqueue it for later processing if necessary.
func (n *node) checkCollision(bp **body, collisions *[][2]**body) {
	if n.particle == bp {
		return // prevent self-intersection
	}

	p := mgl64.Vec3{(*bp).X, (*bp).Y, (*bp).Z}
	if !n.bounds.intersectSphere(p, (*bp).Radius) {
		return
	}

	switch n.kind {
	case external:
		if n.particle != nil && (*n.particle) != nil {
			r := dist(*n.particle, *bp)
			if r <= (*n.particle).Radius+(*bp).Radius {
				*collisions = append(*collisions, [2]**body{bp, n.particle})
			}
		}

	case internal:
		oct := octantBits(n.bounds.center, p)
		if n.children[oct] != nil && n.children[oct].bounds.containsSphere(p, (*bp).Radius) {
			// best case: the octant for the body completely contains
			// the sphere of the body, and therefore the other 7 child
			// nodes (and their children!) do not have to be checked.
			n.children[oct].checkCollision(bp, collisions)
		} else {
			// worse case: check all the children of this node.
			for i := LLL; i <= HHH; i++ {
				if n.children[i] == nil {
					continue
				}
				n.children[i].checkCollision(bp, collisions)
			}
		}
	}
}

// builds a tree by pushing all bodies into the root
func maketree(bodies []*body, simulationBound nodebound) (root *node) {
	root = &node{bounds: simulationBound}
	for i := 0; i < len(bodies); i++ {
		if bodies[i] == nil {
			continue
		}
		if !root.push(&(bodies[i])) {
			bodies[i] = nil // remove out-of-bounds bodies from the simulation
		}
	}
	return
}
