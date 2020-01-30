package main

import (
	"fmt"
	"math"
	"math/rand"
)

/*

physics section

*/

// initializes n bodies.
func makebodies(n int, cores []body) []*body {
	const orbitalVDampening = 0.9
	const meanMass = 1e3
	nc := len(cores)
	bodies := make([]*body, n+nc)

	for i := nc; i < len(bodies); i++ {
		m := math.Abs(rand.NormFloat64()*500 + meanMass)

		core := body{}
		if nc > 0 {
			group := rand.Intn(nc)
			core = cores[group]
		}

		bodies[i] = &body{}
		bodies[i].Mass = m
		bodies[i].X = rand.NormFloat64()*(1000*(1-math.Abs(core.fx))+100*math.Abs(core.fx)) + core.X
		bodies[i].Y = rand.NormFloat64()*(1000*(1-math.Abs(core.fy))+100*math.Abs(core.fy)) + core.Y
		bodies[i].Z = rand.NormFloat64()*(1000*(1-math.Abs(core.fz))+100*math.Abs(core.fz)) + core.Z

		if nc > 0 {
			// apply initial orbital velocity around center point
			// in a direction perpendicular to the body-center vector and
			// the axis of rotation in core.f.
			dx, dy, dz := core.X-bodies[i].X, core.Y-bodies[i].Y, core.Z-bodies[i].Z
			d := math.Sqrt(dx*dx + dy*dy + dz*dz)
			if d == 0 {
				d = 1
			}
			dx /= d
			dy /= d
			dz /= d
			dx, dy, dz = cross(dx, dy, dz, core.fx, core.fy, core.fz)
			core.fx, core.fy, core.fz = 0, 0, 0 // re-zero core.f

			v := math.Sqrt(G * core.Mass / d)
			bodies[i].Vx = dx*v*orbitalVDampening + core.Vx
			bodies[i].Vy = dy*v*orbitalVDampening + core.Vy
			bodies[i].Vz = dz*v*orbitalVDampening + core.Vz
		}
	}

	// insert centers as bodies in the front of the slice
	for i := range cores {
		cores[i].fx, cores[i].fy, cores[i].fz = 0, 0, 0
		bodies[i] = &cores[i]
	}

	return bodies
}

// cross product.
func cross(x1, y1, z1, x2, y2, z2 float64) (x3, y3, z3 float64) {
	x3 = y1*z2 - z1*y2
	y3 = z1*x2 - x1*z2
	z3 = x1*y2 - y1*x2
	return
}

type body struct {
	Mass       float64 // kg
	X, Y, Z    float64 // m
	Vx, Vy, Vz float64 // m/s
	fx, fy, fz float64 // accumulated force
}

// update body velocity and position, reset accumulated force.
func (b *body) update(dt float64) {
	// a = F/m
	// dv = a*dt
	b.Vx += b.fx / b.Mass * dt
	b.Vy += b.fy / b.Mass * dt
	b.Vz += b.fz / b.Mass * dt

	// dp = v*dt
	b.X += b.Vx * dt
	b.Y += b.Vy * dt
	b.Z += b.Vz * dt

	// clear force
	b.fx, b.fy, b.fz = 0, 0, 0
}

func (b body) String() string {
	return fmt.Sprintf("m: %.4f\np: [%.2f, %.2f, %.2f]\nv: [%.2f, %.2f, %.2f]\n",
		b.Mass, b.X, b.Y, b.Z, b.Vx, b.Vy, b.Vz)
}

// distance between two bodies.
func dist(a, b *body) float64 {
	dx := b.X - a.X
	dy := b.Y - a.Y
	dz := b.Z - a.Z
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

// G = 6.67408 × 10-11 m3 kg-1 s-2
//   = 6.67408e-11 m³/(kg·s²)
const G = 6.67408e-11

// adds gravitational force to a and b.
func gravity(r float64, a, b *body) {
	f := G * (a.Mass * b.Mass) / (r * r) // magnitude of force

	dfx := (b.X - a.X) / r * f
	dfy := (b.Y - a.Y) / r * f
	dfz := (b.Z - a.Z) / r * f

	a.fx += dfx
	a.fy += dfy
	a.fz += dfz

	b.fx -= dfx
	b.fy -= dfy
	b.fz -= dfz
}

// calculates the final velocity of a and b in a perfectly inelastic collision.
func inelasticCollision(ma, va, mb, vb float64) (vc float64) {
	return (ma*va + mb*vb) / (ma + mb)
}

// combine a and b into a new body.
func combine(a, b *body) body {
	return body{
		Mass: a.Mass + b.Mass,
		X:    a.X,
		Y:    a.Y,
		Z:    a.Z,
		Vx:   inelasticCollision(a.Mass, a.Vx, b.Mass, b.Vx),
		Vy:   inelasticCollision(a.Mass, a.Vy, b.Mass, b.Vy),
		Vz:   inelasticCollision(a.Mass, a.Vz, b.Mass, b.Vz),
		fx:   a.fx, // NOTE: may be inaccurate. maybe add b.f?
		fy:   a.fy,
		fz:   a.fz,
	}
}
