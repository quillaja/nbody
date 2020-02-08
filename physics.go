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
	const orbitalVDampening = 1.0
	const meanMass = 50e3   //1e3 // 50e3kg@2m =1492 kg/m3; 6e3kg@1m = 1432kg/m3
	const defaultRadius = 2 // given mean mass, this will produce very "nondense" bodies (1000kg@1m radius ≈ 238 kg/m³)
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
		bodies[i].ID = uint64(i)
		bodies[i].Mass = m
		bodies[i].Radius = defaultRadius
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
		bodies[i].ID = uint64(i)
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

// uniformly (no bias towards center) sample a disk with the given radius.
func uniformSampleDisk(radius float64) (x, y float64) {
	r := math.Sqrt(radius * rand.Float64())
	theta := 2 * math.Pi * rand.Float64()
	sin, cos := math.Sincos(theta)
	return r * cos, r * sin
}

type body struct {
	ID         uint64
	Mass       float64 // kg
	Radius     float64 // m
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

// adds to body a the gravitational force acting on a from b.
func gravity(r float64, a, b *body) {
	f := G * (a.Mass * b.Mass) / (r * r) // magnitude of force

	dfx := (b.X - a.X) / r * f
	dfy := (b.Y - a.Y) / r * f
	dfz := (b.Z - a.Z) / r * f

	a.fx += dfx
	a.fy += dfy
	a.fz += dfz
}

// calculates the final velocity of a and b in a perfectly inelastic collision.
func inelasticCollision(ma, va, mb, vb float64) (vc float64) {
	return (ma*va + mb*vb) / (ma + mb)
}

// combine a and b into a.
func combine(a, b *body) {
	a.Mass = a.Mass + b.Mass
	a.Radius = math.Cbrt(a.Radius*a.Radius*a.Radius + b.Radius*b.Radius*b.Radius)
	a.Vx = inelasticCollision(a.Mass, a.Vx, b.Mass, b.Vx)
	a.Vy = inelasticCollision(a.Mass, a.Vy, b.Mass, b.Vy)
	a.Vz = inelasticCollision(a.Mass, a.Vz, b.Mass, b.Vz)
	a.fx = a.fx + b.fx
	a.fy = a.fy + b.fy
	a.fz = a.fz + b.fz
}

// sphere volume from radius
func volume(radius float64) float64 {
	return 4.0 / 3.0 * math.Pi * (radius * radius * radius)
}

// sphere radius from volume
func radius(volume float64) float64 {
	return math.Cbrt((3.0 * volume) / (4.0 * math.Pi))
}

// sphere radius given a mass and density
func radiusMassDensity(mass, density float64) float64 {
	return math.Cbrt((3.0 * mass) / (4.0 * math.Pi * density))
}
