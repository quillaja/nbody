package main

import (
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/draw"
	"image/png"
	"math"
	"math/rand"
	"os"
	"sync"
	"time"
)

func main() {
	// rand.Seed(time.Now().UnixNano())
	numbodies := flag.Int("n", 10, "number of bodies")
	years := flag.Int("y", 1, "number of years to simulate")
	flag.Parse()

	// setup image output workers
	ch := make(chan *frameJob, 8)
	workers := 4
	wg := sync.WaitGroup{}
	wg.Add(workers)
	for i := 0; i < workers; i++ {
		go frameOutput(&wg, ch)
	}

	// simulation parameters
	const dt = (60 * 60)       // 1 hour step
	steps := *years * 365 * 24 // years total simulation time
	bodies := makebodies(*numbodies, []body{
		// uses body.f to generate initial velocities of child bodies.
		// {mass: 1e10, x: 0, y: 0, z: 0, fz: 1},
		{mass: 1e8, x: -900, y: -900, z: 0, fz: 1},
		{mass: 1e8, x: 900, y: 600, z: 0, fz: -1},
	})

	fmt.Printf("bodies: %d\nstep: %d sec\ntotal steps: %d\nsimulation time: %.1f days\n",
		len(bodies),
		dt,
		steps,
		(time.Duration(dt*steps)*time.Second).Hours()/24)

	start := time.Now()

	for frame := 0; frame < steps; frame++ {
		// enque bodies for image output
		bcopy := make([]body, 0, len(bodies))
		for i := 0; i < len(bodies); i++ {
			if bodies[i] != nil {
				bcopy = append(bcopy, *bodies[i])
			}
		}
		ch <- &frameJob{
			frame:  frame,
			bodies: bcopy,
		}

		// O(n^2) gravity
		for i := 0; i < len(bodies)-1; i++ {
			if bodies[i] == nil {
				continue
			}

			for j := i + 1; j < len(bodies); j++ {
				if bodies[j] == nil {
					continue
				}

				r := dist(bodies[i], bodies[j])
				if r < 4.0 {
					*bodies[i] = combine(bodies[i], bodies[j])
					bodies[j] = nil // "delete" other body
				} else {
					gravity(r, bodies[i], bodies[j])
				}
			}
		}

		// update positions/velocities
		for i := 0; i < len(bodies); i++ {
			if bodies[i] != nil {
				bodies[i].update(dt)
			}
		}

		// progress
		avgTimePerFrame := time.Since(start).Milliseconds() / int64(frame+1)
		estTimeLeft := time.Duration(avgTimePerFrame*int64(steps-frame)) * time.Millisecond
		fmt.Printf("%.1f%%, %d bodies, %dms/step, %s remaining           \r",
			100*float64(frame)/float64(steps),
			len(bcopy),
			avgTimePerFrame,
			estTimeLeft.Truncate(time.Second),
		)
	}
	close(ch)

	wg.Wait()
	fmt.Printf("\nDone. Took %s\n", time.Since(start).Truncate(time.Second))
}

/*

physics section

*/

// initializes n bodies.
func makebodies(n int, cores []body) []*body {
	nc := len(cores)
	bodies := make([]*body, n+nc)

	for i := nc; i < len(bodies); i++ {
		m := math.Abs(rand.NormFloat64()*4 + 1e4)

		core := body{}
		if nc > 0 {
			group := rand.Intn(nc)
			core = cores[group]
		}

		bodies[i] = &body{}
		bodies[i].mass = m
		bodies[i].x = rand.NormFloat64()*500 + core.x
		bodies[i].y = rand.NormFloat64()*500 + core.y
		bodies[i].z = rand.NormFloat64()*100 + core.z

		if nc > 0 {
			// apply initial orbital velocity around center point
			// in a direction perpendicular to the body-center vector and
			// the positive z-axis (out-of-screen).
			dx, dy, dz := core.x-bodies[i].x, core.y-bodies[i].y, core.z-bodies[i].z
			d := math.Sqrt(dx*dx + dy*dy + dz*dz)
			dx /= d
			dy /= d
			dz /= d
			dx, dy, dz = cross(dx, dy, dz, core.fx, core.fy, core.fz)
			core.fx, core.fy, core.fz = 0, 0, 0 // re-zero core.f

			v := math.Sqrt(G * core.mass / d)
			bodies[i].vx = dx * v
			bodies[i].vy = dy * v
			bodies[i].vz = dz * v
		}
	}

	// insert centers as bodies in the front of the slice
	for i := range cores {
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
	mass       float64 // kg
	x, y, z    float64 // m
	vx, vy, vz float64 // m/s
	fx, fy, fz float64 // accumulated force
}

// update body velocity and position, reset accumulated force.
func (b *body) update(dt float64) {
	// a = F/m
	// dv = a*dt
	b.vx += b.fx / b.mass * dt
	b.vy += b.fy / b.mass * dt
	b.vz += b.fz / b.mass * dt

	// dp = v*dt
	b.x += b.vx * dt
	b.y += b.vy * dt
	b.z += b.vz * dt

	// clear force
	b.fx, b.fy, b.fz = 0, 0, 0
}

func (b body) String() string {
	return fmt.Sprintf("m: %.4f\np: [%.2f, %.2f, %.2f]\nv: [%.2f, %.2f, %.2f]\n",
		b.mass, b.x, b.y, b.z, b.vx, b.vy, b.vz)
}

// distance between two bodies.
func dist(a, b *body) float64 {
	dx := b.x - a.x
	dy := b.y - a.y
	dz := b.z - a.z
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

// G = 6.67408 × 10-11 m3 kg-1 s-2
//   = 6.67408e-11 m³/(kg·s²)
const G = 6.67408e-11

// adds gravitational force to a and b.
func gravity(r float64, a, b *body) {
	// r := dist(a, b)
	// if r < 1 {
	// 	r = 1
	// }

	f := G * (a.mass * b.mass) / (r * r) // magnitude of force

	dfx := (b.x - a.x) / r * f
	dfy := (b.y - a.y) / r * f
	dfz := (b.z - a.z) / r * f

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
		mass: a.mass + b.mass,
		x:    a.x,
		y:    a.y,
		z:    a.z,
		vx:   inelasticCollision(a.mass, a.vx, b.mass, b.vx),
		vy:   inelasticCollision(a.mass, a.vy, b.mass, b.vy),
		vz:   inelasticCollision(a.mass, a.vz, b.mass, b.vz),
		fx:   a.fx, // NOTE: may be inaccurate
		fy:   a.fy,
		fz:   a.fz,
	}
}

/*

image output section

*/

type frameJob struct {
	frame  int
	bodies []body
}

type stat struct {
	avg, min, max float64
}

func frameOutput(wg *sync.WaitGroup, ch chan *frameJob) {
	const (
		width  = 1920
		height = 1080
		scale  = 4
	)

	for job := range ch {
		film := image.NewRGBA(image.Rect(0, 0, width, height))
		draw.Draw(film, film.Bounds(), image.NewUniform(color.Black), image.ZP, draw.Src) // fill black

		stats := calculateStats(job.bodies)

		for i := 0; i < len(job.bodies); i++ {
			film.Set(
				int((job.bodies[i].x-stats[0].avg)/scale)+width/2,
				int((job.bodies[i].y-stats[1].avg)/scale)+height/2,
				c(job.bodies[i].mass))

		}
		// film.Set(
		// 	int(stats[0].avg/scale)+width/2,
		// 	int(stats[1].avg/scale)+height/2,
		// 	color.RGBA{0, 255, 0, 255})

		file, err := os.Create(fmt.Sprintf("img/%010d.png", job.frame))
		if err != nil {
			panic(err)
		}
		png.Encode(file, film)
		file.Close()
	}

	wg.Done()
}

func fit(b body, s [3]stat, width float64) (int, int, color.Color) {
	return int(lerp(b.x, s[0].min, s[0].max) * width),
		int(lerp(b.y, s[1].min, s[1].max) * width),
		c(b.mass)
}

func lerp(x, min, max float64) float64 {
	return (x - min) / (max - min)
}

func calculateStats(bodies []body) (stats [3]stat) {
	n := len(bodies)
	stats[0].min = math.Inf(1)
	stats[0].max = math.Inf(-1)
	stats[1].min = math.Inf(1)
	stats[1].max = math.Inf(-1)
	stats[2].min = math.Inf(1)
	stats[2].max = math.Inf(-1)

	summass := 0.0
	for i := 0; i < n; i++ {
		summass += bodies[i].mass
	}

	for i := 0; i < n; i++ {
		stats[0].avg += bodies[i].x * (bodies[i].mass / summass)
		stats[1].avg += bodies[i].y * (bodies[i].mass / summass)
		stats[2].avg += bodies[i].z * (bodies[i].mass / summass)

		stats[0].min = math.Min(stats[0].min, bodies[i].x)
		stats[0].max = math.Max(stats[0].max, bodies[i].x)
		stats[1].min = math.Min(stats[1].min, bodies[i].y)
		stats[1].max = math.Max(stats[1].max, bodies[i].y)
		stats[2].min = math.Min(stats[2].min, bodies[i].z)
		stats[2].max = math.Max(stats[2].max, bodies[i].z)
	}

	// stats[0].avg /= float64(n)
	// stats[1].avg /= float64(n)
	// stats[2].avg /= float64(n)

	return
}

var (
	green  = color.RGBA{0, 255, 0, 255}
	blue   = color.RGBA{0, 0, 255, 255}
	yellow = color.RGBA{255, 255, 0, 255}
	red    = color.RGBA{255, 0, 0, 255}
	purple = color.RGBA{255, 0, 255, 255}
)

func c(m float64) color.Color {
	switch {
	case m > 1e6:
		return green
	case m > 1e5:
		return purple
	case m > 18000:
		return red
	case m > 10005:
		return yellow
	case m < 9995:
		return blue
	default:
		return color.White
	}
}
