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
	bodies := makebodies(*numbodies, [][3]float64{
		{-800, -800, 0},
		{800, 500, 0},
	})

	fmt.Printf("bodies: %d\nstep: %d sec\ntotal steps: %d\nsimulation time: %.1f days\n",
		len(bodies),
		dt,
		steps,
		(time.Duration(dt*steps)*time.Second).Hours()/24)

	start := time.Now()

	for frame := 0; frame < steps; frame++ {
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

func makebodies(n int, centers [][3]float64) []*body {
	bodies := make([]*body, n)
	for i := range bodies {
		m := rand.NormFloat64()*4 + 1e4
		if m < 100 {
			m = 1e4
		}

		center := centers[rand.Intn(len(centers))]

		bodies[i] = &body{}
		bodies[i].mass = m
		bodies[i].x = rand.NormFloat64()*500 + center[0]
		bodies[i].y = rand.NormFloat64()*500 + center[1]
		bodies[i].z = rand.NormFloat64()*500 + center[2]
		// bodies[i].vx = rand.NormFloat64() * 0.25
		// bodies[i].vy = rand.NormFloat64() * 0.25
		// bodies[i].vz = rand.NormFloat64() * 0.25
	}

	return bodies
}

type body struct {
	mass       float64 // kg
	x, y, z    float64 // m
	vx, vy, vz float64 // m/s
	fx, fy, fz float64 // accumulated force
}

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

func dist(a, b *body) float64 {
	dx := b.x - a.x
	dy := b.y - a.y
	dz := b.z - a.z
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

// G = 6.67408 × 10-11 m3 kg-1 s-2
//   = 6.67408e-11 m³/(kg·s²)
const G = 6.67408e-11

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

func inelasticCollision(ma, va, mb, vb float64) (vc float64) {
	return (ma*va + mb*vb) / (ma + mb)
}

func combine(a, b *body) body {
	return body{
		mass: a.mass + b.mass,
		x:    a.x,
		y:    a.y,
		z:    a.z,
		vx:   inelasticCollision(a.mass, a.vx, b.mass, b.vx),
		vy:   inelasticCollision(a.mass, a.vy, b.mass, b.vy),
		vz:   inelasticCollision(a.mass, a.vz, b.mass, b.vz),
		fx:   a.fx,
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

func frameOutput(wg *sync.WaitGroup, ch chan *frameJob) {

	for job := range ch {
		film := image.NewRGBA(image.Rect(0, 0, 1920, 1080))
		draw.Draw(film, film.Bounds(), image.NewUniform(color.Black), image.ZP, draw.Src) // fill black

		for i := 0; i < len(job.bodies); i++ {
			film.Set(
				int(job.bodies[i].x/4)+(film.Bounds().Dx()/2),
				int(job.bodies[i].y/4)+(film.Bounds().Dy()/2),
				c(job.bodies[i].mass))
		}

		file, err := os.Create(fmt.Sprintf("img/%010d.png", job.frame))
		if err != nil {
			panic(err)
		}
		png.Encode(file, film)
		file.Close()
	}

	wg.Done()
}

var (
	blue   = color.RGBA{128, 128, 255, 255}
	yellow = color.RGBA{255, 255, 128, 255}
	red    = color.RGBA{255, 128, 128, 255}
)

func c(m float64) color.Color {
	switch {
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
