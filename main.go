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
	bodies := makebodies(*numbodies)

	fmt.Printf("bodies: %d\nstep: %d sec\ntotal steps: %d\nsimulation time: %.1f days\n",
		len(bodies),
		dt,
		steps,
		(time.Duration(dt*steps)*time.Second).Hours()/24)

	start := time.Now()

	for frame := 0; frame < steps; frame++ {
		// O(n^2) gravity
		for i := 0; i < len(bodies)-1; i++ {
			for j := i + 1; j < len(bodies); j++ {
				gravity(&bodies[i], &bodies[j])
			}
		}

		// update positions/velocities
		for i := 0; i < len(bodies); i++ {
			bodies[i].update(dt)
		}

		// enque bodies for image output
		ch <- &frameJob{
			frame:  frame,
			bodies: bodies,
		}

		// progress
		avgTimePerFrame := time.Since(start).Milliseconds() / int64(frame+1)
		estTimeLeft := time.Duration(avgTimePerFrame*int64(steps-frame)) * time.Millisecond
		fmt.Printf("%.1f%%, %dms/step, %s remaining           \r",
			100*float64(frame)/float64(steps),
			avgTimePerFrame,
			estTimeLeft.Truncate(time.Second),
		)
	}
	close(ch)

	wg.Wait()
	fmt.Println("\nDone")
}

func makebodies(n int) []body {
	bodies := make([]body, n)
	for i := range bodies {
		m := rand.NormFloat64() + 1e4
		if m < 100 {
			m = 1e4
		}

		bodies[i].mass = m
		bodies[i].x = rand.NormFloat64() * 1e3
		bodies[i].y = rand.NormFloat64() * 1e3
		bodies[i].z = rand.NormFloat64() * 1e3
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

func gravity(a, b *body) {
	r := dist(a, b)
	if r < 1 {
		r = 1
	}

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
				color.White)
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
