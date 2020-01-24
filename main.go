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
	years := flag.Float64("y", 1, "number of years to simulate")
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
	const dt = (60 * 60)                        // 1 hour step per simulation iteration
	const iterPerFrame = 1                      // times to advance the simulation per rendered frame
	frames := int(*years*365*24) / iterPerFrame // total number of rendered frames
	// bodies := solarsystem()
	bodies := makebodies(*numbodies, []body{
		// uses body.f to generate initial velocities of child bodies.
		{mass: 1e10, x: 0, y: 0, z: 0, fz: 1},
		// {mass: 1e10, x: -2000, y: -2000, z: 0, fz: 1},
		// {mass: 1e10, x: 2000, y: 1800, z: 0, fy: -1},
	})

	fmt.Printf("bodies: %d\nstep: %d sec\nframes: %d\nsimulation time: %.1f days\n",
		len(bodies),
		dt,
		frames,
		(time.Duration(dt*frames*iterPerFrame)*time.Second).Hours()/24)

	start := time.Now()

	for frame := 0; frame < frames; frame++ {
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

		for iter := 0; iter < iterPerFrame; iter++ {
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
			// bz := body{}
			for i := 0; i < len(bodies); i++ {
				if bodies[i] == nil {
					continue
				}

				bodies[i].update(dt)
				// if dist(bodies[i], &bz) > 1e4 {
				// 	bodies[i] = nil
				// }
			}
		}

		// progress
		avgTimePerFrame := time.Since(start).Milliseconds() / int64(frame+1)
		estTimeLeft := time.Duration(avgTimePerFrame*int64(frames-frame)) * time.Millisecond
		fmt.Printf("%.1f%%, %d bodies, %dms/frame, %s remaining           \r",
			100*float64(frame)/float64(frames),
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
	const meanMass = 1e3
	nc := len(cores)
	bodies := make([]*body, n+nc)

	for i := nc; i < len(bodies); i++ {
		m := math.Abs(rand.NormFloat64()*4 + meanMass)

		core := body{}
		if nc > 0 {
			group := rand.Intn(nc)
			core = cores[group]
		}

		bodies[i] = &body{}
		bodies[i].mass = m
		bodies[i].x = rand.NormFloat64()*(1000*(1-math.Abs(core.fx))+100*math.Abs(core.fx)) + core.x
		bodies[i].y = rand.NormFloat64()*(1000*(1-math.Abs(core.fy))+100*math.Abs(core.fy)) + core.y
		bodies[i].z = rand.NormFloat64()*(1000*(1-math.Abs(core.fz))+100*math.Abs(core.fz)) + core.z

		if nc > 0 {
			// apply initial orbital velocity around center point
			// in a direction perpendicular to the body-center vector and
			// the axis of rotation in core.f.
			dx, dy, dz := core.x-bodies[i].x, core.y-bodies[i].y, core.z-bodies[i].z
			d := math.Sqrt(dx*dx + dy*dy + dz*dz)
			if d == 0 {
				d = 1
			}
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
		fx:   a.fx, // NOTE: may be inaccurate. maybe add b.f?
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
		scale  = 5 // normal n-body
		// scale  = 5e9 // full solar system
		// scale = 5e8 // inner solar system
	)

	// create background image with x and y axis
	bg := image.NewRGBA(image.Rect(0, 0, width, height))
	draw.Draw(bg, bg.Bounds(), image.NewUniform(color.Black), image.ZP, draw.Src)
	for x := 0; x < width; x++ {
		bg.Set(x, height/2, darkgray)
	}
	for y := 0; y < height; y++ {
		bg.Set(width/2, y, darkgray)
	}

	for job := range ch {
		film := image.NewRGBA(image.Rect(0, 0, width, height))
		draw.Draw(film, film.Bounds(), bg, image.ZP, draw.Src) // fill black

		// stats := calculateStats(job.bodies)

		for i := 0; i < len(job.bodies); i++ {
			film.Set(
				int((job.bodies[i].x /*-stats[0].avg*/)/scale)+width/2,
				int((job.bodies[i].y /*-stats[1].avg*/)/scale)+height/2,
				c(job.bodies[i].mass))
			// film.Set(fit(job.bodies[i], stats, width, height))
		}
		// draw center of mass
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

// kinda goofy and doesn't work well
func fit(b body, s [3]stat, width, height float64) (int, int, color.Color) {
	return int(lerp(b.x, s[0].min, s[0].max) * width),
		int(lerp(b.y, s[1].min, s[1].max) * height),
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
		stats[0].avg += bodies[i].x * bodies[i].mass
		stats[1].avg += bodies[i].y * bodies[i].mass
		stats[2].avg += bodies[i].z * bodies[i].mass

		stats[0].min = math.Min(stats[0].min, bodies[i].x)
		stats[0].max = math.Max(stats[0].max, bodies[i].x)
		stats[1].min = math.Min(stats[1].min, bodies[i].y)
		stats[1].max = math.Max(stats[1].max, bodies[i].y)
		stats[2].min = math.Min(stats[2].min, bodies[i].z)
		stats[2].max = math.Max(stats[2].max, bodies[i].z)
	}

	stats[0].avg /= summass
	stats[1].avg /= summass
	stats[2].avg /= summass

	return
}

var (
	lightgray = color.RGBA{196, 196, 196, 255}
	gray      = color.RGBA{128, 128, 128, 255}
	darkgray  = color.RGBA{64, 64, 64, 255}
	green     = color.RGBA{0, 255, 0, 255}
	blue      = color.RGBA{0, 0, 255, 255}
	yellow    = color.RGBA{255, 255, 0, 255}
	red       = color.RGBA{255, 0, 0, 255}
	purple    = color.RGBA{255, 0, 255, 255}
)

func c(m float64) color.Color {
	switch {
	case m > 5e6:
		return color.White
	case m > 1e6:
		return purple
	case m > 5e5:
		return red
	case m > 2e4:
		return yellow
	case m > 1e4:
		return green
	case m > 5e3:
		return blue
	default:
		return gray
	}
}

// use physically realistic data to simulate sun and planets of solar system.
func solarsystem() []*body {
	return []*body{
		&body{ //sun
			mass: 1.9885e30,
		},

		&body{ //mercury
			mass: 3.3011e23,
			x:    (69816900*1e3 + 46001200*1e3) / 2,
			vy:   47.362 * 1e3,
		},

		&body{ //venus
			mass: 4.8675e24,
			x:    -(108939000*1e3 + 107477000*1e3) / 2,
			vy:   -35.02 * 1e3,
		},

		&body{ //earth
			mass: 5.97237e24,
			x:    (152100000*1e3 + 147095000*1e3) / 2,
			vy:   29.78 * 1e3,
		},

		&body{ //mars
			mass: 6.4171e23,
			x:    -(249200000*1e3 + 206700000*1e3) / 2,
			vy:   -24.007 * 1e3,
		},

		&body{ //jupiter
			mass: 1.8982e27,
			x:    (816.2*1e6*1e3 + 740.52*1e6*1e3) / 2,
			vy:   13.07 * 1e3,
		},

		&body{ //saturn
			mass: 5.6834e26,
			x:    -(1514.5*1e6*1e3 + 1352.55*1e6*1e3) / 2,
			vy:   -9.68 * 1e3,
		},

		&body{ //uranus
			mass: 8.681e25,
			x:    (3.008*1e9*1e3 + 2.742*1e9*1e3) / 2,
			vy:   6.8 * 1e3,
		},

		&body{ //neptune
			mass: 1.02413e26,
			x:    -(4.54e9*1e3 + 4.46e9*1e3) / 2,
			vy:   -5.43 * 1e3,
		},
	}
}
