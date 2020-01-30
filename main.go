package main

import (
	"encoding/gob"
	"flag"
	"fmt"
	"github.com/go-gl/mathgl/mgl64"
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
	stateSave := flag.Bool("s", false, "set to save the final simulation state")
	stateFilename := flag.String("state", "", "simulation state to load")
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
		{Mass: 1e10, X: 0, Y: 0, Z: 0, fz: -1},
		// {Mass: 1e10, X: -8000, Y: -500, Z: 0, fz: 1},
		// {Mass: 1e10, X: 8000, Y: 500, Z: 0, fy: -1},
	})

	// import data if available and update necessary simulation state
	var lastFrame *frameJob = &frameJob{}
	startFrame := 0
	if *stateFilename != "" {
		lastFrame = importFrameData(*stateFilename)
		overwriteBodies(&bodies, lastFrame.Bodies)
		startFrame = lastFrame.Frame
		frames += startFrame
	}

	// print parameters
	fmt.Printf("bodies: %d\nstep: %d sec\nframes: %d\nsimulation time: %.1f days\n",
		len(bodies),
		dt,
		frames-startFrame,
		(time.Duration(dt*iterPerFrame*(frames-startFrame))*time.Second).Hours()/24)

	start := time.Now()

	for frame := startFrame; frame <= frames; frame++ {
		// enque bodies for image output
		bcopy := make([]body, 0, len(bodies))
		for i := 0; i < len(bodies); i++ {
			if bodies[i] != nil {
				bcopy = append(bcopy, *bodies[i])
			}
		}
		lastFrame = &frameJob{
			Frame:  frame,
			Bodies: bcopy,
		}
		ch <- lastFrame

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
					if r <= 10.0 {
						*bodies[i] = combine(bodies[i], bodies[j])
						bodies[j] = nil // "delete" other body
					} else {
						gravity(r, bodies[i], bodies[j])
					}
				}
			}

			// update positions/velocities
			for i := 0; i < len(bodies); i++ {
				if bodies[i] == nil {
					continue
				}

				bodies[i].update(dt)
			}
		}

		// progress
		avgTimePerFrame := time.Since(start).Milliseconds() / int64(frame-startFrame+1)
		estTimeLeft := time.Duration(avgTimePerFrame*int64(frames-frame)) * time.Millisecond
		fmt.Printf("%.1f%%, %d bodies, %dms/frame, %s remaining           \r",
			100*float64(frame-startFrame)/float64(frames-startFrame),
			len(bcopy),
			avgTimePerFrame,
			estTimeLeft.Truncate(time.Second),
		)
	}
	close(ch)

	wg.Wait()
	fmt.Printf("\nDone. Took %s\n", time.Since(start).Truncate(time.Second))

	// export final state of simulation
	if *stateSave {
		exportFrameData(lastFrame)
	}
}

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

/*

image output section

*/

type frameJob struct {
	Frame  int
	Bodies []body
}

type stat struct {
	avg, min, max float64
}

func frameOutput(wg *sync.WaitGroup, ch chan *frameJob) {
	const (
		width  = 1920.0
		height = 1080.0
		scale  = 5 // normal n-body
		// scale  = 5e9 // full solar system
		// scale = 5e8 // inner solar system
	)

	view := mgl64.LookAt(500, 500, 1e3, -1000, 0, 0, 0, 1, 0) // will cause float div-by-zero error if a point is exactly on the camera's position
	proj := mgl64.Perspective(mgl64.DegToRad(90), width/height, 0.1, 100)
	// proj := mgl64.Ortho(-width, width, -height, height, 0.1, 100)
	vpmat := proj.Mul4(view)

	greybg := image.NewUniform(color.Black)
	bg := image.NewRGBA(image.Rect(0, 0, width, height))
	// func to redraw bg with correctly rotated axes
	rotateBG := func(rvp mgl64.Mat4) {
		zero := mgl64.TransformCoordinate(mgl64.Vec3{}, rvp)
		posXAxis := mgl64.TransformCoordinate(mgl64.Vec3{1e2, 0, 0}, rvp)
		posYAxis := mgl64.TransformCoordinate(mgl64.Vec3{0, 1e2, 0}, rvp)
		posZAxis := mgl64.TransformCoordinate(mgl64.Vec3{0, 0, 1e2}, rvp)
		zerox, zeroy := mgl64.GLToScreenCoords(zero.X(), zero.Y(), width, height)
		xx, xy := mgl64.GLToScreenCoords(posXAxis.X(), posXAxis.Y(), width, height)
		yx, yy := mgl64.GLToScreenCoords(posYAxis.X(), posYAxis.Y(), width, height)
		zx, zy := mgl64.GLToScreenCoords(posZAxis.X(), posZAxis.Y(), width, height)

		// create background image with x,y, and z axes
		draw.Draw(bg, bg.Bounds(), greybg, image.ZP, draw.Src) // clear
		plotline(bg, red, zerox, zeroy, xx, xy)                // draw
		plotline(bg, green, zerox, zeroy, yx, yy)
		plotline(bg, blue, zerox, zeroy, zx, zy)
	}

	for job := range ch {

		// calculate the body's radius as percieved by the (perspective) camera.
		// (eg large looks small at far distance, small looks large at near distance.)
		// do this by transforming the body's center and a point on the "surface"
		// to view/camera space, then finding the distance between those points
		// radius := func(b body) float64 {
		// 	// 1600 kg/m^3 for rock
		// 	r := math.Pow((3*b.Mass)/(4*math.Pi*1600), 1.0/3.0)
		// 	center := mgl64.Vec3{b.X/scale, b.Y/scale, b.Z/scale}
		// 	edge := mgl64.Vec3{center.X + r/scale, center.Y, center.Z}
		// 	center = mgl64.TransformCoordinate(center, vpmat)
		// 	edge = mgl64.TransformCoordinate(edge, vpmat)
		// 	return center.Sub(edge).Len()
		// }

		rot := mgl64.HomogRotate3DY(mgl64.DegToRad(float64(job.Frame)) / 4)
		rvp := vpmat.Mul4(rot) // final rotated view-projection matrix for this frame

		film := image.NewRGBA(image.Rect(0, 0, width, height))
		rotateBG(rvp)
		draw.Draw(film, film.Bounds(), bg, image.ZP, draw.Src) // fill film with background image

		var world, camera, tail mgl64.Vec3
		for i := 0; i < len(job.Bodies); i++ {
			// world positions of body and a "tail"
			world[0] = job.Bodies[i].X / scale
			world[1] = job.Bodies[i].Y / scale
			world[2] = job.Bodies[i].Z / scale
			tail[0] = (job.Bodies[i].X - job.Bodies[i].Vx*(60*30)) / scale
			tail[1] = (job.Bodies[i].Y - job.Bodies[i].Vy*(60*30)) / scale
			tail[2] = (job.Bodies[i].Z - job.Bodies[i].Vz*(60*30)) / scale
			// transform points into camera space
			camera = mgl64.TransformCoordinate(world, rvp)
			tail = mgl64.TransformCoordinate(tail, rvp)
			if camera.Z() < 0 {
				continue // don't draw stuff behind the camera. camera looks down +Z axis in camera space(?)
			}
			// get film coords
			x, y := mgl64.GLToScreenCoords(camera.X(), camera.Y(), width, height)
			// xt, yt := mgl64.GLToScreenCoords(tail.X(), tail.Y(), width, height)
			col := c(job.Bodies[i].Mass)
			// draw
			film.Set(x, y, col)
			// plotline(film, col, x, y, xt, yt)
		}

		file, err := os.Create(fmt.Sprintf("img/%010d.png", job.Frame))
		if err != nil {
			panic(err)
		}
		png.Encode(file, film)
		file.Close()
	}

	wg.Done()
}

// lerp x to [0,1]
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
		summass += bodies[i].Mass
	}

	for i := 0; i < n; i++ {
		stats[0].avg += bodies[i].X * bodies[i].Mass
		stats[1].avg += bodies[i].Y * bodies[i].Mass
		stats[2].avg += bodies[i].Z * bodies[i].Mass

		stats[0].min = math.Min(stats[0].min, bodies[i].X)
		stats[0].max = math.Max(stats[0].max, bodies[i].X)
		stats[1].min = math.Min(stats[1].min, bodies[i].Y)
		stats[1].max = math.Max(stats[1].max, bodies[i].Y)
		stats[2].min = math.Min(stats[2].min, bodies[i].Z)
		stats[2].max = math.Max(stats[2].max, bodies[i].Z)
	}

	stats[0].avg /= summass
	stats[1].avg /= summass
	stats[2].avg /= summass

	return
}

var (
	lightgray = color.RGBA{192, 192, 192, 255}
	gray      = color.RGBA{128, 128, 128, 255}
	darkgray  = color.RGBA{64, 64, 64, 255}
	vdarkgray = color.RGBA{32, 32, 32, 255}
	red       = color.RGBA{255, 0, 0, 255}
	green     = color.RGBA{0, 255, 0, 255}
	blue      = color.RGBA{0, 0, 255, 255}
	yellow    = color.RGBA{255, 255, 0, 255}
	purple    = color.RGBA{255, 0, 255, 255}
	cyan      = color.RGBA{0, 255, 255, 255}
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

// plotline draws a simple line on img from (x0,y0) to (x1,y1).
//
// This is basically a copy of a version of Bresenham's line algorithm
// from https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm.
func plotline(img draw.Image, c color.Color, x0, y0, x1, y1 int) {
	dx := abs(x1 - x0)
	sx := -1
	if x0 < x1 {
		sx = 1
	}
	dy := -abs(y1 - y0)
	sy := -1
	if y0 < y1 {
		sy = 1
	}
	err := dx + dy
	for {
		img.Set(x0, y0, c)
		if x0 == x1 && y0 == y1 {
			break
		}
		e2 := 2 * err
		if e2 >= dy {
			err += dy
			x0 += sx
		}
		if e2 <= dx {
			err += dx
			y0 += sy
		}
	}
}

// abs cuz no integer abs function in the Go standard library.
func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

// plotcircle draws a filled circle at (x0,y0) of radius r.
//
// This seems to perform just slightly faster than other versions I've tried.
func plotcircle(img draw.Image, c color.Color, x0, y0, r int) {
	rsqr := float64(r * r)
	for y := r; y >= 0; y-- {
		xright := int(math.Sqrt(rsqr - float64(y*y)))
		for x := -xright; x <= xright; x++ {
			img.Set(x0+x, y0+y, c)
			img.Set(x0+x, y0-y, c)
		}
	}
}

/*

helpers to save and restore simulation state

*/

func exportFrameData(frame *frameJob) {
	fname := fmt.Sprintf("%010d.data", frame.Frame)
	file, err := os.Create(fname)
	if err != nil {
		panic(err)
	}
	defer file.Close()

	enc := gob.NewEncoder(file)
	err = enc.Encode(*frame)
	if err != nil {
		os.Remove(fname)
		panic(err)
	}
}

func importFrameData(filename string) *frameJob {
	file, err := os.Open(filename)
	if err != nil {
		panic(err)
	}
	defer file.Close()

	var frame frameJob
	dec := gob.NewDecoder(file)
	err = dec.Decode(&frame)
	if err != nil {
		panic(err)
	}

	return &frame
}

func overwriteBodies(old *[]*body, new []body) {
	*old = make([]*body, 0, len(new))
	for i := 0; i < len(new); i++ {
		*old = append(*old, &new[i])
	}
}

// use physically realistic data to simulate sun and planets of solar system.
func solarsystem() []*body {
	return []*body{
		&body{ //sun
			Mass: 1.9885e30,
		},

		&body{ //mercury
			Mass: 3.3011e23,
			X:    (69816900*1e3 + 46001200*1e3) / 2,
			Vy:   47.362 * 1e3,
		},

		&body{ //venus
			Mass: 4.8675e24,
			X:    -(108939000*1e3 + 107477000*1e3) / 2,
			Vy:   -35.02 * 1e3,
		},

		&body{ //earth
			Mass: 5.97237e24,
			X:    (152100000*1e3 + 147095000*1e3) / 2,
			Vy:   29.78 * 1e3,
		},

		&body{ //mars
			Mass: 6.4171e23,
			X:    -(249200000*1e3 + 206700000*1e3) / 2,
			Vy:   -24.007 * 1e3,
		},

		&body{ //jupiter
			Mass: 1.8982e27,
			X:    (816.2*1e6*1e3 + 740.52*1e6*1e3) / 2,
			Vy:   13.07 * 1e3,
		},

		&body{ //saturn
			Mass: 5.6834e26,
			X:    -(1514.5*1e6*1e3 + 1352.55*1e6*1e3) / 2,
			Vy:   -9.68 * 1e3,
		},

		&body{ //uranus
			Mass: 8.681e25,
			X:    (3.008*1e9*1e3 + 2.742*1e9*1e3) / 2,
			Vy:   6.8 * 1e3,
		},

		&body{ //neptune
			Mass: 1.02413e26,
			X:    -(4.54e9*1e3 + 4.46e9*1e3) / 2,
			Vy:   -5.43 * 1e3,
		},
	}
}
