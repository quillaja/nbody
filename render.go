package main

import (
	"fmt"
	"image"
	"image/color"
	"image/draw"
	"image/png"
	"math"
	"os"
	"sort"
	"sync"

	"github.com/go-gl/mathgl/mgl64"
)

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
		width               = 1920.0
		height              = 1080.0
		sqrt3over3          = 0.57735027 // sqrt(3)/3, normalized {1,1,1}
		camRadiusFromOrigin = 0x1p15     //60e3
		axisLength          = camRadiusFromOrigin / 10.0
		// scale  = 5e9 // full solar system
		// scale = 5e8 // inner solar system
	)

	campos := mgl64.Vec3{1, 1, 5}.
		Normalize().
		Mul(camRadiusFromOrigin)
	view := mgl64.LookAtV(
		campos,
		mgl64.Vec3{0, 0, 0},
		mgl64.Vec3{0, 1, 0}) // will cause float div-by-zero error if a point is exactly on the camera's position
	proj := mgl64.Perspective(mgl64.DegToRad(60), width/height, 0.1, 100)
	// proj := mgl64.Ortho(-width, width, -height, height, 0.1, 100)
	vp := proj.Mul4(view)

	greybg := image.NewUniform(color.Black)
	bg := image.NewRGBA(image.Rect(0, 0, width, height))
	zero := mgl64.Vec3{}
	posXAxis := mgl64.Vec3{axisLength, 0, 0}
	posYAxis := mgl64.Vec3{0, axisLength, 0}
	posZAxis := mgl64.Vec3{0, 0, axisLength}
	corners := nodebound{
		center: mgl64.Vec3{},
		width:  mgl64.Vec3{0x1p16, 0x1p16, 0x1p16}}.corners()
	cornerOrder := [12][2]uint8{
		{0, 1}, {2, 3}, {4, 5}, {6, 7},
		{0, 2}, {1, 3}, {4, 6}, {5, 7},
		{0, 4}, {1, 5}, {2, 6}, {3, 7},
	}
	// func to redraw bg with correctly rotated axes
	rotateBG := func(rvp mgl64.Mat4) {

		// create background image with x,y, and z axes
		draw.Draw(bg, bg.Bounds(), greybg, image.ZP, draw.Src) // clear
		plotline3d(bg, red, rvp, zero, posXAxis)               // draw
		plotline3d(bg, green, rvp, zero, posYAxis)
		plotline3d(bg, blue, rvp, zero, posZAxis)

		// draw 12 lines of the simulation bound
		for i := 0; i < 12; i++ {
			plotline3d(bg, gray, rvp, corners[cornerOrder[i][0]], corners[cornerOrder[i][1]])
		}
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

		// sort by low-to-high mass, so "important" bodies are drawn last/on top
		sort.Slice(job.Bodies, func(i, j int) bool {
			return job.Bodies[i].Mass < job.Bodies[j].Mass
		})

		rot := mgl64.HomogRotate3DY(mgl64.DegToRad(float64(job.Frame)) / 4)
		rvp := vp.Mul4(rot) // final rotated view-projection matrix for this frame

		film := image.NewRGBA(image.Rect(0, 0, width, height))
		rotateBG(rvp)
		draw.Draw(film, film.Bounds(), bg, image.ZP, draw.Src) // fill film with background image

		var world, tail mgl64.Vec3
		for i := 0; i < len(job.Bodies); i++ {
			// world positions of body and a "tail"
			world[0] = job.Bodies[i].X
			world[1] = job.Bodies[i].Y
			world[2] = job.Bodies[i].Z
			if job.Bodies[i].Mass >= 1e9 {
				tail[0] = job.Bodies[i].X - job.Bodies[i].Vx*(60*60*4)
				tail[1] = job.Bodies[i].Y - job.Bodies[i].Vy*(60*60*4)
				tail[2] = job.Bodies[i].Z - job.Bodies[i].Vz*(60*60*4)
			}
			// draw
			col := c(job.Bodies[i].Mass)
			if job.Bodies[i].Mass >= 1e9 {
				plotline3d(film, col, rvp, world, tail)
			} else {
				plotpoint3d(film, col, rvp, world)
			}
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
	const step = 1e10 / 7
	switch {
	case m > 6*step:
		return red
	case m > 5*step:
		return purple
	case m > 4*step:
		return yellow
	case m > 3*step:
		return green
	case m > 2*step:
		return blue
	case m > 1*step:
		return cyan
	default:
		return color.White
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

// plotcirclefilled draws a filled circle at (x0,y0) of radius r.
//
// This seems to perform just slightly faster than other versions I've tried.
func plotcirclefilled(img draw.Image, c color.Color, x0, y0, r int) {
	rsqr := float64(r * r)
	for y := r; y >= 0; y-- {
		xright := int(math.Sqrt(rsqr - float64(y*y)))
		for x := -xright; x <= xright; x++ {
			img.Set(x0+x, y0+y, c)
			img.Set(x0+x, y0-y, c)
		}
	}
}

// plotcircle draws an unfilled circle at (x0,y0) of radius r.
func plotcircle(img draw.Image, c color.Color, x0, y0, r int) {
	x := r
	for y := 0; y <= x; y++ {
		img.Set(x0+x, y0+y, c)
		img.Set(x0+x, y0-y, c)
		img.Set(x0-x, y0+y, c)
		img.Set(x0-x, y0-y, c)

		img.Set(x0+y, y0+x, c)
		img.Set(x0+y, y0-x, c)
		img.Set(x0-y, y0+x, c)
		img.Set(x0-y, y0-x, c)
		d := 2*(x*x+y*y-r*r+2*y+1) + 1 - 2*x
		if d > 0 {
			x--
		}
	}
}

// plotpoint3d draws a point at p
func plotpoint3d(img draw.Image, c color.Color, vp mgl64.Mat4, p mgl64.Vec3) {
	// p = mgl64.TransformCoordinate(p, vp)
	t := p.Vec4(1)
	t = vp.Mul4x1(t)
	if t[3] < 0 {
		return
	}
	t = t.Mul(1 / t[3]) // t in NDC space

	x, y := mgl64.GLToScreenCoords(t.X(), t.Y(), img.Bounds().Dx(), img.Bounds().Dy())
	img.Set(x, y, c)
}

// plotline3d draws a line from p1 to p2
func plotline3d(img draw.Image, c color.Color, vp mgl64.Mat4, p1, p2 mgl64.Vec3) {
	// p1 = mgl64.TransformCoordinate(p1, vp)
	// p2 = mgl64.TransformCoordinate(p2, vp)
	t1 := p1.Vec4(1)
	t1 = vp.Mul4x1(t1)
	t2 := p2.Vec4(1)
	t2 = vp.Mul4x1(t2)

	// fix lines going behind the camera by clipping them to the 3D
	// point on the W=0.1 plane
	fix2 := false
	switch {
	case t1[3] <= 0 && t2[3] <= 0:
		return

	case t1[3] < 0: // t1 low, t2 high
		lerpwto0(&t1, &t2)
		t2, t1 = t1, t2 // swap so only x2,y2 need be checked later
		fix2 = true

	case t2[3] < 0: // t2 low, t1 high
		lerpwto0(&t2, &t1)
		fix2 = true
	}

	t1 = t1.Mul(1 / t1[3]) // t in NDC space
	t2 = t2.Mul(1 / t2[3])

	x1, y1 := mgl64.GLToScreenCoords(t1.X(), t1.Y(), img.Bounds().Dx(), img.Bounds().Dy())
	x2, y2 := mgl64.GLToScreenCoords(t2.X(), t2.Y(), img.Bounds().Dx(), img.Bounds().Dy())

	// correct x2,y2 by clipping to the image bounds
	if fix2 {
		dx := float64(x1 - x2)
		dy := float64(y1 - y2)
		var tx, ty float64
		switch {
		case dx == 0:
			tx = -1
		case dx < 0: // x2 beyond image right
			tx = t(float64(img.Bounds().Dx()), float64(x2), float64(x1))
		case dx > 0: // x2 beyond image left
			tx = t(0, float64(x2), float64(x1))
		}
		switch {
		case dy == 0:
			ty = -1
		case dy < 0: // y2 beyond image bottom
			ty = t(float64(img.Bounds().Dy()), float64(y2), float64(y1))
		case dy > 0: // y2 beyond image top
			ty = t(0, float64(y2), float64(y1))
		}
		t := math.Max(tx, ty)
		x2 += int(t * dx)
		y2 += int(t * dy)
	}
	plotline(img, c, x1, y1, x2, y2)
}

func lerpwto0(low, high *mgl64.Vec4) {
	t := (0.1 - low[3]) / (high[3] - low[3])
	low[0] += t * (high[0] - low[0])
	low[1] += t * (high[1] - low[1])
	low[2] += t * (high[2] - low[2])
	low[3] = 0.1 //t * (high[3] - low[3])
}

func t(x, low, high float64) float64 {
	return (x - low) / (high - low)
}
