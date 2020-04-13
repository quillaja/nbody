package main

import (
	"fmt"
	"image"
	"image/color"
	"image/draw"
	"image/png"
	"math"
	"os"
	"sync"

	"github.com/go-gl/mathgl/mgl64"
)

/*

image output section

*/

/*
88 frames = 1.8mb disk used (for comparison against using DB or gob)
*/

type frameJob struct {
	Frame  int
	Bodies []body
}

type stat struct {
	avg, min, max float64
}

func frameToImages(wg *sync.WaitGroup, ch chan *frameJob) {
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
	proj := mgl64.Perspective(mgl64.DegToRad(90), width/height, 0.1, 100)
	// proj := mgl64.Ortho(-width, width, -height, height, 0.1, 100)
	vp := proj.Mul4(view)

	zbuffer := make([]float64, width*height)
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
		plotline3d(bg, zbuffer, red, rvp, zero, posXAxis)      // draw
		plotline3d(bg, zbuffer, green, rvp, zero, posYAxis)
		plotline3d(bg, zbuffer, blue, rvp, zero, posZAxis)

		// draw 12 lines of the simulation bound
		for i := 0; i < 12; i++ {
			plotline3d(bg, zbuffer, gray, rvp, corners[cornerOrder[i][0]], corners[cornerOrder[i][1]])
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

		initZBuffer(zbuffer)

		// sort by low-to-high mass, so "important" bodies are drawn last/on top
		// sort.Slice(job.Bodies, func(i, j int) bool {
		// 	return job.Bodies[i].Mass < job.Bodies[j].Mass
		// })

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
				plotline3d(film, zbuffer, col, rvp, world, tail)
			} else {
				plotpoint3d(film, zbuffer, col, rvp, world)
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
func plotpoint3d(img draw.Image, zbuffer []float64, c color.Color, vp mgl64.Mat4, p mgl64.Vec3) {
	// p = mgl64.TransformCoordinate(p, vp)
	t := p.Vec4(1)
	t = vp.Mul4x1(t)
	if t[3] <= 0 {
		return
	}
	t = t.Mul(1 / t[3]) // t in NDC space

	x, y := mgl64.GLToScreenCoords(t.X(), t.Y(), img.Bounds().Dx(), img.Bounds().Dy())
	if i := index(x, y, img.Bounds().Dx()); zbuffer != nil &&
		0 <= i && i < len(zbuffer) &&
		t.Z() <= zbuffer[i] {

		img.Set(x, y, c)
		zbuffer[i] = t.Z() // set new closest z for this pizel
	}
}

// plotline3d draws a line from p1 to p2
func plotline3d(img draw.Image, zbuffer []float64, c color.Color, vp mgl64.Mat4, p1, p2 mgl64.Vec3) {
	// p1 = mgl64.TransformCoordinate(p1, vp)
	// p2 = mgl64.TransformCoordinate(p2, vp)
	t1 := p1.Vec4(1)
	t1 = vp.Mul4x1(t1)
	t2 := p2.Vec4(1)
	t2 = vp.Mul4x1(t2)

	// fix lines going behind the camera by clipping them to the 3D
	// point on the W=0 plane
	switch {
	case t1[3] <= 0 && t2[3] <= 0: // both behind W=0. don't do anything
		return

	case t1[3] <= 0: // t1 behind W=0, t2 in front
		clipWto0(&t1, &t2)

	case t2[3] <= 0: // t2 behind W=0, t1 in front
		clipWto0(&t2, &t1)
	}

	p1 = t1.Mul(1 / t1[3]).Vec3() // convert t to NDC space
	p2 = t2.Mul(1 / t2[3]).Vec3()

	if clip(&p1, &p2) { // use NDC to clip lines to the image bounds
		return
	}

	x1, y1 := mgl64.GLToScreenCoords(p1.X(), p1.Y(), img.Bounds().Dx(), img.Bounds().Dy())
	x2, y2 := mgl64.GLToScreenCoords(p2.X(), p2.Y(), img.Bounds().Dx(), img.Bounds().Dy())
	plotlinezbuff(img, c, x1, y1, x2, y2, p1.Z(), p2.Z(), zbuffer)
}

func plotlinezbuff(img draw.Image, c color.Color, x0, y0, x1, y1 int, z0, z1 float64, zbuffer []float64) {
	x, y, z := x0, y0, z0
	fullLength := distInt(x0, y0, x1, y1)

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
		if i := index(x0, y0, img.Bounds().Dx()); zbuffer != nil &&
			0 <= i && i < len(zbuffer) &&
			z0 <= zbuffer[i] {

			img.Set(x0, y0, c)
			zbuffer[i] = z0 // set new closest z
		}

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

		curLength := distInt(x, y, x0, y0)
		t := curLength / fullLength
		z0 = z + (z1-z)*t
	}
}

func distInt(x0, y0, x1, y1 int) float64 {
	dx := float64(x1 - x0)
	dy := float64(y1 - y0)
	return math.Sqrt(dx*dx + dy*dy)
}

func clipWto0(behind, front *mgl64.Vec4) {
	// t := (0.0 - behind[3]) / (front[3] - behind[3])
	t := lerp(0, behind[3], front[3])
	behind[0] += t * (front[0] - behind[0])
	behind[1] += t * (front[1] - behind[1])
	behind[2] += t * (front[2] - behind[2])
	behind[3] = 1
}

// x & y clipping planes in NDC space
var planes = [4][2]mgl64.Vec3{
	// {{normal}, {point}}
	{{1, 0, 0}, {-1, 0, 0}}, // low x
	{{-1, 0, 0}, {1, 0, 0}}, // high x
	{{0, 1, 0}, {0, -1, 0}}, // low y
	{{0, -1, 0}, {0, 1, 0}}, // high y
}

// will clip p1 and/or p2 to the ±X and ±Y clipping NDC clipping planes.
// returns true if both p1 and p2 are completely outside of the NDC space
// (and therefore do no have to be drawn at all).
func clip(p1, p2 *mgl64.Vec3) (totallyClippedOutOfExistence bool) {
	for i := range planes {
		n := planes[i][0]
		p := planes[i][1]
		p1behind := p1.Sub(p).Dot(n) < 0
		p2behind := p2.Sub(p).Dot(n) < 0
		switch {
		case p1behind && p2behind:
			return true
		case p1behind:
			dline := p2.Sub(*p1)
			dplane := p.Sub(*p1)
			dlinex := dline.Dot(n)   // length of x component of dline
			dplanex := dplane.Dot(n) // length of x component of dplane
			*p1 = p1.Add(dline.Mul(dplanex / dlinex))
		case p2behind:
			dline := p1.Sub(*p2)
			dplane := p.Sub(*p2)
			dlinex := dline.Dot(n)   // length of x component of dline
			dplanex := dplane.Dot(n) // length of x component of dplane
			*p2 = p2.Add(dline.Mul(dplanex / dlinex))
		}
	}
	return false
}

func index(x, y, w int) int {
	return x + y*w
}

func initZBuffer(zbuffer []float64) {
	for i := range zbuffer {
		zbuffer[i] = math.Inf(1) // fill with minimum float64
	}
}
