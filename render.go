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

	campos := mgl64.Vec3{500, 500, 1e3}.Mul(3)
	view := mgl64.LookAtV(
		campos,
		mgl64.Vec3{0, 0, 0},
		mgl64.Vec3{0, 1, 0}) // will cause float div-by-zero error if a point is exactly on the camera's position
	proj := mgl64.Perspective(mgl64.DegToRad(90), width/height, 0.1, 100)
	// proj := mgl64.Ortho(-width, width, -height, height, 0.1, 100)
	vpmat := proj.Mul4(view)

	greybg := image.NewUniform(color.Black)
	bg := image.NewRGBA(image.Rect(0, 0, width, height))
	// func to redraw bg with correctly rotated axes
	rotateBG := func(rvp mgl64.Mat4) {
		zero := mgl64.TransformCoordinate(mgl64.Vec3{}, rvp)
		posXAxis := mgl64.TransformCoordinate(mgl64.Vec3{1e3 / scale, 0, 0}, rvp)
		posYAxis := mgl64.TransformCoordinate(mgl64.Vec3{0, 1e3 / scale, 0}, rvp)
		posZAxis := mgl64.TransformCoordinate(mgl64.Vec3{0, 0, 1e3 / scale}, rvp)
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
