package main

import (
	"fmt"
	"image"
	"image/color"
	"image/draw"
	"image/png"
	"math"
	"os"
	"path/filepath"
	"sort"
	"sync"

	"github.com/go-gl/mathgl/mgl64"
	"golang.org/x/image/font"
	"golang.org/x/image/font/inconsolata"
	"golang.org/x/image/math/fixed"
)

/*

image output section

*/

/*
88 frames = 1.8mb disk used (for comparison against using DB or gob) (~35kb per image)
*/

const filenameFormat = "%010d.png"

type frameJob struct {
	Frame  int
	Bodies []body
}

type stat struct {
	avg, min, max float64
}

// light is a point or directional light in the world.
type light struct {
	directional bool       // true if the light is a directional
	intensity   float64    // light intensity in candela
	pos         mgl64.Vec3 // point light world position, or directional light's direction relative to the origin
	color       mgl64.Vec3 // fractional color (RGB in [0,1])
}

// luminance for p's distance from light, as seen from cam.
// Candella/m^2, but assume 1m^s.
func (l light) lo(p, dirToCam mgl64.Vec3) mgl64.Vec3 {
	if l.directional {
		frac := fractilum(dirToCam, l.pos)
		return l.color.Mul(frac) // directional light intensity doesn't decrease with distance
	}
	lp := l.pos.Sub(p)
	frac := fractilum(dirToCam, lp.Normalize())
	c := l.color.Mul(math.Pi * l.intensity * frac / lp.LenSqr())
	return c
}

// portion in [0,1] of p illuminated by a light in dirToLight as seen from
// a camera in dirToCam from p.
// aka "percent full" or "percent illuminated".
// similar to the cos(theta) component to the normal lighting equation.
func fractilum(dirToCam, dirToLight mgl64.Vec3) float64 {
	return (dirToCam.Dot(dirToLight) + 1.0) / 2.0 // transform [-1 -> 0, 1 -> 1]
}

// sums all luminance on p and returns a color.
// could add a reflectance value for p (aka p's "surface color")
// but for now treat all bodies as white .
func sumLo(lights []light, cam, p mgl64.Vec3, c color.RGBA) color.Color {
	var sum mgl64.Vec3
	toCam := cam.Sub(p)
	dirToCam := toCam.Normalize()
	for i := range lights {
		sum = sum.Add(lights[i].lo(p, dirToCam))
	}
	// TODO: gamma correction? pow(R, 1/2.2)
	sum[0] = mgl64.Clamp(sum[0]*float64(c.R), 0.0, 255.0) // prevent uint8 overflow
	sum[1] = mgl64.Clamp(sum[1]*float64(c.G), 0.0, 255.0)
	sum[2] = mgl64.Clamp(sum[2]*float64(c.B), 0.0, 255.0)
	return color.RGBA{
		uint8(sum[0]),
		uint8(sum[1]),
		uint8(sum[2]),
		255,
	}
}

func fovFromRadiusBase(r, b float64) float64 { return 2 * math.Atan(0.5*b/r) }

// frameToImages renders a frame into a PNG image. File name format is:
//     img/FrameNumber.png
func frameToImages(imgDir string, wg *sync.WaitGroup, ch chan *frameJob) {
	const (
		width               = 1920.0 // pixels
		height              = 1080.0 // pixels
		bgcolor             = 0      // [0,255]
		fov                 = 15     // degrees
		camRadiusFromOrigin = 0x1p15 // meters // 60e3
		axisLength          = camRadiusFromOrigin / 10.0
		simWidth            = 0x1p16 // meters
	)

	// this values seems to be ok. "blows out" when object is <1000m from light source
	// (luminous flux) lumens / 4π steradians = candela (luminance/luminous intensity)
	const lightIntensity = 1e6 //1e6
	lights := make([]light, 6)
	lights[0] = light{
		pos:       mgl64.Vec3{axisLength, 0, 0}, // end of x (red) axis
		color:     mgl64.Vec3{1.0, 0.001, 0.001},
		intensity: lightIntensity,
	}
	lights[1] = light{
		pos:       mgl64.Vec3{0, axisLength, 0}, // end of Y (green) axis
		color:     mgl64.Vec3{0.001, 1.0, 0.001},
		intensity: lightIntensity,
	}
	lights[2] = light{
		pos:       mgl64.Vec3{0, 0, axisLength}, // end of Z (blue) axis
		color:     mgl64.Vec3{0.001, 0.001, 1.0},
		intensity: lightIntensity,
	}

	// setup for camera and view-projection matrix
	camRotAxis := mgl64.Vec3{0, 1, 0}
	startCamPos := mgl64.Vec3{1, 1, 5}.
		Normalize().
		Mul(camRadiusFromOrigin)

	makeViewProjMatrix := func(camPos, target mgl64.Vec3, fov float64) mgl64.Mat4 {
		view := mgl64.LookAtV(
			camPos,
			target,
			mgl64.Vec3{0, 1, 0}) // will cause float div-by-zero error if a point is exactly on the camera's position
		proj := mgl64.Perspective(fov, width/height, 0.1, 100)
		// proj := mgl64.Ortho(-4*width, 4*width, -4*height, 4*height, 0.1, 100)
		return proj.Mul4(view)
	}

	// setup for rendering to "film"
	film := image.NewRGBA(image.Rect(0, 0, width, height))
	zbuffer := make([]float64, width*height)

	// func to redraw bg with correctly rotated axes
	drawBG := func(vp mgl64.Mat4) {
		// background, axes, bounds
		greybg := image.NewUniform(color.RGBA{bgcolor, bgcolor, bgcolor, 255})
		zero := mgl64.Vec3{}
		posXAxis := mgl64.Vec3{axisLength, 0, 0}
		posYAxis := mgl64.Vec3{0, axisLength, 0}
		posZAxis := mgl64.Vec3{0, 0, axisLength}
		corners := nodebound{
			center: mgl64.Vec3{},
			width:  mgl64.Vec3{simWidth, simWidth, simWidth}}.corners()
		cornerOrder := [12][2]uint8{
			{0, 1}, {2, 3}, {4, 5}, {6, 7},
			{0, 2}, {1, 3}, {4, 6}, {5, 7},
			{0, 4}, {1, 5}, {2, 6}, {3, 7},
		}

		// create background image with x,y, and z axes
		draw.Draw(film, film.Bounds(), greybg, image.ZP, draw.Src) // clear
		plotline3d(film, zbuffer, red, vp, zero, posXAxis)         // draw
		plotline3d(film, zbuffer, green, vp, zero, posYAxis)
		plotline3d(film, zbuffer, blue, vp, zero, posZAxis)

		// draw 12 lines of the simulation bound
		for i := 0; i < 12; i++ {
			plotline3d(film, zbuffer, gray, vp, corners[cornerOrder[i][0]], corners[cornerOrder[i][1]])
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

		// TODO: remove this. just for getting recovered bodies in expected order!
		// high-to-low mass
		sort.Slice(job.Bodies, func(i, j int) bool {
			return job.Bodies[i].Mass > job.Bodies[j].Mass
		})

		initZBuffer(zbuffer)
		g0 := mgl64.Vec3{job.Bodies[0].X, job.Bodies[0].Y, job.Bodies[0].Z}
		g1 := mgl64.Vec3{job.Bodies[1].X, job.Bodies[1].Y, job.Bodies[1].Z}
		middle := g0.Add(g1).Mul(0.5)

		// move camera and create a view-projection matrix
		angle := mgl64.DegToRad(float64(job.Frame)) / 4.0
		camPos := mgl64.QuatRotate(-angle, camRotAxis).Rotate(startCamPos)
		fov := fovFromRadiusBase(camPos.Sub(middle).Len(), g0.Sub(g1).Len()*1.5)
		vp := makeViewProjMatrix(camPos, middle, fov)

		// light at center of each galaxy
		lights[3] = light{
			pos:       g0,
			color:     mgl64.Vec3{0.9, 0.9, 0.5},
			intensity: 0.075 * lightIntensity,
		}
		lights[4] = light{
			pos:       g1,
			color:     mgl64.Vec3{0.9, 0.9, 0.5},
			intensity: 0.075 * lightIntensity,
		}

		// directional light from camera position can be used to simulate "ambient light source"
		lights[5] = light{
			directional: true,
			// pos:         mgl64.Vec3{0, 0, -1}, // light at infinite distance in -z of simulation
			pos:   camPos.Normalize(),        // from camera
			color: mgl64.Vec3{0.1, 0.1, 0.1}, // very dim 'ambient' illumination
		}

		drawBG(vp)
		plotText(film, col(0), row(1), white, fmt.Sprintf("frame:  %d", job.Frame))
		plotText(film, col(0), row(2), white, fmt.Sprintf("bodies: %d", len(job.Bodies)))
		plotText(film, col(0), row(3), white, fmt.Sprintf("rot:    %.1f°", mgl64.RadToDeg(angle)))
		plotText(film, col(0), row(4), white, fmt.Sprintf("fov:    %.1f°", mgl64.RadToDeg(fov)))

		var world mgl64.Vec3
		for i := 0; i < len(job.Bodies); i++ {
			// world positions of body
			world[0] = job.Bodies[i].X
			world[1] = job.Bodies[i].Y
			world[2] = job.Bodies[i].Z

			// draw
			// colbase := c(job.Bodies[i].Mass)
			col := sumLo(lights, camPos, world, white)
			plotpoint3d(film, zbuffer, col, vp, world)
		}

		filename := filepath.Join(imgDir, fmt.Sprint(filenameFormat, job.Frame))
		file, err := os.Create(filename)
		if err != nil {
			panic(err)
		}
		png.Encode(file, film)
		file.Close()
	}

	wg.Done()
}

// invLerp x to [0,1]
func invLerp(x, min, max float64) float64 {
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
	white     = color.RGBA{255, 255, 255, 255}
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

func c(m float64) color.RGBA {
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
		return white
	}
}

// plotText on img at (x,y). Text is 8x16 inconsolata, anchored at the
// top left of the text.
func plotText(img draw.Image, x, y int, col color.Color, label string) {
	// https://stackoverflow.com/questions/38299930/how-to-add-a-simple-text-label-to-an-image-in-go
	point := fixed.Point26_6{
		X: fixed.Int26_6(x * 64),
		Y: fixed.Int26_6((16 + y) * 64)}

	d := &font.Drawer{
		Dst:  img,
		Src:  image.NewUniform(col),
		Face: inconsolata.Regular8x16,
		Dot:  point,
	}
	d.DrawString(label)
}

func col(n int) int { return n * 8 }
func row(n int) int { return n * 16 }

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
	t := invLerp(0, behind[3], front[3])
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
		zbuffer[i] = math.Inf(1) // fill with maximum float64
	}
}
