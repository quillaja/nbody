// implements a simple n-body simulation.
package main

import (
	"encoding/gob"
	"flag"
	"fmt"
	"os"
	"sync"
	"time"

	"github.com/go-gl/mathgl/mgl64"
)

func main() {
	// rand.Seed(time.Now().UnixNano())
	numbodies := flag.Int("n", 10, "number of bodies")
	years := flag.Float64("y", 1, "number of years to simulate")
	stateSave := flag.Bool("s", false, "set to save the final simulation state")
	stateFilename := flag.String("state", "", "simulation state to load")
	tree := flag.Bool("tree", false, "use tree")
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
		{Mass: 1e10, Radius: 1.0, X: 0, Y: 0, Z: 0, fz: -1},
		// {Mass: 1e10, Radius: 1.0, X: -8000, Y: -500, Z: 0, fz: 1},
		// {Mass: 1e10, Radius: 1.0, X: 8000, Y: 500, Z: 0, fy: -1},
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
	fmt.Printf("tree: %t\nbodies: %d\nstep: %d sec\nframes: %d\nsimulation time: %.1f days\n",
		*tree,
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
			if !*tree {
				// // O(n^2) gravity
				for i := 0; i < len(bodies)-1; i++ {
					if bodies[i] == nil {
						continue
					}

					for j := i + 1; j < len(bodies); j++ {
						if bodies[j] == nil {
							continue
						}

						r := dist(bodies[i], bodies[j])
						if r <= bodies[i].Radius+bodies[j].Radius {
							*bodies[i] = combine(bodies[i], bodies[j])
							bodies[j] = nil // "delete" other body
						} else {
							gravity(r, bodies[i], bodies[j])
							gravity(r, bodies[j], bodies[i])
						}
					}
				}
			} else {
				// tree gravity O(n*log(n))
				root := maketree(bodies, nodebound{center: mgl64.Vec3{}, width: mgl64.Vec3{1e6, 1e6, 1e6}})
				collisions := make([][2]**body, 0, 4)
				for i := 0; i < len(bodies); i++ {
					if bodies[i] == nil {
						continue
					}
					root.gravity(&(bodies[i]), 0.2, &collisions) // arbitrary test theta
				}

				for i := 0; i < len(collisions); i++ {
					if *collisions[i][0] == nil || *collisions[i][1] == nil {
						continue
					}

					**(collisions[i][0]) = combine(*(collisions[i][0]), *(collisions[i][1]))
					*(collisions[i][1]) = nil
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
