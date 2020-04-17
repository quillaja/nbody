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
	stateFilename := flag.String("state", "", "simulation state to load")

	stateSave := flag.Bool("save", false, "set to save the final simulation state")
	tree := flag.Bool("tree", false, "use tree")
	norender := flag.Bool("norender", false, "do not render frames")
	nocollision := flag.Bool("nocollision", false, "do not perform collision testing")
	flag.Parse()

	// simulation parameters
	const dt = (60 * 60)                        // 1 hour step per simulation iteration
	const iterPerFrame = 1                      // times to advance the simulation per rendered frame
	frames := int(*years*365*24) / iterPerFrame // total number of rendered frames
	// bodies := solarsystem()
	bodies := makebodies(*numbodies, []body{
		// uses body.f to generate initial velocities of child bodies.
		// {Mass: 1e10, Radius: 1.0, X: 0, Y: 0, Z: 0, fy: 1}, // for this mass, radius must be ~1e6m to be similar to density of the sun, 1410kg/m3
		{Mass: 1e10, Radius: 1.0, X: -9000, Y: -100, Z: -2000, fy: 1, Vx: 0.004, Vz: -0.001},
		{Mass: 1e10, Radius: 1.0, X: 9000, Y: 100, Z: 2000, fz: -1, Vx: -0.003, Vz: 0.002},
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

	// setup image output workers
	ch := make(chan *frameJob, 32)
	workers := 2
	wg := sync.WaitGroup{}
	wg.Add(workers)
	// db := opendb("bodies.sqlite")
	god := newGodOfBuckets(frames, 48)
	for i := 0; i < workers; i++ {
		// go frameToImages(&wg, ch)
		// go frameToSqlite(db, &wg, ch)
		go frameToMemory(god, &wg, ch)
	}

	// print parameters
	fmt.Printf("render: %t\ncollisions: %t\ntree: %t\nbodies: %d\nstep: %d sec\nframes: %d\nsimulation time: %.1f days\n",
		!*norender,
		!*nocollision,
		*tree,
		len(bodies),
		dt,
		frames-startFrame,
		(time.Duration(dt*iterPerFrame*(frames-startFrame))*time.Second).Hours()/24)

	const theta = 1 // apparently a "decent" default theta
	simbound := nodebound{
		center: mgl64.Vec3{},
		width:  mgl64.Vec3{0x1p16, 0x1p16, 0x1p16}} // 1p17=65,000*2, 1p20~1e6 but in powers of 2 for perfect division into octants
	collisions := make([][2]**body, 0, 16)
	start := time.Now()

	for frame := startFrame; frame <= frames; frame++ {
		// enque bodies for image output
		remainingBodies := 0
		if *norender {
			for i := 0; i < len(bodies); i++ {
				if bodies[i] != nil {
					remainingBodies++
				}
			}
		} else {
			bcopy := make([]body, 0, len(bodies))
			for i := 0; i < len(bodies); i++ {
				if bodies[i] != nil {
					bcopy = append(bcopy, *bodies[i])
				}
			}
			remainingBodies = len(bcopy)
			lastFrame = &frameJob{
				Frame:  frame,
				Bodies: bcopy,
			}
			ch <- lastFrame
		}

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

						// 1) apply gravity
						r := dist(bodies[i], bodies[j])
						gravity(r, bodies[i], bodies[j])
						gravity(r, bodies[j], bodies[i])

						// 2) resolve collisions
						if !*nocollision && r <= bodies[i].Radius+bodies[j].Radius {
							combine(bodies[i], bodies[j])
							bodies[j] = nil // "delete" other body
						}
					}
				}
			} else {
				// tree gravity O(n*log(n))
				// 1) figure out the gravitational forces
				// make 4 workers, give each worker 1/4 of the bodies
				root := maketree(bodies, simbound)
				const groups = 4
				gravgroups := sync.WaitGroup{}
				groupsize := len(bodies) / groups
				gravgroups.Add(groups)
				for g := 0; g < groups; g++ {
					go func(bgroup []*body) {
						for i := 0; i < len(bgroup); i++ {
							if bgroup[i] == nil {
								continue
							}
							root.gravity(&(bgroup[i]), theta)
						}
						gravgroups.Done()
					}(bodies[g*groupsize : (g+1)*groupsize])
				}
				gravgroups.Wait()

				// 2) find and process any collisions that take place
				if !*nocollision {
					collisions = collisions[:0] // keep underlying memory, reset length
					for i := 0; i < len(bodies); i++ {
						if bodies[i] == nil {
							continue
						}
						root.checkCollision(&bodies[i], &collisions)
					}
					for i := 0; i < len(collisions); i++ {
						if *collisions[i][0] == nil || *collisions[i][1] == nil {
							continue
						}
						combine(*(collisions[i][0]), *(collisions[i][1]))
						*(collisions[i][1]) = nil
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
		fmt.Printf("%.1f%%, %d bodies, %dms/frame, %s remaining, %s elapsed                    \r",
			100*float64(frame-startFrame)/float64(frames-startFrame),
			remainingBodies,
			avgTimePerFrame,
			estTimeLeft.Truncate(time.Second),
			time.Since(start).Truncate(time.Second),
		)
	}
	close(ch)

	wg.Wait()
	// createIndices(db)
	// db.Close()
	// writeRenderStoreToDisk("allframes.gob")
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
