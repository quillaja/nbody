package main

import (
	"encoding/gob"
	"os"
	"sync"
)

/*
322mb to write renderstore of 88 frames * 50,002 bodies.
Total runtime was 8s (simulation + disk io).
calculated 316mb just for the 88*50,002 bodies (72bytes each)

197mb for same params as above, but zeroing velocity fields before writing.
(gob doesn't write 0-value fields to gob encoding, so 48bytes each.)

when no bodies are discarded in the simulation, final data usage is just about
exactly 48bytes per body. so very good speed and compactness.

1 workers seems fine, 2 is ok too, but more doesn't seem to help.

using 24 byte "render body", 88 frames * 5,002 bodies:
map-map: 15mb? calculated 8-10mb
map-slice: 15mb
slice-slice: 15mb

88 frames * 50,002 bodies, calculated 105mb:
slice-slice: 145mb (storing body id only in struct)
map-map: 157mb ("double" storing body id in map index and struct)
map-map: 141mb (storing body id only in map index)

1752 frames * 100,002 bodies caused the process to be killed just as it
reached 100% done on the simulation due to using all the system memory (15.6gb). =)
*/

type renderindex map[uint32]map[uint32]renderbody

var renderstore renderindex

type renderbody struct {
	ID           uint32
	X, Y, Z      float32
	Mass, Radius float32
}

func frameToMemory(wg *sync.WaitGroup, ch chan *frameJob) {
	if renderstore == nil {
		renderstore = make(renderindex)
	}

	for job := range ch {
		frame := uint32(job.Frame)
		rbs := make(map[uint32]renderbody)

		for _, b := range job.Bodies {
			rbs[uint32(b.ID)] = renderbody{
				// ID:     uint32(b.ID),
				X:      float32(b.X),
				Y:      float32(b.Y),
				Z:      float32(b.Z),
				Mass:   float32(b.Mass),
				Radius: float32(b.Radius),
			}
		}
		renderstore[frame] = rbs
	}
	wg.Done()
}

func writeRenderStoreToDisk(filename string) {
	file, err := os.Create(filename)
	if err != nil {
		panic(err)
	}
	defer file.Close()

	enc := gob.NewEncoder(file)
	enc.Encode(renderstore)
}
