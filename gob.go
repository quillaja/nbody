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
*/
var renderstore map[int][]body

func frameToMemory(wg *sync.WaitGroup, ch chan *frameJob) {
	if renderstore == nil {
		renderstore = make(map[int][]body)
	}

	for job := range ch {
		for i := range job.Bodies {
			job.Bodies[i].Vx = 0
			job.Bodies[i].Vy = 0
			job.Bodies[i].Vz = 0
		}
		renderstore[job.Frame] = job.Bodies
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
