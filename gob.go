package main

import (
	"compress/zlib"
	"encoding/gob"
	"fmt"
	"os"
	"sync"
	"time"
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

using GOD of buckets (chunked compressed gobs):
88 frames * 50,002 bodies: 88mb on disk, took 29 sec (48mb/chunk, best compression)
876 frames * 50,002 bodies: ??874mb on disk, took 2.5 min (48mb/chunk, default compression, 22-23s to write each chunk)
*/

//
// NOTE this bucketing stuff needs serious help!
//

type renderindex map[uint32]map[uint32]renderbody

type renderbody struct {
	// ID           uint32
	X, Y, Z      float32
	Mass, Radius float32
}

type godOfBuckets struct {
	bucketSize          int
	expectedBucketSizes []int
	buckets             []int
	renderstore         renderindex

	dumperWG *sync.WaitGroup
	sem      chan struct{}
	m        *sync.Mutex
}

// totalFrames = last frame number + 1
func newGodOfBuckets(lastFrame, framesPerBucket int) *godOfBuckets {
	god := godOfBuckets{
		bucketSize:          framesPerBucket,
		expectedBucketSizes: make([]int, lastFrame/framesPerBucket+1),
		buckets:             make([]int, lastFrame/framesPerBucket+1),
		renderstore:         make(renderindex, lastFrame+1), // pre-allocated

		dumperWG: &sync.WaitGroup{},
		sem:      make(chan struct{}, 4),
		m:        &sync.Mutex{},
	}
	// do it the dumb way for now
	for frame := 0; frame <= lastFrame; frame++ {
		bnum := frame / god.bucketSize
		god.expectedBucketSizes[bnum]++
	}
	return &god
}

func (god *godOfBuckets) finishedFrame(frame uint32, frameData map[uint32]renderbody) {
	// prevent race on buckets manipulation
	god.m.Lock()
	bnum := int(frame) / god.bucketSize
	god.buckets[bnum]++
	full := god.buckets[bnum] == god.expectedBucketSizes[bnum]
	god.m.Unlock()

	// i think manipulating renderstore will not cause race condition
	// since i preallocate memory (no need for it to reallocate itself)
	// and only one goroutine will access a value (frame) at a time
	god.renderstore[frame] = frameData

	// allow metered file writing and keeping track of
	// remaining "active" dumpers
	if full {
		// dump bucket
		go func() {
			god.sem <- struct{}{}
			god.dumperWG.Add(1)
			god.dumper(bnum)
			god.dumperWG.Done()
			<-god.sem
		}()
	}
}

func (god *godOfBuckets) dumper(bucket int) {
	start := time.Now()
	l, h := bucketToFrames(bucket, god.bucketSize) // indices to dump
	dump := make(renderindex, god.bucketSize)
	for ; l <= h; l++ {
		// transfer subset of frames to "dump"
		// to compressed gob file
		dump[uint32(l)] = god.renderstore[uint32(l)]
		delete(god.renderstore, uint32(l))
	}

	err := os.Mkdir("chunks", 0755)
	// if err != nil {
	// 	panic(err)
	// }
	file, err := os.Create(fmt.Sprintf("chunks/%010d.chunk", h))
	if err != nil {
		panic(err)
	}
	// buf := bytes.NewBuffer(make([]byte, 100*1<<20)) // 100mb buffer to start
	zw, err := zlib.NewWriterLevel(file, zlib.DefaultCompression)
	if err != nil {
		panic(err)
	}
	enc := gob.NewEncoder(zw)
	if err := enc.Encode(dump); err != nil {
		panic(err)
	}
	zw.Close()
	size := 0 //buf.Len()
	// err = ioutil.WriteFile(fmt.Sprintf("chunks/%010d.chunk", h), buf.Bytes(), 0644)
	// if err != nil {
	// 	panic(err)
	// }
	zw.Close()
	// file.Close()
	fmt.Printf("%s to dump bucket %d, %d bytes\n", time.Since(start), bucket, size)
}

// func frameToBucket(frame uint32, bucketSize int) int { return int(frame) / bucketSize }

func bucketToFrames(bucketNumber, bucketSize int) (lowIndex, highIndex int) {
	// inclusive indicies
	return bucketNumber * bucketSize, (bucketNumber+1)*bucketSize - 1
}

func frameToMemory(god *godOfBuckets, wg *sync.WaitGroup, ch chan *frameJob) {
	if god == nil {
		panic("there is no god")
	}

	for job := range ch {
		frame := uint32(job.Frame)
		frameData := make(map[uint32]renderbody, len(job.Bodies))

		for _, b := range job.Bodies {
			frameData[uint32(b.ID)] = renderbody{
				// ID:     uint32(b.ID),
				X:      float32(b.X),
				Y:      float32(b.Y),
				Z:      float32(b.Z),
				Mass:   float32(b.Mass),
				Radius: float32(b.Radius),
			}
		}
		god.finishedFrame(frame, frameData)
	}

	god.dumperWG.Wait() // for for all dumpers before continuing
	wg.Done()
}

// func writeRenderStoreToDisk(filename string) {
// 	file, err := os.Create(filename)
// 	if err != nil {
// 		panic(err)
// 	}
// 	defer file.Close()

// 	enc := gob.NewEncoder(file)
// 	enc.Encode(renderstore)
// }
