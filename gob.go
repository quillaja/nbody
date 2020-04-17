package main

import (
	"bytes"
	"compress/zlib"
	"encoding/gob"
	"fmt"
	"io/ioutil"
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

using GOD of buckets (chunked compressed gobs):
~33 bytes/body uncompressed, 77mb per chunk
88 frames * 50,002 bodies: 88mb on disk, took 29 sec (48mb/chunk, best compression)
876 frames * 50,002 bodies: 874mb on disk, ~21byte/body took 2.5 min encode/compress/direct-to-file (48mb/chunk, default compression (6), 22-23s to write each chunk)
876 frames * 50,002 bodies: 874mb on disk, ~21byte/body took 2.1 min encode/compress/buffer/file (48mb/chunk, default compression, 17-18s to write each chunk)
same settings, lvl 2 compressiong, 21 bytes/body, 5-6s to for each 49mb chunk. (much better!)
*/

type renderindex map[uint32]map[uint32]renderbody

type renderbody struct {
	// ID           uint32
	X, Y, Z      float32
	Mass, Radius float32
}

type godOfBuckets struct {
	compressionLvl      int
	bucketSize          int
	expectedBucketSizes []int
	buckets             []int
	renderstore         renderindex

	dumperWG *sync.WaitGroup
	sem      chan *bytes.Buffer
	m        *sync.Mutex
}

func newGodOfBuckets(lastFrame, framesPerBucket, compressionLvl int) *godOfBuckets {
	god := godOfBuckets{
		compressionLvl:      compressionLvl,
		bucketSize:          framesPerBucket,
		expectedBucketSizes: make([]int, lastFrame/framesPerBucket+1),
		buckets:             make([]int, lastFrame/framesPerBucket+1),
		renderstore:         make(renderindex, lastFrame+1), // pre-allocated

		dumperWG: &sync.WaitGroup{},
		sem:      make(chan *bytes.Buffer, 4),
		m:        &sync.Mutex{},
	}

	// determine the number of frames expected in each bucket.
	// do it the dumb way for now
	for frame := 0; frame <= lastFrame; frame++ {
		bnum := frame / god.bucketSize
		god.expectedBucketSizes[bnum]++
	}

	// fill the semaphore with buffers for dumpers.
	// buffers serve double purpose as semaphore and scratch space.
	for i := 0; i < cap(god.sem); i++ {
		god.sem <- bytes.NewBuffer(make([]byte, 0, 50*1<<20)) // 50mb buffer CAPACITY to start
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
		go god.dumper(bnum)
	}
}

func (god *godOfBuckets) dumper(bucket int) {
	god.dumperWG.Add(1)
	buf := <-god.sem

	// start := time.Now()

	l, h := bucketToFrames(bucket, god.bucketSize) // indices to dump
	dump := make(renderindex, god.bucketSize)
	for i := l; i <= h; i++ {
		// transfer subset of frames to "dump"
		// to compressed gob file
		dump[uint32(i)] = god.renderstore[uint32(i)]
		delete(god.renderstore, uint32(i))
	}

	err := os.Mkdir("chunks", 0755)

	// write compressed data to buffer first,
	// then to disk (is faster than directly to disk)
	zw, err := zlib.NewWriterLevel(buf, god.compressionLvl)
	if err != nil {
		panic(err)
	}
	enc := gob.NewEncoder(zw)
	if err := enc.Encode(dump); err != nil {
		panic(err)
	}
	zw.Close()
	err = ioutil.WriteFile(fmt.Sprintf("chunks/%010d.chunk", h), buf.Bytes(), 0644)
	if err != nil {
		panic(err)
	}

	// debug status print
	// size := buf.Len()
	// bods := len(dump[uint32(l)])
	// fmt.Printf("%s to dump bucket %d, %d bodies/frame, %d bytes, approx %d bytes per body\n", time.Since(start), bucket, bods, size, size/(bods*god.bucketSize))

	buf.Reset() // reset len, keep cap
	god.sem <- buf
	god.dumperWG.Done()
}

func bucketToFrames(bucketNumber, bucketSize int) (lowIndex, highIndex int) {
	// inclusive indicies
	return bucketNumber * bucketSize, (bucketNumber+1)*bucketSize - 1
}

func idealBucketSize(numBodies int) int {
	const (
		sizePerBody    = 24           // bytes
		idealChunkSize = 32 * 1 << 20 // 32MB
		minBucketSize  = 2
	)
	framesPerChunk := idealChunkSize / (sizePerBody * numBodies)
	extra := minBucketSize - framesPerChunk%minBucketSize
	return framesPerChunk + extra
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

func readChunk(filename string) (renderindex, error) {
	file, err := os.Open(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()
	zr, err := zlib.NewReader(file)
	if err != nil {
		return nil, err
	}
	defer zr.Close()

	frameData := make(renderindex)
	dec := gob.NewDecoder(zr)
	err = dec.Decode(&frameData)
	if err != nil {
		return nil, err
	}
	return frameData, nil
}
