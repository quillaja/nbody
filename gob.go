package main

import (
	"bytes"
	"compress/zlib"
	"encoding/gob"
	"fmt"
	"io/ioutil"
	"os"
	"path/filepath"
	"strconv"
	"strings"
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

const (
	chunkFilenameFormat = "%010d.chunk"
)

type framedata map[uint32]renderbody
type renderindex map[uint32]framedata

type renderbody struct {
	// ID           uint32
	X, Y, Z      float32
	Mass, Radius float32
}

func (fd framedata) bodies() []body {
	n := len(fd)
	bs := make([]body, 0, n)
	for id, b := range fd {
		bs = append(bs, body{
			ID:     uint64(id),
			Mass:   float64(b.Mass),
			Radius: float64(b.Radius),
			X:      float64(b.X),
			Y:      float64(b.Y),
			Z:      float64(b.Z),
		})
	}
	return bs
}

type chunkRenderer struct {
	outputDir          string
	compressionLvl     int
	chunkSize          int
	expectedChunkSizes []int
	currentChunkSizes  []int
	chunks             []renderindex

	dumperWG *sync.WaitGroup
	sem      chan *bytes.Buffer
	m        *sync.Mutex
}

func newChunkRenderer(lastFrame, framesPerChunk, compressionLvl int, outputDir string) *chunkRenderer {
	god := chunkRenderer{
		outputDir:          outputDir,
		compressionLvl:     compressionLvl,
		chunkSize:          framesPerChunk,
		expectedChunkSizes: make([]int, lastFrame/framesPerChunk+1),
		currentChunkSizes:  make([]int, lastFrame/framesPerChunk+1),
		chunks:             make([]renderindex, lastFrame/framesPerChunk+1),

		dumperWG: &sync.WaitGroup{},
		sem:      make(chan *bytes.Buffer, 4),
		m:        &sync.Mutex{},
	}

	// determine the number of frames expected in each bucket.
	// do it the dumb way for now
	for frame := 0; frame <= lastFrame; frame++ {
		bnum := frame / god.chunkSize
		god.expectedChunkSizes[bnum]++
	}

	// fill the semaphore with buffers for dumpers.
	// buffers serve double purpose as semaphore and scratch space.
	for i := 0; i < cap(god.sem); i++ {
		god.sem <- bytes.NewBuffer(make([]byte, 0, 50*1<<20)) // 50mb buffer CAPACITY to start
	}

	return &god
}

func (god *chunkRenderer) finishedFrame(frame uint32, frameData framedata) {
	// prevent race on buckets manipulation
	god.m.Lock()
	chunkNum := int(frame) / god.chunkSize
	god.currentChunkSizes[chunkNum]++
	full := god.currentChunkSizes[chunkNum] == god.expectedChunkSizes[chunkNum]

	if god.chunks[chunkNum] == nil {
		god.chunks[chunkNum] = make(renderindex, god.chunkSize)
	}
	god.chunks[chunkNum][frame] = frameData
	god.m.Unlock()

	// write full chunk to disk
	if full {
		go god.dumper(chunkNum)
	}
}

func (god *chunkRenderer) dumper(chunkNum int) {
	god.dumperWG.Add(1)
	buf := <-god.sem
	_, h := framesForChunk(chunkNum, god.chunkSize) // indices to dump

	os.Mkdir(god.outputDir, 0755) // ignore errors (such as the directory existing)

	// write compressed data to buffer first,
	// then to disk (is faster than directly to disk)
	zw, err := zlib.NewWriterLevel(buf, god.compressionLvl)
	if err != nil {
		panic(err)
	}
	enc := gob.NewEncoder(zw)
	if err := enc.Encode(god.chunks[chunkNum]); err != nil {
		panic(err)
	}
	zw.Close()
	god.chunks[chunkNum] = nil // (hopefully) deallocated memory for this chunk

	filename := filepath.Join(god.outputDir, fmt.Sprintf(chunkFilenameFormat, h))
	err = ioutil.WriteFile(filename, buf.Bytes(), 0644)
	if err != nil {
		panic(err)
	}

	buf.Reset() // reset len, keep cap
	god.sem <- buf
	god.dumperWG.Done()
}

func framesForChunk(chunkNumber, chunkSize int) (lowIndex, highIndex int) {
	// inclusive indicies
	return chunkNumber * chunkSize, (chunkNumber+1)*chunkSize - 1
}

func idealChunkSize(numBodies int) int {
	const (
		sizePerBody    = 24           // bytes
		idealChunkSize = 32 * 1 << 20 // 32MB
		minBucketSize  = 2
	)
	framesPerChunk := idealChunkSize / (sizePerBody * numBodies)
	extra := minBucketSize - framesPerChunk%minBucketSize
	return framesPerChunk + extra
}

func frameToMemory(god *chunkRenderer, wg *sync.WaitGroup, ch chan *frameJob) {
	if god == nil {
		panic("there is no god")
	}

	for job := range ch {
		frame := uint32(job.Frame)
		frameData := make(framedata, len(job.Bodies))

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

	god.dumperWG.Wait() // for all dumpers before continuing
	wg.Done()
}

// return low and high frame number included in the chunkData.
func readChunkFromFrame(dir string, frameNumber int) (chunkData renderindex, low, high int, err error) {
	size, err := determineChunkSizeFromFolder(dir)
	if err != nil {
		return
	}
	low, high = framesForChunk(frameNumber/size, size)
	filename := filepath.Join(dir, fmt.Sprintf(chunkFilenameFormat, high))
	chunkData, err = readChunkFromFilename(filename)
	return
}

func readChunkFromFilename(filename string) (renderindex, error) {
	// filename with path
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

// determineChunkSizeFromFolder scans dir for files matching "*.chunk"
// and attempts to use the sequential numbering of the filenames to determine
// the chunk size used for their generation.
func determineChunkSizeFromFolder(dir string) (chunkSize int, err error) {
	matches, err := filepath.Glob(filepath.Join(dir, "*.chunk"))
	if err != nil {
		return
	}
	switch n := len(matches); {
	case n < 1:
		return 0, fmt.Errorf("found %d chunk files in '%s'", n, dir)

	case n < 2:
		frames, err := filenamesToInts(matches[0])
		if err != nil {
			return 0, err
		}
		return frames[0] + 1, nil

	default:
		frames, err := filenamesToInts(matches[0], matches[1])
		if err != nil {
			return 0, err
		}
		return frames[1] - frames[0], nil
	}
}

// cleans and converts to integers filenames whose name are integers.
func filenamesToInts(files ...string) ([]int, error) {
	frames := make([]int, 0, len(files))
	for _, file := range files {
		base := filepath.Base(file)
		base = strings.TrimSuffix(base, filepath.Ext(base))
		n, err := strconv.Atoi(base)
		if err != nil {
			return nil, err
		}
		frames = append(frames, n)
	}
	return frames, nil
}
