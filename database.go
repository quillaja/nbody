package main

import (
	"database/sql"
	"fmt"
	"math"
	"os"
	"sync"

	_ "github.com/mattn/go-sqlite3"
)

/*
note: all below rounds float for sqlite real->integer space optimization.

24.7mb calculated for (88 frames * 5,002 bodies)rows @ (7 fields * 8 bytes) per row
34 mb on disk (with indices)
37.7% larger than "expected" (i think due to indices)

246.5mb calculated for (88 frames * 50,002 bodies)rows @ (7 fields * 8 bytes) per row
270mb on disk when types were "REAL" (with indices)
116mb on disk when types were "INTEGER" (no indices created)
116mb on disk when types were "REAL" (no indicies created)

really only 1 worker is useful for sqlite since it allows only 1 writer at a time.
*/

const schema = `
CREATE TABLE bodies (
	frame 	INTEGER,
	id 		INTEGER, -- body id
	x 		REAL,
	y 		REAL,
	z 		REAL,
	mass 	REAL,
	radius 	REAL);
`

const indices = `	
CREATE INDEX idx_frame ON bodies (frame, id);
CREATE INDEX idx_id ON bodies (id);
CREATE INDEX idx_mass ON bodies (mass);
`

const insert = `INSERT INTO bodies VALUES (?, ?, ?, ?, ?, ?, ?);`
const queryFrame = `SELECT * FROM bodies WHERE frame = ? ORDER BY id ASC;`

// opens and initializes db in filename. returns db handle for use.
func opendb(filename string) *sql.DB {
	info, err := os.Stat(filename)
	if info != nil {
		fmt.Printf("%s exists\n", filename)
		os.Exit(1)
	}
	//_journal_mode=OFF&_synchronous=OFF
	//mode=memory&cache=shared
	db, err := sql.Open("sqlite3", "file:"+filename+"?_journal_mode=OFF&_synchronous=OFF")
	if err != nil {
		panic(err)
	}
	createTables(db)
	return db
}

// runs create table statements on db.
func createTables(db *sql.DB) {
	_, err := db.Exec(schema)
	if err != nil {
		panic(err)
	}
}

// runs create index statements on db.
func createIndices(db *sql.DB) {
	_, err := db.Exec(indices)
	if err != nil {
		panic(err)
	}
}

// creates, initializes, and closes a db in filename.
func makedb(filename string) {
	db := opendb(filename)
	createTables(db)
	db.Close()
}

// outputs a frame to an sqlite database instead of an image.
func frameToSqlite(db *sql.DB, wg *sync.WaitGroup, ch chan *frameJob) {
	stmt, err := db.Prepare(insert)
	if err != nil {
		panic(err)
	}

	for job := range ch {
		tx, err := db.Begin()
		if err != nil {
			panic(err)
		}

		for _, b := range job.Bodies {
			_, err = stmt.Exec(
				job.Frame,
				b.ID,
				math.Round(b.X),
				math.Round(b.Y),
				math.Round(b.Z),
				math.Round(b.Mass),
				math.Round(b.Radius))
			if err != nil {
				break
			}
		}

		if err != nil {
			tx.Rollback()
			panic(err)
		} else {
			tx.Commit()
		}
	}
	wg.Done()
}
