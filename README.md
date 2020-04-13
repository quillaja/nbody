## About
A simple (maybe?) n-body simulator for my own experimentation and enjoyment.

## Todo
- [ ] improve renderer
    - [x] implement z-buffer for correct drawing of 3d line and points
    - [ ] actually use written data to render
        - [x] !! write minimal frame data to sqlite database for later rendering (slow)
        - [x] write data in gob format for later rendering (fast but uses a lot of RAM)
    - [ ] improved camera control (ex camera pos & look follow bezier spline)
    - [½] axes grow/shrink in proportion to camera distance (so axes look about the same size from any distance from origin)
    - [ ] creation of color palettes for mass
    - [ ] color/mass "key" in corner of frame. other textual info?
    - [x] draw bounding cubes. at least simulation bound, perhaps oct-tree node bounds.
- [ ] better simulation state save/load
- [ ] better control of simulation params (json file?)
    - [ ] more realistic body masses, positions, etc
- [x] multi-thread gravity calculations using tree
- [ ] use of body ID + map instead of pointers + list
- [ ] convert body/physics to use mgl.Vec3 types
- [ ] examine memory use, potential GC gains from using sync.Pool
- [ ] general organization and commenting