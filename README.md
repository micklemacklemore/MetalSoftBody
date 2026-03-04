# Simple soft-body simulation in Metal

A real-time soft-body physics playground written in Swift and rendered with Metal,
inspired by Matthias Müller’s TenMinutePhysics tutorial: [Simple and Unbreakable Simulation of Soft Bodies](https://www.youtube.com/watch?v=uCaHXkS2cUg&feature=youtu.be).

https://github.com/user-attachments/assets/39d9bf94-f9a3-4a33-9bac-9215a4f4330f

## Overview

MetalSoftBody is my 2-week exploration of: 
- soft-body deformations
- extended position based dynamics (XPBD) simulations
- writing Metal code + iOS development

This project is not at all "production-ready", it is intended to be small and experimental for learning the basics of Metal and soft-body sims. 

The basic rendering & shading code was repurposed from the sample code in *Metal by Tutorials* by Caroline Begbie & Marius Horga.

## Features
- Real-time XPBD soft-body physics simulation
- Interactive grabber (raycasting + vertex manipulation)
- Basic lighting

## Controls
- Drag (over bunny): grab soft-body
- Drag (over empty space): rotate camera
- Pinch: Zoom in/out

## Code & PBD Explanation

If you're interested in how the XPBD soft-body solver works, you'll find the interesting & relevent code in **SoftBody.swift**, which contains the `simulate()` function that updates the soft-body mesh per frame. 

```swift
// -- SoftBody.swift --

// only slightly modified for brevity
func simulate(dt: Double) {
    let gravity = SIMD3<Float>(0, -9.81, 0)

    preSolve(dt: sdt, gravity: [gravity.x, gravity.y, gravity.z])
    solve(dt: sdt)
    postSolve(dt: sdt)
}
```

You can see that `simulate()` is very simple. The actual work is done in the steps `preSolve()`, `solve()` and `postSolve()`

Here's a basic outline of those steps, which is also the general algorithm of (extended) position based dynamics:  

<img width="462" height="419" alt="image" src="https://github.com/user-attachments/assets/8cb31896-3a59-494f-967b-a359a3007ca6" />

where: 
- $v_i$: the current *velocity* of particle *i*
- $\triangle t$: the timestep, essentially the time in seconds between each frame. This app runs in 60fps so it is usually ~0.01667 seconds
- $g$: the force of gravity, represented as a 3D vector `[0, -9.81, 0]`
- $p_i$: the *previous position* of particle *i*
- $x_i$: the *current position* of particle *i*
- $C$: the constraints present in the simulation. This simulation uses only two: ***distance constraints*** and ***volume constraints***.

## References + Thanks
- Matthias Müller — *Simple and Unbreakable Simulation of Soft Bodies*
- Apple Metal Documentation
- Metal by Tutorials by Caroline Begbie & Marius Horga
