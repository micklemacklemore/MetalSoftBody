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

## Explanation

### Simulation Model

This app simulates a **low-poly Stanford bunny mesh** that has been **tetrahedralized using Delaunay triangulation**.

Tetrahedralization converts the original surface mesh into a **volumetric mesh made of tetrahedra**, meaning the bunny is not hollow. Instead, the interior volume is filled with tetrahedra.

In the simulation:

- Each **vertex** of the tetrahedral mesh is treated as a **particle**.
- Each **edge** is treated as a **distance constraint**.
- Each **tetrahedron** is treated as a **volume constraint**.

*Constraints* are explained in Matthias's video. These constraints are solved using **Extended Position Based Dynamics (XPBD)**.

### XPBD Solver

The soft-body solver follows the an XPBD pipeline:

```
let dt = the time step, which is the time (in seconds) since the last simulation step
let gravity_force = a 3D vector that represents the force of gravity

function simulate():
    // pre-solve
    for each particle i:
        i.velocity = i.velocity + dt * gravity_force
        i.previousPosition = i.position
        i.position = position + dt * velocity

    // solve
    solve all distance constraints & volume constraints
    update particles with new positions

    // post-solve
    for each particle i:
        i.velocity = (i.position - i.previousPosition) / dt
```
  
You'll find the interesting & relevent code in **SoftBody.swift**, which contains the `simulate()`, `preSolve()`, `solve()` and `postSolve()` methods that updates the soft-body mesh per frame. 


## References + Thanks
- Matthias Müller — *Simple and Unbreakable Simulation of Soft Bodies*
- Apple Metal Documentation
- Metal by Tutorials by Caroline Begbie & Marius Horga
