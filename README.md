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
let [particles] = an array of particles. each particle has a mass, velocity, position and previousPosition

function simulate():
    // pre-solve
    for each particle i:
        i.velocity = i.velocity + dt * gravity_force
        i.previousPosition = i.position
        i.position = position + dt * velocity

    // solve
    solve all distance constraints & volume constraints
    update particles with adjusted positions

    // post-solve
    for each particle i:
        i.velocity = (i.position - i.previousPosition) / dt
```

In the **preSolve()** step, we iterate through all particles. The force of gravity is applied to the current velocity and the initial *unconstrained* position is calculated from the new velocity (using *explicit integration*). 

In the **solve()** step, the initial *unconstrained* particle positions are adjusted into *constrained* positions after all distance constraints and volume constraints are solved. 

While we kind of expect softbodies to move around like jello, we still need constraints to maintain the overall shape of the mesh and prevent it from collapsing into a pancake. This is the most computationally expensive step. 

In the **postSolve()** step, we iterate through all the particles once more. The velocity is once again updated, using the new *constrained* particle positions after solving the constraints. The new velocity is simply the vector from the previous position to the new position (divided by the timestep) 

You'll find the interesting & relevent code in **SoftBody.swift**, which contains the `simulate()`, `preSolve()`, `solve()` and `postSolve()` methods that updates the soft-body mesh per frame. 

### Constraints

Constraints are basically what stops the mesh from collapsing into a flat pancake. We only use two types of constraints in this solver: **distance constraints** and **volume constraints**. 

#### Distance Constraints

A distance constraint is between two particles ($p_1$ and $p_2$). Each particle has a *mass* ($m_1$ and $m_2$) and we declare that there is a *rest distance* ($d$) between these two particles, in other words the distance between these particles when there are no forces involved. 

<img width="400" height="177" alt="image" src="https://github.com/user-attachments/assets/d809c17c-a17d-4991-9faa-f207ddaf66b1" />

#### Volume Conservation Constraints

A volume conservation constraint conserves the *volume* of a 3D shape, if we treat its vertices like particles ($x_1, x_2, x_3, x_4$). Tetrahedras are used in this case because they're the most simple 3D shape. 

Like distance constraints, volume constraints maintains a *rest volume* between the particles. 

<img width="256" height="225" alt="image" src="https://github.com/user-attachments/assets/8fd6d95b-1f68-4b1c-95c2-2ac96d77fddb" />


## References + Thanks
- Matthias Müller — *Simple and Unbreakable Simulation of Soft Bodies*
- Apple Metal Documentation
- Metal by Tutorials by Caroline Begbie & Marius Horga
