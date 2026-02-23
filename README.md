# Simple soft-body simulation in Metal

A real-time soft-body physics playground written in Swift and rendered with Metal,
inspired by Matthias Müller’s TenMinutePhysics tutorial: [Simple and Unbreakable Simulation of Soft Bodies](https://www.youtube.com/watch?v=uCaHXkS2cUg&feature=youtu.be).

https://github.com/user-attachments/assets/39d9bf94-f9a3-4a33-9bac-9215a4f4330f

## Overview

MetalSoftBody is my 2-week exploration of: 
- soft-body deformations
- position based dynamics (PBD) simulations
- writing Metal code + iOS development

This project is not at all "production-ready", it is intended to be small and experimental for learning the basics of Metal and soft-body sims. 

The basic rendering & shading code was repurposed from the sample code in *Metal by Tutorials* by Caroline Begbie & Marius Horga.

## Features
- Real-time PBD soft-body physics simulation
- Interactive grabber (raycasting + vertex manipulation)
- Basic lighting

## Controls
- Drag (over bunny): grab soft-body
- Drag (over empty space): rotate camera
- Pinch: Zoom in/out

## References + Thanks
- Matthias Müller — *Simple and Unbreakable Simulation of Soft Bodies*
- Apple Metal Documentation
- Metal by Tutorials by Caroline Begbie & Marius Horga
