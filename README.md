# SwarmCore
**A High-Performance Data-Oriented Spatial Engine for Python**

> ⚠️ **Status: Active Development (Sprint 1)**
> This project is currently undergoing a 7-day development sprint to transition from a single-threaded prototype to a fully multithreaded, SIMD-accelerated C++ backend. 
> *Current Milestone: Infrastructure & Core DOD Data Structures initialized.*

## Overview
SwarmCore is a lightweight, blazing-fast C++ physics and spatial query engine controlled entirely via Python. 

By aggressively exploiting **Data-Oriented Design (DOD)**, contiguous memory (Structure of Arrays), and cache locality, this engine bypasses the traditional bottlenecks of Object-Oriented C++. It is designed to compute proximity, collisions, and movement for up to 1,000,000 entities in real-time (60+ FPS) on consumer hardware.

## Target Use Cases
*   **Gaming & VFX:** Simulating massive particle systems, flocking behaviors (boids), or horde dynamics.
*   **Epidemiology:** Tracking disease vector spread in densely populated urban models.
*   **Robotics:** Real-time collision avoidance for massive fleets of autonomous drones.

## Core Architecture
SwarmCore separates logic from data. Python acts as the "Command and Control" frontend, while C++ acts as the high-performance backend.
1. **Frontend:** Python scripts define the simulation parameters and agent initial states.
2. **The Bridge:** `PyBind11` seamlessly passes Python data into the native C++ engine.
3. **Memory:** C++ shreds high-level objects into flat, cache-friendly `std::vector<float>` arrays (SoA).
4. **Spatial Hashing:** Agents are sorted into a 1D grid using an $O(N)$ Radix Sort, completely eliminating the need for slow `std::unordered_map` allocations.
5. **Execution:** Stateless C++ utility classes execute math over contiguous `std::span` memory views.

## Quick Start (Coming Soon)
*Build instructions using CMake will be provided upon the completion of Sprint 1.*