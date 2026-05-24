# Product Requirements Document (PRD): SwarmCore v1.0

## 1. Problem Statement
Traditional Object-Oriented Programming (OOP) in spatial simulations scales poorly. Representing entities as arrays of objects (AoS) and using standard hash maps for spatial grids results in severe CPU cache misses, pointer-chasing, and continuous heap allocations. This limits real-time Python simulations to roughly 5,000-10,000 entities before frame rates drop below 60 FPS.

## 2. Objective
Build a Python-controllable C++ engine capable of simulating 1,000,000+ spatial entities at real-time speeds (>60 FPS) by strictly adhering to Data-Oriented Design (DOD) principles.

## 3. Technical Requirements
*   **Language Stack:** C++20 (Backend) / Python 3.10+ (Frontend)
*   **Build System:** CMake (Cross-platform compatibility)
*   **Bindings:** PyBind11
*   **Precision:** 32-bit `float` for all spatial coordinates and math to maximize L1 cache density and SIMD vectorization.
*   **Memory Architecture:** Strict Structure of Arrays (SoA). Zero `malloc`/`new` calls during the main execution loop.
*   **Spatial Indexing:** Flat array spatial grid grouped via $O(N)$ Radix Sort or highly optimized `std::sort`.

## 4. Key Performance Indicators (KPIs)
*   **Initialization:** Python to C++ data handoff in under 100ms.
*   **Math Throughput:** Update 1,000,000 agent positions in < 2ms per frame.
*   **Grid Rebuild:** Hash and sort 1,000,000 agents in < 10ms per frame.
*   **Target Frame Time:** < 16.6ms (60 FPS minimum) for 1 million entities.

## 5. Out of Scope
*   **Rendering:** The C++ engine will not render graphics. It will output coordinates back to Python, where a lightweight library (e.g., VisPy, PyGame) will handle visualization.
*   **Complex Physics:** Soft-body dynamics or complex concave hitboxes are excluded. Collisions are restricted to fast radius-based proximity checks.