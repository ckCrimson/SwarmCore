# High-Performance Spatial Dynamics Engine (C++/Python)

A data-oriented, highly parallelized spatial simulation engine capable of tracking and processing **1,000,000 active agents at ~57 FPS (17.4ms/frame)** on consumer hardware. 

This project demonstrates a transition from high-level Object-Oriented design constraints to low-level **Data-Oriented Design (DOD)**, optimizing memory layouts for modern CPU architecture.

##  Architectural Transformation & Benchmarks

| Milestone | Architecture Strategy | Sorting Algorithm | Throughput (100k Agents) | Throughput (1M Agents) |
| :--- | :--- | :--- | :--- | :--- |
| **Phase 1** | Naive Python / C++ OOP | $O(N \log N)$ `std::sort` | ~80 FPS | Choked (<7 FPS) |
| **Phase 2** | Structure of Arrays (SoA) | 32-bit Linear Radix Sort | ~212 FPS | ~18 FPS |
| **Phase 3** | Parallel DOD (Cache Aligned) | 4-Pass Radix Sort ($O(N)$) | **~287 FPS** | **~57 FPS** |

---

##  Hardware & Code-Level Optimizations Implemented

### 1. Memory Density: Array of Structures (AoS) ➡️ Structure of Arrays (SoA)
Traditional Object-Oriented layouts group entity attributes inside massive class objects, polluting the CPU L1/L2 cache lines with cold data (e.g., meta flags, IDs) during spatial math updates. 
* **Our Solution:** Decoupled spatial coordinates into contiguous `float` arrays (`PointCloudSoA`). This doubled cache line density and allowed the CPU to stream coordinates uninterrupted.
* **Result:** Zero runtime memory allocation ($O(1)$ loop footprint) inside the simulation execution frame.

### 2. Algorithmic Overhaul: Non-Comparison 4-Pass Radix Sort
Replaced the comparison-based `std::sort` ($O(N \log N)$), which suffers from branch mispredictions at scale, with a custom 32-bit linear-time Radix Sort ($O(N)$).
* By utilizing **Prefix-Sum Count Arrays** and reducing the spatial hash from 64-bit to 32-bit, the sorting pipeline computes the exact destination indices mathematically in just 4 passes using fast bitwise shifts (`>>` and `& 0xFF`).
* Features a pointer-swapping **Ping-Pong Buffer** mechanism to eliminate element copying overhead between sorting passes.

### 3. Multi-Core Scaling & False Sharing Mitigation
Leveraged C++20 parallel execution policies (`std::execution::par`) to chunk calculation loops across all available physical CPU cores.
* **Cache Line Alignment:** Initial parallel setups suffered from *False Sharing* due to multiple cores writing to overlapping 64-byte cache boundaries. 
* Resolving this via an independent, pre-allocated `execution_indices` matrix eliminated hardware-level cache invalidation stalls, yielding an immediate performance jump.

---

## Tech Stack
* **Core Engine:** C++20 (Modern `std::span`, parallel execution algorithms)
* **Bindings:** PyBind11 (Zero-copy memory views exposed natively to Python)
* **Frontend/Testing:** Python 3 (NumPy, performance profiling scripts)
* **Build System:** CMake (Optimized MSVC compiler flags: `/O2 /Oi /GL /fp:fast /openmp`)
