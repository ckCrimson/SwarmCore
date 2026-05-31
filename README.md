# High-Performance Spatial Dynamics Engine (C++/Python)

A data-oriented, highly parallelized spatial simulation engine capable of tracking and processing massive N-body field interactions on consumer hardware.

This project demonstrates a transition from high-level Object-Oriented design constraints to low-level **Data-Oriented Design (DOD)** and pure **Entity-Component-System (ECS)** architecture, optimizing memory layouts and threading models for modern CPU architecture.

## 🚀 Architectural Transformation & Benchmarks

| Milestone | Architecture Strategy | Grid Search | Throughput (100k Agents) |
| --- | --- | --- | --- |
| **Phase 1** | Naive OOP & `std::sort` | $O(N^2)$ Global Loop | Choked (<7 FPS) |
| **Phase 2** | Structure of Arrays (SoA) | Single-Threaded Hash | ~37 FPS |
| **Phase 3** | Parallel ECS & Bitwise Grid | Multi-Core Local Accumulator | **~157 FPS (6.38ms)** |

---

## 🧠 Hardware & Code-Level Optimizations Implemented

### 1. Memory Density: Array of Structures (AoS) ➡️ Structure of Arrays (SoA)

Traditional Object-Oriented layouts group entity attributes inside massive class objects, polluting the CPU L1/L2 cache lines with cold data (e.g., meta flags, IDs) during spatial math updates.

* **Our Solution:** Decoupled spatial coordinates into contiguous `float` arrays (`PointCloudSoA`). This doubled cache line density and allowed the CPU to stream coordinates uninterrupted.
* **Result:** Zero runtime memory allocation ($O(1)$ loop footprint) inside the simulation execution frame.

### 2. Algorithmic Overhaul: Non-Comparison 4-Pass Radix Sort

Replaced the comparison-based `std::sort` ($O(N \log N)$), which suffers from branch mispredictions at scale, with a custom 32-bit linear-time Radix Sort ($O(N)$).

* By utilizing **Prefix-Sum Count Arrays**, the sorting pipeline computes the exact destination indices mathematically in just 4 passes using fast bitwise shifts (`>>` and `& 0xFF`).

### 3. Defeating the Pointer-Chasing Penalty (Cache-Aligned Traversal)

Iterating through particles via their random creation IDs caused massive L1 cache misses during the 9-cell neighborhood search because the memory jumps across the map.

* **Our Solution:** The outer physics loop traverses the `PointCloudSoA` strictly using the Radix-sorted spatial indices.
* **Result:** Flawless hardware prefetching. The CPU cache reads particles exactly as they exist sequentially in physical space, keeping the L1 cache constantly fed.

### 4. Lock-Free Parallel Physics (Dropping Newton's 3rd Law)

Enforcing Newton's 3rd Law (Particle A writes force to Particle B) created severe memory data races across threads, artificially locking the engine to a single core.

* **Our Solution:** Dropped the 3rd Law. Every thread calculates forces strictly for its own particle using internal CPU registers (`local_fx`, `local_fy`).
* **Result:** By doing *double* the math, we completely eliminated RAM synchronization locks. This unleashed 100% utilization across all CPU cores via `std::execution::par_unseq`, resulting in a massive **3.5x speedup**.

### 5. O(1) Spatial Directory & Bitwise Masking

Modulo operators (`%`) for spatial hash bucket lookups cost 40-80 CPU cycles, heavily bottlenecking the grid resolution.

* **Our Solution:** Clamped the Hash Table strictly to powers of 2 (e.g., $2^{18}$ or 262,144 buckets) and replaced the division step with a 1-cycle Bitwise AND mask (`& HASH_MASK`).
* **Result:** Eliminated division overhead entirely while keeping the spatial grid footprint at exactly **1 Megabyte**, ensuring the `std::fill` directory reset fits perfectly inside the CPU's ultra-fast L3 cache.

### 6. The ECS Bridge (`PhysicsContext`)

Transitioned to a pure Data-Oriented ECS architecture. The memory manager (`Engine2D`) generates a zero-copy, lightweight `PhysicsContext` using C++20 `std::span`. This context is handed to the pure-math `PhysicsSystem`, entirely decoupling mathematical logic from memory allocation.

---

## 🛠 Tech Stack

* **Core Engine:** C++20 (Modern `std::span`, parallel execution algorithms)
* **Bindings:** PyBind11 (Zero-copy memory views exposed natively to Python)
* **Frontend/Testing:** Python 3 (NumPy, performance profiling scripts)
* **Build System:** CMake (Optimized MSVC compiler flags: `/O2 /Oi /GL /fp:fast /openmp`)

---

## 🚧 Roadmap / Next Steps

* **Phase 4 (Current):** Implementing the **"Singularity Shield"**—a hard collision resolution phase utilizing elastic vector math to prevent high-density overlapping during extreme attractive field interactions.
* **Phase 5:** Python UI Visualization binding.
