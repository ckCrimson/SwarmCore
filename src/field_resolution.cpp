#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <execution>
#include <span>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// =========================================================================
// 1. DATA STRUCTURES & HASHING (2D)
// =========================================================================

struct Agent2D {
    int id;
    float x, y;
    float vx, vy;
    float fx, fy;
    float mass;

    Agent2D(int _id, float _x, float _y, float _vx, float _vy, float _mass)
        : id(_id), x(_x), y(_y), vx(_vx), vy(_vy), fx(0), fy(0), mass(_mass) {}
};

struct PointCloud2D {
    std::vector<float> x, y;
    std::vector<float> vx, vy;
    std::vector<float> fx, fy;
    std::vector<float> mass;
};

struct GridCell2D {
    int x, y;
    bool operator==(const GridCell2D&) const = default;
};

struct AgentLocation2D {
    uint32_t cell_hash;
    size_t agent_index;
};

struct GridHasher2D {
    uint32_t operator()(const GridCell2D& grid) const {
        const uint32_t P1 = 73856093;
        const uint32_t P2 = 19349663;
        return (static_cast<uint32_t>(grid.x) * P1) ^ 
               (static_cast<uint32_t>(grid.y) * P2);
    }
};

inline GridCell2D get_grid_cell_2d(float x, float y, float cell_size) {
    return {
        static_cast<int>(std::floor(x / cell_size)),
        static_cast<int>(std::floor(y / cell_size))
    };
}

// =========================================================================
// 2. THE ECS BRIDGE: PHYSICS CONTEXT
// =========================================================================

struct PhysicsContext {
    std::span<float> x, y;
    std::span<float> vx, vy;
    std::span<float> fx, fy;
    std::span<float> mass;
    std::span<size_t> execution_indices;

    std::span<AgentLocation2D> spatial_grid_flat;
    std::span<uint32_t> cell_start;
    std::span<uint32_t> cell_count;

    size_t active_count;
    float cell_size;
    size_t hash_table_size;
};

// =========================================================================
// 3. THE SYSTEM: PURE MATH & LOGIC
// =========================================================================

struct PhysicsKernels2D {
    static inline float linear_repulsion(float distance, float kernel_radius, float base_strength) {
        if (distance >= kernel_radius || distance == 0.0f) return 0.0f;
        return (1.0f - (distance / kernel_radius)) * base_strength;
    }
};

class PhysicsSystem {
public:
    static void resolve_fields(PhysicsContext& ctx, float kernel_radius, float base_strength) {
        float k_sq = kernel_radius * kernel_radius;
        const uint32_t HASH_MASK = static_cast<uint32_t>(ctx.hash_table_size - 1);

        // MULTI-THREADED & CACHE-ALIGNED
        std::for_each(std::execution::par_unseq, ctx.execution_indices.begin(), ctx.execution_indices.end(),
            [&](size_t sorted_idx) {
                GridHasher2D hasher;
                
                // Fetch physical ID based on sorted memory to maintain L1 Cache prefetching
                size_t i = ctx.spatial_grid_flat[sorted_idx].agent_index;
                
                float ax = ctx.x[i];
                float ay = ctx.y[i];
                
                // Local Accumulators (Prevent Thread Data Races)
                float local_fx = 0.0f;
                float local_fy = 0.0f;
                
                GridCell2D g = get_grid_cell_2d(ax, ay, ctx.cell_size);
                
                for (int nx = g.x - 1; nx <= g.x + 1; ++nx) {
                    for (int ny = g.y - 1; ny <= g.y + 1; ++ny) {
                        
                        // Bitwise Mask (1 CPU Cycle)
                        uint32_t bucket = hasher({nx, ny}) & HASH_MASK;
                        
                        uint32_t start = ctx.cell_start[bucket];
                        uint32_t count = ctx.cell_count[bucket];
                        
                        for (uint32_t idx = 0; idx < count; ++idx) {
                            size_t j = ctx.spatial_grid_flat[start + idx].agent_index;
                            
                            // Self-check only. Newton's 3rd Law is disabled for thread safety.
                            if (i == j) continue; 
                            
                            float dx = ctx.x[j] - ax;
                            float dy = ctx.y[j] - ay;
                            float d_sq = (dx*dx) + (dy*dy);
                            
                            if (d_sq >= k_sq || d_sq == 0.0f) continue;
                            
                            float d = std::sqrt(d_sq);
                            float force = PhysicsKernels2D::linear_repulsion(d, kernel_radius, base_strength);
                            
                            float force_on_A = force / ctx.mass[i];
                            local_fx -= force_on_A * (dx / d);
                            local_fy -= force_on_A * (dy / d);
                        }
                    }
                }
                
                // Write out to main RAM exactly once per particle
                ctx.fx[i] = local_fx;
                ctx.fy[i] = local_fy;
            });
    }

    static void integrate(PhysicsContext& ctx, float dt) {
        std::for_each(std::execution::par_unseq, ctx.execution_indices.begin(), ctx.execution_indices.end(),
            [&](size_t i) {
                ctx.vx[i] += ctx.fx[i] * dt;
                ctx.vy[i] += ctx.fy[i] * dt;
                
                ctx.x[i] += ctx.vx[i] * dt;
                ctx.y[i] += ctx.vy[i] * dt;
            });
    }
};

// =========================================================================
// 4. FAST O(N) RADIX SORT
// =========================================================================

inline void radix_sort_2d(AgentLocation2D* src, AgentLocation2D* dst, size_t count) {
    constexpr int PASSES = sizeof(uint32_t);
    constexpr size_t RADIX = 256;
    
    AgentLocation2D* current_src = src;
    AgentLocation2D* current_dst = dst;
    
    for (int pass = 0; pass < PASSES; ++pass) {
        size_t counts[RADIX] = {0};
        int shift = pass * 8;
        
        for (size_t i = 0; i < count; ++i) {
            size_t bucket = (current_src[i].cell_hash >> shift) & 0xFF;
            counts[bucket]++;
        }
        
        size_t offsets[RADIX] = {0};
        size_t total = 0;
        for (size_t i = 0; i < RADIX; ++i) {
            offsets[i] = total;
            total += counts[i];
        }
        
        for (size_t i = 0; i < count; ++i) {
            size_t bucket = (current_src[i].cell_hash >> shift) & 0xFF;
            current_dst[offsets[bucket]++] = current_src[i];
        }
        
        std::swap(current_src, current_dst);
    }
    
    if constexpr (PASSES % 2 != 0) {
        for (size_t i = 0; i < count; ++i) {
            src[i] = dst[i];
        }
    }
}

// =========================================================================
// 5. THE ORCHESTRATOR: MEMORY MANAGER
// =========================================================================

class Engine2D {
private:
    PointCloud2D particles;
    size_t active_count = 0;
    float cell_size;
    std::vector<size_t> execution_indices;
    
    std::vector<AgentLocation2D> spatial_grid_flat;
    std::vector<AgentLocation2D> spatial_grid_temp; 
    
    // Exactly 1MB directory perfectly sized for CPU L3 Cache
    static constexpr size_t HASH_TABLE_SIZE = 262144; // 2^18
    static_assert((HASH_TABLE_SIZE & (HASH_TABLE_SIZE - 1)) == 0, "HASH_TABLE_SIZE must be a power of 2");
    
    std::vector<uint32_t> cell_start;
    std::vector<uint32_t> cell_count;

public:
    Engine2D(const std::vector<Agent2D>& agents_array, float cs) : cell_size(cs) {
        size_t capacity = agents_array.size();
        
        particles.x.reserve(capacity);
        particles.y.reserve(capacity);
        particles.vx.reserve(capacity);
        particles.vy.reserve(capacity);
        particles.fx.reserve(capacity);
        particles.fy.reserve(capacity);
        particles.mass.reserve(capacity);
        
        spatial_grid_flat.resize(capacity);
        spatial_grid_temp.resize(capacity);
        execution_indices.resize(capacity);
        
        cell_start.resize(HASH_TABLE_SIZE, 0);
        cell_count.resize(HASH_TABLE_SIZE, 0);

        for (size_t i = 0; i < capacity; ++i) {
            const Agent2D& agent = agents_array[i];
            particles.x.push_back(agent.x);
            particles.y.push_back(agent.y);
            particles.vx.push_back(agent.vx);
            particles.vy.push_back(agent.vy);
            particles.fx.push_back(agent.fx);
            particles.fy.push_back(agent.fy);
            particles.mass.push_back(agent.mass);
            
            execution_indices[i] = i;
            active_count++;
        }
        
        rebuild_spatial_index();
    }

    void rebuild_spatial_index() {
        GridHasher2D hasher;
        
        std::for_each(std::execution::par_unseq, execution_indices.begin(), execution_indices.begin() + active_count,
            [this, hasher](size_t i) {
                GridCell2D cell = get_grid_cell_2d(particles.x[i], particles.y[i], cell_size);
                spatial_grid_flat[i] = { hasher(cell), i }; 
            });
        
        radix_sort_2d(spatial_grid_flat.data(), spatial_grid_temp.data(), active_count);
        
        std::fill(cell_count.begin(), cell_count.end(), 0);
        
        const uint32_t HASH_MASK = HASH_TABLE_SIZE - 1;
        
        for (size_t i = 0; i < active_count; ++i) {
            cell_count[spatial_grid_flat[i].cell_hash & HASH_MASK]++;
        }
        
        uint32_t total = 0;
        for (size_t i = 0; i < HASH_TABLE_SIZE; ++i) {
            cell_start[i] = total;
            total += cell_count[i];
        }
    }

    PhysicsContext get_context() {
        return {
            std::span<float>(particles.x), std::span<float>(particles.y),
            std::span<float>(particles.vx), std::span<float>(particles.vy),
            std::span<float>(particles.fx), std::span<float>(particles.fy),
            std::span<float>(particles.mass), std::span<size_t>(execution_indices),
            std::span<AgentLocation2D>(spatial_grid_flat),
            std::span<uint32_t>(cell_start), std::span<uint32_t>(cell_count),
            active_count, cell_size, HASH_TABLE_SIZE
        };
    }

    void step_simulation(float dt, float kernel_radius, float base_strength) {
        rebuild_spatial_index(); 
        PhysicsContext ctx = get_context();
        PhysicsSystem::resolve_fields(ctx, kernel_radius, base_strength);
        PhysicsSystem::integrate(ctx, dt);
    }
    
    std::vector<Agent2D> get_state() const {
        std::vector<Agent2D> result;
        result.reserve(active_count);
        for(size_t i = 0; i < active_count; ++i) {
            result.emplace_back(i, particles.x[i], particles.y[i], particles.vx[i], particles.vy[i], particles.mass[i]);
            result.back().fx = particles.fx[i];
            result.back().fy = particles.fy[i];
        }
        return result;
    }
};

// =========================================================================
// 6. PYBIND11 MODULE
// =========================================================================

PYBIND11_MODULE(physics_engine_2d, m) {
    m.doc() = "2D Data-Oriented ECS Physics Engine for Python";

    py::class_<Agent2D>(m, "Agent2D")
        .def(py::init<int, float, float, float, float, float>())
        .def_readwrite("id", &Agent2D::id)
        .def_readwrite("x", &Agent2D::x)
        .def_readwrite("y", &Agent2D::y)
        .def_readwrite("vx", &Agent2D::vx)
        .def_readwrite("vy", &Agent2D::vy)
        .def_readwrite("fx", &Agent2D::fx)
        .def_readwrite("fy", &Agent2D::fy)
        .def_readwrite("mass", &Agent2D::mass);

    py::class_<Engine2D>(m, "Engine2D")
        .def(py::init<const std::vector<Agent2D>&, float>(), py::arg("agents"), py::arg("cell_size"))
        .def("rebuild_spatial_index", &Engine2D::rebuild_spatial_index)
        .def("step_simulation", &Engine2D::step_simulation, py::arg("dt"), py::arg("kernel_radius"), py::arg("base_strength"))
        .def("get_state", &Engine2D::get_state);
}