#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <execution>
#include <span>
#include <chrono>
#include <random>
#include <iomanip>

// =========================================================================
// 1. DATA STRUCTURES & HASHING (2D)
// =========================================================================

struct Agent2D {
    int id;
    float x, y, vx, vy, fx, fy, mass;

    Agent2D(int _id, float _x, float _y, float _vx, float _vy, float _mass)
        : id(_id), x(_x), y(_y), vx(_vx), vy(_vy), fx(0), fy(0), mass(_mass) {}
};

struct PointCloud2D {
    std::vector<float> x, y, vx, vy, fx, fy, mass;
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
        return (static_cast<uint32_t>(grid.x) * P1) ^ (static_cast<uint32_t>(grid.y) * P2);
    }
};

inline GridCell2D get_grid_cell_2d(float x, float y, float cell_size) {
    return { static_cast<int>(std::floor(x / cell_size)), static_cast<int>(std::floor(y / cell_size)) };
}

// =========================================================================
// 2. THE ECS BRIDGE: PHYSICS CONTEXT
// =========================================================================

struct PhysicsContext {
    std::span<float> x, y, vx, vy, fx, fy, mass;
    std::span<size_t> execution_indices;
    std::span<AgentLocation2D> spatial_grid_flat;
    std::span<uint32_t> cell_start, cell_count;
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

        // MULTI-THREADING UNLEASHED: Processing every particle in parallel
        std::for_each(std::execution::par_unseq, ctx.execution_indices.begin(), ctx.execution_indices.end(),
            [&](size_t i) {
                GridHasher2D hasher; // Local hasher per thread
                
                float ax = ctx.x[i];
                float ay = ctx.y[i];
                
                // LOCAL ACCUMULATORS: Keep writes in ultra-fast CPU registers
                float local_fx = 0.0f;
                float local_fy = 0.0f;
                
                GridCell2D g = get_grid_cell_2d(ax, ay, ctx.cell_size);
                
                for (int nx = g.x - 1; nx <= g.x + 1; ++nx) {
                    for (int ny = g.y - 1; ny <= g.y + 1; ++ny) {
                        
                        uint32_t bucket = hasher({nx, ny}) & HASH_MASK;
                        uint32_t start = ctx.cell_start[bucket];
                        uint32_t count = ctx.cell_count[bucket];
                        
                        for (uint32_t idx = 0; idx < count; ++idx) {
                            size_t j = ctx.spatial_grid_flat[start + idx].agent_index;
                            
                            // Self-check only (No Newton's 3rd Law cull)
                            if (i == j) continue; 
                            
                            float dx = ctx.x[j] - ax;
                            float dy = ctx.y[j] - ay;
                            float d_sq = (dx*dx) + (dy*dy);
                            
                            if (d_sq >= k_sq || d_sq == 0.0f) continue;
                            
                            float d = std::sqrt(d_sq);
                            float force = PhysicsKernels2D::linear_repulsion(d, kernel_radius, base_strength);
                            
                            // Only apply force to our local accumulator (No data races!)
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
                ctx.vx[i] += ctx.fx[i] * dt; ctx.vy[i] += ctx.fy[i] * dt;
                ctx.x[i] += ctx.vx[i] * dt;  ctx.y[i] += ctx.vy[i] * dt;
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
        
        for (size_t i = 0; i < count; ++i) counts[(current_src[i].cell_hash >> shift) & 0xFF]++;
        
        size_t offsets[RADIX] = {0};
        size_t total = 0;
        for (size_t i = 0; i < RADIX; ++i) { offsets[i] = total; total += counts[i]; }
        for (size_t i = 0; i < count; ++i) current_dst[offsets[(current_src[i].cell_hash >> shift) & 0xFF]++] = current_src[i];
        
        std::swap(current_src, current_dst);
    }
    if constexpr (PASSES % 2 != 0) { for (size_t i = 0; i < count; ++i) src[i] = dst[i]; }
}

// =========================================================================
// 5. THE ORCHESTRATOR
// =========================================================================

class Engine2D {
private:
    PointCloud2D particles;
    size_t active_count = 0;
    float cell_size;
    std::vector<size_t> execution_indices;
    std::vector<AgentLocation2D> spatial_grid_flat, spatial_grid_temp; 
    
    static constexpr size_t HASH_TABLE_SIZE = 262144; // 2^18 buckets (Exactly 1 Megabyte!)
    static_assert((HASH_TABLE_SIZE & (HASH_TABLE_SIZE - 1)) == 0, "HASH_TABLE_SIZE must be a power of 2");

    std::vector<uint32_t> cell_start, cell_count;

public:
    Engine2D(const std::vector<Agent2D>& agents_array, float cs) : cell_size(cs) {
        size_t capacity = agents_array.size();
        particles.x.reserve(capacity); particles.y.reserve(capacity);
        particles.vx.reserve(capacity); particles.vy.reserve(capacity);
        particles.fx.reserve(capacity); particles.fy.reserve(capacity);
        particles.mass.reserve(capacity);
        
        spatial_grid_flat.resize(capacity); spatial_grid_temp.resize(capacity);
        execution_indices.resize(capacity);
        cell_start.resize(HASH_TABLE_SIZE, 0); cell_count.resize(HASH_TABLE_SIZE, 0);

        for (size_t i = 0; i < capacity; ++i) {
            const auto& a = agents_array[i];
            particles.x.push_back(a.x); particles.y.push_back(a.y);
            particles.vx.push_back(a.vx); particles.vy.push_back(a.vy);
            particles.fx.push_back(a.fx); particles.fy.push_back(a.fy);
            particles.mass.push_back(a.mass);
            execution_indices[i] = i; active_count++;
        }
        rebuild_spatial_index();
    }

    void rebuild_spatial_index() {
        GridHasher2D hasher;
        std::for_each(std::execution::par_unseq, execution_indices.begin(), execution_indices.begin() + active_count,
            [&](size_t i) { spatial_grid_flat[i] = { hasher(get_grid_cell_2d(particles.x[i], particles.y[i], cell_size)), i }; });
        
        radix_sort_2d(spatial_grid_flat.data(), spatial_grid_temp.data(), active_count);
        
        std::fill(cell_count.begin(), cell_count.end(), 0);
        
        const uint32_t HASH_MASK = HASH_TABLE_SIZE - 1;
        
        // OPTIMIZATION 1: Bitwise AND for bucket assignment
        for (size_t i = 0; i < active_count; ++i) { 
            cell_count[spatial_grid_flat[i].cell_hash & HASH_MASK]++; 
        }
        
        uint32_t total = 0;
        for (size_t i = 0; i < HASH_TABLE_SIZE; ++i) { cell_start[i] = total; total += cell_count[i]; }
    }

    PhysicsContext get_context() {
        return { std::span(particles.x), std::span(particles.y), std::span(particles.vx), std::span(particles.vy),
                 std::span(particles.fx), std::span(particles.fy), std::span(particles.mass), std::span(execution_indices),
                 std::span(spatial_grid_flat), std::span(cell_start), std::span(cell_count), active_count, cell_size, HASH_TABLE_SIZE };
    }

    void step_simulation(float dt, float kernel_radius, float base_strength) {
        rebuild_spatial_index(); 
        PhysicsContext ctx = get_context();
        PhysicsSystem::resolve_fields(ctx, kernel_radius, base_strength);
        PhysicsSystem::integrate(ctx, dt);
    }
};

// =========================================================================
// 6. BENCHMARK RUNNER
// =========================================================================

int main() {
    const int NUM_PARTICLES = 100000;
    const float WORLD_SIZE = 20000.0f;
    const float CELL_SIZE = 10.0f;
    const float KERNEL_RADIUS = 10.0f;
    const float BASE_STRENGTH = 5.0f;
    const float DT = 0.016f; // ~60 FPS

    std::cout << "Initializing Benchmark...\n";

    // 1. Scatter Particles Randomly
    std::mt19937 rng(42); 
    std::uniform_real_distribution<float> dist_pos(0.0f, WORLD_SIZE);
    std::uniform_real_distribution<float> dist_vel(-5.0f, 5.0f);

    std::vector<Agent2D> initial_agents;
    initial_agents.reserve(NUM_PARTICLES);
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        initial_agents.emplace_back(i, dist_pos(rng), dist_pos(rng), dist_vel(rng), dist_vel(rng), 1.0f);
    }

    // 2. Initialize Engine
    Engine2D engine(initial_agents, CELL_SIZE);

    // 3. Benchmark Parameters
    const int NUM_ITERATIONS = 100;
    const int FRAMES_PER_ITERATION = 4;
    double total_time_ms = 0.0;

    std::cout << "Simulating " << NUM_PARTICLES << " particles.\n";
    std::cout << "Running " << NUM_ITERATIONS << " iterations (Each doing " << FRAMES_PER_ITERATION << " Physics Frames)...\n\n";

    // 4. Run Loop
    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        
        auto start = std::chrono::high_resolution_clock::now();
        
        // Execute 4 frames
        for (int f = 0; f < FRAMES_PER_ITERATION; ++f) {
            engine.step_simulation(DT, KERNEL_RADIUS, BASE_STRENGTH);
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> ms = end - start;
        total_time_ms += ms.count();
    }

    // 5. Results
    double avg_time_per_iteration = total_time_ms / NUM_ITERATIONS;
    double avg_time_per_frame = avg_time_per_iteration / FRAMES_PER_ITERATION;

    std::cout << "========================================\n";
    std::cout << "Total Benchmark Time:    " << total_time_ms << " ms\n";
    std::cout << "Avg Time per 4 Frames:   " << avg_time_per_iteration << " ms\n";
    std::cout << "Avg Time per 1 Frame:    " << avg_time_per_frame << " ms\n";
    std::cout << "Theoretical Max FPS:     " << std::fixed << std::setprecision(0) << (1000.0 / avg_time_per_frame) << " FPS\n";
    std::cout << "========================================\n";

    return 0;
}