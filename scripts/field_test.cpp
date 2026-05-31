#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <execution>
#include <span>
#include <iomanip>

struct Agent2D {
    int id; float x, y, vx, vy, fx, fy, mass;
    Agent2D(int _id, float _x, float _y, float _vx, float _vy, float _mass)
        : id(_id), x(_x), y(_y), vx(_vx), vy(_vy), fx(0), fy(0), mass(_mass) {}
};

struct PointCloud2D { std::vector<float> x, y, vx, vy, fx, fy, mass; };
struct GridCell2D { int x, y; bool operator==(const GridCell2D&) const = default; };
struct AgentLocation2D { uint32_t cell_hash; size_t agent_index; };

struct GridHasher2D {
    uint32_t operator()(const GridCell2D& grid) const {
        return (static_cast<uint32_t>(grid.x) * 73856093) ^ (static_cast<uint32_t>(grid.y) * 19349663);
    }
};

inline GridCell2D get_grid_cell_2d(float x, float y, float cell_size) {
    return { static_cast<int>(std::floor(x / cell_size)), static_cast<int>(std::floor(y / cell_size)) };
}

struct PhysicsContext {
    std::span<float> x, y, vx, vy, fx, fy, mass;
    std::span<size_t> execution_indices;
    std::span<AgentLocation2D> spatial_grid_flat;
    std::span<uint32_t> cell_start, cell_count;
    size_t active_count; float cell_size; size_t hash_table_size;
};

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

        std::for_each(std::execution::par_unseq, ctx.execution_indices.begin(), ctx.execution_indices.end(),
            [&](size_t i) {
                GridHasher2D hasher;
                float ax = ctx.x[i], ay = ctx.y[i];
                float local_fx = 0.0f, local_fy = 0.0f;
                GridCell2D g = get_grid_cell_2d(ax, ay, ctx.cell_size);
                
                for (int nx = g.x - 1; nx <= g.x + 1; ++nx) {
                    for (int ny = g.y - 1; ny <= g.y + 1; ++ny) {
                        uint32_t bucket = hasher({nx, ny}) & HASH_MASK;
                        uint32_t start = ctx.cell_start[bucket], count = ctx.cell_count[bucket];
                        
                        for (uint32_t idx = 0; idx < count; ++idx) {
                            size_t j = ctx.spatial_grid_flat[start + idx].agent_index;
                            if (i == j) continue; 
                            
                            float dx = ctx.x[j] - ax, dy = ctx.y[j] - ay;
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
                ctx.fx[i] = local_fx; ctx.fy[i] = local_fy;
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

inline void radix_sort_2d(AgentLocation2D* src, AgentLocation2D* dst, size_t count) {
    constexpr int PASSES = sizeof(uint32_t); constexpr size_t RADIX = 256;
    AgentLocation2D* current_src = src; AgentLocation2D* current_dst = dst;
    for (int pass = 0; pass < PASSES; ++pass) {
        size_t counts[RADIX] = {0}; int shift = pass * 8;
        for (size_t i = 0; i < count; ++i) counts[(current_src[i].cell_hash >> shift) & 0xFF]++;
        size_t offsets[RADIX] = {0}; size_t total = 0;
        for (size_t i = 0; i < RADIX; ++i) { offsets[i] = total; total += counts[i]; }
        for (size_t i = 0; i < count; ++i) current_dst[offsets[(current_src[i].cell_hash >> shift) & 0xFF]++] = current_src[i];
        std::swap(current_src, current_dst);
    }
    if constexpr (PASSES % 2 != 0) { for (size_t i = 0; i < count; ++i) src[i] = dst[i]; }
}

class Engine2D {
private:
    PointCloud2D particles;
    size_t active_count = 0; float cell_size;
    std::vector<size_t> execution_indices;
    std::vector<AgentLocation2D> spatial_grid_flat, spatial_grid_temp; 
    static constexpr size_t HASH_TABLE_SIZE = 262144; 
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
    }

    void rebuild_spatial_index() {
        GridHasher2D hasher;
        std::for_each(std::execution::par_unseq, execution_indices.begin(), execution_indices.begin() + active_count,
            [&](size_t i) { spatial_grid_flat[i] = { hasher(get_grid_cell_2d(particles.x[i], particles.y[i], cell_size)), i }; });
        radix_sort_2d(spatial_grid_flat.data(), spatial_grid_temp.data(), active_count);
        std::fill(cell_count.begin(), cell_count.end(), 0);
        const uint32_t HASH_MASK = HASH_TABLE_SIZE - 1;
        for (size_t i = 0; i < active_count; ++i) { cell_count[spatial_grid_flat[i].cell_hash & HASH_MASK]++; }
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

    void print_diagnostics(int frame) {
        std::cout << "--- Frame " << frame << " ---\n";
        for (size_t i = 0; i < active_count; ++i) {
            std::cout << "P" << i 
                      << " | Pos: (" << particles.x[i] << ", " << particles.y[i] << ")"
                      << " | Vel: (" << particles.vx[i] << ", " << particles.vy[i] << ")"
                      << " | Frc: (" << particles.fx[i] << ", " << particles.fy[i] << ")\n";
        }
        std::cout << "\n";
    }
};

int main() {
    std::cout << std::fixed << std::setprecision(3);
    
    // Exact 3-4-5 Triangle setup
    std::vector<Agent2D> initial_agents = {
        Agent2D(0, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f),
        Agent2D(1, 4.0f, 3.0f, 0.0f, 0.0f, 1.0f)
    };

    Engine2D engine(initial_agents, 10.0f);
    
    float kernel_radius = 10.0f;
    float base_strength = 5.0f;
    float dt = 0.1f;

    std::cout << "Initial State:\n";
    engine.print_diagnostics(0);

    engine.step_simulation(dt, kernel_radius, base_strength);
    engine.print_diagnostics(1);

    return 0;
}