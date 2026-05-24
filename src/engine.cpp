#include <iostream>
#include <vector>
#include <unordered_map>
#include <span> 
#include <cmath>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> 
#include <algorithm> 

namespace py = pybind11;

struct Agent {
    int id; 
    float x, y, z;
    float health;
    int target_node_id;
    char team_color;

    // Explicit constructor to guarantee PyBind11 maps fields cleanly
    Agent(int _id, float _x, float _y, float _z, float _h, int _t, char _c)
        : id(_id), x(_x), y(_y), z(_z), health(_h), target_node_id(_t), team_color(_c) {}
};

struct PointCloudSoA {
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
};

struct AgentMetaData {
    float health;
    int target_node_id;
    char color;
};

struct GridCell3D {
    int x, y, z;
    bool operator==(const GridCell3D&) const = default;
};

struct GridHasher {
    size_t operator()(const GridCell3D& grid) const {
        const size_t P1 = 73856093;
        const size_t P2 = 19349663;
        const size_t P3 = 83492791;
        return (grid.x * P1) ^ (grid.y * P2) ^ (grid.z * P3);
    }
};

inline GridCell3D get_grid_cell(float x, float y, float z, float cell_size) {
    return {
        static_cast<int>(std::floor(x / cell_size)),
        static_cast<int>(std::floor(y / cell_size)),
        static_cast<int>(std::floor(z / cell_size))
    };
}

struct AgentLocation {
    size_t cell_hash; 
    size_t agent_index; 
};

// =========================================================================
// FAST O(N) RADIX SORT HELPER (STRICT DOD)
// =========================================================================
inline void radix_sort_agent_locations(AgentLocation* src, AgentLocation* dst, size_t count) {
    constexpr int PASSES = sizeof(size_t);
    constexpr size_t RADIX = 256;
    
    AgentLocation* current_src = src;
    AgentLocation* current_dst = dst;
    
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

struct AgentFastRef {
    std::span<float> x;
    std::span<float> y;
    std::span<float> z;
    std::span<AgentMetaData> meta_data;
};

class AgentUtility {
public:
    static void increase_all_health(AgentFastRef fast_ref, float health_boost) {
        for (AgentMetaData& meta : fast_ref.meta_data) {
            meta.health += health_boost;
        }
    }
    static void move_agents_forward(AgentFastRef fast_ref, float delta_time) {
        for (size_t i = 0; i < fast_ref.x.size(); ++i) {
            fast_ref.x[i] += (5.0f * delta_time); 
        }
    }
};

// =========================================================================
// 3. THE CONTAINER: Stateful Memory Manager (PURE DOD)
// =========================================================================
class CollectionOfAgents {
private:
    PointCloudSoA agent_positions;
    std::vector<AgentMetaData> agent_meta_data;
    std::unordered_map<int, size_t> id_to_index; 
    size_t active_agents_count = 0;
    float cell_size;
    
    std::vector<AgentLocation> spatial_grid_flat;
    std::vector<AgentLocation> spatial_grid_temp; 

public:
    CollectionOfAgents(const std::vector<Agent>& agents_array, float cs) : cell_size(cs) {
        size_t capacity = agents_array.size() * 2;
        agent_positions.x.reserve(capacity);
        agent_positions.y.reserve(capacity);
        agent_positions.z.reserve(capacity);
        agent_meta_data.reserve(capacity);
        
        spatial_grid_flat.resize(capacity);
        spatial_grid_temp.resize(capacity);

        for (size_t i = 0; i < agents_array.size(); ++i) {
            const Agent& agent = agents_array[i];
            agent_positions.x.push_back(agent.x);
            agent_positions.y.push_back(agent.y);
            agent_positions.z.push_back(agent.z);
            agent_meta_data.push_back({agent.health, agent.target_node_id, agent.team_color});
            
            id_to_index[agent.id] = i;
            active_agents_count++;
        }
        rebuild_spatial_index();
    }
    
    void rebuild_spatial_index() {
        GridHasher hasher;
        
        for(size_t i = 0; i < active_agents_count; i++) {
            GridCell3D cell = get_grid_cell(
                agent_positions.x[i], 
                agent_positions.y[i], 
                agent_positions.z[i], 
                cell_size
            );
            spatial_grid_flat[i] = { hasher(cell), i }; 
        }
        
        radix_sort_agent_locations(spatial_grid_flat.data(), spatial_grid_temp.data(), active_agents_count);
    }

    AgentFastRef generate_fast_ref() {
        return {
            std::span<float>(agent_positions.x.data(), active_agents_count),
            std::span<float>(agent_positions.y.data(), active_agents_count),
            std::span<float>(agent_positions.z.data(), active_agents_count),
            std::span<AgentMetaData>(agent_meta_data.data(), active_agents_count)
        };
    }

    void simulate(int frames, float dt) {
        AgentFastRef ref = generate_fast_ref();
        for (int i = 0; i < frames; ++i) {
            AgentUtility::move_agents_forward(ref, dt);
            rebuild_spatial_index();
        }
    }
    
    void debug_print_state() const {
        std::cout << "\n=== ENGINE MEMORY STATE ===\n";
        std::cout << "Active Agents: " << active_agents_count << "\n";
        std::cout << "Spatial Grid is now a Pure DOD Radix Sorted array.\n";
        std::cout << "===========================\n\n";
    }
};

PYBIND11_MODULE(fast_spatial_engine, m) {
    m.doc() = "Data-Oriented C++ Engine for Python";

    py::class_<Agent>(m, "Agent")
        .def(py::init<int, float, float, float, float, int, char>())
        .def_readwrite("id", &Agent::id)
        .def_readwrite("x", &Agent::x)
        .def_readwrite("y", &Agent::y)
        .def_readwrite("z", &Agent::z);

    py::class_<CollectionOfAgents>(m, "CollectionOfAgents")
        .def(py::init<const std::vector<Agent>&, float>())
        .def("rebuild_spatial_index", &CollectionOfAgents::rebuild_spatial_index)
        .def("debug_print_state", &CollectionOfAgents::debug_print_state)
        .def("simulate", &CollectionOfAgents::simulate);

    m.def("move_agents_forward", [](CollectionOfAgents& world, float delta_time) {
        AgentUtility::move_agents_forward(world.generate_fast_ref(), delta_time);
    });
}