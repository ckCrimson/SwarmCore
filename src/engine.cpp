#include <iostream>
#include <vector>
#include <unordered_map>
#include <span> // C++20: The modern header for your "fast_ref" concept
#include<cmath>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // Automatically converts Python Lists <-> C++ std::vector
#include <algorithm> // Required for std::sort
using namespace std;

namespace py = pybind11;

struct Agent {
    int id; 
    double x, y, z;
    double health;
    int target_node_id;
    char team_color;
};



struct PointCloudSoA {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
};

// Cold Data
struct AgentMetaData {
    double health;
    int target_node_id;
    char color;
};

struct GridCell3D {
    int x, y, z;
    
    bool operator==(const GridCell3D& other)const{
        return (x==other.x && y==other.y && z==other.z);
    }
    
    friend std::ostream& operator<<(std::ostream& os, const GridCell3D& cell) {
        os << "[" << cell.x << ", " << cell.y << ", " << cell.z << "]";
        return os;
    }
};

struct GridHasher{
    
  size_t operator()(const GridCell3D& grid) const{
    const size_t P1 = 73856093;
    const size_t P2 = 19349663;
    const size_t P3 = 83492791;
     return (grid.x * P1) ^(grid.y * P2) ^ (grid.z * P3);
  };
    
};

GridCell3D get_grid_cell(double x, double y, double z, double R) {
    return {
        static_cast<int>(std::floor(x / R)),
        static_cast<int>(std::floor(y / R)),
        static_cast<int>(std::floor(z / R))
    };
}


// =========================================================================
// 1. YOUR CONCEPT: The FastRef (Modernized with std::span)
// This is a lightweight, non-owning view of the data. It copies instantly.
// =========================================================================
struct AgentFastRef {
    std::span<double> x;
    std::span<double> y;
    std::span<double> z;
    std::span<AgentMetaData> meta_data;
};

// =========================================================================
// 2. THE SYSTEM: Stateless Utility Class
// It knows nothing about CollectionOfAgents. It only operates on memory views.
// =========================================================================
class AgentUtility {
public:
    // Takes the fast_ref by value (spans are extremely cheap to copy)
    static void increase_all_health(AgentFastRef fast_ref, double health_boost) {
        
        // Because std::span knows its own size, we can use clean range-based loops
        // instead of raw pointer arithmetic and active_counts.
        for (AgentMetaData& meta : fast_ref.meta_data) {
            meta.health += health_boost;
        }
        
        // Note: If you were doing multithreading, you would chunk the fast_ref.meta_data 
        // into smaller spans and pass them to different threads here.
    }
    static void move_agents_forward(AgentFastRef fast_ref, double delta_time) {
        // Simple example: Move everyone forward on the X axis
        for (size_t i = 0; i < fast_ref.x.size(); ++i) {
            fast_ref.x[i] += (5.0 * delta_time); 
        }
    }
};

// =========================================================================
// 3. THE CONTAINER: Stateful Memory Manager
// =========================================================================
// =========================================================================
// 3. THE CONTAINER: Stateful Memory Manager (PURE DOD)
// =========================================================================
class CollectionOfAgents {
private:
    PointCloudSoA agent_positions;
    std::vector<AgentMetaData> agent_meta_data;
    std::unordered_map<int, size_t> id_to_index; 
    size_t active_agents_count = 0;
    double cell_size;
    
    // PURE DOD: One flat array. No maps. No memory allocation in the loop.
    struct AgentLocation {
        size_t cell_hash;
        size_t agent_index;
    };
    std::vector<AgentLocation> spatial_grid_flat;

public:
    CollectionOfAgents(const std::vector<Agent>& agents_array, double cs) : cell_size(cs) {
        size_t capacity = agents_array.size() * 2;
        agent_positions.x.reserve(capacity);
        agent_positions.y.reserve(capacity);
        agent_positions.z.reserve(capacity);
        agent_meta_data.reserve(capacity);
        
        // Pre-allocate the flat grid exactly to the capacity
        spatial_grid_flat.resize(capacity);

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
        
        // 1. Overwrite the array with new hashes (Zero memory allocations!)
        for(size_t i = 0; i < active_agents_count; i++) {
            GridCell3D cell = get_grid_cell(
                agent_positions.x[i], 
                agent_positions.y[i], 
                agent_positions.z[i], 
                cell_size
            );
            spatial_grid_flat[i] = { hasher(cell), i }; 
        }
        
        // 2. Sort the array by hash. 
        // Agents in the same cell will now sit exactly next to each other in RAM.
        std::sort(spatial_grid_flat.begin(), spatial_grid_flat.begin() + active_agents_count, 
            [](const AgentLocation& a, const AgentLocation& b) {
                return a.cell_hash < b.cell_hash;
            });
    }

    AgentFastRef generate_fast_ref() {
        return {
            std::span<double>(agent_positions.x.data(), active_agents_count),
            std::span<double>(agent_positions.y.data(), active_agents_count),
            std::span<double>(agent_positions.z.data(), active_agents_count),
            std::span<AgentMetaData>(agent_meta_data.data(), active_agents_count)
        };
    }
    
    void debug_print_state() const {
        std::cout << "\n=== ENGINE MEMORY STATE ===\n";
        std::cout << "Active Agents: " << active_agents_count << "\n";
        std::cout << "Spatial Grid is now a Pure DOD sorted array.\n";
        std::cout << "===========================\n\n";
    }
};

// THIS IS THE BRIDGE
PYBIND11_MODULE(fast_spatial_engine, m) {
    m.doc() = "Data-Oriented C++ Engine for Python";

    // 1. Expose the Agent struct so Python can create them
    py::class_<Agent>(m, "Agent")
        .def(py::init<int, double, double, double, double, int, char>())
        .def_readwrite("id", &Agent::id)
        .def_readwrite("x", &Agent::x)
        .def_readwrite("y", &Agent::y)
        .def_readwrite("z", &Agent::z);

    // 2. Expose the Container
    py::class_<CollectionOfAgents>(m, "CollectionOfAgents")
        .def(py::init<const std::vector<Agent>&, double>())
        .def("rebuild_spatial_index", &CollectionOfAgents::rebuild_spatial_index)
        .def("debug_print_state", &CollectionOfAgents::debug_print_state);

    // 3. Expose the Utility functions
    // Notice how we use a lambda function here. We have to call world.generate_fast_ref() 
    // inside C++ because Python doesn't know what a 'std::span' is!
    m.def("move_agents_forward", [](CollectionOfAgents& world, double delta_time) {
        AgentUtility::move_agents_forward(world.generate_fast_ref(), delta_time);
    });
}