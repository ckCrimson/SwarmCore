import fast_spatial_engine

print("Engine imported successfully!\n")

# 1. Create the data using the C++ struct we exposed!
# Arguments: (id, x, y, z, health, target_node_id, team_color)
agents = [
    fast_spatial_engine.Agent(1, 0.0, 0.0, 0.0, 100.0, 12, 'g'),
    fast_spatial_engine.Agent(2, 2.0, 2.0, 2.0, 100.0, 12, 'r'),
    fast_spatial_engine.Agent(3, 12.0, 0.0, 0.0, 100.0, 12, 'b')
]

# 2. Initialize the C++ Engine 
# (PyBind11 automatically converts our Python List into the C++ std::vector!)
cell_size = 5.0
world = fast_spatial_engine.CollectionOfAgents(agents, cell_size)

print("--- BEFORE MOVING ---")
# Call the C++ method directly from Python
world.debug_print_state()

# 3. Step the engine forward (using a big time step: 2.0)
# This will move the agents and rebuild the spatial index directly in C++
delta_time = 2.0
world.simulate(1, delta_time)

print("--- AFTER MOVING ---")
world.debug_print_state()