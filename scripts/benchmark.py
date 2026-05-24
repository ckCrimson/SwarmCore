import fast_spatial_engine
import time
import random

NUM_AGENTS = 100_000
CELL_SIZE = 5.0
FRAMES = 100
DELTA_TIME = 0.016 # 16ms step

print(f"--- GENERATING {NUM_AGENTS} AGENTS ---")
start_gen = time.perf_counter()

# Generate random starting positions
agents = []
for i in range(NUM_AGENTS):
    x = random.uniform(0.0, 1000.0)
    y = random.uniform(0.0, 1000.0)
    z = random.uniform(0.0, 100.0)
    # Using 'g' for color just as a placeholder
    agents.append(fast_spatial_engine.Agent(i, x, y, z, 100.0, 0, 'g'))

end_gen = time.perf_counter()
print(f"Python Generation Time: {(end_gen - start_gen):.4f} seconds\n")

print("--- INITIALIZING C++ ENGINE ---")
# This times how long it takes PyBind11 to copy 100,000 Python objects 
# into our highly optimized C++ Structure-of-Arrays (SoA).
start_init = time.perf_counter()
world = fast_spatial_engine.CollectionOfAgents(agents, CELL_SIZE)
end_init = time.perf_counter()
print(f"C++ SoA Memory Allocation & First Grid Build: {(end_init - start_init):.4f} seconds\n")

print(f"--- RUNNING {FRAMES} FRAMES ---")
start_sim = time.perf_counter()


total_math_time = 0.0
total_grid_time = 0.0

for _ in range(FRAMES):
    # Time the contiguous SoA math operations
    t0 = time.perf_counter()
    fast_spatial_engine.move_agents_forward(world, DELTA_TIME)
    
    # Time the unordered_map memory operations
    t1 = time.perf_counter()
    world.rebuild_spatial_index()
    t2 = time.perf_counter()
    
    total_math_time += (t1 - t0)
    total_grid_time += (t2 - t1)

total_sim_time = total_math_time + total_grid_time

print(f"Total Math Time (SoA Span): {total_math_time:.4f} seconds ({(total_math_time/total_sim_time)*100:.1f}%)")
print(f"Total Grid Time (Map Hash): {total_grid_time:.4f} seconds ({(total_grid_time/total_sim_time)*100:.1f}%)")
print(f"\nAverage time per frame: {(total_sim_time / FRAMES) * 1000:.4f} ms")