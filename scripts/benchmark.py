import time
import random
import fast_spatial_engine

# --- CONFIGURATION ---
NUM_AGENTS = 1000000  # Pushing the 1 Million Limit!
FRAMES = 100
DELTA_TIME = 0.016
CELL_SIZE = 10.0

print(f"--- GENERATING {NUM_AGENTS} AGENTS ---")
start_time = time.time()
raw_agents = []
for i in range(NUM_AGENTS):
    raw_agents.append(
        fast_spatial_engine.Agent(
            i,                          # ID
            random.uniform(0, 1000),    # X
            random.uniform(0, 1000),    # Y
            random.uniform(0, 100),     # Z
            100.0,                      # Health
            -1,                         # Target Node
            b'R' if i % 2 == 0 else b'B' # Team Color (bytes character)
        )
    )
print(f"Python Generation Time: {time.time() - start_time:.4f} seconds\n")

print("--- INITIALIZING C++ ENGINE ---")
start_init = time.time()
world = fast_spatial_engine.CollectionOfAgents(raw_agents, CELL_SIZE)
print(f"C++ SoA Allocation & First Radix Build: {time.time() - start_init:.4f} seconds\n")

print(f"--- RUNNING {FRAMES} FRAMES INSIDE C++ BACKEND ---")
start_sim = time.time()

# ONE SINGLE CALL: Python hands execution to C++ completely
world.simulate(FRAMES, DELTA_TIME)

total_sim_time = time.time() - start_sim
avg_frame_time = (total_sim_time / FRAMES) * 1000

print(f"Total Simulation Time: {total_sim_time:.4f} seconds")
print(f"Average time per frame: {avg_frame_time:.4f} ms")
print(f"Estimated Frames Per Second: {1000.0 / avg_frame_time:.2f} FPS")