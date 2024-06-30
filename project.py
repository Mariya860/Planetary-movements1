import numpy as np

# Constants
G = 6.67430e-11  # gravitational constant
dt = 86400  # time step (one day in seconds)

# Initial conditions for two bodies (e.g., sun and Earth)
# Masses in kg, positions in m, velocities in m/s
masses = np.array([1.989e30, 5.972e24], dtype=np.float64)  # Ensure using float64 for masses
positions = np.array([[0, 0], [1.496e11, 0]], dtype=np.float64)  # Ensure using float64 for positions
velocities = np.array([[0, 0], [0, 29780]], dtype=np.float64)  # Ensure using float64 for velocities

def compute_forces(masses, positions):
    n = len(masses)
    forces = np.zeros_like(positions)
    for i in range(n):
        for j in range(n):
            if i != j:
                r_vec = positions[j] - positions[i]
                r_mag = np.linalg.norm(r_vec)
                r_hat = r_vec / r_mag
                force_mag = G * masses[i] * masses[j] / r_mag**2
                forces[i] += force_mag * r_hat
    return forces

def update_system(masses, positions, velocities, dt):
    forces = compute_forces(masses, positions)
    velocities += forces / masses[:, np.newaxis] * dt
    positions += velocities * dt
    return positions, velocities

# Simulation loop
for _ in range(365):  # Run for one year
    positions, velocities = update_system(masses, positions, velocities, dt)
    print(f"Day {_+1}: position of Earth: {positions[1]}")