import numpy as np
import matplotlib.pyplot as plt
from sklearn.mixture import GaussianMixture
import cvxpy as cp
import time  # Added for timing


# 2. Learn SEDS parameters with timing
# ------------------------------------
def learn_seds(positions, velocities, n_components=3):
    data = np.hstack((positions, velocities))
    gmm = GaussianMixture(n_components=n_components, 
                         covariance_type='full',
                         random_state=0)
    gmm.fit(data)
    
    A_matrices = []
    for k in range(n_components):
        Sigma = gmm.covariances_[k]
        A_k = Sigma[2:, :2] @ np.linalg.inv(Sigma[:2, :2])
        A_matrices.append(A_k)
    
    # Enforce stability constraints
    stable_A_matrices = []
    for A_k in A_matrices:
        A_var = cp.Variable((2, 2))
        constraints = [A_var + A_var.T << -1e-6*np.eye(2)]
        prob = cp.Problem(cp.Minimize(cp.norm(A_var - A_k, 'fro')), constraints)
        prob.solve(solver=cp.SCS)
        stable_A_matrices.append(A_var.value)
    
    target = np.array([1.0, 1.0])  # Known target for synthetic data
    return gmm, stable_A_matrices, target


# 3. Velocity query and visualization with timing
# -----------------------------------------------
def query_velocity(gmm, stable_A_matrices, target, query_pos):
    """Fixed version handling 4D input expectations"""
    query_4d = np.hstack((query_pos, [0, 0]))  # Shape: (4,)
    responsibilities = gmm.predict_proba(query_4d.reshape(1, -1))[0]
    
    velocity = np.zeros(2)
    for k, gamma_k in enumerate(responsibilities):
        displacement = query_pos - target
        velocity += gamma_k * (stable_A_matrices[k] @ displacement)
    
    return velocity

if __name__ == "__main__":

    # 1. Generate synthetic straight line trajectory data
    # --------------------------------------------------
    T = 100  # Number of time steps
    t = np.linspace(0, 1, T)

    # Create a straight line from (0,0) to (1,1) with constant velocity
    positions = np.column_stack((t, t))
    velocities = np.column_stack((np.ones(T), np.ones(T)))

    # Create a straight line from (0,0) to (1,1) with constant velocity
    # positions = np.column_stack((t, t))
    # velocities = np.column_stack((np.ones(T), np.ones(T)))



    # Time the learning process
    start_time = time.time()
    gmm, stable_A, target = learn_seds(positions, velocities)
    learn_time = time.time() - start_time
    print(f"Learning time: {learn_time:.2f} seconds")



    # Create grid for vector field
    x = np.linspace(0, 1.2, 15)
    y = np.linspace(0, 1.2, 15)
    X, Y = np.meshgrid(x, y)
    grid_points = np.column_stack([X.ravel(), Y.ravel()])

    # Time the velocity queries
    start_time = time.time()
    V = np.array([query_velocity(gmm, stable_A, target, p) for p in grid_points])
    query_time = time.time() - start_time
    print(f"Query time for {len(grid_points)} points: {query_time:.2f} seconds")
    print(f"Average time per query: {query_time/len(grid_points)*1000:.2f} ms")

    # Normalize velocities for better visualization
    norms = np.linalg.norm(V, axis=1, keepdims=True)
    V_normalized = V / np.where(norms > 0, norms, 1)

    # Plot results with timing info in title
    plt.figure(figsize=(10, 8))
    plt.quiver(grid_points[:,0], grid_points[:,1], 
            V_normalized[:,0], V_normalized[:,1], 
            color='blue', scale=30)
    plt.plot(positions[:,0], positions[:,1], 'r-', linewidth=2, label='Training Trajectory')
    plt.scatter(target[0], target[1], c='green', s=200, marker='*', label='Target')
    plt.title(f'SEDS Velocity Vector Field\nLearning: {learn_time:.2f}s | Queries: {query_time:.2f}s')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()
