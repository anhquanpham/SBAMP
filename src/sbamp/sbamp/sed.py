import numpy as np
import matplotlib.pyplot as plt
from sklearn.mixture import GaussianMixture
import cvxpy as cp
import time

def learn_seds(start,  goal, n_components=2, num_interp_points=20):
    
    # Interpolate points linearly through the three points
    first_half = num_interp_points // 2
    second_half = num_interp_points - first_half
    
    positions = np.linspace(start, goal, first_half, endpoint=False)
    

    
    # Calculate velocities by numerical differentiation
    velocities = np.diff(positions, axis=0)
    velocities = np.vstack([velocities, velocities[-1]])  # Keep array size consistent
    
    # Normalize velocities to unit vectors
    norms = np.linalg.norm(velocities, axis=1, keepdims=True)
    velocities_normalized = velocities / np.where(norms > 0, norms, 1)
    
    # Prepare data for GMM fitting
    data = np.hstack((positions, velocities_normalized))
    
    # Fit Gaussian Mixture Model
    gmm = GaussianMixture(n_components=n_components, 
                          covariance_type='full',
                          random_state=0,
                          max_iter=100)  # Limit iterations for speed
    gmm.fit(data)
    
    # Extract A matrices from covariance
    A_matrices = []
    for k in range(n_components):
        Sigma = gmm.covariances_[k]
        A_k = Sigma[2:, :2] @ np.linalg.inv(Sigma[:2, :2])
        A_matrices.append(A_k)
    
    # Enforce stability constraints using convex optimization
    stable_A_matrices = []
    for A_k in A_matrices:
        A_var = cp.Variable((2, 2))
        constraints = [A_var + A_var.T << -1e-6 * np.eye(2)]
        prob = cp.Problem(cp.Minimize(cp.norm(A_var - A_k, 'fro')), constraints)
        
        # Use SCS solver with parameters optimized for speed
        prob.solve(solver=cp.SCS, eps=1e-3, max_iters=500, verbose=False)
        stable_A_matrices.append(A_var.value)
    
    return gmm, stable_A_matrices, goal

def query_velocity(gmm, stable_A_matrices, target, query_pos):
    """
    Query velocity at current position using learned SEDS model.
    
    Args:
        gmm: Fitted Gaussian Mixture Model from learn_seds
        stable_A_matrices: Stable A matrices from learn_seds
        target: Goal point from learn_seds
        query_pos: Current position [x, y]
        
    Returns:
        velocity: Computed velocity vector [vx, vy]
    """
    # Create 4D query vector with zero velocity placeholder
    query_4d = np.hstack((query_pos, [0, 0]))
    
    # Get GMM component responsibilities at this position
    responsibilities = gmm.predict_proba(query_4d.reshape(1, -1))[0]
    
    # Compute weighted velocity from all components
    velocity = np.zeros(2)
    for k, gamma_k in enumerate(responsibilities):
        displacement = query_pos - target
        velocity += gamma_k * (stable_A_matrices[k] @ displacement)
    
    return velocity

# Example usage and visualization
if __name__ == "__main__":
    # Define three points
    start = np.array([1.0, 1.0])
    
    goal = np.array([0.0, 0.0])
    
    # Time the learning process
    start_time = time.time()
    gmm, stable_A, target = learn_seds(start,  goal)
    learn_time = time.time() - start_time
    print(f"Learning time: {learn_time:.4f} seconds")
    
    # Create grid for vector field visualization
    x = np.linspace(-0.25, 1.25, 15)
    y = np.linspace(-0.25, 1.25, 15)
    X, Y = np.meshgrid(x, y)
    grid_points = np.column_stack([X.ravel(), Y.ravel()])
    
    # Time velocity queries
    start_time = time.time()
    V = np.array([query_velocity(gmm, stable_A, target, p) for p in grid_points])
    query_time = time.time() - start_time
    print(f"Query time for {len(grid_points)} points: {query_time:.4f} seconds")
    print(f"Average query time: {query_time/len(grid_points)*1000:.4f} ms/point")
    
    # Normalize for visualization
    norms = np.linalg.norm(V, axis=1, keepdims=True)
    V_normalized = V / np.where(norms > 0, norms, 1)
    
    # Plot results
    plt.figure(figsize=(10, 8))
    plt.quiver(grid_points[:,0], grid_points[:,1], 
               V_normalized[:,0], V_normalized[:,1], 
               color='blue', scale=30)
               
    # Generate and plot the trajectory for visualization
    traj_points = np.vstack([
       
        np.linspace(start, goal, 50)
    ])
    plt.plot(traj_points[:,0], traj_points[:,1], 'r-', linewidth=2, label='Trajectory')
    
    # Plot points
    plt.scatter(start[0], start[1], c='red', s=100, marker='o', label='Start')
    
    plt.scatter(goal[0], goal[1], c='green', s=200, marker='*', label='Goal')
    
    plt.title(f'SEDS Velocity Field (Learn: {learn_time:.2f}s, Query: {query_time:.4f}s)')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.show()
