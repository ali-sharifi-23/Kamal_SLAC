import subprocess
import numpy as np
from scipy.optimize import minimize
from scipy.optimize import Bounds
from scipy.optimize import differential_evolution

# Function to run the C++ program and return the error
def run_cpp_program(x):
    # Convert parameters to strings
    params = [str(xi) for xi in x]
    
    # Call the C++ executable with subprocess
    print(['./MyGTSAM'] + params)
    result = subprocess.run(['./MyGTSAM'] + params, 
                            capture_output=True, text=True)
    
    # If there is an error, return a large number (simulating a failure case)
    if result.returncode != 0:
        print(f"Error running C++ program: {result.stderr}")
        return np.inf

    # Print stdout for debugging
    print(f"C++ program output: {result.stdout}")
    
    # Get the output (error) and return it as a float
    return float(result.stdout.strip())

# Objective function for optimization (we want to minimize the error)
def objective_function(x):
    return run_cpp_program(x)

# Initial guess for parameters
initial_guess = [0.09, 0.02, 0.02, 0.015, 0.01, 0.001]

# Define the bounds for each parameter
# Example: lower bound is 0 for all parameters, upper bound is 10 for all
lower_bounds = [0.001, 0.001]
upper_bounds = [0.09, 0.09]

# Set bounds using scipy's Bounds class
# bounds = Bounds(lower_bounds, upper_bounds)

bounds = [(low, high) for low, high in zip(lower_bounds, upper_bounds)]

# Use scipy.optimize to minimize the objective function with bounds
result = differential_evolution(objective_function, bounds)
# result = minimize(objective_function, initial_guess, method='L-BFGS-B', bounds=bounds, options={'eps': 1e-4, 'ftol': 1e-8, 'gtol': 1e-8})

# Print the result
if result.success:
    print("Optimal parameters found:", result.x)
    print("Minimum error:", result.fun)
else:
    print("Optimization failed:", result.message)
