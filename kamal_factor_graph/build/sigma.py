import subprocess
import numpy as np
from scipy.optimize import minimize
from scipy.optimize import Bounds
from scipy.optimize import differential_evolution

def run_cpp_program(x):

    params = [str(xi) for xi in x]

    print(['./MyGTSAM'] + params)
    result = subprocess.run(['./MyGTSAM'] + params, 
                            capture_output=True, text=True)

    if result.returncode != 0:
        print(f"Error running C++ program: {result.stderr}")
        return np.inf

    print(f"C++ program output: {result.stdout}")
    
    return float(result.stdout.strip())

def objective_function(x):
    return run_cpp_program(x)

initial_guess = [0.09, 0.02, 0.02, 0.015, 0.01, 0.001]

lower_bounds = [0.001, 0.001]
upper_bounds = [0.09, 0.09]

bounds = [(low, high) for low, high in zip(lower_bounds, upper_bounds)]

result = differential_evolution(objective_function, bounds)

if result.success:
    print("Optimal parameters found:", result.x)
    print("Minimum error:", result.fun)
else:
    print("Optimization failed:", result.message)
