# Factor Graph Coding
The C++ code for the factor graph of the proposed problem is implemented in this section.
- `main.cpp`: This file contains the code for implementing the factor graph. To compile and run the code, follow these steps:
    1. Navigate to the `build` directory:
        ```bash
        cd build
        ```
    2. Generate the build files using CMake:
        ```bash
        cmake ..
        ```
    3. Compile the project and run the executable:
        ```bash
        make && ./KamalSLAC <arg1> <arg2>
        ```
        `<arg1>`: The value of the standard deviation of the cable length offsets.
        `<arg2>`: The number of position samples to include in the optimization.
        Replace `<arg1>` and `<arg2>` with appropriate values.
<br>
- `verification/init.ipynb`: This notebook perturbs the initial estimates of the anchor point locations and runs the compiled executable from the `build` directory to perform the optimization. It then receives and plots the optimization results in a figure for visualization.
<br>
- `verification/calib.ipynb`: This notebook is designed to compare the optimization results with the ground truth values to verify the calibration process.