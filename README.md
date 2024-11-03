# Graph-Based Simultaneous Localization and Calibration for Planar Cable-Driven Parallel Robots

<!-- [![ICRoM](https://img.shields.io/badge/Conference-ICRoM-orange)](https://icrom.ir/) -->
[![C++](https://img.shields.io/badge/Language-C++-blue.svg)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Language-Python-green.svg)](https://www.python.org/)
[![GTSAM](https://img.shields.io/badge/Library-GTSAM-brightgreen.svg)](https://gtsam.org/)
[![Symforce](https://img.shields.io/badge/Library-Symforce-yellow.svg)](https://symforce.org/)

## Overview
This repository contains the code and supplementary material for the paper: "*Graph-Based Simultaneous Localization and Calibration of Planar Cable-Driven Parallel Robots*". The paper addresses issue of deployability of planar **C**able-**D**riven **P**arallel **R**obots (**CDPR**s) by fusing state estimates with joint measurements, resulting in a unified framework for both calibration and localization. This is achieved through a factor-graph formulation that integrates these processes. The proposed method has been implemented on Kamal-ol-Molk planar CDPR, developed by [ARAS](https://aras.kntu.ac.ir/) Robotic group at [K.N. Toosi University](https://kntu.ac.ir/).

<div align="center">
    <img src="Kamal Structure.jpg" width="500"/>
    <p><em>Fig. 1.&emsp;The structure of the Kamal-ol-Molk planar cable-driven parallel robot.</em></p>
</div>

## Table of Contents
- [Prerequisites](#prerequisites)
- [Code Structure](#code-structure)
- [Results](#results)
- [Citation](#citation)

## Prerequisites

Before using this code, ensure you have installed the `gtsam` and `symforce` libraries. Follow these steps:

1. Install `gtsam`:
   Refer to the official [GTSAM](https://github.com/borglab/gtsam) repository for detailed instructions on installing GTSAM for your platform.

2. Install `symforce`:
   You can install `symforce` using the provided tutorial on [this](https://github.com/symforce-org/symforce) repository.


## Code Structure
The code is divided into two main sections as follows:

- **Model Generation:** This section, found in the `system_model` directory, is responsible for generating the cost functions and their derivatives (refer to the [document](system_model/README.md) for more information).
- **Factor Graph Coding:** The factor graph formulation is implemented in the `kamal_factor_graph` directory (see the [documentation](kamal_factor_graph/README.md) for more details).

## Results
The root mean square error (RMSE) for the **V**isual **K**inematic **F**usion (**VKF**), **F**orward **K**inematics (**FK**) and **V**isual **O**dometry (**VO**) in estimating the end-effector position has been mentioned in the table below:

||VKF (ours)|FK|VO|
|--|--|--|--|
|RMSE (m)|**$0.003$**|$0.004$|$0.007$|

These results demonstrate that VKF has improved accuracy by $25$% compared to kinematics alone and by $57$% compared to scale-aligned VO.

Furthermore, we assess the discrepancy between the cable length changes computed from the robot's inverse kinematics, using the optimized anchor points, and the values recorded by the encoders in order to verify the kinematic calibration. The results indicated that the RMSE for the first, second, and third cables was $0.003m$, $0.005m$, and $0.003m$, respectively.

## Citation
...
