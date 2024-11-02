# Graph-Based Simultaneous Localization and Calibration of Planar Cable-Driven Parallel Robots

[![ICRoM](https://img.shields.io/badge/Conference-ICRoM-orange)](https://icrom.ir/)
[![C++](https://img.shields.io/badge/Language-C++-blue.svg)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Language-Python-green.svg)](https://www.python.org/)
[![GTSAM](https://img.shields.io/badge/Library-GTSAM-brightgreen.svg)](https://gtsam.org/)
[![Symforce](https://img.shields.io/badge/Library-Symforce-yellow.svg)](https://symforce.org/)

## Introduction
This repository contains the code and supplementary material for the paper: "*Graph-Based Simultaneous Localization and Calibration of Planar Cable-Driven Parallel Robots*". The paper addresses issue of deployability of planar **C**able-**D**riven **P**arallel **R**obots (**CDPR**s) by fusing state estimates with joint measurements, resulting in a unified framework for both calibration and localization. This is achieved through a factor-graph formulation that integrates these processes. The proposed method has been implemented on Kamal-ol-Molk planar CDPR, developed by [ARAS](https://aras.kntu.ac.ir/) Robotic group at [K.N. Toosi University](https://kntu.ac.ir/).

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Code Structure](#code-structure)
- [Citing](#citing)
- [License](#license)

## Installation

Before using this code, ensure you have installed the `gtsam` and `symforce` libraries. Follow these steps:

1. Install `gtsam`:
   Refer to the official [GTSAM](https://github.com/borglab/gtsam) repository for detailed instructions on installing GTSAM for your platform.

2. Install `symforce`:
   You can install `symforce` using the provided tutorial on their [repo](https://github.com/symforce-org/symforce).

After installing these libraries, clone the repository:
```bash
git clone https://github.com/ali-sharifi-23/Kamal_SLAC.git
cd Kamal_SLAC
```

## Usage
The code is divided into two main sections. The first section involves generating the model of the factors and their derivatives in C++ header files (refer to the [documentation](system_model/README.md) for more details).
