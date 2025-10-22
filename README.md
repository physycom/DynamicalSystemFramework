# DynamicalSystemFramework
[![Latest Release](https://img.shields.io/github/v/release/physycom/DynamicalSystemFramework)](https://github.com/physycom/DynamicalSystemFramework/releases/latest)
[![PyPI version](https://img.shields.io/pypi/v/dsf-mobility)](https://pypi.org/project/dsf-mobility/)
[![Standard](https://img.shields.io/badge/C%2B%2B-20/23-blue.svg)](https://en.wikipedia.org/wiki/C%2B%2B#Standardization)
[![TBB](https://img.shields.io/badge/TBB-2021.11.0-blue.svg)](https://github.com/oneapi-src/oneTBB)
[![SPDLOG](https://img.shields.io/badge/spdlog-1.12.0-blue.svg)](https://github.com/gabime/spdlog)
[![CSV](https://img.shields.io/badge/rapidcsv-8.89-blue.svg)](https://github.com/d99kris/rapidcsv)
[![JSON](https://img.shields.io/badge/simdjson-3.6.4-blue.svg)](https://github.com/simdjson/simdjson)
[![codecov](https://codecov.io/gh/physycom/DynamicalSystemFramework/graph/badge.svg?token=JV53J6IUJ3)](https://codecov.io/gh/physycom/DynamicalSystemFramework)

The aim of this project is to rework the original [Traffic Flow Dynamics Model](https://github.com/Grufoony/TrafficFlowDynamicsModel).
This rework consists of a full code rewriting, in order to implement more features (like *intersections*) and get advantage from the latest C++ updates.

## Table of Contents
- [Installation](#installation)
- [Installation (from source)](#installation-from-source)
- [Testing](#testing)
- [Benchmarking](#benchmarking)
- [Citing](#citing)
- [Bibliography](#bibliography)

## Installation
The library is available on `PyPI`:
```shell
pip install dsf-mobility
```

To check the installation you can simply run
```python
import dsf

print(dsf.__version__)
```

## Installation (from source)

### Requirements
The project requires `C++20` or greater, `cmake`, `tbb` `simdjson`, and `spdlog` (with `fmt`).
To install requirements on Ubuntu:
```shell
sudo apt install libtbb-dev libspdlog-dev libsimdjson-dev cmake
```
To install requirements on macOS:
```shell
brew install tbb simdjson spdlog cmake
```

Utilities are written in python. To install their dependencies:
```shell
pip install -r ./requirements.txt
```
### Installation (C++)
The library can be installed using CMake. To build and install the project in the default folder run:
```shell
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)
sudo cmake --install build
```
Otherwise, it is possible to customize the installation path:
```shell
cmake -B build -DCMAKE_INSTALL_PREFIX=/path/to/install
```
then building and installing it (eventually in sudo mode) with:
```shell
cmake --build build
cmake --install build
```

## Installation (Python)
If you want to use the library from Python, you can build the Python bindings using [pybind11](https://github.com/pybind/pybind11). Make sure you have Doxygen installed to generate the docstrings:
```shell
sudo apt install doxygen
```

Then, the installation is automatic via `pip`:
```shell
pip install .
```

After installation, you should be able to import the module in Python:
```python
import dsf

print(dsf.__version__)
```

If you encounter issues, ensure that the installation path is in your `PYTHONPATH` environment variable.

## Testing
This project uses [Doctest](https://github.com/doctest/doctest) for testing.

To compile tests run:
```shell
cd test
cmake -B build && make -C build -j$(nproc)
```
To run all the tests together use the command:
```shell
./dsf_tests.out
```

## Benchmarking
Some functionalities of the library have been benchmarked in order to assess their efficiency.  
The benchmarks are performed using a small toolkit developed by @sbaldu, in order to keep them simple and
without needing to rely on large external libraries.  
To compile the benchmarks use the commands:
```shell
cd benchmark
cmake -B build && make -C build
```
To run all the benchmarks together use the command:
```shell
for f in ./*.out ; do ./$f ; done
```

## Citing

```BibTex
@misc{DSF,
  author = {Berselli, Gregorio and Balducci, Simone},
  title = {Framework for modelling dynamical complex systems.},
  year = {2023},
  url = {https://github.com/physycom/DynamicalSystemFramework},
  publisher = {GitHub},
  howpublished = {\url{https://github.com/physycom/DynamicalSystemFramework}}
}
```

## Bibliography
- **Mungai, Veronica** (2024) *Studio dell'ottimizzazione di una rete semaforica*. University of Bologna, Bachelor's Degree in Physics [L-DM270]. [Link to Thesis](https://amslaurea.unibo.it/id/eprint/32525/).
- **Berselli, Gregorio** (2024) *Advanced queuing traffic model for accurate congestion forecasting and management*. University of Bologna, Master's Degree in Physics [LM-DM270]. [Link to Thesis](https://amslaurea.unibo.it/id/eprint/32191/).
- **Berselli, Gregorio** (2022) *Modelli di traffico per la formazione della congestione su una rete stradale*. University of Bologna, Bachelor's Degree in Physics [L-DM270]. [Link to Thesis](https://amslaurea.unibo.it/id/eprint/26332/).