# DynamicalSystemFramework
[![codecov](https://codecov.io/github/sbaldu/DynamicalSystemFramework/graph/badge.svg?token=JV53J6IUJ3)](https://codecov.io/github/sbaldu/DynamicalSystemFramework)

The aim of this project is to rework the original [Traffic Flow Dynamics Model](https://github.com/Grufoony/TrafficFlowDynamicsModel).
This rework consists of a full code rewriting, in order to implement more features (like *intersections*) and get advantage from the latest C++ updates.

## Requirements

The project only requires `C++20` or greater and `cmake`.

Utilities are written in python. To install their dependencies:
```shell
pip install -r ./requirements.txt
```

## Installation
The library can be installed using CMake. To do this build it with the commands:
```shell
cmake -B build -DCMAKE_INSTALL_PREFIX=/path/to/install
cmake --build build
```
and then install it with:
```shell
cmake --install build
```

## Testing
To compile tests one can run:
```shell
cd test
cmake -B build && make -C build
```
To run all the tests together use the command:
```shell
./dsm_tests.out
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
@misc{DSM,
  author = {Berselli, Gregorio and Balducci, Simone},
  title = {Framework for modelling dynamical complex systems.},
  year = {2023},
  url = {https://github.com/sbaldu/DynamicalSystemFramework},
  publisher = {GitHub},
  howpublished = {\url{https://github.com/sbaldu/DynamicalSystemFramework}}
}
```
