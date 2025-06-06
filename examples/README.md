# DSM - Examples
Here are reported some example studies using the dsf framework.
Many of these simulations were used as thesis work.

To compile all simulations, use cmake inside the *examples* folder:
```shell
cmake -B build -DCMAKE_BUILD_TYPE=Release && make -C build
```
If anything goes wrong, try to build the example in debug mode:
```shell
cmake -B build -DCMAKE_BUILD_TYPE=Debug && make -C build
```

## Simulation files
C++ files (`cpp`) are simulation files and need to be compiled.

**NOTE**: you may have to modify some hard-coded parameters and recompile to set certain variables.

### Slow Charge
Aim of these simulations is to load the road network in an adiabatic way, i.e. slowly increasing the mean density over streets.
You can try it both using all roundabouts intersections (RB case) or traffic lights (TL case).
To run the simulation you can use the Makefile:
```shell
make slow_charge_tl
```
### Stalingrado
Aim of this program is to make a realistic simulation of *Via Stalingrado*, in Bologna, which is a street full of traffic lights that have high influence on the traffic.
Also in this case, once compiled one can run it using the Makefile:
```shell
make stalingrado
```