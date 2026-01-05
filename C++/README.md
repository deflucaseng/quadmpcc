# MPCC
This is a C++ implementation of the MPCC controller. The implementation is NOT the version which was used in the paper [Optimization‐based autonomous racing of 1:43 scale RC cars](https://onlinelibrary.wiley.com/doi/abs/10.1002/oca.2123) but a new version with some additional features inspired the work we did in this paper [AMZ Driverless: The Full Autonomous Racing System](https://arxiv.org/abs/1905.05150)

## Use With Full-Sized Car Model
The code in the master branch does not work well with the parameters of a full-sized car. However, I added a new branch (full-size) that uses a different SQP formulation which works for model parameters of a full-sized car. The simulation in this branch is performed for a formula student style car on a full-sized race track, following a pre-computed ideal line.

## Difference to Matlab implementation

### Solver
This version only supports hpipm as a solver. However, the goal is to add an [acados](https://github.com/acados/acados) qp interface, which would allow us to use the large list of solvers supported by acoados.

### Input rate cost and constraints - Dynamics
Instead of lifting the state and using the difference in inputs as new inputs, this version uses the continuous time approach where the new inputs are the rate of change of the inputs, similar to [AMZ Driverless: The Full Autonomous Racing System](https://arxiv.org/abs/1905.05150).

In detail we use the following dynamics,
<img src="https://github.com/alexliniger/MPCC/blob/master/Images/model_cpp.jpg" width="700" />

this also includes some changes in the notation, to match better the literature. Mainly the yaw rate is not `r` and the progress `s`. Thus, we have the following states and inputs,

<img src="https://github.com/alexliniger/MPCC/blob/master/Images/state_input_cpp.jpg" width="700" />

We also split up the force in x-direction into two components, the force at the wheel `F_r,x` and the friction force `F_fric`, which are defined as follows,
<img src="https://github.com/alexliniger/MPCC/blob/master/Images/forces_cpp.jpg" width="700" />

### Tire constraints
The C++ implementation adds the tire constraints used in [AMZ Driverless: The Full Autonomous Racing System](https://arxiv.org/abs/1905.05150). More precisely, I added a slip angle constraint for the front wheel (since the 1:43 scale cars are rear wheel drive and have no brakes), and a tire friction ellipse constraint for the rear wheel. Thus, the MPC problem the following three constraints, on top of state and input bounds,
<img src="https://github.com/alexliniger/MPCC/blob/master/Images/constraints_cpp.jpg" width="700" />

Note that if the car is all wheel drive or has brakes at the front wheel, also a tire ellipse constraint should be used for the front tire.

### Beta Cost
We added an additional regularization cost, which penalizes high sideslip angles. This second regularization cost augments the small cost on the yaw rate. These regularization costs force the car to behave more reasonably and help the NLP to converge better.
<img src="https://github.com/alexliniger/MPCC/blob/master/Images/cost_cpp.jpg" width="700" />

### Obstacle Avoidance
There is no obstacle avoidance available yet in the C++ version

### Options
Currently, only one track and car model is implemented. However, adapting the parameters only requires changing the json files in the Params folder.

## Installation & Setup

### Docker (Recommended)
The easiest way to get started is using Docker, which comes pre-configured with all dependencies.

1. **Run the container**:
   ```bash
   bash docker_run.sh
   ```
   This script will mount the repository, install pre-commit hooks, and fetch dependencies via `gitman` automatically on the first run.

### Local Installation
If you prefer to run locally, you will need `cmake`, `git`, `python3`, and `gitman`.

1. **Install Gitman & Pre-commit**:
   ```bash
   pip install gitman pre-commit
   ```

2. **Fetch Dependencies**:
   ```bash
   gitman install
   ```
   This will download all external libraries into the `External/` directory.

3. **Install Pre-commit Hooks**:
   ```bash
   pre-commit install --config .pre-commit-config.yaml
   ```

## Building

Once dependencies are fetched, build the project using CMake.
```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```
> [!TIP]
> If you build both inside and outside of Docker, it is recommended to use different build directories (e.g., `build` and `build_docker`) to avoid CMake cache conflicts.
> [!NOTE]
> If you encounter Eigen-related compilation errors with GCC 13, the build system is configured to automatically fetch a compatible version.
> [!NOTE]
> If you use a different target than X64_INTEL_HASWELL, you need to set the target in the CMake command line, e.g., `cmake .. -DTARGET=ARMV8A_APPLE_M1`.

## Usage

### 1. Generating the Model (ADCodeGen)
If you have modified the model equations or parameters, you need to re-generate the C++ code for the dynamics:
```bash
# Inside the build directory
./ADCodeGen/ADCodeGen
```
This tool reads the configuration files from `Params/` and generates optimized code in the `Generated/` folder.

### 2. Running the MPCC Controller
Run the main executable to start the simulation:
```bash
# Inside the build directory
./MPCC
```
The output will be saved to the `visualization/` directory.

### TODO

There are still several things that should be added to the project. Most of them are marked with TODO in the code.
