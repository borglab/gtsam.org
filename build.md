---
layout: page
title: Build
permalink: /build/
---

To build GTSAM from source, clone or download the [GTSAM 4.3.0 release](https://github.com/borglab/gtsam/releases/tag/4.3.0). The `develop` branch contains changes intended for the next release and may include API changes.

## Quick Start

Install Git, CMake, and a C++17 compiler first. From the repository root, use an out-of-source build. This Linux/macOS recipe disables the optional Boost features and installs under your home directory:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/.local" \
  -DGTSAM_USE_BOOST_FEATURES=OFF \
  -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF
cmake --build build --target check
cmake --build build --target install
```

`check` is optional, but recommended when you are validating a local build. If you choose a system install prefix instead, the install step may require administrator privileges; the build itself should not run as administrator. For GPU support in Python, use the separate [CUDA Python recipe](#cuda-with-python) below.

## Supported Configurations

| Current baseline guidance | Minimum recommendation |
| --- | --- |
| Linux | GCC 11, 13, 14, or 15; Clang 11, 14, or 16 |
| macOS | Xcode 16 |
| Windows | MSVC toolset 14.40 |

## Required Dependencies

Install these first:

1. [CMake](https://cmake.org/download/) 3.16 or newer
2. A C++17 toolchain for your platform

Ubuntu development packages:

```sh
sudo apt-get install build-essential cmake git
```

## Optional Boost Dependency

Boost is now optional. Two CMake flags govern its use:

- `GTSAM_USE_BOOST_FEATURES`
- `GTSAM_ENABLE_BOOST_SERIALIZATION`

Both default to `ON` in ordinary CMake builds; the quick-start recipe above explicitly disables them. If either is `ON`, install [Boost](https://www.boost.org/users/download/) 1.70 or newer before configuring.

Platform-specific guidance:

- macOS: `brew install boost`
- Ubuntu: `sudo apt-get install libboost-all-dev`
- Windows: prefer [vcpkg](https://github.com/microsoft/vcpkg)

## Optional Dependencies

### Intel TBB

If TBB is installed and detectable by CMake, GTSAM will use it automatically. Confirm that CMake prints `Use Intel TBB : Yes`.

- Disable it with `GTSAM_WITH_TBB=OFF`.
- On Ubuntu, install it with `sudo apt-get install libtbb-dev`.
- On other platforms, see [oneTBB](https://github.com/uxlfoundation/oneTBB).

### Intel MKL

GTSAM can be configured to use MKL with `GTSAM_WITH_EIGEN_MKL` and `GTSAM_WITH_EIGEN_MKL_OPENMP`, but it does not always improve performance. Benchmark your workload before enabling it.

Before configuring, initialize the oneAPI environment using the script appropriate to your installation. For the component directory layout, a typical system installation uses:

```sh
source /opt/intel/oneapi/setvars.sh
```

For the unified directory layout, use the versioned installation's `oneapi-vars.sh` instead. Paths depend on the installation; see [Intel's environment setup instructions](https://www.intel.com/content/www/us/en/docs/oneapi/programming-guide/2025-1/use-the-setvars-and-oneapi-vars-scripts-with-linux.html). Then enable MKL from the GTSAM source root:

```sh
cmake -S . -B build -DGTSAM_WITH_EIGEN_MKL=ON
```

No `LD_PRELOAD` workaround is required by this recipe.

## CUDA with Python

CUDA acceleration is **experimental and opt-in**. To use it from Python, **compile both GTSAM and its Python wrapper on a CUDA-equipped machine**. The standard GTSAM 4.3.0 PyPI wheels do not include `gtsam.cuda`; installing a CUDA toolkit or a Python CUDA package alongside those wheels does not enable the bindings.

### Prerequisites

The following recipe is for a Linux machine with an NVIDIA GPU:

- A GPU with compute capability 6.0 or newer, supported by your chosen CUDA toolkit, and a compatible NVIDIA driver. Newer toolkits can drop support for older GPUs.
- The CUDA **development toolkit**, including `nvcc`, headers, and libraries—not just a driver or runtime package. Follow [NVIDIA's Linux installation guide](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/) and use a host C++ compiler supported by that toolkit.
- Git, a C++17 toolchain, and CMake 3.24 or newer for the recipe below. GTSAM's CUDA minimum is 3.17; 3.24 adds native GPU-architecture selection.
- Python 3.11 or newer with its development headers, `venv`, and pip. On Ubuntu, install the matching `python3-dev` and `python3-venv` packages for your interpreter.

Check the GPU driver and toolkit in the shell you will use to build:

```sh
nvidia-smi
nvcc --version
```

The CUDA version reported by `nvidia-smi` describes driver compatibility; it does not establish that the development toolkit is installed. Use `nvcc --version` to check the compiler. This recipe is not for a Mac without an NVIDIA CUDA device; use a suitable GPU workstation or server instead.

### Build and install the wrapper

Use a fresh checkout and Python environment so an existing CPU-only wheel cannot mask the local build. These commands are for a Bash-compatible shell:

```sh
git clone --branch 4.3.0 --depth 1 https://github.com/borglab/gtsam.git gtsam-cuda
cd gtsam-cuda
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
python -m pip install -r python/dev_requirements.txt

cmake -S . -B build-cuda \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$VIRTUAL_ENV" \
  -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python" \
  -DPython_FIND_VIRTUALENV=ONLY \
  -DGTSAM_BUILD_PYTHON=ON \
  -DGTSAM_ENABLE_CUDA=ON \
  -DGTSAM_ENABLE_CUDSS=OFF \
  -DGTSAM_USE_BOOST_FEATURES=OFF \
  -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF
cmake --build build-cuda --parallel 6
cmake --install build-cuda
cmake --build build-cuda --target python-install
```

Check that CMake selects the active environment's Python interpreter and the intended CUDA compiler and GPU architecture before building. If several toolkits are installed, add `-DCMAKE_CUDA_COMPILER=/path/to/cuda/bin/nvcc` to the initial configure command. The default targets the GPU(s) on the build machine; to target other GPUs, specify their supported architecture numbers with `CMAKE_CUDA_ARCHITECTURES`.

This recipe builds the stable library and wrapper, without Boost features, unstable modules, tests, or example executables. It enables matrix-free PCG and the CUDA SfM dense-Cholesky backend; cuDSS is a separate option below. Reduce `--parallel 6` if compilation exhausts memory.

`cmake --install` installs the C++ libraries under the virtual environment; `python-install` installs the wrapper into the Python environment selected at configure time. No system-wide installation or `sudo pip` is needed. Keep the checkout, build directory, and environment in place: locally installed wrappers can retain build-tree library paths. This is a local installation, not a portable wheel.

### Verify the Python bindings

With the same environment active, check the imported package and both CUDA optimizer bindings:

```sh
python - <<'PY'
import sys
import gtsam
from importlib.metadata import version

print("Python:", sys.executable)
print("GTSAM:", version("gtsam"), gtsam.__file__)
assert hasattr(gtsam, "cuda"), "This GTSAM build has no CUDA bindings"
print(gtsam.cuda.SparseLevenbergMarquardtOptimizer)
print(gtsam.cuda.SfmLevenbergMarquardtOptimizer)
PY
```

This verifies the bindings, not GPU execution. If `gtsam.cuda` is absent, check the printed Python/package paths and the CMake cache: both `GTSAM_BUILD_PYTHON` and `GTSAM_ENABLE_CUDA` must be `ON`. Installing a standard wheel into this environment afterward can replace the CUDA-enabled wrapper.

For a GPU run, follow the [Sparse LM notebook](https://borglab.github.io/gtsam/sparselevenbergmarquardtoptimizer/) or [CUDA SfM guide](https://borglab.github.io/gtsam/cudasfmlevenbergmarquardtoptimizer/). The general sparse optimizer defaults to cuDSS, so with the PCG-only sparse build above, explicitly select PCG. For example, use these parameters with the notebook's graph and initial values:

```python
linear = gtsam.cuda.LinearSolverOptions()
linear.backend = gtsam.cuda.LinearSolverType.Pcg
params = gtsam.cuda.SparseLevenbergMarquardtParams()
params.linear = linear
params.fallbackOnUnsupported = False
```

Disabling fallback prevents an unsupported graph or runtime from silently continuing with CPU LM. After optimization, inspect `optimizer.result()` for backend and fallback diagnostics. The specialized SfM optimizer instead defaults to the CUDA dense-Cholesky backend.

### Optional cuDSS backend

cuDSS is a separate NVIDIA sparse-direct-solver library, not part of the CUDA toolkit. Install **cuDSS 0.8.0 or newer**, including its development headers and linkable library, compatible with your toolkit. See the [GTSAM cuDSS installation guide](https://github.com/borglab/gtsam/blob/4.3.0/doc/CUDA_LINEAR_SOLVERS.md#installing-cudss). Runtime-only pip packages are not sufficient to compile this backend.

From the same source directory and active environment, reconfigure the existing build, then rebuild and reinstall both components:

```sh
cmake -S . -B build-cuda \
  -DGTSAM_ENABLE_CUDSS=ON \
  -DCUDSS_ROOT=/path/to/cudss
cmake --build build-cuda --parallel 6
cmake --install build-cuda
cmake --build build-cuda --target python-install
```

Replace `/path/to/cudss` with your installation prefix, or omit `CUDSS_ROOT` if CMake already finds the correct installation. Confirm the reported cuDSS version and paths, and ensure the runtime library is visible to the system loader when using Python. The [upstream CUDA guide](https://github.com/borglab/gtsam/blob/4.3.0/doc/CUDA_LINEAR_SOLVERS.md) documents supported graphs, backend selection, and diagnostics.

## Ubuntu Packages and PPAs

Ubuntu users can either build from source or use the BorgLab Launchpad archives:

- [BorgLab Launchpad PPAs](https://launchpad.net/~borglab)
- [Nightly develop PPA](https://launchpad.net/~borglab/+archive/ubuntu/gtsam-develop)

PPAs are convenient, but they may lag the main repository or carry different package variants depending on the Ubuntu series. For the most current build options, source builds are the safest path.

## Arch Linux

GTSAM is also available in the [AUR](https://aur.archlinux.org/packages/gtsam/).

```sh
yay -S gtsam
```

For Intel-accelerated builds:

```sh
yay -S gtsam-mkl
```

## Running Tests

`check` builds and runs all tests. Tests are only built for the `check` targets so that `install` does not build them unnecessarily.

Examples:

- Configure first: `cmake -S . -B build`
- Run all tests: `cmake --build build --target check`
- Build timing targets: `cmake --build build --target timing`

If you are working directly with the generated Makefiles, the classic targets still work:

- `make check`
- `make check.geometry`
- `make testMatrix.run`

## Windows Notes

On Windows, the preferred modern route is CMake with Ninja from a Developer shell:

```powershell
cmake -S . -B build -G Ninja
cmake --build build --target check
cmake --build build --target install
```

Visual Studio builds are also supported, but require a recent Visual Studio installation with C++ tooling and a modern CMake.

## Important CMake Options

### `CMAKE_BUILD_TYPE`

```sh
cmake -DCMAKE_BUILD_TYPE=Debug ..
```

Supported values:

- `Debug`: full error checking, no optimization
- `Release`: optimized, no debug symbols
- `Timing`: enables timing statistics
- `Profiling`: intended for profiling runs
- `RelWithDebInfo`: release build with debug symbols

### `CMAKE_INSTALL_PREFIX`

Set a user-writable install location (from the source root):

```sh
cmake -S . -B build -DCMAKE_INSTALL_PREFIX="$HOME/.local"
```

### MATLAB toolbox

Enable the wrapper with `GTSAM_INSTALL_MATLAB_TOOLBOX=ON`; setting its install path alone does not build it. Point `Matlab_ROOT_DIR` at your MATLAB installation and optionally choose the toolbox destination:

```sh
cmake -S . -B build \
  -DGTSAM_INSTALL_MATLAB_TOOLBOX=ON \
  -DMatlab_ROOT_DIR=/path/to/MATLAB \
  -DGTSAM_TOOLBOX_INSTALL_PATH="$HOME/gtsam_toolbox"
```

See the [MATLAB wrapper README](https://github.com/borglab/gtsam/blob/4.3.0/matlab/README.md) for prerequisites, installation, and MATLAB path setup.

### `GTSAM_BUILD_UNSTABLE`

```sh
cmake -DGTSAM_BUILD_UNSTABLE:OPTION=ON ..
```

- `ON` (default): builds and installs `libgtsam_unstable`
- `OFF`: excludes unstable code from build and install

### `MEX_COMMAND`

Path to the MATLAB `mex` compiler. If `mex` is not already in `PATH`, point it at `$MATLABROOT/bin/mex`.

### `GTSAM_BUILD_PYTHON`

Enable the Python wrapper with:

```sh
cmake -S . -B build -DGTSAM_BUILD_PYTHON=1
```

Create and activate a Python environment first, and install `python/dev_requirements.txt` into it. Use `-DPython3_EXECUTABLE=/path/to/python` to select that interpreter; if you need a specific version, add `-DGTSAM_PYTHON_VERSION=<version>`. After building, the `python-install` target installs the wrapper into the interpreter selected at configuration time. The [Python wrapper README](https://github.com/borglab/gtsam/blob/4.3.0/python/README.md) has the complete CPU build instructions; the [CUDA recipe above](#cuda-with-python) adds GPU support.

### `GTSAM_USE_BOOST_FEATURES` and `GTSAM_ENABLE_BOOST_SERIALIZATION`

These flags control the optional Boost dependency. If both are `OFF`, GTSAM can be built without Boost.

## Debugging Tips

GTSAM makes extensive use of debug assertions, so development work should usually happen in `Debug` mode. Switch back to `Release` when benchmarking or running finished code.

Another useful option is `_GLIBCXX_DEBUG`, which enables additional standard library checks. If you use it to compile GTSAM, anything linking against GTSAM must also use it.

## Performance Tips

1. Use `Release` mode for production workloads.
2. Enable TBB on multi-core systems and benchmark with and without it.
3. Consider `-DGTSAM_BUILD_WITH_MARCH_NATIVE=ON` if portability is not a concern; the resulting binaries may not run on other CPU architectures.
4. Only enable MKL if you have measured a real benefit.
5. If you use TBB and memory growth is a concern, try `-DGTSAM_TBB_BOUNDED_MEMORY_GROWTH=ON`.

## API Documentation

For API documentation, see:

- [C++ API docs](/doxygen/)
- [Python API docs](https://borglab.github.io/gtsam/)

Wrapper-specific build details are documented in the upstream repository:

- [Python wrapper README](https://github.com/borglab/gtsam/blob/develop/python/README.md)
- [MATLAB wrapper README](https://github.com/borglab/gtsam/blob/develop/matlab/README.md)
