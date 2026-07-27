---
layout: page
title: Build
permalink: /build/
---

To build GTSAM from source, clone or download the latest release from the [GTSAM GitHub repository](https://github.com/borglab/gtsam). The current stable release is 4.2, while the main development line is in pre-4.3 mode.

## Quick Start

From the repository root, use an out-of-source build:

```sh
cmake -S . -B build
cmake --build build --target check
cmake --build build --target install
```

`check` is optional, but recommended when you are validating a local build.

## Supported Configurations

| Current baseline guidance | Minimum recommendation |
| --- | --- |
| Linux | clang-11 or gcc-9 |
| macOS | Xcode 14.2 or newer |
| Windows | MSVC 14.2 or newer |

## Required Dependencies

Install these first:

1. [CMake](https://cmake.org/download/) 3.10 or newer
2. A current C++ toolchain for your platform

Ubuntu package:

```sh
sudo apt-get install cmake
```

## Optional Boost Dependency

Boost is now optional. Two CMake flags govern its use:

- `GTSAM_USE_BOOST_FEATURES`
- `GTSAM_ENABLE_BOOST_SERIALIZATION`

If either of those is `ON`, install [Boost](https://www.boost.org/users/download/) 1.70 or newer.

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

To use MKL on Linux, Intel provides installation guidance through its package repositories. If you are building the Python wrapper, you may also need:

```sh
source /opt/intel/mkl/bin/mklvars.sh intel64
export LD_PRELOAD="$LD_PRELOAD:/opt/intel/mkl/lib/intel64/libmkl_core.so:/opt/intel/mkl/lib/intel64/libmkl_sequential.so"
```

Then pass:

```sh
cmake -DGTSAM_WITH_EIGEN_MKL=ON ..
```

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

Set the install location:

```sh
cmake -DCMAKE_INSTALL_PREFIX:PATH=$HOME ..
```

### `GTSAM_TOOLBOX_INSTALL_PATH`

Set the MATLAB toolbox install path:

```sh
cmake -DGTSAM_TOOLBOX_INSTALL_PATH:PATH=$HOME/toolbox ..
```

### `GTSAM_BUILD_CONVENIENCE_LIBRARIES`

```sh
cmake -DGTSAM_BUILD_CONVENIENCE_LIBRARIES:OPTION=ON ..
```

- `ON` (default): faster for developers iterating on tests
- `OFF`: avoids rebuilding the full library twice, better for most users

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

If you need a specific interpreter version, add `-DGTSAM_PYTHON_VERSION=<version>`.

### `GTSAM_USE_BOOST_FEATURES` and `GTSAM_ENABLE_BOOST_SERIALIZATION`

These flags control the optional Boost dependency. If both are `OFF`, GTSAM can be built without Boost.

## Debugging Tips

GTSAM makes extensive use of debug assertions, so development work should usually happen in `Debug` mode. Switch back to `Release` when benchmarking or running finished code.

Another useful option is `_GLIBCXX_DEBUG`, which enables additional standard library checks. If you use it to compile GTSAM, anything linking against GTSAM must also use it.

## Performance Tips

1. Use `Release` mode for production workloads.
2. Enable TBB on multi-core systems and benchmark with and without it.
3. Consider `-march=native` in `GTSAM_CMAKE_CXX_FLAGS` if portability is not a concern.
4. Only enable MKL if you have measured a real benefit.
5. If you use TBB and memory growth is a concern, try `-DGTSAM_TBB_BOUNDED_MEMORY_GROWTH=ON`.

## API Documentation

For API documentation, see:

- [C++ API docs](/doxygen/)
- [Python API docs](https://borglab.github.io/gtsam/)

Wrapper-specific build details are documented in the upstream repository:

- [Python wrapper README](https://github.com/borglab/gtsam/blob/develop/python/README.md)
- [MATLAB wrapper README](https://github.com/borglab/gtsam/blob/develop/matlab/README.md)
