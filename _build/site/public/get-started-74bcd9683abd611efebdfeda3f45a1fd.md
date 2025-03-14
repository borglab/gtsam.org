---
title: Get Started
---

Welcome to GTSAM! Please follow this guide to setup GTSAM on your platform.

<br><br>

# Prerequisites

First, make sure to use a compatible software configuration:
```{csv-table}
:header: "Tested Compilers", "Tested Systems"

"GCC 4.2-7.3", "Ubuntu 16.04 - 18.04"
"OS X Clang 2.9-10.0", "MacOS 10.6 - 10.14, 15.2-15.3"
"OS X GCC 4.2", "Windows 7, 8, 8.1, 10"
"MSVC 2017", ""
```

Then install the following libraries on your system:

1. [Boost](http://www.boost.org/users/download/) $\geq$ **1.43** *(install through Linux repositories or MacPorts)*
2. [CMake](http://www.cmake.org/cmake/resources/software.html) $\geq$ **3.0**
     - (MacOS) Support for **XCode 4.3** command line tools requires **CMake 2.8.8 or higher**

:::{note} Additional optional libraries
:class: dropdown
*These additional libraries are used automatically—if findable by CMake.*

3. Intel Threaded Building Blocks [(oneTBB)](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onetbb.html#gs.kym212)
   - If oneTBB is installed and detectable by CMake, GTSAM will use it automatically. Ensure that CMake prints `Use Intel TBB : Yes`.
   - To disable the use of TBB, disable the CMake flag `GTSAM_WITH_TBB` *(enabled by default)*.
   - *On **Ubuntu**, TBB may be installed from the Ubuntu repositories, and for other platforms it may be downloaded from [here](https://github.com/uxlfoundation/oneTBB).*
4. Intel Math Kernel Library [(MKL)](http://software.intel.com/en-us/intel-mkl)
    - ***MKL may not provide a speedup in all cases.*** Make sure to benchmark your problem with and without MKL.
    - GTSAM may be configured to use MKL by toggling `GTSAM_WITH_EIGEN_MKL` and `GTSAM_WITH_EIGEN_MKL_OPENMP` to `ON`. However, best performance is usually achieved with MKL *disabled*. We therefore advise you to benchmark your problem before using MKL.
    - *See [Installing MKL on Linux](/Content/MKL-linux) for more installation information.*
:::

:::{hint} Installing libraries on **Ubuntu**
:class: dropdown
```bash
sudo apt-get install libboost-all-dev   # Boost
sudo apt-get install cmake              # Cmake
```

For optional libraries:
- TBB: `sudo apt-get install libtbb-dev`
- MKL: [install using APT](https://software.intel.com/en-us/articles/installing-intel-free-libs-and-python-apt-repo)
:::

<br><br>

# Install GTSAM

::::{tab-set}
:::{tab-item} from Source
Clone or download the latest release from the [GTSAM Github repo](https://github.com/borglab/gtsam).

Then execute the commands below in the root directory for an out-of-source build.

```bash
#!bash
$ mkdir build
$ cd build
$ cmake ..
$ make check    # optional, runs unit tests
$ make install
```

This will build the library and unit tests (to the default system install path), run all of the unit tests, and then install the library itself.
:::

:::{tab-item} from Ubuntu PPA
GTSAM can be installed on Ubuntu via [these PPA repositories](https://launchpad.net/~borglab) as well.
As of November 2020, packages for Xenial (u16.04), Bionic (u18.04), and Focal (u20.04) are published.

Follow the code below to add PPA for your preferred branch:


**Latest 4.x stable release**
```bash
# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev
```


**Nightly builds (develop branch)**
```bash
# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-develop
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev
```
:::

:::{tab-item} from Arch Linux AUR
```{warning} Installing GTSAM on Arch Linux has not been tested by GTSAM developers.
```

GTSAM is available in the Arch User Repository
([AUR](https://wiki.archlinux.org/index.php/Arch_User_Repository)) as
[`gtsam`](https://aur.archlinux.org/packages/gtsam/).

You can manually install the package by following the instructions on the
[Arch Wiki](https://wiki.archlinux.org/index.php/Arch_User_Repository#Installing_packages)
or use an [AUR helper](https://wiki.archlinux.org/index.php/AUR_helpers) like
[`yay`](https://aur.archlinux.org/packages/yay/)
(recommended for ease of install).

It is also recommended to use the
[arch4edu](https://wiki.archlinux.org/index.php/Unofficial_user_repositories#arch4edu)
repository. They are hosting many packages related to education and research,
including robotics such as ROS. Adding a repository allows for you to install
binaries of packages, instead of compiling them from source.
This will greatly speed up your installation time. Visit [here](https://github.com/arch4edu/arch4edu/wiki/Add-arch4edu-to-your-Archlinux) to add and use arch4edu. 

**Install GTSAM**
```sh
yay -S gtsam
```

or

**Install GTSAM with Intel Accelerations**

```sh
yay -S gtsam-mkl
```

To discuss any issues related to this package refer to the comments section on
the AUR page of `gtsam` [here](https://aur.archlinux.org/packages/gtsam/).
:::
::::

## Documentation

GTSAM has [Doxygen](http://www.doxygen.nl/) documentation. To generate, run `make doc` from your build directory, or refer to the [statically generated version on this website](https://gtsam.org/doxygen/).

:::{note} ***For developers:*** Debug Assertions
:class: dropdown
GTSAM makes extensive use of debug assertions, so we highly recommend you work in **Debug mode** while developing (which is not enabled by default). 

Likewise, it is imperative that you switch back to release mode when running finished code and for timing. ***GTSAM will run up to 10x faster in Release mode!***

[See the end of this site for additional debugging tips.](#debugging-tips)
:::

<br><br>

# CMake Configuration Options and Details

GTSAM has a number of options that can be configured, which is best done with
one of the following:

  - `ccmake`:      the [curses GUI](https://manpages.debian.org/testing/cmake-curses-gui/ccmake.1.en.html) for cmake
  - `cmake-gui`:   a real GUI for cmake

## Important options

:::{glossary}
CMAKE_BUILD_TYPE
: We support several build configurations for GTSAM (case insensitive)
  ```
  cmake -DCMAKE_BUILD_TYPE=[Option] ..
  ```
  - `Debug`: All error checking options on, no optimization. Use for development of new features and fixing issues.
  - `Release`: Optimizations turned on, no debug symbols.
  - `Timing`: Adds ENABLE_TIMING flag to provide statistics on operation
  - `Profiling`: Standard configuration for use during profiling
  - `RelWithDebInfo`: Same as Release, but with the - g flag
  for debug symbols

CMAKE_INSTALL_PREFIX
: The install folder. The default is typically `/usr/local/`.
  To configure to install to your home directory, you could execute:
  ```
  cmake -DCMAKE_INSTALL_PREFIX:PATH=$HOME ..
  ```

GTSAM_TOOLBOX_INSTALL_PATH
: The Matlab toolbox will be installed in a subdirectory of this folder, called `gtsam`.
  ```
  cmake -DGTSAM_TOOLBOX_INSTALL_PATH:PATH=$HOME/toolbox ..
  ```

GTSAM_BUILD_CONVENIENCE_LIBRARIES
: This is a build option to allow for tests in subfolders to be linked against convenience libraries rather than the full `libgtsam`.
  Set with the command line as follows:
  ```
  cmake -DGTSAM_BUILD_CONVENIENCE_LIBRARIES:OPTION=ON ..
  ```
    - `ON` (Default): This builds convenience libraries and links tests against them.
      - This option is suggested for gtsam developers, as it is possible to build and run tests without first building the rest of the library, and speeds up compilation for a single test. The downside of this option is that it will build the entire library again to build the full libgtsam library, so build / install will be slower.
    - `OFF`: This will build all of `libgtsam` before any of the tests, and then link all of the tests at once.
      - This option is best
    for users of GTSAM, as it avoids rebuilding the entirety of gtsam an extra time.

GTSAM_BUILD_UNSTABLE
: Enable build and install for `libgtsam_unstable` library.
  Set with the command line as follows:
  ```
  cmake -DGTSAM_BUILD_UNSTABLE:OPTION=ON ..
  ```
  - `ON` (Default): When enabled, `libgtsam_unstable` will be built and installed with the same options as libgtsam. In addition, if tests are enabled, the unit tests will be built as well. The Matlab toolbox will also be generated
  if the matlab toolbox is enabled, installing into a folder called `gtsam_unstable`.
  - `OFF`: If disabled, no `gtsam_unstable` code will be included in build or install.

MEX_COMMAND
: Path to the mex compiler. Defaults to assume the path is included in your shell's `PATH` environment variable. `mex` is installed with matlab at `$MATLABROOT/bin/mex`.

  The correct value for  `MATLABROOT` can be found by executing the command `matlabroot` in MATLAB.
:::

<br><br>

# Running the unit tests

 `make check` will build and run all of the tests. Note that the tests will only be built when using the "check" targets, to prevent `make install` from building the tests unnecessarily.

You can also run `make timing` to build all of the timing scripts.
To run check on a particular module only, run `make check.[subfolder]`, so to run just the geometry tests, run:
```
make check.geometry
```

Individual tests can be run by appending **.run** to the name of the test. For example, to run `testMatrix`, execute:
```
make testMatrix.run
```

<br><br>

# Performance

Here are some tips to get the best possible performance out of GTSAM.

1. Build in `Release` mode.
   - GTSAM will run up to ***10x faster*** compared to `Debug` mode.

1. Enable **TBB**.
   - On modern processors with multiple cores, this can easily speed up optimization 30-50%.
     - Please note that this may not be true for very small problems where the overhead of dispatching work to multiple threads outweighs the benefit. We recommend that you benchmark your problem with / without TBB.

2. Add `-march=native` to `GTSAM_CMAKE_CXX_FLAGS`. A performance gain of 25 - 30% can be expected on modern processors.
   - Note that this affects the portability of your executable. It may not run when copied to another system with older / different processor architecture. Also note that all dependent projects **must** be compiled with the same flag, or segfaults and other undefined behavior may result.

3. Possibly enable **MKL**.
   - Please note that our benchmarks have shown that this helps only in very limited cases, and actually hurts performance in the usual case. We therefore recommend that you do *not* enable MKL, unless you have benchmarked it on your problem and have verified that it improves performance.

# Debugging tips

Another useful debugging symbol is `_GLIBCXX_DEBUG`, which enables debug checks and safe containers in the standard C++ library and makes problems much easier to find.

```{warning} 
The native Snow Leopard g++ compiler / library contains a bug that makes it impossible to use `_GLIBCXX_DEBUG`. MacPorts g++ compilers do work with it though.

If `_GLIBCXX_DEBUG` is used to compile gtsam, anything that links against gtsam will need to be compiled with `_GLIBCXX_DEBUG` as well, due to the use of header - only Eigen.
```
