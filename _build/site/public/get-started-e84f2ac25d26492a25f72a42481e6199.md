---
title: Get Started
---
FirstMake sure you are using compatible OS and compiler

# Prerequisites

## Compatible Software Configuration

```{csv-table}
:header: "Tested Compilers", "Tested Systems"

"GCC 4.2-7.3", "Ubuntu 16.04 - 18.04"
"OS X Clang 2.9-10.0", "MacOS 10.6 - 10.14, 15.2-15.3"
"OS X GCC 4.2", "Windows 7, 8, 8.1, 10"
"MSVC 2017", ""
```

## Libraries

GTSAM requires the following libraries to be installed on your system:
   
   1. [Boost](http://www.boost.org/users/download/) $\geq$ **1.43** *(install through Linux repositories or MacPorts)*
   2. [CMake](http://www.cmake.org/cmake/resources/software.html) $\geq$ **3.0**
   - (MacOS): Support for **XCode 4.3** command line tools requires **CMake 2.8.8 or higher**

:::{note} Optional libraries *(used automatically if findable by CMake)*
:class: dropdown
4. [Intel Threaded Building Blocks (TBB)](http://www.threadingbuildingblocks.org/)
   - If TBB is installed and detectable by CMake GTSAM will use it automatically. Ensure that CMake prints `Use Intel TBB : Yes`.
    - To disable the use of TBB, disable the CMake flag `GTSAM_WITH_TBB` (enabled by default).
    - On Ubuntu, TBB may be installed from the Ubuntu repositories, and for other platforms it may be downloaded from [here](https://www.threadingbuildingblocks.org/).
5. [Intel Math Kernel Library (MKL)](http://software.intel.com/en-us/intel-mkl)
    - *See [Build](/Content/build-project) for more installation information.*
    - ***MKL may not provide a speedup in all cases.*** Make sure to benchmark your problem with and without MKL.
    - GTSAM may be configured to use MKL by toggling `GTSAM_WITH_EIGEN_MKL` and `GTSAM_WITH_EIGEN_MKL_OPENMP` to `ON`.
    However, best performance is usually achieved with MKL *disabled*. We therefore advise you to benchmark your problem before using MKL.
:::

:::{hint} (Ubuntu) Library installation
:class: dropdown
```bash
sudo apt-get install libboost-all-dev   # Boost
sudo apt-get install cmake              # Cmake
```

For optional prerequisites:
- TBB: `sudo apt-get install libtbb-dev`
- MKL: [install using APT](https://software.intel.com/en-us/articles/installing-intel-free-libs-and-python-apt-repo)
:::

<br><br>


# Install GTSAM from Source

To build GTSAM from source, clone or download the latest release from the [GTSAM Github repo](https://github.com/borglab/gtsam).
Then follow the build & install instructions below.

In the root library folder execute:

```bash
#!bash
$ mkdir build
$ cd build
$ cmake ..
$ make check    # optional, runs unit tests
$ make install
```

<br><br>

# Install GTSAM from Ubuntu PPA

GTSAM can be installed on Ubuntu via [these PPA repositories](https://launchpad.net/~borglab) as well.
As of November 2020, packages for Xenial (u16.04), Bionic (u18.04), and Focal (u20.04) are published.

***Follow the code below to add PPA for your preferred branch:***
::::{tab-set}

:::{tab-item} latest 4.x stable release
```bash
# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev
```
:::

:::{tab-item} Nightly builds (develop branch)
```bash
# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-develop
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev
```
:::

::::

<br><br>

# Install GTSAM from Arch Linux AUR

```{warning} Installing GTSAM on Arch Linux has not been tested by the GTSAM developers.
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

::::{tab-set}

:::{tab-item} Install GTSAM
```sh
yay -S gtsam
```
:::

:::{tab-item} Install GTSAM with Intel Accelerations

```sh
yay -S gtsam-mkl
```
:::
::::

To discuss any issues related to this package refer to the comments section on
the AUR page of `gtsam` [here](https://aur.archlinux.org/packages/gtsam/).
