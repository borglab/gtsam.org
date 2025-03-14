---
title: Get Started
---

# Prerequisites:

1. A modern compiler, i.e., at least gcc 4.7.3 on Linux.
2. [Boost](http://www.boost.org/users/download/) $\geq$ 1.43
3. [CMake](http://www.cmake.org/cmake/resources/software.html) $\geq$ 3.0

:::{note} Optional prerequisites - used automatically if findable by CMake:
:class: dropdown
4. [Intel Threaded Building Blocks (TBB)](http://www.threadingbuildingblocks.org/) (Ubuntu: `sudo apt-get install libtbb-dev`)
5. [Intel Math Kernel Library (MKL)](http://software.intel.com/en-us/intel-mkl)
    - *See [Build](/Content/build-project) for more installation information.*
    - Note that MKL may not provide a speedup in all cases. Make sure to benchmark your problem with and without MKL.
:::

:::{hint} Ubuntu Prerequisite Setup:
:class: dropdown
```bash
sudo apt-get install libboost-all-dev   # Boost
sudo apt-get install cmake              # CMake
```

```
sudo apt-get install libtbb-dev         # TBB
```
MKL
[installing using APT](https://software.intel.com/en-us/articles/installing-intel-free-libs-and-python-apt-repo)
:::

<br><br>

# Install GTSAM from Source

In the root library folder execute:

```bash
#!bash
$ mkdir build
$ cd build
$ cmake ..
$ make check (optional, runs unit tests)
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
