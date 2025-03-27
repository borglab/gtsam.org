---
layout: page
title: Get Started
permalink: /get_started/
---

## Install GTSAM from Source

{% include quick_start_module.md %}

Prerequisites:

- [Boost](http://www.boost.org/users/download/) >= 1.43 (Ubuntu: `sudo apt-get install libboost-all-dev`)
- [CMake](http://www.cmake.org/cmake/resources/software.html) >= 3.0 (Ubuntu: `sudo apt-get install cmake`)
- A modern compiler, i.e., at least gcc 4.7.3 on Linux.

Optional prerequisites - used automatically if findable by CMake:

- [Intel Threaded Building Blocks (TBB)](http://www.threadingbuildingblocks.org/) (Ubuntu: `sudo apt-get install libtbb-dev`)
- [Intel Math Kernel Library (MKL)](http://software.intel.com/en-us/intel-mkl) (Ubuntu: [installing using APT](https://software.intel.com/en-us/articles/installing-intel-free-libs-and-python-apt-repo))
    - See [Build](build.md) for more installation information
    - Note that MKL may not provide a speedup in all cases. Make sure to benchmark your problem with and without MKL.

## Install GTSAM from Ubuntu PPA

GTSAM can be installed on Ubuntu via [these PPA repositories](https://launchpad.net/~borglab) as well.
At present (Nov 2020), packages for Xenial (u16.04), Bionic (u18.04), and Focal (u20.04) are published.

#### Add PPA for GTSAM nightly builds (develop branch)

```sh
# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-develop
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

#### Add PPA for the latest GTSAM 4.x stable release

```sh
# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

## Install GTSAM from Arch Linux AUR

Note: Installing GTSAM on Arch Linux is not tested by the GTSAM developers.

GTSAM is available in the Arch User Repository
([AUR](https://wiki.archlinux.org/index.php/Arch_User_Repository)) as
[`gtsam`](https://aur.archlinux.org/packages/gtsam/).

Note you can manually install the package by following the instructions on the
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

#### Install GTSAM
```sh
yay -S gtsam
```

or

#### Install GTSAM with Intel Accelerations

```sh
yay -S gtsam-mkl
```

To discuss any issues related to this package refer to the comments section on
the AUR page of `gtsam` [here](https://aur.archlinux.org/packages/gtsam/).

## Install GTSAM via conda-forge (conda/mamba installation) on Linux/MacOS/Windows

Note: Installing GTSAM via conda-forge is not tested by the GTSAM developers.

GTSAM is available on [conda-forge](https://conda-forge.org) via the 
[`gtsam` package](https://anaconda.org/conda-forge/gtsam) on Linux, MacOS and Windows platforms.

If you already have a working installation of conda/mamba/micromamba, installation is as easy as

```sh
conda install gtsam -c conda-forge  # if you use conda
mamba install gtsam -c conda-forge  # if you use mamba
micromamba install gtsam -c conda-forge  # if you use micromamba, recommended
```

If you do not yet have a working installation of conda/mamba/micromamba, 
we recommend installing micromamba which is as easy as 
(this works on any platform in your favourite shell):

```sh
"${SHELL}" <(curl -L micro.mamba.pm/install.sh)
```

For more information about micromamba, please visit their [website](https://mamba.readthedocs.io/en/latest/micromamba-installation.html).
