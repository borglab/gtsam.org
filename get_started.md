---
layout: page
title: Get Started
permalink: /get_started/
---

The current stable release of GTSAM is 4.2. The active development line is pre-4.3 and recent alpha releases follow that track.

## Fastest Start: Python Package

If you want the easiest way to try GTSAM, install the Python package:

```sh
pip install gtsam
```

For the newest development build:

```sh
pip install gtsam-develop
```

This works well in local Python environments and in Google Colab.

## Build from Source

Clone the source from the [GTSAM GitHub repository](https://github.com/borglab/gtsam):

```sh
git clone https://github.com/borglab/gtsam.git
cd gtsam
```

{% include quick_start_module.md %}

### Requirements

- A modern compiler:
  - Linux: at least clang-11 or gcc-9
  - macOS: at least Xcode 14.2
  - Windows: at least MSVC 14.2
- [CMake](https://cmake.org/download/) >= 3.10

Ubuntu package:

```sh
sudo apt-get install cmake
```

### Optional Boost Dependency

Boost is now optional and only needed when either of these flags is enabled:

- `GTSAM_USE_BOOST_FEATURES`
- `GTSAM_ENABLE_BOOST_SERIALIZATION`

If you need Boost, install version 1.70 or newer:

- Ubuntu: `sudo apt-get install libboost-all-dev`
- macOS: `brew install boost`
- Windows: prefer [vcpkg](https://github.com/microsoft/vcpkg)

### Optional Dependencies

- [Intel Threaded Building Blocks (TBB)](https://github.com/uxlfoundation/oneTBB)
  - Ubuntu: `sudo apt-get install libtbb-dev`
- [Intel Math Kernel Library (MKL)](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onemkl.html)
  - Benchmark before enabling it; it does not help all workloads.

For more build flags and platform notes, see the [Build](/build/) page.

## Ubuntu Packages and PPAs

Ubuntu users can also use the BorgLab Launchpad archives:

- [BorgLab Launchpad PPAs](https://launchpad.net/~borglab)
- [Nightly develop PPA](https://launchpad.net/~borglab/+archive/ubuntu/gtsam-develop)

For the most current configuration options, source builds are still the safest path.

## Arch Linux

GTSAM is available in the [AUR](https://aur.archlinux.org/packages/gtsam/).

```sh
yay -S gtsam
```

For Intel-accelerated builds:

```sh
yay -S gtsam-mkl
```

## Conda-forge

Community-maintained GTSAM packages are available from
[conda-forge](https://conda-forge.org) for Linux, macOS, and Windows. These
packages are not tested by the GTSAM developers.

Install the [`gtsam` package](https://anaconda.org/conda-forge/gtsam) with your
preferred conda-compatible package manager:

```sh
conda install -c conda-forge gtsam
mamba install -c conda-forge gtsam
micromamba install -c conda-forge gtsam
```

If you need a package manager, see the
[micromamba installation guide](https://mamba.readthedocs.io/en/stable/installation/micromamba-installation.html).
It recommends this installer for Linux, macOS, and Git Bash on Windows:

```sh
"${SHELL}" <(curl -L micro.mamba.pm/install.sh)
```

For Windows PowerShell, use:

```powershell
Invoke-Expression ((Invoke-WebRequest -Uri https://micro.mamba.pm/install.ps1).Content)
```

## Wrappers

GTSAM also ships with Python and MATLAB wrappers:

- [Python wrapper README](https://github.com/borglab/gtsam/blob/develop/python/README.md)
- [MATLAB wrapper README](https://github.com/borglab/gtsam/blob/develop/matlab/README.md)
