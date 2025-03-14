# Installing MKL on Linux

Intel has a guide
for installing MKL on Linux through APT repositories [here](https://software.intel.com/en-us/articles/installing-intel-free-libs-and-python-apt-repo).

After following the instructions, add the following to your `~/.bashrc` (and afterwards, open a new terminal before compiling GTSAM):
 `LD_PRELOAD`
need only be set if you are building the cython wrapper to use GTSAM from python.

```sh
source /opt/intel/mkl/bin/mklvars.sh intel64
export LD_PRELOAD="$LD_PRELOAD:/opt/intel/mkl/lib/intel64/libmkl_core.so:/opt/intel/mkl/lib/intel64/libmkl_sequential.so"
```

To use MKL in GTSAM pass the flag `-DGTSAM_WITH_EIGEN_MKL=ON` to cmake.

The `LD_PRELOAD` fix seems to be related to a well known problem with MKL which leads to lots of undefined symbol errors, for example:

- <https://software.intel.com/en-us/forums/intel-math-kernel-library/topic/300857>
- <https://software.intel.com/en-us/forums/intel-distribution-for-python/topic/628976>
- <https://groups.google.com/a/continuum.io/forum/#!topic/anaconda/J3YGoef64z8>

Failing to specify `LD_PRELOAD` may lead to errors such as:

 ```
 ImportError: /opt/intel/mkl/lib/intel64/libmkl_vml_avx2.so: undefined symbol: mkl_serv_getenv
 ```

or

 ```
 Intel MKL FATAL ERROR: Cannot load libmkl_avx2.so or libmkl_def.so.
 ```

when importing GTSAM using the cython wrapper in python.