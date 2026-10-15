# Flexiv RDK v2.2 installation — Ubuntu 22.04

The official **C++ SDK v2.2** is installed locally at:

```text
/home/kiara/rdk/flexiv_enlight_readonly_setup/.rdk-v2.2
```

Installation uses Ubuntu 22.04.5 x86_64, GCC 11.4.0, CMake 3.22.1, Eigen 3.4.0, and C++20. No system packages, firewall settings, network profiles, or robot settings were changed during this installation.

## Release status and Python limitation

[Flexiv's public v2.2 release](https://github.com/flexivrobotics/flexiv_rdk/releases/tag/v2.2) is titled **v2.2 Draft** with body **WIP**. Its Linux x86_64 shared library is publicly downloadable. The repository's `release/v2.2` branch declares SDK version 2.2.

[PyPI](https://pypi.org/project/flexivrdk/) and `python -m pip index versions flexivrdk` both show 2.1.0 as the latest Python release and no 2.2 release. GitHub's v2.2 assets include C++ shared libraries but no Python wheel. Therefore this installation does **not** upgrade `import flexivrdk` to 2.2. A genuine vendor-supplied Python 2.2 wheel is needed to complete that part of the request.

Existing system Python package remains 1.7.0, and project `.venv` remains 2.1.0. Do not rename a 2.1 wheel, symlink this C++ library over a Python extension, or infer Python package version from a Git tag. Do not attempt `pip install git+...` as a substitute: no official Python 2.2 distribution was identified here.

## Pinned source and integrity

- Source: [official release/v2.2 commit](https://github.com/flexivrobotics/flexiv_rdk/tree/36639f38525631b5caac2608a88b23af60d5dd5f).
- Commit: `36639f38525631b5caac2608a88b23af60d5dd5f`.
- Untouched source checkout: `third_party/flexiv_rdk_v2.2/`.
- Binary: `libflexiv_rdk.x86_64-linux-gnu.so` from the official v2.2 GitHub release.
- SHA-256: `8205d3491c651b4490995f3d1f8cf7125e1e039309812869ddf8dc59fd1376e5`.

The initial `v2.2` tag points to `ff92fe421e19daee6cdf27794a332c8fef7cf831`, whose checked-in binary checksum differs from the current published asset. The pinned official release-branch commit explicitly updates that checksum and matches GitHub release metadata and the downloaded bytes. It was used without modifying source or disabling verification. This is a draft branch snapshot, not a claim that the original tag and current asset are identical.

The original `third_party/flexiv_rdk` checkout and its state-example inspection record remain unchanged. The v2.2 example also contains `ClearFault()` and `ServoOn()` and was not executed. No Robot object was instantiated during any installation test. Controller compatibility and live states remain unverified.

## Reproduce local installation

The checkout was created from the previously fetched official local Git repository, then pinned to the commit above. For a fresh online checkout, use:

```bash
git clone --branch release/v2.2 https://github.com/flexivrobotics/flexiv_rdk.git third_party/flexiv_rdk_v2.2
git -C third_party/flexiv_rdk_v2.2 checkout --detach 36639f38525631b5caac2608a88b23af60d5dd5f
cmake -S third_party/flexiv_rdk_v2.2 -B .build/rdk-v2.2 -DCMAKE_INSTALL_PREFIX="$PWD/.rdk-v2.2"
cmake --build .build/rdk-v2.2 --target install --config Release --parallel 2
```

Run from the project root. The CMake download verifies the binary against the pinned SHA-256. Existing build prerequisites satisfied the official source; no sudo or OS package installation was needed.

## Use from a C++ project

```cmake
find_package(flexiv_rdk 2.2 EXACT CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE flexiv::flexiv_rdk)
```

Configure with the explicit prefix:

```bash
cmake -S <YOUR_CPP_PROJECT> -B <YOUR_BUILD_DIRECTORY> -DCMAKE_PREFIX_PATH=/home/kiara/rdk/flexiv_enlight_readonly_setup/.rdk-v2.2
```

This does not authorize any robot operation. Observe the original read-only checklist before hardware contact.

## Verification performed

- CMake configure/build/install succeeded with SDK package version 2.2.
- Downloaded binary SHA-256 matched pinned source and official release metadata.
- `ldd` found all shared-library dependencies on Ubuntu 22.04.
- Bounded `ctypes.CDLL` load succeeded without constructing a Robot.
- A separate offline consumer used `find_package(flexiv_rdk 2.2 EXACT)`, included the official robot header, linked with `--no-as-needed` so the SDK really loaded, and printed PASS. It contains no robot object or API calls.
- Offline source/executable: `.build/rdk-v2.2-smoke-src/`, `.build/rdk-v2.2-smoke/rdk22_offline_check`.

Exact consumer checks executed:

```bash
cmake -S .build/rdk-v2.2-smoke-src -B .build/rdk-v2.2-smoke -DCMAKE_PREFIX_PATH=/home/kiara/rdk/flexiv_enlight_readonly_setup/.rdk-v2.2
cmake --build .build/rdk-v2.2-smoke --parallel 2
.build/rdk-v2.2-smoke/rdk22_offline_check
ldd .build/rdk-v2.2-smoke/rdk22_offline_check
```

Next single safe verification command:

```bash
/home/kiara/rdk/flexiv_enlight_readonly_setup/.build/rdk-v2.2-smoke/rdk22_offline_check
```
