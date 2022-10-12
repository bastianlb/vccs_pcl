#!/bin/bash

set -e
set -x

rm -rf build
mkdir build

pushd build

CONAN_CPU_COUNT=12 CXX=/usr/bin/g++-7 CC=/usr/bin/gcc-7 CUDACXX=/usr/local/cuda-11.1/bin/nvcc conan install .. -s build_type=Debug --build "*" -o python_dev_config:python=$(python -c 'import sys; print(sys.executable)')
CUDACXX=/usr/local/cuda-11.1/bin/nvcc CXX=/usr/bin/g++-7 CC=/usr/bin/gcc-7 cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build . --parallel 12

popd
