#!/bin/bash
set -e

# Ensure we are in the script's directory
cd "$(dirname "$0")"
ROOT_DIR=$(pwd)

echo "Building BLASFEO for Mac M1..."
cd "${ROOT_DIR}/External/blasfeo"
rm -rf build lib
mkdir -p build
mkdir -p lib
cd build

cmake .. \
    -DCMAKE_INSTALL_PREFIX="${ROOT_DIR}/External/blasfeo/lib" \
    -DTARGET=ARMV8A_APPLE_M1 \
    -DLA=HIGH_PERFORMANCE \
    -DMF=PANELMAJ \
    -DBLASFEO_EXAMPLES=OFF

make -j4
make install

echo "Building HPIPM for Mac M1..."
cd "${ROOT_DIR}/External/hpipm"
rm -rf build lib
mkdir -p build
mkdir -p lib
cd build

cmake .. \
    -DCMAKE_INSTALL_PREFIX="${ROOT_DIR}/External/hpipm/lib" \
    -DTARGET=ARMV8A_APPLE_M1 \
    -DBLASFEO_PATH="${ROOT_DIR}/External/blasfeo/lib" \
    -DHPIPM_TESTING=OFF

make -j4
make install

echo "Build complete. Libraries installed in External/blasfeo/lib and External/hpipm/lib"
