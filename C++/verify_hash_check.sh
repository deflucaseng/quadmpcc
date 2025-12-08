#!/bin/bash
set -e

# Build ADCodeGen
echo "Building ADCodeGen..."
cd ADCodeGen/build
make
cd ../..

# Build MPCC
echo "Building MPCC..."
cd build
make
cd ..

# Run ADCodeGen
echo "Running ADCodeGen..."
cd ADCodeGen/build
./ADCodeGen
cd ../..

# Run MPCC (Baseline)
echo "Running MPCC (Baseline)..."
cd build
./MPCC
cd ..

# Backup config.json
cp Params/config.json Params/config.json.bak

# Modify config.json
echo "Modifying config.json..."
echo " " >> Params/config.json

# Run MPCC (Expect Failure)
echo "Running MPCC (Expect Failure)..."
cd build
if ./MPCC; then
    echo "Error: MPCC should have failed but succeeded."
    mv ../Params/config.json.bak ../Params/config.json
    exit 1
else
    echo "Success: MPCC failed as expected."
fi
cd ..

# Run ADCodeGen (Recovery)
echo "Running ADCodeGen (Recovery)..."
cd ADCodeGen/build
./ADCodeGen
cd ../..

# Run MPCC (Success Check)
echo "Running MPCC (Success Check)..."
cd build
./MPCC
cd ..

# Restore config.json
mv Params/config.json.bak Params/config.json

echo "Verification successful!"
