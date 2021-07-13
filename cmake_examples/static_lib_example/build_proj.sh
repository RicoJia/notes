rm -rf build
mkdir -p build
cd build
arch=$(uname -m)
echo "Building for $arch"
cmake .. || exit 1
make -j$(grep -c ^processor /proc/cpuinfo) || exit 1
cd ..
