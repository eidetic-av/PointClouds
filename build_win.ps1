cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE="/opt/vcpkg/scripts/buildsystems/vcpkg.cmake" `
	-G "Ninja Multi-Config" `
	-DCMAKE_C_COMPILER=clang `
	-DCMAKE_CXX_COMPILER=clang++ `
	-DVCPKG_TARGET_TRIPLET=x64-windows `
	-DVCPKG_OVERLAY_PORTS="$pwd\ports" `
	-DWITH_DRACO=off `
	-DBUILD_TESTS=off
