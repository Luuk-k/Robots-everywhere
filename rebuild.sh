echo "Building"
cmake -S . -B ./build
make clean -C ./build
make -C ./build
