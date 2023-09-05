BUILD_TYPE=$1

if [ $BUILD_TYPE != 'Release' ] && [ $BUILD_TYPE != 'Debug' ]
then
  echo "Expected Release or Debug as argument"
  exit 1
fi

C_COMPILER="/usr/bin/clang"
CXX_COMPILER="/usr/bin/clang++"
FLAGS="-Werror -Wall -Wextra -Wc++17-extensions"

cmake -G Ninja -D CMAKE_C_COMPILER=$C_COMPILER -D CMAKE_CXX_COMPILER=$CXX_COMPILER -D CMAKE_CXX_FLAGS_INIT=$FLAGS -D CMAKE_BUILD_TYPE=$BUILD_TYPE -S ./RoMoController -B ./build
cmake --build ./build --config $BUILD_TYPE --target all --

echo 'Adding the default samples to the executable directory..'
sudo cp ~/Mobots-3dPrinter/Samples/hello-world.ngc ~/Mobots-3dPrinter/build/
sudo mv ~/Mobots-3dPrinter/build/hello-world.ngc  ~/Mobots-3dPrinter/build/gcode.ngc
sudo cp ~/Mobots-3dPrinter/Samples/config.ini ~/Mobots-3dPrinter/build/
