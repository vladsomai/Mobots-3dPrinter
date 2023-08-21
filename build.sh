C_COMPILER="/usr/bin/clang"
CXX_COMPILER="/usr/bin/clang++"
FLAGS="-Werror -Wall -Wextra -Wc++17-extensions"

cmake -G Ninja -D CMAKE_C_COMPILER=$C_COMPILER -D CMAKE_CXX_COMPILER=$CXX_COMPILER -D CMAKE_CXX_FLAGS_INIT=$FLAGS -S ./RoMoController -B ./out

echo 'Adding the default samples to the executable directory..'
sudo cp ~/Mobots-3dPrinter/Samples/hello-world.ngc ~/Mobots-3dPrinter/out/
sudo mv ~/Mobots-3dPrinter/out/hello-world.ngc  ~/Mobots-3dPrinter/out/gcode.ngc
sudo cp ~/Mobots-3dPrinter/Samples/config.ini ~/Mobots-3dPrinter/out/
