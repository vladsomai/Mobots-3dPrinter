set C_COMPILER="C:/Program Files (x86)/Microsoft Visual Studio/2019/Professional/VC/Tools/Llvm/bin/clang.exe"
set CXX_COMPILER="C:/Program Files (x86)/Microsoft Visual Studio/2019/Professional/VC/Tools/Llvm/bin/clang++.exe"
set FLAGS="-Werror -Wall -Wextra -Wc++17-extensions"
set FLAGS="-Werror -Wall -Wextra -Wc++17-extensions"
cmake -G Ninja -D CMAKE_C_COMPILER=%C_COMPILER% -D CMAKE_CXX_COMPILER=%CXX_COMPILER% -D CMAKE_CXX_FLAGS_INIT=%FLAGS% -D CMAKE_BUILD_TYPE=Release -S ./RoMoController -B ./out

echo Adding the default samples to the executable directory..
copy .\Samples\hello-world.ngc .\out
move .\out\hello-world.ngc  .\out\gcode.ngc
copy .\Samples.\config.ini .\out
