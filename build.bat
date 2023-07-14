set C_COMPILER="C:/Program Files (x86)/Microsoft Visual Studio/2019/Professional/VC/Tools/Llvm/bin/clang.exe"
set CXX_COMPILER="C:/Program Files (x86)/Microsoft Visual Studio/2019/Professional/VC/Tools/Llvm/bin/clang++.exe"
set FLAGS="-Werror -Wall -Wextra -Wc++17-extensions"
cmake -G Ninja -D CMAKE_C_COMPILER=%C_COMPILER% -D CMAKE_CXX_COMPILER=%CXX_COMPILER% -D CMAKE_CXX_FLAGS_INIT=%FLAGS% -S ./RoMoController -B ./out