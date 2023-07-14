C_COMPILER="/usr/bin/clang"
CXX_COMPILER="/usr/bin/clang++"
FLAGS="-Werror -Wall -Wextra -Wc++17-extensions"
cmake -G Ninja -D CMAKE_C_COMPILER=$C_COMPILER -D CMAKE_CXX_COMPILER=$CXX_COMPILER -D CMAKE_CXX_FLAGS_INIT=$FLAGS -S ./RoMoController -B ./out