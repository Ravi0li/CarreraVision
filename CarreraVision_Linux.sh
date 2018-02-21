echo "Ordner erstellen"
mkdir -p bin
echo "Kompilieren"
g++-7.2.0 $(pkg-config --libs --cflags opencv) -std=c++11 -o bin/CarreraVision src/*.cpp
echo "Fertig"
