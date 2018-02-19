echo "Ordner erstellen"
mkdir -p bin
echo "Kompilieren"
g++-7.2.0 $(pkg-config --libs --cflags opencv) -std=c++98 -o bin/CarreraVision src/main.cpp
echo "Fertig"
