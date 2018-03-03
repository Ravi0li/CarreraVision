echo "Ordner erstellen"
mkdir -p bin
mkdir -p /bin/demo
cp /para.xml /bin/
cp /demo /bin/demo
echo "Kompilieren vom Hauptprogramm"
g++-7.2.0 $(pkg-config --libs --cflags opencv) -std=c++11 -o bin/CarreraVision src/*.cpp
echo "Kompilieren vom Unittest"
g++-7.2.0 $(pkg-config --libs --cflags opencv) -std=c++11 -o bin/CarreraVision_UnitTest unit/*.cpp src/trackdetectionClass.cpp src/debugWinOrganizerClass.cpp
echo "Fertig"
