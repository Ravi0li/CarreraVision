echo "Ordner erstellen"
mkdir -p bin
cp para.xml bin/
cp -r demo bin/
echo "Kompilieren vom Hauptprogramm"
g++-7.2.0 $(pkg-config --libs --cflags opencv) -std=c++11 -o bin/CarreraVision src/*.cpp -L/usr/local/lib/ -lboost_filesystem -lboost_system -lboost_thread -lpthread
echo "Kompilieren vom Unittest"
g++-7.2.0 $(pkg-config --libs --cflags opencv) -std=c++11 -o bin/CarreraVision_UnitTest unit/*.cpp src/trackdetectionClass.cpp src/debugWinOrganizerClass.cpp -L/usr/local/lib/ -lboost_filesystem -lboost_system -lboost_thread -lpthread
echo "Fertig"
