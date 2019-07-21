g++ -I. `pkg-config --cflags opencv` stream.cpp tcplib.c server.cpp  -o stream.exe `pkg-config --libs opencv` -lpthread

