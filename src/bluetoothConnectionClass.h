#pragma once
#include <string>
#include <boost/asio/serial_port.hpp> 
#include <boost/asio.hpp> 
#include <boost/thread/mutex.hpp>
#include <opencv2/core.hpp>
#include <chrono>
#include <mutex>

class BluetoothConnectionClass
{
public:
	// C'tor & D'tor
	BluetoothConnectionClass(cv::FileNode _para);
	~BluetoothConnectionClass();
	
	// Methoden
	int connectBLE();
	int disconnectBLE();
	void setSendValue(int channel, int set);
	void sendValuesBothChannels();
	void loopingThread();
	void stopThread();
	int getFrameRate();
	int getSetValue1();
	int getSetValue2();

private:
	cv::FileNode para;				// Parameter aus XML
	boost::asio::io_service* io;	// IO Service f�r BT-Verbindung
	boost::asio::serial_port* port;	// Serieller Port f�r BT-Verbindung
	std::string serialPortString;	// Zeichenkette f�r COM-Port
	int baudRate;					// Baudrate f�r serielle Kommunikation
	int setValue1, setValue2;		// Aktuelle Stellwerte f�r beide Kan�le
	char* sendString;				// Zeichenkette mit Stellwerten f�r beide Kan�le
	int sendStringLength;			// L�nge der Zeichenkette mit Stellwerten
	bool connected = false;			// Zeigt an ob eine Verbindung besteht
	std::mutex lockValues;			// sperrt die Values w�rend des senden
	
	// Frameraten messung
	std::chrono::high_resolution_clock::time_point startFrame;	// Zeit zur Frameratenmessung
	int countFrame = 0;											// Z�hler f�r Frameratenanzeige
	int frameRate = 0;											// Aktuelle Framerate
	std::mutex frameMutex;										// Mutex f�r die Framerate
	bool stop = false;											// anhalten des Threads

	// Methoden
	void updateSendString();	

};

typedef void (BluetoothConnectionClass::*pf_i) (int);