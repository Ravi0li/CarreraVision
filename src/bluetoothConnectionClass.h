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
	boost::asio::io_service* io;	// IO Service für BT-Verbindung
	boost::asio::serial_port* port;	// Serieller Port für BT-Verbindung
	std::string serialPortString;	// Zeichenkette für COM-Port
	int baudRate;					// Baudrate für serielle Kommunikation
	int setValue1, setValue2;		// Aktuelle Stellwerte für beide Kanäle
	char* sendString;				// Zeichenkette mit Stellwerten für beide Kanäle
	int sendStringLength;			// Länge der Zeichenkette mit Stellwerten
	bool connected = false;			// Zeigt an ob eine Verbindung besteht
	std::mutex lockValues;			// sperrt die Values wärend des senden
	
	// Frameraten messung
	std::chrono::high_resolution_clock::time_point startFrame;	// Zeit zur Frameratenmessung
	int countFrame = 0;											// Zähler für Frameratenanzeige
	int frameRate = 0;											// Aktuelle Framerate
	std::mutex frameMutex;										// Mutex für die Framerate
	bool stop = false;											// anhalten des Threads

	// Methoden
	void updateSendString();	

};

typedef void (BluetoothConnectionClass::*pf_i) (int);