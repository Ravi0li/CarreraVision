#pragma once
#include <string>
#include <boost/asio/serial_port.hpp> 
#include <boost/asio.hpp> 
#include <boost/thread/mutex.hpp>
#include <opencv2/core.hpp>

class BluetoothConnectionClass
{
public:
	// C'tor & D'tor
	BluetoothConnectionClass(cv::FileNode _para);
	~BluetoothConnectionClass();
	
	// Methoden
	int connect();
	int disconnect();
	void sendChannel1(int set);
	void sendChannel2(int set);
	void sendChannel12(int set1, int set2);
	int getSetValue1();
	int getSetValue2();

private:
	cv::FileNode para;				// Parameter aus XML
	boost::mutex mtx_;
	boost::asio::io_service* io;	// IO Service für BT-Verbindung
	boost::asio::serial_port* port;	// Serieller Port für BT-Verbindung
	std::string serialPortString;	// Zeichenkette für COM-Port
	int baudRate;					// Baudrate für serielle Kommunikation
	int setValue1, setValue2;		// Aktuelle Stellwerte für beide Kanäle
	char* sendString;				// Zeichenkette mit Stellwerten für beide Kanäle
	int sendStringLength;			// Länge der Zeichenkette mit Stellwerten
	
	// Methoden
	void updateSendString();	

};