#pragma once
#include <string>
#include <boost/asio/serial_port.hpp> 
#include <boost/asio.hpp> 

class BluetoothConnectionClass
{
public:
	// C'tor & D'tor
	BluetoothConnectionClass();
	~BluetoothConnectionClass();
	
	// Methoden
	int connect();
	int disconnect();
	void sendChannel1(int set);
	void sendChannel2(int set);
	void sendChannel12(int set1, int set2);

private:
	boost::asio::io_service* io;	// IO Service f�r BT-Verbindung
	boost::asio::serial_port* port;	// Serieller Port f�r BT-Verbindung
	std::string serialPortString;	// Zeichenkette f�r COM-Port
	int baudRate;					// Baudrate f�r serielle Kommunikation
	int setValue1, setValue2;		// Aktuelle Stellwerte f�r beide Kan�le
	char* sendString;				// Zeichenkette mit Stellwerten f�r beide Kan�le
	int sendStringLength;			// L�nge der Zeichenkette mit Stellwerten
	
	// Methoden
	void updateSendString();	

};