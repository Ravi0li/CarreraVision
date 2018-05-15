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
	void connect();
	void disconnect();
	void sendChannel1();
	void sendChannel2();

private:
	boost::asio::io_service* io;	// IO Service f�r BT-Verbindung
	boost::asio::serial_port* port;	// Serieller Port f�r BT-Verbindung
	std::string serialPortString;	// Zeichenkette f�r COM-Port

};