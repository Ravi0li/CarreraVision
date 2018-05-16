#include "bluetoothConnectionClass.h"
#include <string>
#include <sstream>
#include <iostream>
#include <boost/asio/serial_port.hpp> 
#include <boost/asio.hpp> 

BluetoothConnectionClass::BluetoothConnectionClass()
{	
	io = new boost::asio::io_service();
	port = new boost::asio::serial_port(*io);
	serialPortString = "COM4";
	baudRate = 115200;
	setValue1 = 0;
	setValue2 = 0;
	sendString = new char[1];
	sendStringLength = 1;

	updateSendString();
}

// --------------------------------------------------------------------------
// Serielle Bluetooth Verbindung herstellen
// --------------------------------------------------------------------------
int BluetoothConnectionClass::connect()
{
	try{
		port->open(serialPortString);
		port->set_option(boost::asio::serial_port_base::baud_rate(baudRate));
	}
	catch (boost::system::system_error const& ex)
	{
		std::cout << " Could not open COM-Port: " << ex.what() << std::endl;
		return -1;
	}

	Sleep(100);
	return 1;
}

// --------------------------------------------------------------------------
// Serielle Bluetooth Verbindung trennen
// --------------------------------------------------------------------------
int BluetoothConnectionClass::disconnect()
{
	try {
		port->close();
	}
	catch (boost::system::system_error const& ex)
	{
		std::cout << " Could not close COM-Port: " << ex.what() << std::endl;
		return -1;
	}

	Sleep(100);
	return 1;
}

// --------------------------------------------------------------------------
// Sende Stellsignal an Kanal 1, lasse Stellsignal von Kanal 2 unverändert
// --------------------------------------------------------------------------
void BluetoothConnectionClass::sendChannel1(int set)
{
	try {
		setValue1 = set;
		updateSendString();
		boost::asio::write(*port, boost::asio::buffer(sendString, sendStringLength));
		
	}
	catch (boost::system::system_error const&  ex)
	{
		std::cout << " Cannout send data via channel 1: " << ex.what() << std::endl;
	}

	Sleep(1);
}

// --------------------------------------------------------------------------
// Sende Stellsignal an Kanal 2, lasse Stellsignal von Kanal 1 unverändert
// --------------------------------------------------------------------------
void BluetoothConnectionClass::sendChannel2(int set)
{
	try {
		setValue2 = set;
		updateSendString();
		boost::asio::write(*port, boost::asio::buffer(sendString, sendStringLength));

	}
	catch (boost::system::system_error const& ex)
	{
		std::cout << " Cannout send data via channel 2: " << ex.what() << std::endl;
	}

	Sleep(1);
}

// --------------------------------------------------------------------------
// Sende Stellsignal an Kanal 1 und 2 
// --------------------------------------------------------------------------
void BluetoothConnectionClass::sendChannel12(int set1, int set2)
{
	try {
		setValue1 = set1;
		setValue2 = set2;
		updateSendString();
		boost::asio::write(*port, boost::asio::buffer(sendString, sendStringLength));

	}
	catch (boost::system::system_error const& ex)
	{
		std::cout << " Cannout send data via channel 1 and 2: " << ex.what() << std::endl;
	}

	Sleep(1);
}

// --------------------------------------------------------------------------
// Schreibe Stellwerte beider Kanäle in die Zeichenkette zum Versenden
// --------------------------------------------------------------------------
void BluetoothConnectionClass::updateSendString()
{
	if (sendString != NULL)
	{ 
		delete[] sendString;
	}
	
	// Bastle String aus Stellsignalen zusammen
	std::stringstream stream;

	stream << setValue1;
	stream << ",";
	stream << setValue2;

	// Konvertiere Stream zu String Objekt
	std::string str = stream.str();
	
	// Konvertiere String Objekt zu char*
	sendStringLength = str.length() + 1;
	sendString = new char[sendStringLength];
	std::strcpy(sendString, str.c_str());
	sendString[sendStringLength - 1] = '\n';
}



BluetoothConnectionClass::~BluetoothConnectionClass()
{
	// Gebe allokierte Objekte wieder frei
	delete io;
	delete port;
	delete[] sendString;
}