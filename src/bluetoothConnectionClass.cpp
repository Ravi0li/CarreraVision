#include "bluetoothConnectionClass.h"
#include <string>
#include <sstream>
#include <iostream>
#include <boost/asio/serial_port.hpp> 
#include <boost/asio.hpp> 
#include <chrono>
#include <thread>

BluetoothConnectionClass::BluetoothConnectionClass(cv::FileNode _para)
{	
	para = _para;
	io = new boost::asio::io_service();
	port = new boost::asio::serial_port(*io);
	serialPortString = (std::string)para["serial_port_name"];
	std::cout << serialPortString;
	baudRate = (int)para["serial_baut_rate"];
	setValue1 = 0;
	setValue2 = 0;
	sendString = new char[2];	// Länge 1 ist erstmal noch Platzhalter
	sendStringLength = 2;

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

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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

	std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

	std::this_thread::sleep_for(std::chrono::milliseconds(1));
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

	std::this_thread::sleep_for(std::chrono::milliseconds(1));
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

	std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

// --------------------------------------------------------------------------
// Schreibe Stellwerte beider Kanäle in die Zeichenkette zum Versenden
// --------------------------------------------------------------------------
void BluetoothConnectionClass::updateSendString()
{
	mtx_.lock();

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
	sendStringLength = (int)str.length() + 1;
	sendString = new char[sendStringLength];
	std::strcpy(sendString, str.c_str());
	sendString[sendStringLength - 1] = '\n';

	mtx_.unlock();
}

// --------------------------------------------------------------------------
// Gibt den Stellwert 1 zurück
// --------------------------------------------------------------------------
int BluetoothConnectionClass::getSetValue1()
{
	return setValue1;
}

// --------------------------------------------------------------------------
// Gibt den Stellwert 2 zurück
// --------------------------------------------------------------------------
int BluetoothConnectionClass::getSetValue2()
{
	return setValue2;
}

// --------------------------------------------------------------------------
// Allokierten Speicher freigeben
// --------------------------------------------------------------------------
BluetoothConnectionClass::~BluetoothConnectionClass()
{
	// Gebe allokierte Objekte wieder frei
	delete io;
	delete port;
	delete[] sendString;
}