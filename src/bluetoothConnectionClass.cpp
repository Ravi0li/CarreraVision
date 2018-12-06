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
	sendString = new char[2];	// L�nge 1 ist erstmal noch Platzhalter
	sendStringLength = 2;

	updateSendString();
}

// --------------------------------------------------------------------------
// Serielle Bluetooth Verbindung herstellen
// --------------------------------------------------------------------------
int BluetoothConnectionClass::connectBLE()
{
	try{
		port->open(serialPortString);
		port->set_option(boost::asio::serial_port_base::baud_rate(baudRate));
		connected = true;
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
int BluetoothConnectionClass::disconnectBLE()
{
	try {
		port->close();
		connected = false;
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
// Setzt den wert eines Kanals ohne ihn zu senden
// --------------------------------------------------------------------------
void BluetoothConnectionClass::setSendValue(int channel, int set)
{
	std::lock_guard<std::mutex> lockit(lockValues);
	if (channel == 1)
	{
		setValue1 = set;
	}
	else if (channel == 2)
	{
		setValue2 = set;
	}
	else
	{
		std::cout << "ERROR: Es wurde versucht einen Wert f�r einen ung�ltigen Channel zu setzen";
	}
}

// --------------------------------------------------------------------------
// Sende Stellsignal an Kanal 1 und 2 
// --------------------------------------------------------------------------
void BluetoothConnectionClass::sendValuesBothChannels()
{
	if (connected == true)
	{
		try {
			updateSendString();
			boost::asio::write(*port, boost::asio::buffer(sendString, sendStringLength));
		}
		catch (boost::system::system_error const& ex)
		{
			std::cout << " Cannout send data via channel 1 and 2: " << ex.what() << std::endl;
		}
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

// --------------------------------------------------------------------------
// Schreibe Stellwerte beider Kan�le in die Zeichenkette zum Versenden
// --------------------------------------------------------------------------
void BluetoothConnectionClass::updateSendString()
{
	if (sendString != NULL)
	{ 
		delete[] sendString;
	}

	// Bastle String aus Stellsignalen zusammen
	std::string str;
	{
		std::lock_guard<std::mutex> lockit(lockValues);
		std::stringstream stream;
		stream << setValue1;
		stream << ",";
		stream << setValue2;
		str = stream.str();
	}
	
	// Konvertiere String Objekt zu char*
	sendStringLength = (int)str.length() + 1;
	sendString = new char[sendStringLength];
	std::strcpy(sendString, str.c_str());
	sendString[sendStringLength - 1] = '\n';
}

// --------------------------------------------------------------------------
// Dauerhaft durchlaufende Schleife
// --------------------------------------------------------------------------
void BluetoothConnectionClass::loopingThread()
{
	startFrame = std::chrono::high_resolution_clock::now();
	while (!stop)
	{
		// Frameratenmessung
		auto now = std::chrono::high_resolution_clock::now();
		int diff = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - startFrame).count();
		if (diff > 1000)
		{
			frameMutex.lock();
			frameRate = (int)(countFrame / (diff / 1000.0));
			frameMutex.unlock();
			countFrame = 0;
			startFrame = now;
		}
		countFrame++;

		// Senden der Daten
		sendValuesBothChannels();

		std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}
}

// --------------------------------------------------------------------------
// Stopt den Thread
// --------------------------------------------------------------------------
void BluetoothConnectionClass::stopThread()
{
	stop = true;
}

// --------------------------------------------------------------------------
// zeigt die aktuelle Framerate an
// --------------------------------------------------------------------------
int BluetoothConnectionClass::getFrameRate()
{
	frameMutex.lock();
	int ret = frameRate;
	frameMutex.unlock();
	return ret;
}

// --------------------------------------------------------------------------
// Gibt den Stellwert 1 zur�ck
// --------------------------------------------------------------------------
int BluetoothConnectionClass::getSetValue1()
{
	return setValue1;
}

// --------------------------------------------------------------------------
// Gibt den Stellwert 2 zur�ck
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