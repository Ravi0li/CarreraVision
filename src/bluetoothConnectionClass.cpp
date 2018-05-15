#include "bluetoothConnectionClass.h"
//#include <boost/asio/serial_port.hpp> 
//#include <boost/asio.hpp> 


//boost::asio::io_service io;
//boost::asio::serial_port port(io);
//port.open("COM4");
//port.set_option(boost::asio::serial_port_base::baud_rate(115200));
//std::cout << "warten" << std::endl;
//Sleep(5000);
//char c = 'r';
//boost::asio::write(port, boost::asio::buffer(&c, 1));

BluetoothConnectionClass::BluetoothConnectionClass()
{	
	serialPortString = "COM4";
}

// --------------------------------------------------------------------------
// Serielle Bluetooth Verbindung herstellen
// --------------------------------------------------------------------------
void BluetoothConnectionClass::connect()
{

}

// --------------------------------------------------------------------------
// Serielle Bluetooth Verbindung trennen
// --------------------------------------------------------------------------
void BluetoothConnectionClass::disconnect()
{

}

// --------------------------------------------------------------------------
// Sende Stellsignal an Kanal 1 
// --------------------------------------------------------------------------
void BluetoothConnectionClass::sendChannel1()
{

}

// --------------------------------------------------------------------------
// Sende Stellsignal an Kanal 2 
// --------------------------------------------------------------------------
void BluetoothConnectionClass::sendChannel2()
{

}

BluetoothConnectionClass::~BluetoothConnectionClass()
{
	// TODO: IO Service freigeben
}