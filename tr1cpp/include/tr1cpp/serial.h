#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <iostream>
#include <stdio.h>
using namespace std;

class SerialPort
{
	private:
		int fd;
    bool connected;
		int setup(int fd, int speed, int parity);
		void setBlocking(int fd, int should_block);
	public:
		SerialPort(char *portName);
		~SerialPort();

    bool readSerialPort(std::string req, char res[32]);
    bool writeSerialPort(std::string data);
    bool isConnected();
};

#endif // SERIALPORT_H
