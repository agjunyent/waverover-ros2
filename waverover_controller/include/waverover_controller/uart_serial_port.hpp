#pragma once

#include <serial_cpp/serial.h>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <memory>
#include <iostream>

class UARTSerialPort {
public:
    UARTSerialPort(const std::string& port_name, int baudrate);
    ~UARTSerialPort();

    bool isAvailable() const;

    static void listAllPorts();

    void sendRequestSync(const std::string& text);
    bool getResponseSync(const std::string& command, std::string& response);
    bool sendRequestSync(const std::vector<uint8_t>& data);

    bool readResponse();

    bool isOpen() const;
    bool isClosed() const;

private:
    std::unique_ptr<serial_cpp::Serial> _serial;
    std::vector<uint8_t> _response;

    const int _receiveTimeout = 10000; // milliseconds
    const int _writeTimeout = 10000;   // milliseconds
};
