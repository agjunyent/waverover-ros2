#include "waverover_controller/uart_serial_port.hpp"

UARTSerialPort::UARTSerialPort(const std::string& port_name, int baudrate)
{
    std::string port = port_name;
    std::cout << "Opening " << port << " at baudrate " << baudrate << "..." << std::endl;

    try {
        _serial = std::make_unique<serial_cpp::Serial>(port, baudrate, serial_cpp::Timeout::simpleTimeout(1000));
    } catch (const std::exception& e) {
        std::cerr << "Failed to open serial port: " << e.what() << std::endl;
        return;
    }
}

UARTSerialPort::~UARTSerialPort()
{
    if (_serial && _serial->isOpen()) {
        _serial->close();
    }
}

void UARTSerialPort::listAllPorts()
{
    // List all serial ports
    std::vector<serial_cpp::PortInfo> devices_found = serial_cpp::list_ports();
    std::cout << "Detected Serial ports:";

    for (const auto& device : devices_found)
    {
        std::cout << device.port << std::endl;
    }

    std::cout << (devices_found.empty() ? " None." : "\n----") << std::endl;
}

void UARTSerialPort::sendRequestSync(const std::string& text)
{
    std::lock_guard<std::mutex> lock(_serial_mutex);
    if (!isAvailable()) {
        std::cerr << "Serial Port is down!" << std::endl;
        return;
    }

    std::string message = text + "\r\n";
    _serial->write(message);
}

bool UARTSerialPort::getResponseSync(const std::string& command, std::string& response)
{
    std::lock_guard<std::mutex> lock(_serial_mutex);
    if (!isAvailable()) {
        std::cerr << "Serial Port is down!" << std::endl;
        return false;
    }

    _serial->write(command + "\r\n");

    try {
        std::string data = _serial->readline(1024, "\n");
        response = data;
        // std::cout << "Message received: " << response << std::endl;
        return true;
    } catch (...) {
        std::cerr << "Timeout or read error while waiting for response." << std::endl;
        return false;
    }
}

bool UARTSerialPort::sendRequestSync(const std::vector<uint8_t>& data)
{
    std::lock_guard<std::mutex> lock(_serial_mutex);
    _serial->write(data);
    return true;
}

bool UARTSerialPort::readResponse()
{
    std::lock_guard<std::mutex> lock(_serial_mutex);
    std::string data = _serial->read(1024);
    _response.insert(_response.end(), data.begin(), data.end());
    return false;
}

bool UARTSerialPort::isAvailable() const
{
    if (_serial && _serial->isOpen())
        return true;

    std::cerr << "Trying to recover the Serial Port..." << std::endl;
    try {
        _serial->open();
        return _serial->isOpen();
    } catch (...) {
        return false;
    }
}

bool UARTSerialPort::isOpen() const
{
    return _serial && _serial->isOpen();
}

bool UARTSerialPort::isClosed() const
{
    return _serial && !_serial->isOpen();
}
