/**
 * @file SerialPort.hpp
 * @brief Serial port manager using Boost.Asio for async communication
 */

#pragma once

#include "FirmwareProtocol.hpp"
#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <condition_variable>

#ifdef _WIN32
#include <windows.h>
// Prevent Windows API macros from conflicting with C++ enum values
#ifdef IDLE
#undef IDLE
#endif
#endif

namespace robot_controller {
namespace firmware {

using boost::asio::serial_port;
using boost::asio::io_context;

// ============================================================================
// Serial Port Configuration
// ============================================================================

struct SerialConfig {
    std::string portName;
    int baudRate;
    int dataBits;
    int stopBits;
    enum class Parity { None, Odd, Even } parity;
    enum class FlowControl { None, Hardware, Software } flowControl;

    SerialConfig()
        : portName(""),
          baudRate(115200),
          dataBits(8),
          stopBits(1),
          parity(Parity::None),
          flowControl(FlowControl::None) {}
};

// ============================================================================
// Callbacks
// ============================================================================

using DataReceivedCallback = std::function<void(const std::vector<uint8_t>&)>;
using LineReceivedCallback = std::function<void(const std::string&)>;
using ErrorCallback = std::function<void(const std::string&)>;
using ConnectionCallback = std::function<void(bool connected)>;

// ============================================================================
// Serial Port Manager
// ============================================================================

/**
 * Manages serial port communication with async read/write
 */
class SerialPortManager {
public:
    SerialPortManager();
    ~SerialPortManager();

    // ========================================================================
    // Connection Management
    // ========================================================================

    bool open(const SerialConfig& config);
    void close();
    bool isOpen() const;
    static std::vector<std::string> getAvailablePorts();

    // ========================================================================
    // Data Transfer
    // ========================================================================

    void write(const std::vector<uint8_t>& data);
    void writeLine(const std::string& line);
    void writeBytes(const uint8_t* data, size_t length);
    std::string readLine(int timeoutMs = 1000);
    void flush();

    // ========================================================================
    // Callbacks
    // ========================================================================

    void setDataCallback(DataReceivedCallback callback) {
        dataCallback_ = callback;
    }

    void setLineCallback(LineReceivedCallback callback) {
        lineCallback_ = callback;
    }

    void setErrorCallback(ErrorCallback callback) {
        errorCallback_ = callback;
    }

    void setConnectionCallback(ConnectionCallback callback) {
        connectionCallback_ = callback;
    }

    // ========================================================================
    // Statistics
    // ========================================================================

    size_t getBytesReceived() const { return bytesReceived_; }
    size_t getBytesSent() const { return bytesSent_; }
    void resetStatistics() { bytesReceived_ = 0; bytesSent_ = 0; }

private:
    io_context ioContext_;
    std::unique_ptr<serial_port> port_;
    std::thread ioThread_;
    std::atomic<bool> running_;

    // Read buffer
    std::array<uint8_t, 1024> readBuffer_;
    std::string lineBuffer_;
    std::mutex lineBufferMutex_;

    // Received lines queue
    std::queue<std::string> receivedLines_;
    std::mutex receivedLinesMutex_;
    std::condition_variable receivedLinesCv_;

    // Write queue
    std::mutex writeMutex_;
    std::queue<std::vector<uint8_t>> writeQueue_;
    std::atomic<bool> writeInProgress_;

    // Callbacks
    DataReceivedCallback dataCallback_;
    LineReceivedCallback lineCallback_;
    ErrorCallback errorCallback_;
    ConnectionCallback connectionCallback_;

    // Statistics
    std::atomic<size_t> bytesReceived_;
    std::atomic<size_t> bytesSent_;

    // Internal methods
    void startAsyncRead();
    void handleRead(const boost::system::error_code& error, size_t bytesTransferred);
    void processWriteQueue();
    void handleWrite(const boost::system::error_code& error, size_t bytesTransferred);
    void runIoContext();
};

// ============================================================================
// Implementation
// ============================================================================

inline SerialPortManager::SerialPortManager()
    : running_(false),
      writeInProgress_(false),
      bytesReceived_(0),
      bytesSent_(0) {
}

inline SerialPortManager::~SerialPortManager() {
    close();
}

inline bool SerialPortManager::open(const SerialConfig& config) {
    try {
        close();

        port_ = std::make_unique<serial_port>(ioContext_, config.portName);

        // Configure port
        port_->set_option(serial_port::baud_rate(config.baudRate));
        port_->set_option(serial_port::character_size(config.dataBits));

        switch (config.stopBits) {
            case 1: port_->set_option(serial_port::stop_bits(serial_port::stop_bits::one)); break;
            case 2: port_->set_option(serial_port::stop_bits(serial_port::stop_bits::two)); break;
        }

        switch (config.parity) {
            case SerialConfig::Parity::None:
                port_->set_option(serial_port::parity(serial_port::parity::none)); break;
            case SerialConfig::Parity::Odd:
                port_->set_option(serial_port::parity(serial_port::parity::odd)); break;
            case SerialConfig::Parity::Even:
                port_->set_option(serial_port::parity(serial_port::parity::even)); break;
        }

        switch (config.flowControl) {
            case SerialConfig::FlowControl::None:
                port_->set_option(serial_port::flow_control(serial_port::flow_control::none)); break;
            case SerialConfig::FlowControl::Hardware:
                port_->set_option(serial_port::flow_control(serial_port::flow_control::hardware)); break;
            case SerialConfig::FlowControl::Software:
                port_->set_option(serial_port::flow_control(serial_port::flow_control::software)); break;
        }

        running_ = true;
        startAsyncRead();

        ioThread_ = std::thread(&SerialPortManager::runIoContext, this);

        if (connectionCallback_) {
            connectionCallback_(true);
        }

        return true;

    } catch (const std::exception& e) {
        if (errorCallback_) {
            errorCallback_(std::string("Failed to open port: ") + e.what());
        }
        return false;
    }
}

inline void SerialPortManager::close() {
    running_ = false;

    if (port_ && port_->is_open()) {
        boost::system::error_code ec;
        port_->cancel(ec);
        port_->close(ec);
    }

    ioContext_.stop();

    if (ioThread_.joinable()) {
        ioThread_.join();
    }

    ioContext_.restart();
    port_.reset();

    if (connectionCallback_) {
        connectionCallback_(false);
    }
}

inline bool SerialPortManager::isOpen() const {
    return port_ && port_->is_open();
}

inline std::vector<std::string> SerialPortManager::getAvailablePorts() {
    std::vector<std::string> ports;

#ifdef _WIN32
    // Windows: Check COM1-COM256
    for (int i = 1; i <= 256; ++i) {
        char portName[16];
        snprintf(portName, sizeof(portName), "COM%d", i);

        HANDLE hPort = CreateFileA(
            portName, GENERIC_READ | GENERIC_WRITE, 0, nullptr,
            OPEN_EXISTING, 0, nullptr);

        if (hPort != INVALID_HANDLE_VALUE) {
            ports.push_back(portName);
            CloseHandle(hPort);
        }
    }
#else
    // Linux/macOS placeholder
    const char* patterns[] = {
        "/dev/ttyUSB",
        "/dev/ttyACM",
        "/dev/tty.usb"
    };

    for (const char* pattern : patterns) {
        for (int i = 0; i < 10; ++i) {
            std::string portName = std::string(pattern) + std::to_string(i);
            // Check if exists - simplified
            ports.push_back(portName);
        }
    }
#endif

    return ports;
}

inline void SerialPortManager::write(const std::vector<uint8_t>& data) {
    if (!isOpen()) return;

    {
        std::lock_guard<std::mutex> lock(writeMutex_);
        writeQueue_.push(data);
    }

    boost::asio::post(ioContext_, [this]() {
        if (!writeInProgress_) {
            processWriteQueue();
        }
    });
}

inline void SerialPortManager::writeLine(const std::string& line) {
    std::vector<uint8_t> data(line.begin(), line.end());
    data.push_back('\n');
    write(data);
}

inline void SerialPortManager::writeBytes(const uint8_t* data, size_t length) {
    write(std::vector<uint8_t>(data, data + length));
}

inline std::string SerialPortManager::readLine(int timeoutMs) {
    std::unique_lock<std::mutex> lock(receivedLinesMutex_);

    if (receivedLinesCv_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                                   [this] { return !receivedLines_.empty(); })) {
        std::string line = receivedLines_.front();
        receivedLines_.pop();
        return line;
    }

    return "";
}

inline void SerialPortManager::flush() {
#ifdef _WIN32
    if (port_ && port_->is_open()) {
        FlushFileBuffers(port_->native_handle());
    }
#endif
}

inline void SerialPortManager::startAsyncRead() {
    if (!running_ || !port_ || !port_->is_open()) return;

    port_->async_read_some(
        boost::asio::buffer(readBuffer_),
        [this](const boost::system::error_code& error, size_t bytesTransferred) {
            handleRead(error, bytesTransferred);
        });
}

inline void SerialPortManager::handleRead(
    const boost::system::error_code& error, size_t bytesTransferred) {

    if (error) {
        if (error != boost::asio::error::operation_aborted) {
            if (errorCallback_) {
                errorCallback_("Read error: " + error.message());
            }
        }
        return;
    }

    bytesReceived_ += bytesTransferred;

    // Process received data
    if (dataCallback_) {
        std::vector<uint8_t> data(readBuffer_.begin(),
                                   readBuffer_.begin() + bytesTransferred);
        dataCallback_(data);
    }

    // Process lines
    {
        std::lock_guard<std::mutex> lock(lineBufferMutex_);
        for (size_t i = 0; i < bytesTransferred; ++i) {
            char c = static_cast<char>(readBuffer_[i]);
            if (c == '\n' || c == '\r') {
                if (!lineBuffer_.empty()) {
                    if (lineCallback_) {
                        lineCallback_(lineBuffer_);
                    }

                    {
                        std::lock_guard<std::mutex> linesLock(receivedLinesMutex_);
                        receivedLines_.push(lineBuffer_);
                    }
                    receivedLinesCv_.notify_one();

                    lineBuffer_.clear();
                }
            } else {
                lineBuffer_ += c;
            }
        }
    }

    // Continue reading
    startAsyncRead();
}

inline void SerialPortManager::processWriteQueue() {
    std::vector<uint8_t> data;

    {
        std::lock_guard<std::mutex> lock(writeMutex_);
        if (writeQueue_.empty()) {
            writeInProgress_ = false;
            return;
        }
        data = std::move(writeQueue_.front());
        writeQueue_.pop();
        writeInProgress_ = true;
    }

    boost::asio::async_write(
        *port_,
        boost::asio::buffer(data),
        [this](const boost::system::error_code& error, size_t bytesTransferred) {
            handleWrite(error, bytesTransferred);
        });
}

inline void SerialPortManager::handleWrite(
    const boost::system::error_code& error, size_t bytesTransferred) {

    if (error) {
        if (errorCallback_) {
            errorCallback_("Write error: " + error.message());
        }
        writeInProgress_ = false;
        return;
    }

    bytesSent_ += bytesTransferred;

    processWriteQueue();
}

inline void SerialPortManager::runIoContext() {
    while (running_) {
        try {
            ioContext_.run();
            if (running_) {
                ioContext_.restart();
            }
        } catch (const std::exception& e) {
            if (errorCallback_) {
                errorCallback_(std::string("IO error: ") + e.what());
            }
        }
    }
}

} // namespace firmware
} // namespace robot_controller
