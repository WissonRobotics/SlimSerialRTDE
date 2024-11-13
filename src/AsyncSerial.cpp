 
#include "SlimSerialRTDE/AsyncSerial.h"
#include <spdlog/fmt/bin_to_hex.h>
#include <spdlog/fmt/fmt.h> 

#include <string>
#include <algorithm>
#include <thread>
#include <mutex>
#include <iomanip>

#if defined(__linux__)
# include <linux/serial.h>
#endif
using namespace std;
using namespace boost;

template<typename T, int Width>
struct fixed_width_t
{
    fixed_width_t(T val) : val{ val } {}
    T val;
};

template <typename CharT, typename T, int Width>
auto& operator<<(std::basic_ostream<CharT>& out, const fixed_width_t<T, Width>& fixed_width_val)
{
    return out << std::setw(Width) << fixed_width_val.val;
}

template<typename CharT>
auto HexPrinter(std::basic_ostream<CharT>& stream)
{
    stream << std::hex << std::setfill(stream.widen(' '));
    return std::ostream_iterator<fixed_width_t<int, 3 >, CharT>(stream);
}

template<typename CharT>
auto ByteStreamToHexString(uint8_t *pdata,uint32_t databytes)
{
    std::basic_ostringstream<CharT> result{};
    std::copy(pdata, pdata+databytes, HexPrinter(result));
    return result.str();
}

AsyncSerial::AsyncSerial():m_logger(spdlog::default_logger()), circularBuffer(cbBuf, DEFAULT_CIRCULAR_BUF_SIZE),  io_context(), serial_port(io_context),serial_work(io_context),single_buf{}, ioContextThread(nullptr) {
        
};


AsyncSerial::~AsyncSerial() {
    close();
}

boost::system::error_code AsyncSerial::open(std::string dev_node, unsigned int baud, bool autoConnect)
{
    boost::system::error_code e = doOpen(dev_node, baud);
    if (isOpen()) {
        m_logger->info( "Serial {} is opened.",dev_node);
 
    }
    else {
        m_logger->error( "Serial {} fail to open with error: {}", dev_node,e.message());
 
    }
    if(autoConnect){
        m_logger->info( "Serial {} Auto Reconnect is turned on at 1s interval", dev_node);
        startAutoConnect(1000);
    }
    else{
        stopAutoConnect();
    }
    

    return e;

}
void AsyncSerial::stopIOContextThread(){
    if (serial_port.is_open()) {
        try {

            serial_port.cancel();
            m_logger->debug( "serial_port {} jobs canceled", m_portname);
        }
        catch (...) {
            
        }
        try {
            serial_port.close();
            m_logger->debug( "serial_port {} closed", m_portname);
        }
        catch (...) {
            m_logger->error( "serial_port {} closing error", m_portname);
        }
    }

    if(ioContextThread){
        if (!io_context.stopped()) {
            io_context.stop();
        }
        while(!io_context.stopped()){
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            m_logger->debug("Waiting for io_context to stop");
        }
        if(io_context.stopped()){
            m_logger->debug("serial port {} io_context is stopped",m_portname);
        }
 
        if(ioContextThread->joinable()){
  
            ioContextThread->join();
            
        }
        ioContextThread.reset();
        m_logger->debug("serial port {} ioContextThread stopped",m_portname);
    }
}


boost::system::error_code AsyncSerial::doOpen(std::string dev_node, unsigned int baud, flowControlType flowControl, unsigned int characterSize, parityType parity, stopBitsType stopBits)
{
    boost::system::error_code _errorCode;

    stopIOContextThread();

    resetTxRxCount();
    needToReconnectFlag = false;
    writeLocked = false;
  

    m_portname = dev_node;
    m_baudrate = baud;

    serial_port.open(m_portname, _errorCode);
    m_logger->info( "serial {} open with state: {}", m_portname, _errorCode.message());

    if (!serial_port.is_open()) {
        m_logger->error( "serial {} open failed", m_portname);
        return _errorCode;
    }
        

    serial_port.set_option(boost::asio::serial_port_base::baud_rate(m_baudrate));
    serial_port.set_option(boost::asio::serial_port_base::flow_control(flowControl));
    serial_port.set_option(boost::asio::serial_port_base::character_size(characterSize));
    serial_port.set_option(boost::asio::serial_port_base::parity(parity));
    serial_port.set_option(boost::asio::serial_port_base::stop_bits(stopBits));

#if defined(__linux__)
    auto native = serial_port.native_handle();
    struct serial_struct serial;
    ioctl(native, TIOCGSERIAL, &serial);
    serial.flags |= ASYNC_LOW_LATENCY; // (0x2000)
    ioctl(native, TIOCSSERIAL, &serial);
    usleep(2000); //required to make flush work, for some reason
    tcflush(native,TCIOFLUSH);

#endif

   
 
    ioContextThread = std::make_unique<std::jthread>(
        [this]()
        {
            io_context.restart();
            io_context.run(); 
        }
    );

    serial_port.async_read_some(
        boost::asio::buffer(single_buf),
        [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
            asyncReadHandler(error, bytes_transferred);
        });

    return _errorCode;
}

void AsyncSerial::setBaudrate(uint32_t baud) {
    serial_port.set_option(boost::asio::serial_port_base::baud_rate(baud));
}

void AsyncSerial::setAutoConnectPeriod(int autoReconnectTimeMs) {
    m_autoReconnectTimeMs = autoReconnectTimeMs;
}
 
void AsyncSerial::startAutoConnect(int autoReconnectTimeMs) {
    m_autoReconnectTimeMs = autoReconnectTimeMs;
    stopAutoConnect();
    reconnectThread = std::make_unique<std::jthread>(
        [this](std::stop_token stop_token)
        {
            while (!stop_token.stop_requested())
            {
                std::unique_lock<std::mutex> lock(reconnectMutex);
                needToReconnectFlag = false;
                reconnectCV.wait_for(lock, std::chrono::milliseconds(m_autoReconnectTimeMs), [&]() {return (needToReconnectFlag || stop_token.stop_requested()); });


                if (needToReconnectFlag && !stop_token.stop_requested()) {
                    m_logger->warn( "Serialport {} got a reconnect command", m_portname);
                     
                    doClose();

                    m_logger->warn( "Serialport {} reconnecting", m_portname);
                    doOpen(m_portname, m_baudrate);
                }
                else {
                    if (!isOpen()) {
                        m_logger->warn( "Serialport {} detects disconnected", m_portname);
                        
                        doClose();

                        m_logger->warn( "Serialport {} reconnecting", m_portname);
                        doOpen(m_portname, m_baudrate);

                    }
                    else {
                        //LOG_F(2, "serial autoconnect monitoring");
                    }
                }

            };
        }
    );
}
void AsyncSerial::stopAutoConnect() {
    try {
        if (reconnectThread) {
            m_logger->debug( "Serialport {} stopping reconnect thread ", m_portname);
            reconnectThread->request_stop();
            reconnectCV.notify_one();
            if(reconnectThread->joinable()){
                reconnectThread->join();
            }
            reconnectThread.reset();
            m_logger->debug( "Serialport {} cleaning reconnect thread ", m_portname);
        }
    }
    catch (...) {
        m_logger->error( "Serialport {} fail to stop AutoConnect ", m_portname);
    }

}

void AsyncSerial::triggerReconnect() {
    if (reconnectThread) {
        std::unique_lock<std::mutex> lock(reconnectMutex);
        needToReconnectFlag = true;
        reconnectCV.notify_one();
        return;
    }
}

void AsyncSerial::asyncReadHandler(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (error) {
        if(!m_closing_state){
            m_logger->error( "Serialport Error when reading: {}", error.message());
        }
        setError(error.value());
        triggerReconnect();
        return;
    }

    circularBuffer.in(&single_buf[0], bytes_transferred);

    serial_port.async_read_some(
        boost::asio::buffer(single_buf),
        [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
            asyncReadHandler(error, bytes_transferred);
        });

    m_totalRxBytes += bytes_transferred;

    if(rxDataCallback)
        rxDataCallback();
} 

void AsyncSerial::asyncWriteHandler(const boost::system::error_code& error, std::size_t bytes_transferred)
{

    if (error) {
        if(!m_closing_state){
            m_logger->error( "Serialport Error when writing: {}", error.message());
        }
        setError(error.value());
        triggerReconnect();
        return;
    }
     
    std::unique_lock<std::mutex> lk(writeMtx);
    m_totalTxBytes += bytes_transferred;
    m_totalTxFrames++;
    writeLocked = false;
    lk.unlock();
    writeCv.notify_one();
}
bool AsyncSerial::isOpen() const
{
    return serial_port.is_open();
}

void AsyncSerial::doClose()
{
    m_closing_state = true;
    

    stopIOContextThread();
     

    m_closing_state = false;
}

 
void AsyncSerial::close()
{
    stopAutoConnect();
    doClose();

}


 
std::size_t AsyncSerial::transmit(const std::vector<uint8_t>& v)
{
    return transmit((uint8_t *)(v.data()),v.size());
}

std::size_t AsyncSerial::transmit(uint8_t *pData,uint16_t datasize)
{
    if (!isOpen()) {
        m_logger->error( "Fail to transmit, serial port {} is not opened", m_portname);
        return 0;
    }

    try {
        std::unique_lock<std::mutex> lk(writeMtx);

        writeCv.wait(lk, [this](){return !writeLocked;});

        writeLocked = true;

        //keep internal copy

        memcpy((uint8_t *)(&m_txBuffer[0]), pData, datasize); 
        
        m_logger->debug( "Transmitting {} nytes",datasize);
        m_logger->trace("[Tx] {}", spdlog::to_hex(std::begin(m_txBuffer), std::begin(m_txBuffer) + datasize));
   
 
        if (!isOpen()) {
            m_logger->error( "Fail to transmit, serial port {} is not opened", m_portname);
            writeLocked = false;
            writeCv.notify_one();
            return 0;
        }
        auto txedsize =  boost::asio::write(serial_port, boost::asio::buffer((uint8_t *)(&m_txBuffer[0]),datasize));
        m_totalTxBytes += txedsize;
        m_totalTxFrames++;
        if (datasize != txedsize) {
            m_logger->warn( "Transmitted only {} out of {} bytes", txedsize, datasize);
        }

        writeLocked = false;
        writeCv.notify_one();
        return txedsize;
    }
    catch (const std::exception& err) {
        m_logger->error( "Serial port {} start transmit error:{}", m_portname, err.what());
        triggerReconnect();
        return 0;
    }
 
}

void AsyncSerial::transmitAsync(const std::vector<uint8_t>& v){
    transmitAsync((uint8_t *)(v.data()),v.size());
}
 
 
void AsyncSerial::transmitAsync(uint8_t *pData,uint16_t datasize)
{
    if (!isOpen()) {
        m_logger->error( "Fail to transmit, serial port {} is not opened", m_portname);
        return;
    }
    try {
        std::unique_lock<std::mutex> lk(writeMtx);
        while (writeLocked) {
            writeCv.wait(lk);
        }
        writeLocked = true;
        lk.unlock();

        //keep internal copy
        memcpy((uint8_t *)(&m_txBuffer[0]), pData, datasize);
  
        m_logger->debug( "Transmitting {} nytes",datasize);
        m_logger->trace("[Tx] {}", spdlog::to_hex(std::begin(m_txBuffer), std::begin(m_txBuffer) + datasize));
   
        if (!isOpen()) {
            m_logger->error( "Fail to transmit, serial port {} is not opened", m_portname);
            writeLocked = false;
            return ;
        }
        boost::asio::async_write(
            serial_port,
            boost::asio::buffer((uint8_t *)(&m_txBuffer[0]), datasize),
            [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
                asyncWriteHandler(error, bytes_transferred);
            });
    }
    catch (const std::exception& err) {
        m_logger->error( "Serial port {} start transmitAsync error:{}", m_portname, err.what());
    }
}



 


void AsyncSerial::addRxDataCallback(std::function<void(void)> rxDataCallbackExt) {
    rxDataCallback = rxDataCallbackExt;
}


void AsyncSerial::setError(const int error)
{
    std::unique_lock<std::mutex> elk(errMtx);
    error_value = error;

}

int AsyncSerial::getError()  const
{
    std::unique_lock<std::mutex> lk(errMtx);
    return error_value;
}

void AsyncSerial::enableLogger(std::shared_ptr<spdlog::logger> ext_logger){
    m_logger = ext_logger;
}
void AsyncSerial::disableLogger(){
 
    if(spdlog::get("disabledLogger")){
        m_logger = spdlog::get("disabledLogger");
    }
    else{
        m_logger = spdlog::stdout_color_mt("disabledLogger");
    }
    m_logger->set_level(spdlog::level::off);
 
}
 
std::string AsyncSerial::toHexString(const std::vector<uint8_t>& data) {

    return ByteStreamToHexString<char>((uint8_t *)(data.data()),sizeof(data));
}      
std::string AsyncSerial::toHexString(uint8_t *pdata,uint32_t databytes) {

    return ByteStreamToHexString<char>(pdata,databytes);
}      


void AsyncSerial::resetTxRxCount() {
    m_totalRxBytes = 0;
    m_totalTxBytes = 0;
}