//
// created by linxif2008 on 03/22/2024
//

#include "can/can_bus.h"
#include <chrono>
#include <iostream>

#include "assert.h"

namespace usb2can
{


USB2CAN_OpenError CanBus::_openUSB2CAN(const std::string &port)
{
    using namespace std::chrono;
    
    // set basic protocol info
    // the USB2CAN device is realized on the virtual protocol, 
    // so the parameters here does not work at all.
    serial_.setPort(port);
    serial_.setBaudrate(921600);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial_.setTimeout(USB2CAN_TIMEOUT);

    serial_.open();
    if (!serial_.isOpen())
        return USB2CAN_OpenError::FAILED_TO_OPEN;    // failed to open device
    serial_.flush();                                 // flush the serial


    if (device_mode_ == USB2CAN_Mode::CAN)  // initialize the can setting if in can modes
    {
        // launch the can recv thread
        msg_recv_thread_ = std::make_shared<std::thread>(&CanBus::_canMsgRecvCheck, this);
        msg_recv_thread_->detach();

        // ================= stop CAN sending ================
        _write<USB2CAN_CANSTOP>(CAN_STOP_BAG_);
        std::this_thread::sleep_for(100ms);                // waiting for 100ms(clear the buffer)

        // ================ config the can bus ===============
        candevice_baudrate_set_bag_.baudrate = candevice_baudrate_type_;
        _write<CANDEVICE_BaudrateSetBag>(candevice_baudrate_set_bag_);
    } else
    {
        // ========= config the usb2can device ===============
        usb2can_baudrate_set_bag_.data = usb2can_baudrate_set_msg_;
        _write<USB2CAN_BaudrateSetBag>(usb2can_baudrate_set_bag_);

    }


    // the device is opened succesfully if we reach here

    // finally, set the usb2can opened flag
    usb2can_opened_flag_ = true;
    return USB2CAN_OpenError::SUCC;
}


void CanBus::_reset()
{
    // set serial status to not opened;
    if (usb2can_opened_flag_ && device_mode_ == USB2CAN_Mode::CAN)
    {
        msg_recv_thread_required_stop_ = 1;
        // wait until the msg recv thread exit
        while (msg_recv_thread_required_stop_ == 1);
        serial_.close();
    }
    usb2can_opened_flag_ = false;

    // set usb2can device configuration timeout
    usb2can_port_ = "";

    // set the usb2can baudrate
    // TODO: check if the 921600 baudrate is right
    usb2can_baudrate_set_msg_.baudrate = 921600;
    usb2can_baudrate_set_msg_.data_bit = USB2CAN_BaudrateDatabit::BITS_8;
    usb2can_baudrate_set_msg_.parity   = USB2CAN_BaudrateParity::PARITY_NONE;
    usb2can_baudrate_set_msg_.stop_bit = USB2CAN_BaudrateStopbit::STOPBIT_ONE;

    // set the can baudrate
    candevice_baudrate_type_ = CANDEVICE_BaudrateTypes::RATE_1M;

    // set msg recv thread running status
    msg_recv_thread_required_stop_ = 0;

    // set can_recv_handle_
    can_recv_handle_ = nullptr;
    canbag_recv_handle_ = nullptr;
}

USB2CAN_OpenError CanBus::_reconnect()
{
    // close the serial if the serial is opened
    if (usb2can_opened_flag_ && device_mode_ == USB2CAN_Mode::CAN)
    {
        msg_recv_thread_required_stop_ = 1;
        // wait until the msg recv thread exit
        while (msg_recv_thread_required_stop_ == 1);
        serial_.close();
    }
    // set serial status to not opened;
    usb2can_opened_flag_ = false;

    return _openUSB2CAN(usb2can_port_);
}

template<typename T>
void CanBus::_write(const T &data)
{
    int buf;
    const uint8_t *data_ptr = reinterpret_cast<const uint8_t *>(&data);
    for (i_ = 0; i_ < sizeof(data); i_++)
    {
        buf = (int)(data_ptr[i_]);
        if (debug)
            printf("0x%02hx, ", buf);
    }  
    if (debug)
        std::cerr << std::endl;
    serial_.write(reinterpret_cast<const uint8_t *>(&data), sizeof(data));
}


void CanBus::_receiveSuccHandle(const CAN_RecvMsg &recv)
{
    if (recv.can_frame_type == CAN_RecvFrameType::DATA_FRAME)
    {
        if (can_recv_handle_ != nullptr)
            can_recv_handle_(recv.can_id, recv.data, recv.can_data_len);
    } else
    {
        // TODO: handle remote frame
    }
}

void CanBus::_canMsgRecvCheck()
{
    // keep checking until required stop
    while (!msg_recv_thread_required_stop_)
    {
        // read one byte
        serial_.read(&can_recv_bag_.header, 1);
        // read bytes until a start byte is read
        if (can_recv_bag_.header != USB2CAN_RECV_START_BYTE)
            continue;
        
        // read command byte
        serial_.read(reinterpret_cast<uint8_t *>(&can_recv_bag_.command), 1);
        // read command data
        serial_.read(reinterpret_cast<uint8_t *>(&can_recv_bag_.msg), sizeof(can_recv_bag_.msg));
        // read ending byte
        serial_.read(&can_recv_bag_.endding, 1);
        if (can_recv_bag_.endding != USB2CAN_RECV_ENDING_BYTE)
            continue;       // detected bit absence during bag receiving
        
        // if we reached here, the can msg is correctly received
        if (canbag_recv_handle_ != nullptr)
            canbag_recv_handle_(std::ref(can_recv_bag_));
        
        switch (static_cast<CAN_RecvCmdType>(can_recv_bag_.command))
        {
        case CAN_RecvCmdType::HEART_BEAT:
            // handle heart beat command
            // FIXME: not work
            if (debug)
                std::cerr << "======heart beat received=====" << std::endl;
            break;
        case CAN_RecvCmdType::RECV_FAILURE:
            // handle can msg receive failure
            if (debug)
                std::cerr << "receive message failed" << std::endl;
            break;
        case CAN_RecvCmdType::RECV_SUCC:
            // handle can msg receive
            if (debug)
                std::cerr << "receive message successfully" << std::endl;
            _receiveSuccHandle(std::ref(can_recv_bag_.msg));
            break;
        case CAN_RecvCmdType::SEND_FAILURE:
            // handle can msg sending failure
            if (debug)
                std::cerr << "send message failed" << std::endl;
            break;
        case CAN_RecvCmdType::SEND_SUCC:
            // handle can msg sending success
            if (debug)
                std::cerr << "send message successfully" << std::endl;
            break;
        case CAN_RecvCmdType::BAUDRATE_SET_FAILURE:
            // handle baudrate setting failure
            // FIXME: not work
            if (debug)
                std::cerr << "set baudrate failed" << std::endl;
            usb2can_baudrate_setting_status = 1;
            candevice_baudrate_setting_status = 1;
            break;
        case CAN_RecvCmdType::BAUDRATE_SET_SUCC:
            // handle baudrate setting success
            // FIXME: not work
            if (debug)
                std::cerr << "set baudrate successfully" << std::endl;
            usb2can_baudrate_setting_status = 2;
            candevice_baudrate_setting_status = 2;
            break;
        case CAN_RecvCmdType::COMMUNICATION_ERROR:
            // handle communication error
            break;
        default:
            continue;
        }
    }

    msg_recv_thread_required_stop_ = 0;
}



CanBus::CanBus()
{
    usb2can_opened_flag_ = false;
    _reset();
}

CanBus::~CanBus()
{
    close();
}

std::vector<serial::PortInfo> CanBus::listDevices()
{
    return serial::list_ports();
}


USB2CAN_OpenError CanBus::openDeviceWithHID(const std::string &hid)
{
    if (usb2can_opened_flag_)
        return USB2CAN_OpenError::DEVICE_ALREADY_OPENED;
    for (const serial::PortInfo port_info : serial::list_ports())
    {
        // if found the target device
        if (port_info.hardware_id.compare(hid) == 0)
        {
            // update the default port
            usb2can_port_ = port_info.port;
            return _openUSB2CAN(port_info.port);
        }
    }

    // target device not found
    return USB2CAN_OpenError::HARDWIRE_NOT_FOUND;
}

void CanBus::setCanDataFrameRecvCallBack(std::function<void(const uint32_t &, const uint8_t *, const uint8_t &)> can_recv_handle)
{
    assert(device_mode_ == USB2CAN_Mode::CAN);
    can_recv_handle_ = can_recv_handle;
}

void CanBus::setCanBagRecvCallBack(std::function<void(const CAN_RecvBag &)> canbag_recv_handle)
{
    assert(device_mode_ == USB2CAN_Mode::CAN);
    canbag_recv_handle_ = canbag_recv_handle;
}

USB2CAN_OpenError CanBus::enableUARTMode(const USB2CAN_BaudrateSetMsg &usb2can_setting)
{
    device_mode_ = USB2CAN_Mode::UART;
    usb2can_baudrate_set_msg_ = usb2can_setting;

    // check if the device is opened
    if (usb2can_opened_flag_)
    {
        // reopen the device to update the setting
        return _reconnect();
    }
    return USB2CAN_OpenError::SUCC;
}

USB2CAN_OpenError CanBus::enableCANMode(const CANDEVICE_BaudrateTypes &candevice_baudrate_type)
{
    device_mode_ = USB2CAN_Mode::CAN;
    candevice_baudrate_type_ = candevice_baudrate_type;

    // check if the device is opened
    if (usb2can_opened_flag_)
    {
        // reopen the device to update the setting
        return _reconnect();
    }
    return USB2CAN_OpenError::SUCC;   
}

USB2CAN_Mode CanBus::getDeviceMode() const
{
    return device_mode_;
}

bool CanBus::isOpened() const
{
    return usb2can_opened_flag_;
}

template<typename T>
void CanBus::write(const T &data)
{
    assert(usb2can_opened_flag_ == true);
    _write<T>(reinterpret_cast<const uint8_t *>(&data), sizeof(data));
}


void CanBus::write(const uint8_t *data, const size_t &len)
{
    assert(usb2can_opened_flag_ == true);
    serial_.write(data, len);
}


void CanBus::sendStandardCanMsg(const uint32_t &id, const uint8_t *data, const uint8_t &len)
{
    assert(usb2can_opened_flag_ == true && device_mode_ == USB2CAN_Mode::CAN);
    assert(len <= 8);
    can_send_bag_buff_.msg.cmd_type = CAN_SendCmdType::CAN_SEND;
    can_send_bag_buff_.msg.can_id = id;

    memcpy(can_send_bag_buff_.msg.data, data, len);
    if (len < 8)
        memset(can_send_bag_buff_.msg.data + len, 0x00, 8 - len);

    _write<CAN_SendBag>(can_send_bag_buff_);
}

size_t CanBus::readUART(uint8_t *data, const size_t &len)
{
    assert(usb2can_opened_flag_ == true && device_mode_ == USB2CAN_Mode::UART);
    return serial_.read(data, len);
}

void CanBus::close()
{
    // set serial status to not opened;
    if (usb2can_opened_flag_)
    {
        msg_recv_thread_required_stop_ = 1;
        // wait until the msg recv thread exit
        while (msg_recv_thread_required_stop_ == 1);
        serial_.close();
    }
    usb2can_opened_flag_ = false;

    // set can_recv_handle_
    can_recv_handle_ = nullptr;
    canbag_recv_handle_ = nullptr;
}


};