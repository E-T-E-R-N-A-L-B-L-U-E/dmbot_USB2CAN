//
// created by linxif2008 on 03/22/2024
//

#include "can/can_bus.h"
#include <chrono>
#include <iostream>

#include "assert.h"

namespace usb2can
{


USB2CAN_OpenError CanBus::_openUSB2CAN(const std::string &port, const int64_t &timeout_us)
{
    using namespace std::chrono;
    // set basic protocol info
    serial_.setPort(port);
    serial_.setBaudrate(921600);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial_.setTimeout(USB2CAN_TIMEOUT);

    serial_.open();
    if (!serial_.isOpen())
        return USB2CAN_OpenError::FAILED_TO_OPEN;    // failed to open device

    // launch the can recv thread
    msg_recv_thread_ = std::make_shared<std::thread>(&CanBus::_msgRecvCheck, this);
    msg_recv_thread_->detach();

    // ===================================================
    // ================= stop CAN sending ================
    // ===================================================
    _write<USB2CAN_CANSTOP>(CAN_STOP_BAG_);
    std::this_thread::sleep_for(100ms);                // waiting for 100ms

    // ===================================================
    // === shake hand with the usb2can device ============
    // ===================================================
    _write<USB2CAN_SHAKEHANDS>(SHAKE_HAND_BAG_);        // shake hand once
    std::this_thread::sleep_for(1000ms);                // waiting for 1s
    _write<USB2CAN_SHAKEHANDS>(SHAKE_HAND_BAG_);        // shakehand again to setting time


    // ===================================================
    // ========= config the usb2can device ===============
    // ===================================================
    // usb2can_baudrate_setting_status = 0;
    usb2can_baudrate_set_bag_.data = usb2can_baudrate_set_msg_;
    _write<USB2CAN_BaudrateSetBag>(usb2can_baudrate_set_bag_);
    
    // // wait for the reply until timeout
    // std::chrono::system_clock::time_point time_send_configuration = std::chrono::system_clock::now();
    // while (usb2can_baudrate_setting_status == 0)
    //     if (timeout_us != 0 && std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_send_configuration).count() > timeout_us)
    //         break;
    
    // // check the setting result
    // if (usb2can_baudrate_setting_status == 0)   // no reply from the target device
    // {
    //     std::cerr << 1 << std::endl;
    //     serial_.close();
    //     return USB2CAN_OpenError::NO_REPLY;
    // }
    // if (usb2can_baudrate_setting_status == 1)   // failed to configure the target device
    // {
    //     serial_.close();
    //     return USB2CAN_OpenError::BAUDRATE_SETTING_ERROR;
    // }


    // ===================================================
    // ================ config the can bus ===============
    // ===================================================
    // candevice_baudrate_setting_status = 0;
    candevice_baudrate_set_bag_.baudrate = candevice_baudrate_type_;
    _write<CANDEVICE_BaudrateSetBag>(candevice_baudrate_set_bag_);

    // // wait for the reply until timeout
    // std::cerr << "wait time: " << timeout_us << std::endl;
    // std::chrono::system_clock::time_point time_send_configuration = std::chrono::system_clock::now();
    // while (candevice_baudrate_setting_status == 0)
    //     if (timeout_us != 0 && std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_send_configuration).count() > timeout_us)
    //         break;
    
    // // check the setting result
    // if (candevice_baudrate_setting_status == 0)   // no reply from the target device
    // {
    //     std::cerr << 1 << std::endl;
    //     serial_.close();
    //     return USB2CAN_OpenError::NO_REPLY;
    // }
    // if (candevice_baudrate_setting_status == 1)   // failed to configure the target device
    // {
    //     serial_.close();
    //     return USB2CAN_OpenError::BAUDRATE_SETTING_ERROR;
    // }



    // the device is opened succesfully if we reach here

    // finally, set the usb2can opened flag
    usb2can_opened_flag_ = true;
    return USB2CAN_OpenError::SUCC;
}

void CanBus::_receiveSuccHandle()
{
    if (can_recv_msg_.can_frame_type == CAN_RecvFrameType::DATA_FRAME)
    {
        if (can_recv_handle_ != nullptr)
            can_recv_handle_(can_recv_msg_.can_id, can_recv_msg_.data, can_recv_msg_.can_data_len);
    } else
    {
        // TODO: handle remote frame
    }
}

void CanBus::_msgRecvCheck()
{
    // keep checking until required stop
    while (!msg_recv_thread_required_stop_)
    {
        // read one byte
        serial_.read(&recv_buff_, 1);
        // read bytes until a start byte is read
        if (recv_buff_ != USB2CAN_RECV_START_BYTE)
            continue;
        
        // read command byte
        serial_.read(&recv_buff_, 1);
        // read command data
        serial_.read((uint8_t *)&can_recv_msg_, sizeof(can_recv_msg_));
        // read ending byte
        serial_.read(&recv_buff2_, 1);
        if (recv_buff2_ != USB2CAN_RECV_ENDING_BYTE)
            continue;       // detected bit absence during bag receiving
        switch (static_cast<CAN_RecvCmdType>(recv_buff_))
        {
        case CAN_RecvCmdType::HEART_BEAT:
            // handle heart beat command
            break;
        case CAN_RecvCmdType::RECV_FAILURE:
            // handle can msg receive failure
            break;
        case CAN_RecvCmdType::RECV_SUCC:
            // handle can msg receive
            _receiveSuccHandle();
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
            usb2can_baudrate_setting_status = 1;
            candevice_baudrate_setting_status = 1;
            break;
        case CAN_RecvCmdType::BAUDRATE_SET_SUCC:
            // handle baudrate setting success
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
}

void CanBus::_reset()
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

    // set usb2can device configuration timeout
    usb2can_config_timeout_ = 0;
    usb2can_port_ = "";

    // set the usb2can baudrate
    // TODO: check if the 115200 baudrate is right
    usb2can_baudrate_set_msg_.baudrate = 115200;
    usb2can_baudrate_set_msg_.data_bit = USB2CAN_BaudrateDatabit::BITS_8;
    usb2can_baudrate_set_msg_.parity   = USB2CAN_BaudrateParity::PARITY_NONE;
    usb2can_baudrate_set_msg_.stop_bit = USB2CAN_BaudrateStopbit::STOPBIT_ONE;

    // set the can baudrate
    candevice_baudrate_type_ = CANDEVICE_BaudrateTypes::RATE_1M;

    // set msg recv thread running status
    msg_recv_thread_required_stop_ = 0;

    // set can_recv_handle_
    can_recv_handle_ = nullptr;
}

USB2CAN_OpenError CanBus::_reconnect()
{
    // close the serial if the serial is opened
    if (usb2can_opened_flag_)
    {
        msg_recv_thread_required_stop_ = 1;
        // wait until the msg recv thread exit
        while (msg_recv_thread_required_stop_ == 1);
        serial_.close();
    }
    // set serial status to not opened;
    usb2can_opened_flag_ = false;

    return _openUSB2CAN(usb2can_port_, usb2can_config_timeout_);
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


CanBus::CanBus()
{
    usb2can_opened_flag_ = false;
    _reset();
}

CanBus::~CanBus()
{
    close();
}

std::vector<serial::PortInfo> CanBus::listDevices() const
{
    return serial::list_ports();
}


USB2CAN_OpenError CanBus::openDeviceWithHID(const std::string &hid, const int64_t &timeout_us)
{
    if (usb2can_opened_flag_)
        return USB2CAN_OpenError::DEVICE_ALREADY_OPENED;
    for (const serial::PortInfo port_info : serial::list_ports())
    {
        // if found the target device
        if (port_info.hardware_id.compare(hid) == 0)
        {
            // update the default configuration timeout value
            usb2can_config_timeout_ = timeout_us;
            // update the default port
            usb2can_port_ = port_info.port;
            return _openUSB2CAN(port_info.port, usb2can_config_timeout_);
        }
    }

    // target device not found
    return USB2CAN_OpenError::HARDWIRE_NOT_FOUND;
}

void CanBus::setCanRecvCallBack(void (*can_recv_handle)(const uint32_t &, const uint8_t *, const uint8_t &))
{
    can_recv_handle_ = can_recv_handle;
}

USB2CAN_OpenError CanBus::setUSB2CAN(const USB2CAN_BaudrateSetMsg &usb2can_setting)
{
    usb2can_baudrate_set_msg_ = usb2can_setting;

    // check if the device is opened
    if (usb2can_opened_flag_)
    {
        // reopen the device to update the setting
        return _reconnect();
    }
    return USB2CAN_OpenError::SUCC;
}

USB2CAN_OpenError CanBus::setCanDeviceBaudrate(const CANDEVICE_BaudrateTypes &candevice_baudrate_type)
{
    candevice_baudrate_type_ = candevice_baudrate_type;

    // check if the device is opened
    if (usb2can_opened_flag_)
    {
        // reopen the device to update the setting
        return _reconnect();
    }
    return USB2CAN_OpenError::SUCC;   
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
    assert(usb2can_opened_flag_ == true);
    assert(len <= 8);
    can_send_bag_buff_.msg.can_id = id;

    memcpy(can_send_bag_buff_.msg.data, data, len);
    if (len < 8)
        memset(can_send_bag_buff_.msg.data + len, 0x00, 8 - len);

    _write<CAN_SendBag>(can_send_bag_buff_);
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
}


};