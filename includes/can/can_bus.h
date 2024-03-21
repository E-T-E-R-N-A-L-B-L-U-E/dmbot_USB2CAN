//
// created by linxif2008 on 03/21/2024
//

#ifndef DMBOT_USB2CAN_CAN_BUS_H
#define DMBOT_USB2CAN_CAN_BUS_H

#include <thread>
#include <vector>
#include <string>
#include <array>

#include "serial/serial.h"
#include "can/message.h"

namespace usb2can
{

enum class USB2CAN_OpenError : uint8_t
{
    SUCC = 0,                       // the device is opened successfully
    HARDWIRE_NOT_FOUND = 1,         // the target device is not found
    FAILED_TO_OPEN = 2,             // failed to open the target device (probably the target device is in used or permission denied)
    NO_REPLY = 3,                   // no target device reply received
    BAUDRATE_SETTING_ERROR = 4,     // failed to config the target device
    CAN_BAUDRATE_SETTING_ERROR = 5, // failed to config the can bus
    DEVICE_ALREADY_OPENED = 6,      // the device is already opened
};


class CanBus
{
private:
    const bool           debug = true;

    serial::Timeout      USB2CAN_TIMEOUT = serial::Timeout::simpleTimeout(serial::Timeout::max());  // the protocol reading will not timeout
    int64_t              usb2can_config_timeout_;               // the usb2can configuration timeout set by the user
    std::string          usb2can_port_;                         // the usb2can port set by the user
    size_t               i_;                                    // iterator for for loop


    // Declare all the required structs at first to avoid later memory allocation.
    // runtime memory allocation may cause frequent page fault, which is fatal in realtime programming.
    const USB2CAN_CANSTOP      CAN_STOP_BAG_;      // the bag to stop can sending
    const USB2CAN_HEARTBEATS   HEARTBEATS_BAG_;    // the heartbeat bag
    const USB2CAN_SHAKEHANDS   SHAKE_HAND_BAG_;    // the shake hand bag


    USB2CAN_BaudrateSetMsg      usb2can_baudrate_set_msg_;   // msg to set the baudrate of the usb2can device
    USB2CAN_BaudrateSetBag      usb2can_baudrate_set_bag_;   // struct to store the baudrate settings
    uint8_t                     usb2can_baudrate_setting_status;    // the status of baudrate setting
                                                                    // 0: no baudrate setting result received
                                                                    // 1: baudrate setting failure
                                                                    // 2: baudrate setting success


    CANDEVICE_BaudrateSetBag    candevice_baudrate_set_bag_; // msg to set the baudrate of the can device     
    CANDEVICE_BaudrateTypes     candevice_baudrate_type_;    // store the baudrate type
    uint8_t                     candevice_baudrate_setting_status;  // the status of candevice baudrate setting
                                                                    // 0: no baudrate setting result received
                                                                    // 1: baudrate setting failure
                                                                    // 2: baudrate setting success


    uint8_t                         recv_buff_, recv_buff2_;        // byte buffer used to receive data
    CAN_RecvMsg                     can_recv_msg_;                  // store the received can data
    std::shared_ptr<std::thread>    msg_recv_thread_;               // the thread which listen to the protocol
    uint8_t                         msg_recv_thread_required_stop_; // if the msg_recv_thread is required to stop
                                                                    // 0: default
                                                                    // 1: required stop
                                                                    // 2: already stop


    CAN_SendBag                     can_send_bag_buff_;                 // buffer for can bag sending
    CAN_SendMsg                     can_send_standard_can_msg_buff_;    // buffer for function sendStandardCanMsg


    /**
     * @attention this function should not block the program
     * The function ptr for can msg receive callback.
     * This function will be called when a new can message is received in msg_recv_thread_
     * 
     * @param uint32_t:     the received can id
     * @param uint8_t[]:    the received can data
     * @param uint8_t:      the received data len
    */
    void (*can_recv_handle_)(const uint32_t &, const uint8_t *, const uint8_t &);


    serial::Serial  serial_;                                // the serial protocol of the USB2CAN device
    bool            usb2can_opened_flag_;                   // if the usb2can device is opened

    USB2CAN_OpenError _openUSB2CAN(const std::string &port, const int64_t &timeout_us);              // open the usb2can device with port
    
    void _receiveSuccHandle();                               // handle receive success in msgRecvCheck()
    void _msgRecvCheck();                                    // the function checking if new msg is received from usb2can device
                                                            // this function is running in msg_recv_thread
    void _reset();                                          // reset all values to default
    USB2CAN_OpenError _reconnect();                         // close the device and reconnect

    template<typename T>
    void _write(const T &data);                             // write data to protocol without protocol open check
public:
    CanBus();
    ~CanBus();

    /**
     * lists the serial ports info of the found devices
     * 
     * @param return: the serial ports info of the found devices
    */
    std::vector<serial::PortInfo> listDevices() const;

    /**
     * @attention It will be more efficient to config the device before open it
     * open the USB2CAN device with target hardwire id
     * 
     * @param hid: the hardwire id of the device with need to be opened
     * @param timeout: the time(us) waiting for the reply from the usb2can device
     * 
     * @param return: the device is successfully opened or not
    */
    USB2CAN_OpenError openDeviceWithHID(const std::string &hid, const int64_t &timeout_us = 0);


    /**
     * @attention this recall function should not block the program
     * Set the function called when new can message is received
     * 
     * @param uint32_t:     the received can id
     * @param uint8_t[]:    the received can data
     * @param uint8_t:      the received data len
    */
    void setCanRecvCallBack(void (*can_recv_handle)(const uint32_t &, const uint8_t *, const uint8_t &));


    /**
     * Set the basic information of the usb2can device.(Baudrate, databit, stopbit, parity).
     * If the device is not opened, the information will be updated after the device is opened.
     * If the device is opened, the information will be updated at once.
     * 
     * @param usb2can_setting: the setting struct of the usb2can device
     * 
     * @param return: setting successfully or not (setting is always success if device is not opened)
    */
    USB2CAN_OpenError setUSB2CAN(const USB2CAN_BaudrateSetMsg &usb2can_setting);

    /**
     * Set the baudrate of the usb2can device.
     * If the device is not opened, the information will be updated after the device is opened.
     * If the device is opened, the information will be updated at once.
     * 
     * @param usb2can_setting: the setting struct of the usb2can device
     * 
     * @param return: setting successfully or not (setting is always success if device is not opened)
    */
    USB2CAN_OpenError setCanDeviceBaudrate(const CANDEVICE_BaudrateTypes &candevice_baudrate_type);


    /**
     * write something to the protocol
     * 
     * @param T: the type of the data need to write
     * @param data: the data need to write
    */
    template<typename T>
    void write(const T &data);

    /**
     * write bytes to the protocol
     * 
     * @param data: the bytes need to write
     * @param len:  the length of the bytes need to write
    */
    void write(const uint8_t *data, const size_t &len);


    /**
     * Send a new can message
     * 
     * @param id:   the target id of this can message
     * @param data: the data in this can message
     * @param len:  the length of this can message
    */
    void sendStandardCanMsg(const uint32_t &id, const uint8_t *data, const uint8_t &len);

    /**
     * close the USB2CAN device
    */
    void close();
};


};
#endif // DMBOT_USB2CAN_CAN_BUS_H