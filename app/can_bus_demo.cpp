#include <iostream>
#include <functional>

#include "can/can_bus.h"

void canReceiveHandle(const uint32_t &id, const uint8_t *data, const uint8_t &len)
{
    printf("receive msg from id: 0x%04hx, ", (int) (id & 0xffff));
    std::cerr << "recv data: ";
    for (int i = 0; i < len; i++)
        printf("0x%02hx, ", data[i]);
    std::cerr << std::endl;
}

void canBagReceiveHandle(const usb2can::CAN_RecvBag &msg)
{
    printf("receive msg cmd: 0x%02hx, ", (int)msg.command);
    printf("receive msg from id: 0x%04hx, ", (int) (msg.msg.can_id & 0xffff));
    std::cerr << "recv data: ";
    for (int i = 0; i < msg.msg.can_data_len; i++)
        printf("0x%02hx, ", msg.msg.data[i]);
    std::cerr << std::endl;
}


int main(int argc, char ** argv)
{
    // use the hardwire id to select the device you want to open.
    std::string target_hid = "USB VID:PID=2e88:4603 SNR=00000000050C";

    // the data transmit with can bus
    uint8_t data[8] = {0xA2, 0x00, 0x00, 0x00, 0xA0, 0x8C, 0x00, 0x00};
    
    // config the can bus
    usb2can::CanBus can_bus;
    can_bus.enableCANMode(usb2can::CANDEVICE_BaudrateTypes::RATE_1M);
    can_bus.setCanBagRecvCallBack(std::bind(&canBagReceiveHandle, std::placeholders::_1));                // register callback functions
    can_bus.setCanDataFrameRecvCallBack(std::bind(&canReceiveHandle, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));             // register callback functions

    // list the hardwire id of the serial ports on the device
    std::vector<serial::PortInfo> port_list = can_bus.listDevices();
    for (int i = 0; i < port_list.size(); i++)
    {
        std::cout << "port: " << port_list[i].port << ", hid: " << port_list[i].hardware_id << "." << std::endl;
    }

    // try to open the USB2CAN device
    usb2can::USB2CAN_OpenError result = can_bus.openDeviceWithHID(target_hid);
    if (result != usb2can::USB2CAN_OpenError::SUCC)
    {
        std::cerr << "failed to open device, error code: " << static_cast<int>(result) << std::endl;
        return 0;
    } else
        std::cerr << "successfully opened device" << std::endl;

    
    // send can message
    using namespace std::chrono;
    for (int i = 0; i < 10; i++)
    {
        can_bus.sendStandardCanMsg(0x141, data, 8);
        std::this_thread::sleep_for(1ms);
    }
    return 0;
}