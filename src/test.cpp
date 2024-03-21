#include <iostream>

#include "can/can_bus.h"

int main(int argc, char ** argv)
{
    std::string target_hid = "USB VID:PID=2e88:4603 SNR=00000000050C";
    uint8_t data[8] = {0xA2, 0x00, 0x00, 0x00, 0xA0, 0x8C, 0x00, 0x00};
    usb2can::CanBus can_bus;

    std::vector<serial::PortInfo> port_list = can_bus.listDevices();
    for (int i = 0; i < port_list.size(); i++)
    {
        std::cout << "port: " << port_list[i].port << ", hid: " << port_list[i].hardware_id << "." << std::endl;
    }

    usb2can::USB2CAN_OpenError result = can_bus.openDeviceWithHID(target_hid, 1000000);
    if (result != usb2can::USB2CAN_OpenError::SUCC)
    {
        std::cerr << "failed to open device, error code: " << static_cast<int>(result) << std::endl;
        return 0;
    } else
        std::cerr << "successfully opened device" << std::endl;

    

    using namespace std::chrono;
    
    for (int i = 0; i < 1; i++)
    {
        can_bus.sendStandardCanMsg(0x141, data, 8);
        std::this_thread::sleep_for(1s);
    }
    return 0;
}