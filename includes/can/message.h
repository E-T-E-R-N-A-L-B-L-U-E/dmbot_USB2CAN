//
// created by linxif2008 on 03/21/2024
//

#ifndef DMBOT_USB2CAN_CAN_MESSAGE_H
#define DMBOT_USB2CAN_CAN_MESSAGE_H

#include <cstdint>

namespace usb2can
{

const uint8_t USB2CAN_RECV_START_BYTE = 0xAA;
const uint8_t USB2CAN_RECV_ENDING_BYTE = 0x55;
const uint8_t USB2CAN_EMPTY_BYTE = 0x00;


struct USB2CAN_HEARTBEATS
{
    uint8_t  header_low     = 0x55;
    uint8_t  header_high    = 0x04;
    uint8_t  ending_low     = 0xAA;
    uint8_t  ending_high    = 0x55;
} __attribute__((packed));


struct USB2CAN_CANSTOP
{
    uint8_t  header_low     = 0x55;
    uint8_t  header_high    = 0x03;
    uint8_t  ending_low     = 0xAA;
    uint8_t  ending_high    = 0x55;
} __attribute__((packed));


/**
 * 
 * Define the Basic Message Bags between the PC and the USB2CAN device.
 * This Bag is used to set the communication baudrate between the PC and the USB2CAN device.
 * 
 * Here is the format of the data frame:
 * 
 * |                               Baudrate Setting Command                                           |
 * | ------------------------------------------------------------------------------------------------ |
 * | Frame  Head   |            baudrate           | data bit | stop bit | parity |  Frame Endding    |
 * | Data0 | Data1 | Data2 | Data3 | Data4 | Data5 |   Data6  |   Data7  | Data8  |  Data9  |  Data10 |
 * |    16 bits    |            32 bits            |  8 bits  |  8 bits  | 8 bits |  8 bits |  8 bits |
 * | 0x55  | 0xAA  |                               |          |          |        |   0xAA  |  0x55   |
 * 
*/

enum class USB2CAN_BaudrateDatabit : uint8_t
{
    BITS_8 = 0,                 // use 8 bit protocol data frame
    BITS_9 = 1,                 // use 9 bit protocol data frame
};

enum class USB2CAN_BaudrateStopbit : uint8_t
{
    STOPBIT_ONE = 1,            // use one stop bit protocol frame
    STOPBIT_TWO = 2,            // use two stop bit protocol frame
};

enum class USB2CAN_BaudrateParity : uint8_t
{
    PARITY_NONE = 0,            // no parity in protocol frame
    PARITY_ODD = 1,             // only support no parity or with parity two modes
};

struct USB2CAN_BaudrateSetMsg
{
    uint32_t                baudrate;
    USB2CAN_BaudrateDatabit data_bit = USB2CAN_BaudrateDatabit::BITS_8;
    USB2CAN_BaudrateStopbit stop_bit = USB2CAN_BaudrateStopbit::STOPBIT_ONE;
    USB2CAN_BaudrateParity  parity   = USB2CAN_BaudrateParity::PARITY_NONE;
} __attribute__((packed));

struct USB2CAN_BaudrateSetBag
{
    uint8_t  header_low     = 0x55;
    uint8_t  header_high    = 0xAA;
    USB2CAN_BaudrateSetMsg  data;
    uint8_t  ending_low     = 0xAA;
    uint8_t  ending_high    = 0x55;
} __attribute__((packed));



/**
 * 
 * Define the Message Bag to set the communication rate of can devices.
 * 
 * Here is the format of data frame:
 * 
 * |        Can Baudrate Setting Command            |
 * | ---------------------------------------------- |
 * | Frame Head     |   index   |   Frame Endding   |
 * | Data0  | Data1 |   Data2   |   Data3  |  Data4 |
 * |     16 bits    |   8 bits  |       16 bits     |
 * |  0x05  | 0x55  |           |   0xAA   |  0x55  |
*/



enum class CANDEVICE_BaudrateTypes : uint8_t
{
    RATE_1M     = 0,
    RATE_800K   = 1,
    RATE_666K   = 2,
    RATE_500K   = 3,
    RATE_400K   = 4,
    RATE_250K   = 5,
    RATE_200K   = 6,
    RATE_125K   = 7,
    RATE_100K   = 8,
    RATE_80K    = 9,
    RATE_50K    = 10,
    RATE_40K    = 11,
    RATE_20K    = 12,
    RATE_10K    = 13,
    RATE_5K     = 14,
};


struct CANDEVICE_BaudrateSetBag
{
    uint8_t  header_low     = 0x55;
    uint8_t  header_high    = 0x05;
    CANDEVICE_BaudrateTypes baudrate = CANDEVICE_BaudrateTypes::RATE_1M;
    uint8_t  ending_low     = 0xAA;
    uint8_t  ending_high    = 0x55;
} __attribute__((packed));




/**
 * 
 * Define the Message Bag to send a can message.
 * 
 * Here is the format of the data frame
 * 
 * |                                                                                        CAN Send Data Command                                                                                                          |
 * | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |  Frame Head   |  Frame len |  cmd   |           send  times         |          time interval          | id type |              CAN ID                | frame type  |  len  | idAcc  | data acc |    data    |   CRC   |
 * | Data0 | Data1 |   Data2    | Data3  | Data4 | Data5 | Data6 | Data7 | Data8 | Data9 | Data10 | Data11 |  Data12 | Data13 | Data14 | Data15 | Data 16 |    Data17   | Data18| Data19 |  Data20  | Data21-28  |  Data29 |
 * |   16 bits     |    8 bits  | 8 bits |            32   bits          |            32  bits             |  8 bits |             32   bits              |     8 bits  | 8 bits| 8 bits |  8 bits  | 8 bits * 8 |  8 bits |
 * |  55   |  AA   |    1E      |   01   |   01  |   00  |   00  |  00   |   0a  |   00  |   00   |   00   |   00    |   00   |   00   |   00   |    00   |    00       |   08  |  00    |    00    |   data[8]  |   crc   |
 *
*/


enum class CAN_SendCmdType : uint8_t
{
    CAN_SEND    = 0x01,
    SHAKE_HAND  = 0x02,
    CAN_SEND_NO_REPLY = 0x03,
};

enum class CAN_SendIDType : uint8_t
{
    STANDARD    = 0x00,
    EXTENDED    = 0x01,
};

enum class CAN_SendFrameType : uint8_t
{
    DATA_FRAME  = 0x00,
    REMOTE_RATE = 0x01,
};


struct CAN_SendMsg
{
    CAN_SendCmdType cmd_type    = CAN_SendCmdType::CAN_SEND;
    uint32_t send_times         = 0x00000001;
    uint32_t time_interval      = 0x00000000;   // (100us)
    CAN_SendIDType id_type      = CAN_SendIDType::STANDARD;
    uint32_t can_id             = 0;
    CAN_SendFrameType frame_type = CAN_SendFrameType::DATA_FRAME;
    uint8_t data_length         = 0x08;         // the default can data length is 8 bits
    uint8_t id_acc              = 0;
    uint8_t data_acc            = 0;
    uint8_t data[8];                            // the data part
} __attribute__((packed));

struct CAN_SendBag
{
    uint8_t  header_low     = 0x55;
    uint8_t  header_high    = 0xAA;
    uint8_t frame_length    = 0x1e;         // the default frame length is 8 bits
    CAN_SendMsg msg;
    uint8_t crc             = 0;
} __attribute__((packed));



/**
 * 
 * Define the CAN Message receive bag
 * 
 * Here is the format of the data frame
 * 
 * |                                                    CAN Receive Frame Format                                         |
 * | ------------------------------------------------------------------------------------------------------------------- |
 * | Frame Head |  commands | can data len | can ide | canRtr |            CAN ID             |    data    |  Frame End  |
 * |   Data0    |   Data1   |               Data3             | Data3 | Data4 | Data5 | Data6 |  Data7-14  |    Data15   |
 * |  8 bits    |   8 bits  |    6 bits    |  1 bit  | 1 bit  |            32   bits          |  64 bits   |    8  bits  |
 * |     AA     |           |                                 |                               |    data[8] |     55      |
*/

enum class CAN_RecvCmdType : uint8_t
{
    HEART_BEAT = 0x00,
    RECV_FAILURE = 0x01,
    RECV_SUCC = 0x11,
    SEND_FAILURE = 0x02,
    SEND_SUCC = 0x12,
    BAUDRATE_SET_FAILURE = 0x03,
    BAUDRATE_SET_SUCC = 0x13,
    COMMUNICATION_ERROR = 0xEE,
};

enum class CAN_RecvIDType : uint8_t
{
    STANDARD    = 0x00,
    EXTENDED    = 0x01,
};

enum class CAN_RecvFrameType : uint8_t
{
    DATA_FRAME  = 0x00,
    REMOTE_RATE = 0x01,
};

enum class CAN_RecvErrorType : uint8_t
{
    NO_ERROR = 0x00,
    RESET_REQUIRED = 0x01,
};

struct CAN_RecvMsg
{
    uint8_t can_data_len : 6;
    CAN_RecvIDType can_id_type : 1;
    CAN_RecvFrameType can_frame_type : 1;
    uint32_t can_id;
    uint8_t data[8];
} __attribute__((packed));

struct CAN_RecvBag
{
    uint8_t         header;
    CAN_RecvCmdType command;
    CAN_RecvMsg     msg;
    uint8_t         endding;
} __attribute__((packed));


};

#endif // DMBOT_USB2CAN_CAN_MESSAGE_H