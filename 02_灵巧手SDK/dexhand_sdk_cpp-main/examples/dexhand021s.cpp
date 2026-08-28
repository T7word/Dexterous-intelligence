#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>
#include "DexHand.h"
#include "StringUtils.h"

#define RL_BUFSIZE 1024

using namespace DexRobot;
using namespace DexRobot::Dex021;


void CallbackFunc(const DX21StatusRxData * status)
{
    //printf("BoradTemp = %d\n", status->boardTemper);
    printf("Position1 = %d, Speed1 = %d, Current1=%d, MT Temp1=%d\n"
        , status->MotorHallValue(1)
        , status->MotorVelocity(1)
        , status->MotorCurrent(1)
        , status->MotorTemperature(1));

    printf("Position2 = %d, Speed2 = %d, Current2=%d, MT Temp2=%d\n"
        , status->MotorHallValue(2)
        , status->MotorVelocity(2)
        , status->MotorCurrent(2)
        , status->MotorTemperature(2));

    printf("Position3 = %d, Speed3 = %d, Current3=%d, MT Temp3=%d\n"
        , status->MotorHallValue(3)
        , status->MotorVelocity(3)
        , status->MotorCurrent(3)
        , status->MotorTemperature(3));

    printf("Position4 = %d, Speed4 = %d, Current4=%d, MT Temp4=%d\n"
        , status->MotorHallValue(4)
        , status->MotorVelocity(4)
        , status->MotorCurrent(4)
        , status->MotorTemperature(4));

    printf("\n");
}

char *CmdReadLine(void)
{
    int bufsize = RL_BUFSIZE;
    int position = 0;
    char *buffer = (char *)malloc(sizeof(char) * bufsize);
    int c;

    if (!buffer) {
        printf("CmdReadLine: allocation error\n");
        exit(1);
    }

    while (true) {
        c = getchar();
        if (c == EOF || c == '\n') {
            buffer[position] = '\0';
            return buffer;
        }
        else {
            buffer[position] = c;
        }
        position++;
        if (position >= bufsize) {
            bufsize += RL_BUFSIZE;
            buffer = (char *)realloc(buffer, bufsize);
            if (!buffer) {
                printf("CmdReadLine: realloc error\n");
                exit(1);
            }
        }
    }
}

void show_usage()
{
    std::cout << "USAGE: dexhand021s <-zlg2u | -zlgm | -lys> [-l]" << std::endl;
    std::cout << "\t-zlg2u, use ZLG USB CANFD 200U adapter" << std::endl;
    std::cout << "\t-zlgm, use ZLG USB CANFD mini adapter" << std::endl;
    std::cout << "\t-lys, use LYS USB CANFD mini adapter" << std::endl;
    std::cout << "\t-l, optional, use for whether listening to realtime response or not" << std::endl;
}

void CANFDTest(const Dex021::AdapterType atype, bool bListen)
{
    const auto device = DexHand::createInstance(ProductType::DX021_S, atype, 0);
    const auto hand = std::dynamic_pointer_cast<DexHand_021S>(device);

    DH21StatusRxCallBack callback = std::bind(CallbackFunc, std::placeholders::_1);
    hand->setStatusRxCallback(callback);

    if(!hand->connect(true))
    {
        std::cout << "Connection failure." << std::endl;
        exit(-1);
    }

    uint8_t deviceId = 0x01;

    hand->setHandId(AdapterChannel::CHN0, deviceId);
    hand->setRealtimeResponse(deviceId, 0x00, 50, bListen);

    auto firmwareVersion = hand->getFirmwareVersion(deviceId, 0x00);
    std::cout << "Firmware version = " << std::to_string(firmwareVersion) << std::endl;

    auto maxCurrent = hand->getSafeCurrent(deviceId, 0x01);
    std::cout << "Limited Max Current = " << std::to_string(maxCurrent) << std::endl;

    auto maxTemperature = hand->getSafeTemperature(deviceId, 0x01);
    std::cout << "Limited Max Temperature = " << std::to_string(maxTemperature) << std::endl;

    hand->clearFirmwareError(deviceId, 0x00);
    hand->resetJoints(deviceId);

    hand->clearFirmwareError(deviceId, 0x00);

    for(int i=0; i < 10; ++i)
    {
        hand->moveFinger(deviceId, 0x01, 0x03, 1200, 1000, HALL_POSLIMIT_CONTROL_MODE, 10);
        hand->moveFinger(deviceId, 0x02, 0x03, 1200, 1000, HALL_POSLIMIT_CONTROL_MODE, 10);
        hand->moveFinger(deviceId, 0x03, 0x03, 1200, 1000, HALL_POSLIMIT_CONTROL_MODE, 10);
        hand->clearFirmwareError(deviceId, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));

        hand->moveFinger(deviceId, 0x01, 0x03, 0, 1000, HALL_POSLIMIT_CONTROL_MODE, 10);
        hand->moveFinger(deviceId, 0x02, 0x03, 0, 1000, HALL_POSLIMIT_CONTROL_MODE, 10);
        hand->moveFinger(deviceId, 0x03, 0x03, 0, 1000, HALL_POSLIMIT_CONTROL_MODE, 10);
        hand->clearFirmwareError(deviceId, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    hand->resetJoints(deviceId);
}

void RTU485Test(const std::string & portName)
{
    const auto device = DexHand::DexHand::createInstance(ProductType::DX021_S, portName.c_str());
    const auto hand = std::dynamic_pointer_cast<DexHand_021S>(device);

    if(!hand->connect(true))
    {
        std::cout << "Connection failure." << std::endl;
        exit(-1);
    }

    uint8_t deviceId = 0x01;

    /// For 485 user, this step is a must, for 485 protocols does not support to retrieve the
    /// device ID when user does NOT even know it. Device ID to be known is a prerequisite 
    hand->setHandId(AdapterChannel::CHN0, deviceId);

    auto firmwareVersion = hand->getFirmwareVersion(deviceId, 0x00);
    std::cout << "Firmware version = " << std::to_string(firmwareVersion) << std::endl;

    auto maxCurrent = hand->getSafeCurrent(deviceId, 0x01);
    std::cout << "Limited Max Current = " << std::to_string(maxCurrent) << std::endl;

    auto maxTemperature = hand->getSafeTemperature(deviceId, 0x01);
    std::cout << "Limited Max Temperature = " << std::to_string(maxTemperature) << std::endl;

    hand->clearFirmwareError(deviceId, 0x00);
    hand->resetJoints(deviceId);

    hand->clearFirmwareError(deviceId, 0x00);

    for(int i=0; i < 10; ++i)
    {
        hand->moveFinger(deviceId, 0x01, 0x03, 1200, 1000, HALL_POSLIMIT_CONTROL_MODE, 10);
        hand->moveFinger(deviceId, 0x02, 0x03, 1200, 1000, HALL_POSLIMIT_CONTROL_MODE, 10);
        hand->moveFinger(deviceId, 0x03, 0x03, 1200, 1000, HALL_POSLIMIT_CONTROL_MODE, 10);
        hand->clearFirmwareError(deviceId, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));

        hand->moveFinger(deviceId, 0x01, 0x03, 0, 1000, HALL_POSLIMIT_CONTROL_MODE, 10);
        hand->moveFinger(deviceId, 0x02, 0x03, 0, 1000, HALL_POSLIMIT_CONTROL_MODE, 10);
        hand->moveFinger(deviceId, 0x03, 0x03, 0, 1000, HALL_POSLIMIT_CONTROL_MODE, 10);
        hand->clearFirmwareError(deviceId, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    hand->resetJoints(deviceId);
}


int main(int argc, const char ** argv)
{
    bool bListen = false;
    Dex021::AdapterType atype;

    if(argc != 3 && argc != 2)
    {
        show_usage();
        exit(1);
    }

    if(0 == strncmp(argv[1], "-zlg2u", 6))
    {
        atype = AdapterType::ZLG_200U;
    }
    else if(0 == strncmp(argv[1], "-zlgm", 5))
    {
        atype = AdapterType::ZLG_MINI;
    }
    else if(0 == strncmp(argv[1], "-lys", 4))
    {
        atype = AdapterType::LYS_MINI;
    }
    else if(0 == strncmp(argv[1], "-485", 4))
    {
        atype = AdapterType::USB2_485;
    }
    else
    {
        show_usage();
        exit(1);
    }

    if(argc == 3 && (0 == strncmp(argv[2], "-l", 2)))
        bListen = true;

    if(atype != AdapterType::USB2_485)
    {
        CANFDTest(atype, bListen);
    }
    else
    {
        RTU485Test("/dev/ttyUSB0");
    }

    return 0;
}
