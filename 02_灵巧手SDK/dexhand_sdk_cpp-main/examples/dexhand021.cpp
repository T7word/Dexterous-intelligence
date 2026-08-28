#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <cstring>
#ifndef WIN32
#include <unistd.h>
#endif
#include <signal.h>
#include <sys/types.h>

#include "DexHand.h"
#include "StringUtils.h"
#include "DateTimeUtils.h"

#define RL_BUFSIZE 1024

using namespace DexRobot;
using namespace DexRobot::Dex021;

DexHand::PTR gHandInstance = nullptr;

std::fstream logFile;

const static std::map<HandID, std::string> DH021_HandNames = {
    { HandID::LEFT, "左手" },
    { HandID::RIGHT, "右手" },
    { HandID::None, "未知设备" },
};

const static std::map<FingerID, std::string> DH021_FingerNames = {
    { FingerID::LEFT_THUMB  , "左手大拇指"  },
    { FingerID::LEFT_PALM   , "左手分指"   },
    { FingerID::LEFT_INDEX  , "左手食指"   },
    { FingerID::LEFT_MIDDLE , "左手中指"   },
    { FingerID::LEFT_RING   , "左手无名指" },
    { FingerID::LEFT_PINKY  , "左手小拇指" },
    { FingerID::RIGHT_THUMB , "右手大拇指" },
    { FingerID::RIGHT_PALM  , "右手分指"   },
    { FingerID::RIGHT_INDEX , "右手食指"   },
    { FingerID::RIGHT_MIDDLE, "右手中指"   },
    { FingerID::RIGHT_RING  , "右手无名指" },
    { FingerID::RIGHT_PINKY , "右手小拇指" },
};

auto CallbackFunc = [](const DX21StatusRxData * status) ->void {
    char row[1024] = {0};
    if (status != nullptr)
    {
        const auto fingerId = status->deviceId;
        std::string fingerName;
        const auto it = DH021_FingerNames.find(FingerID(fingerId));
        if (it != DH021_FingerNames.end())
        {
            fingerName = it->second;
        }
        else
        {
            fingerName = "未知手指";
        }

        const auto timestamp = current_timestamp();
        const auto datetime = timestamp_to_loc_string(timestamp, TS_RADIX::MICROSECOND);

        const auto current1 = status->MotorCurrent((uint8_t)JointMotor::DIST);
        const auto current2 = status->MotorCurrent((uint8_t)JointMotor::PROX);
        const auto degree1 = status->JointDegree((uint8_t)JointMotor::DIST);
        const auto degree2 = status->JointDegree((uint8_t)JointMotor::PROX);
        const auto speed1 = status->MotorVelocity((uint8_t)JointMotor::DIST);
        const auto speed2 = status->MotorVelocity((uint8_t)JointMotor::PROX);
        const auto temperature1 = status->MotorTemperature((uint8_t)JointMotor::DIST);
        const auto temperature2 = status->MotorTemperature((uint8_t)JointMotor::PROX);

        sprintf(row, "[%s]-[%s 远端关节]: 电机电流=%d, 角度=%f, 电机速度=%d, 温度=%d\n", datetime.c_str(), fingerName.c_str(), current1, degree1, speed1, temperature1);
        sprintf(row, "[%s]-[%s 近端关节]: 电机电流=%d, 角度=%f, 电机速度=%d, 温度=%d\n", datetime.c_str(), fingerName.c_str(), current2, degree2, speed2, temperature2);
        logFile << row;
        logFile.flush();
    }
};

/*
 * Use individual control command to drive each fingers, with call of moveFinger() one by one
 */
bool SingleFingerCTL_Test021(const AdapterChannel channel, const std::shared_ptr<DexHand_021> & hand, const int repeatRound = 10)
{
    const auto handId = hand->handID(channel);

    for (int i = 0; i < repeatRound; ++i)
    {
        hand->moveFinger(handId, 0x01, 0x03, 750, 900, MotorControlMode::HALL_POSLIMIT_CONTROL_MODE);
        hand->moveFinger(handId, 0x01, 0x03,   0,   0, MotorControlMode::HALL_POSLIMIT_CONTROL_MODE, 1000);
        hand->moveFinger(handId, 0x02, 0x03, 900, 100, MotorControlMode::HALL_POSLIMIT_CONTROL_MODE, 2000);
        hand->moveFinger(handId, 0x02, 0x03,   0,   0, MotorControlMode::HALL_POSLIMIT_CONTROL_MODE, 1000);
        hand->moveFinger(handId, 0x03, 0x03, 750, 750, MotorControlMode::HALL_POSLIMIT_CONTROL_MODE, 10);
        hand->moveFinger(handId, 0x04, 0x03, 750, 750, MotorControlMode::HALL_POSLIMIT_CONTROL_MODE, 10);
        hand->moveFinger(handId, 0x05, 0x03, 750, 750, MotorControlMode::HALL_POSLIMIT_CONTROL_MODE, 10);
        hand->moveFinger(handId, 0x06, 0x03, 750, 750, MotorControlMode::HALL_POSLIMIT_CONTROL_MODE, 10);

        hand->moveFinger(handId, 0x03, 0x03,   0,   0, MotorControlMode::HALL_POSLIMIT_CONTROL_MODE, 1000);
        hand->moveFinger(handId, 0x04, 0x03,   0,   0, MotorControlMode::HALL_POSLIMIT_CONTROL_MODE, 1000);
        hand->moveFinger(handId, 0x05, 0x03,   0,   0, MotorControlMode::HALL_POSLIMIT_CONTROL_MODE, 1000);
        hand->moveFinger(handId, 0x06, 0x03,   0,   0, MotorControlMode::HALL_POSLIMIT_CONTROL_MODE, 1000);

        hand->clearFirmwareError(channel, handId);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    hand->resetJoints(channel);

    return true;
}

/*
 * Use broadcast control command to drive all fingers with one single call of moveMultipleFingers()
 */
bool MultipleFingerCTL_Test021(const std::shared_ptr<DexHand_021> & hand, int repeatRound = 10)
{
    auto handId0 = hand->handID(AdapterChannel::CHN0);
    auto handId1 = hand->handID(AdapterChannel::CHN1);

    HandControlDesc controlDesc;
    controlDesc.mode = CASCADED_PID_CONTROL_MODE;
    controlDesc.controlParam.params.HandType = 0;
    controlDesc.controlParam.params.ClearErr = 1;
    controlDesc.controlParam.params.Feedback = 0;

    controlDesc.enableMap.enables.ThumbDist = 1;
    controlDesc.enableMap.enables.ThumbProx = 1;
    controlDesc.enableMap.enables.ThumbSwng = 1;
    controlDesc.enableMap.enables.FngerSwng = 1;
    controlDesc.enableMap.enables.IndexDist = 1;
    controlDesc.enableMap.enables.IndexProx = 1;
    controlDesc.enableMap.enables.MddleDist = 1;
    controlDesc.enableMap.enables.MddleProx = 1;
    controlDesc.enableMap.enables.RingFDist = 1;
    controlDesc.enableMap.enables.RingFProx = 1;
    controlDesc.enableMap.enables.LttleDist = 1;
    controlDesc.enableMap.enables.LttleProx = 1;

    controlDesc.thumb.DistPos = 2000;
    controlDesc.thumb.ProxPos = 2000;
    controlDesc.thumb.DistSpd = 5000;
    controlDesc.thumb.ProxSpd = 5000;
    controlDesc.thumb.DistCur = 200;
    controlDesc.thumb.ProxCur = 200;

    controlDesc.swing.DistPos = 5000;
    controlDesc.swing.ProxPos = 0;
    controlDesc.swing.DistSpd = 10000;
    controlDesc.swing.ProxSpd = 10000;
    controlDesc.swing.DistCur = 200;
    controlDesc.swing.ProxCur = 200;

    controlDesc.index.DistPos = 5000;
    controlDesc.index.ProxPos = 5000;
    controlDesc.index.DistSpd = 10000;
    controlDesc.index.ProxSpd = 10000;
    controlDesc.index.DistCur = 200;
    controlDesc.index.ProxCur = 200;

    controlDesc.middle.DistPos = 5000;
    controlDesc.middle.ProxPos = 5000;
    controlDesc.middle.DistSpd = 10000;
    controlDesc.middle.ProxSpd = 10000;
    controlDesc.middle.DistCur = 200;
    controlDesc.middle.ProxCur = 200;
    controlDesc.ring.DistPos = 5000;
    controlDesc.ring.ProxPos = 5000;
    controlDesc.ring.DistSpd = 10000;
    controlDesc.ring.ProxSpd = 10000;
    controlDesc.ring.DistCur = 200;
    controlDesc.ring.ProxCur = 200;

    controlDesc.little.DistPos = 5000;
    controlDesc.little.ProxPos = 5000;
    controlDesc.little.DistSpd = 10000;
    controlDesc.little.ProxSpd = 10000;
    controlDesc.little.DistCur = 200;
    controlDesc.little.ProxCur = 200;

    int sign = -1;

    for (int i = 0; i < 2*repeatRound; ++i)
    {
        if (handId0 != (uint8_t)HandID::None)
            hand->moveMultipleFingers(handId0, controlDesc);

        if (handId1 != (uint8_t)HandID::None)
            hand->moveMultipleFingers(handId1, controlDesc);

        std::this_thread::sleep_for(std::chrono::seconds(2));

        controlDesc.thumb.DistPos  += sign * 1000;
        controlDesc.thumb.ProxPos  += sign * 1000;
        controlDesc.swing.DistPos  += sign * 5000;
        controlDesc.swing.ProxPos  += sign * 1000;
        controlDesc.index.DistPos  += sign * 5000;
        controlDesc.index.ProxPos  += sign * 5000;
        controlDesc.middle.DistPos += sign * 5000;
        controlDesc.middle.ProxPos += sign * 5000;
        controlDesc.ring.DistPos   += sign * 5000;
        controlDesc.ring.ProxPos   += sign * 5000;
        controlDesc.little.DistPos += sign * 5000;
        controlDesc.little.ProxPos += sign * 5000;

        sign *= -1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    hand->resetJoints(AdapterChannel::CHN0);
    hand->resetJoints(AdapterChannel::CHN1);
    return true;
}

void CppInterfaceTests_021()
{
    const auto timestamp = current_timestamp();
    const auto datetime = timestamp_to_loc_string(timestamp, TS_RADIX::MICROSECOND);
    const std::string logFileName = "./sensor_data_stream-" + datetime + ".log";
    logFile.open(logFileName, std::ios_base::out | std::ios_base::app);

    /// In this example, I used a ZLG USBCANFD-200U adapter to connect 2 DexHand-021 devices, this adapter has 2 independent channels.
    /// If ZLG USBCANFD-100U-Mini is the type of adapter on your hand, but there are 2 DexHand-021 devices, you will have to use 2 ZLG Mini adapters to connect them.
    /// To achive this, your code must be like below:
    ///          const auto hand1 = std::dynamic_pointer_cast<DexHand_021>(DexHand::createInstance(ProductType::DX021, AdapterType::ZLG_MINI, 0));
    ///          const auto hand2 = std::dynamic_pointer_cast<DexHand_021>(DexHand::createInstance(ProductType::DX021, AdapterType::ZLG_MINI, 1));
    /// Beaware, the last argument (3rd one) of createInstance() indicates the index of the ZLG adapter which starts by 0. This index is somehow assigend/deteremined
    /// by some kind of rule inside ZLG SDK, or maybe no rule, in which we are not able to predict. Thus, if you have 1 left hand and 1 right hand are connected to PC
    /// via 2 ZLG adapters, you will not able to identify them by the adapter's index, until SDK reads the the finger ID sequence from each device. E.g. for left hand,
    /// the fingers' ID starts from 1 to 6, and for right hand, the ID starts from 7 to 12, that's how we identify the hands.
    /// Altough, one single CANFD adapter is able to connect more than 1 DexHand devices, theoratically. Our SDK just does NOT support that because by doing so, it
    /// might introduce management issues because our device firmware supports identification only via finger IDs sequence, and a group of IDs sequence might not be
    /// always reliable, if one or more of the control boards are down.
    const auto handPair = std::dynamic_pointer_cast<DexHand_021>(DexHand::createInstance(ProductType::DX021, AdapterType::ZLG_200U, 0));
    if(handPair == nullptr)
    {
        std::cout << "Failed to open device." << std::endl;
        return;
    }

    handPair->setTxLogLevel(LOG_LEVEL::DX_INFO);
    handPair->setStatusRxCallback(CallbackFunc);
    if(!handPair->connect(true))
    {
        std::cout << "Connection failure." << std::endl;
        exit(-1);
    }

    /// Get information of DexHand device with all connected fingers on channel 0
    std::vector<FingerID> connectedFingers0;
    constexpr auto channel0 = AdapterChannel::CHN0;
    auto handId0 = static_cast<HandID>(handPair->handID(channel0));
    if (handId0 != None)
    {
        if(const auto it = DH021_HandNames.find(handId0); it != DH021_HandNames.end())
        {
            std::cout << it->second << " is connected on channel " << std::to_string((int)channel0) << std::endl;
            int idx = (handId0 == LEFT) ? 1 : 7;
            const int endIdx = idx + 6;
            for (; idx < endIdx; ++idx)
            {
                connectedFingers0.push_back(static_cast<FingerID>(idx));
            }
        }
    }
    else
    {
        std::cout << "There's no operational DexHand 021 device on channel " << std::to_string((int)channel0) << std::endl;
    }

    /// Get information of DexHand device with all connected fingers on channel 1
    std::vector<FingerID> connectedFingers1;
    constexpr auto channel1 = AdapterChannel::CHN1;
    const auto handId1 = static_cast<HandID>(handPair->handID(channel1));
    if (handId1 != None)
    {
        if(const auto it = DH021_HandNames.find(handId1); it != DH021_HandNames.end())
        {
            std::cout << it->second << " is connected " << std::to_string((int)channel1) << std::endl;
            int idx = (handId1 == LEFT) ? 1 : 7;
            const int endIdx = idx + 6;
            for (; idx < endIdx; ++idx)
            {
                connectedFingers1.push_back(static_cast<FingerID>(idx));
            }
        }
    }
    else
    {
        std::cout << "There's no operational DexHand 021 device on channel " << std::to_string((int)channel1) << std::endl;
    }

    // Clear firmware errors before control the device, incase there were some errors cause motor protection to prevent control.
    if (handId0 != None)
    {
        handPair->clearFirmwareError(channel0, handId0);
        for (const auto & fingerId : connectedFingers0)
        {
            /// Read the firmware version from device
            const auto firmwareVersion = handPair->getFirmwareVersion(handId0, fingerId);
            std::cout << "Firmware Version = " << std::to_string(firmwareVersion) << std::endl;

            /// Set safe working temperature of device, and confirm the setting succeeds or not
            uint8_t maxAllowedTemp = 80;
            handPair->setSafeTemperature(handId0, fingerId, 0x00, maxAllowedTemp);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            const auto safeTemp1 = handPair->getSafeTemperature(handId0, fingerId, 0x00);
            if (maxAllowedTemp == safeTemp1)
            {
                std::cout << "Safe temperature of finger " << DH021_FingerNames.at(fingerId)
                    << " is set to " << std::to_string(safeTemp1) << std::endl;
            }
            else
            {
                std::cout << "Failed to set safe temperature to " << std::to_string(safeTemp1)
                    << " for finger " << DH021_FingerNames.at(fingerId) << ". Please try again." << std::endl;
            }
        }
        handPair->resetJoints(channel0);
    }

    if (handId1 != None)
    {
        handPair->clearFirmwareError(channel1, handId1);
        for (const auto & fingerId : connectedFingers1)
        {
            /// Read the firmware version from device
            const auto firmwareVersion = handPair->getFirmwareVersion(handId1, fingerId);
            std::cout << "Firmware Version = " << std::to_string(firmwareVersion) << std::endl;

            /// Set safe working temperature of device, and confirm the setting succeeds or not
            uint8_t maxAllowedTemp = 80;
            handPair->setSafeTemperature(handId1, fingerId, 0x00, maxAllowedTemp);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            const auto safeTemp1 = handPair->getSafeTemperature(handId1, fingerId, 0x00);
            if (maxAllowedTemp == safeTemp1)
            {
                std::cout << "Safe temperature of finger " << DH021_FingerNames.at(fingerId)
                    << " is set to " << std::to_string(safeTemp1) << std::endl;
            }
            else
            {
                std::cout << "Failed to set safe temperature to " << std::to_string(safeTemp1)
                    << " for finger " << DH021_FingerNames.at(fingerId) << ". Please try again." << std::endl;
            }
        }
        handPair->resetJoints(channel1);
    }

    for (const auto & fingerId : connectedFingers0)
    {
        handPair->setRealtimeResponse(handId0, fingerId, 100, true);
    }

    if (handId0 != None)
    {
        SingleFingerCTL_Test021(channel0, handPair, 3);
    }

    if (handId1 != None)
    {
        SingleFingerCTL_Test021(channel1, handPair, 3);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    MultipleFingerCTL_Test021(handPair, 5);

    for (const auto & fingerId : connectedFingers0)  // Close auto feed back of device on channel 0
    {
        handPair->setRealtimeResponse(handId0, fingerId, 50, false);
    }

    for (const auto & fingerId : connectedFingers1)  // Close auto feed back of device on channel 1
    {
        handPair->setRealtimeResponse(handId1, fingerId, 50, false);
    }

    logFile.close();
}

void signalHandler(int signum)
{
    std::cout << "\nGet Message: " << signum << std::endl;

    if (signum == SIGINT) {
        std::cout << "Closing socket..." << std::endl;
        if(gHandInstance != nullptr)
            gHandInstance->disconnect();

        exit(0);
    }
    else if (signum == SIGABRT) {
        std::cout << "Closing socket..." << std::endl;
        if(gHandInstance != nullptr)
            gHandInstance->disconnect();

        exit(signum);
    }
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

int main(int argc, const char ** argv)
{
    CppInterfaceTests_021();
    return 0;
}
