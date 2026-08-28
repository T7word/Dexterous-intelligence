# DexHand CPP SDK

## 概览

DexHand CPP SDK为DexRobot灵巧手系列产品的二次开发人员提供了基础C++开发套件，使得开发人员可以通过该SDK对DexRobot的灵巧手系列产品进行基本的运动控制，目前支持的DexRobot灵巧手系列产品型号包括五指灵巧手DexHand-021, 和三指灵巧手DexHand-021S。

## 如何获得并使用DexHand CPP SDK

### 获得DexHand CPP SDK

DexHand CPP SDK可以通过DexRobot的github社区获得，也可以从DexRobot的官方网站下载。

- github地址：<https://github.com/DexRobot/dexhand_sdk_cpp.git>  
  可以通过`git clone <https://github.com/DexRobot/dexhand_sdk_cpp.git>`将远程仓库拉取到本地。
- DexRobot官网下载地址：<https://www.dex-robot.com/downloadCenter/detail>
  也可以通过官网下载页面下载压缩包到本地。

### 目录结构

如果下载的是压缩包，完成下载后，请先将压缩包解压。SDK下有三个子目录：include, lib, 以及examples。
其中include目录下包含了开发人员所需该SDK的所有头文件，lib目录下则包含了该SDK所有的动态库二进制文件。examples目录中有关于如何使用该SDK对DexRobot所有灵巧手系列产品进行基本运动控制的示例代码，包括五指灵巧手DexHand-021和三指灵巧手DexHand-021S。

### 使用流程

目前DexRobot的系列灵巧手产品均支持使用CANFD进行通讯，该SDK提供了使用CANFD-USB适配器与灵巧手设备进行通讯和运动控制的接口。此外，三指灵巧手DexHand021-S产品支持485串口通讯，SDK中同样提供了使用485串口USB适配器与三指灵巧手进行控制通讯的API接口。

- #### 创建灵巧手设备实例
  
使用DexHand::createInstance()接口创建一个指定设备类型的灵巧手实例，如果该调用的返回值不为空，则实例创建成功，否则实例创建失败：

```CPP
    auto gHandInstance = DexHand::createInstance(ProductType::DX021, AdapterType::ZLG_200U, 0);
    if(nullptr == gHandInstance)
    {
        std::cout << "Device not available." << std::endl;
        exit(-1);
    }
```

该函数的第一个参数指定灵巧手设备的类型，如DX021即代表五指灵巧手DexHand-021M，DX021S则代表三指灵巧手DexHand-021S。第二个参数指定连接灵巧手设备所使用的CANFD适配器类型，第三个参数指定CANFD适配器上的连接了灵巧手设备的通道号。

若用户需要使用485串口适配器与三指灵巧手设备进行通讯控制，则可以使用以下接口创建三指灵巧手对象实例：

```CPP
    const auto hand = std::dynamic_pointer_cast<DexHand_021S>(DexHand::createInstance(ProductType::DX021_S, "/dev/ttyUSB0"));
```

其第二个参数是485串口适配器连接上PC后所分配的串口设备号。

实例创建成功后不代表可以直接进行控制，还需要对该灵巧手进行相应的设置，并发起连接后，才可以进行后续的控制。

- #### 设置灵巧手实例

通常在对灵巧手设备进行运动控制的同时，我们还需要了解灵巧手各手指及关节的运动状态，角度，方便进行运动学解算等工作，并对灵巧手内部零部件的工作状态如电机的电流，转速，温度等进行监控，以方便对灵巧手的工作负载进行管理。这要求我们能够获得灵巧手固件反馈的这些状态数据并进行处理。用户可以通过调用setStatusRxCallback()接口，来向该实例注册一个回调函数用于处理灵巧手固件反馈的状态数据：

```CPP
    DH21StatusRxCallback callback = std::bind(CallbackFunc, std::placeholders::_1);
    gHandInstance->setStatusRxCallback(callback);
```

关于状态数据具体包括哪些信息，以及对该回调函数的参数和返回值形式要求等详细信息，需参阅DexRobotd的灵巧手用户手册及SDK使用手册。这些文档均可在官网下载中心获得: <https://www.dex-robot.com/downloadCenter/detail>

由于485串口并非全双工通讯，且目前三指灵巧手DexHand-021S所使用串口波特率最高为115200，SDK在对灵巧手发送控制指指令时，无法异步并实时地从后台获取灵巧手设备的状态数据，因而使用485控制时，SDK不支持使用回调函数对灵巧手设备的状态数据进行处理，以上接口对使用485串口通讯的三指灵巧手设备无效。

如用户希望获得设备的灵巧手设备的状态数据，需主动调用HandStatus485()函数来获取。


- #### 发起连接并设置固件
  
完成回调函数设置后，即可通过该实例向灵巧手设备发起连接:

```CPP
    if(!gHandInstance->connect(true))
    {
        std::cout << "Connection failure." << std::endl;
        exit(-1);
    }
```

对于DexHand-021型灵巧手来说，实例连接成功后，会自动将左手ID设为1, 右手ID设为2，并与CANFD适配器上相应的通道进行绑定，开发者可以通过handID(channel)接口获取指定通道上所连接的手的ID。用户同时也可以使用setHandId()接口来自行决定该映射关系：

```CPP
    uint8_t handId = 0x01;
    gHandInstance->setHandId(AdapterChannel::CHN0, handId);
    uint8_t actualHandId = gHandInstance->handID(AdapterChannel::CHN0); // The returned actualHandId should be the same as handId.
```

而对于DexHand-021S型三指灵巧手设备，当使用485串口进行通讯时，由于485协议目前不支持在设备ID未知时通过接口获取其ID，因而需要用户通过使用CANFD通讯的方式，或者使用轮询的方式来确认当前设备的ID。在创建好使用485串口通讯的灵巧手对象实例后，必须使用正确的设备ID传入setHandId()接口来设置好当前对象的ID，以保证后续控制过程中SDK能向灵巧手设备发送正确的发送485指令，如果传入的ID并非当前设备的真实ID，SDK将无法正确的与灵巧手设备通讯，该函数也不会将传入的错误ID写入到设备的固件中。

开发者可以根据自己的需求，进一步对灵巧手的固件进行相应的设置。例如，在默认状态下，灵巧手设备不会实时的上报自身的状态数据，而是只会在执行每一帧运动指令后进行反馈。如果开发者希望实时的获得灵巧手的状态数据以方便进行运动解算及监控，则需要进行一下设置：

```CPP
    gHandInstance->setRealtimeResponse(handId, 50, true);
```

该调用将两巧手的固件设置为实时上报状态数据的模式，采样率为50Hz。若第三个参数设置为false，则实时采样模式关闭。如开发者不希望实时监控灵巧手的状态，则该调用不是必须的。

- #### 控制灵巧手
  
完成上述所有操作后，开发者便可以开始根据自己的需要对灵巧手的各个手指进行运动控制，控制的接口为moveFinger()。调用方式如下：

```CPP
    gHandInstance->moveFinger(handId, fingerId, jointId, value1, value2, MotorControlMode::CASCADED_PID_CONTROL_MODE);
```

五指灵巧手DexHand-021的手指ID设置为每只手有6个控制单元共12个主动自由度，分别代表5个手指及手掌的分指机构，各自有独立的手指ID。其中左手大拇指ID为1，分指机构ID为2，食指到小拇指依次为3，4，5，6。各手指有两个主动自由度，靠近手掌的近端关节MIP，在该SDK中标识为JointMotor::PROX，取值为2，靠近远端的指腹关节PIP在该SDK中标识为JointMotor::DIST，取值为1。其中对分指机构的DIST为大拇指翻转，而PROX则为其余4指的开合。对于右手，大拇指的ID为7，分指机构ID为8，其余以与左手对称的方式依次标识为9，10，11，12。关于DexHand-021的结构模型及ID分布的详细说明，请参阅用户手册。

该调用中，第二个参数为指定的手指ID，第三个参数代表远近端关节使能，取值为JointMotor::DIST标识只运动远端关节，取值PROX时只运动近端关节，取值ALL时，则所有关节均参与运动。第四个和第五个参数分别代表远端关节和近端关节运动的目标值，该值在不同的控制模式下具有不同的语义。目前对用户开放的控制模式共有4种，代码分别为0x44, 0x55, 0x66, 0x77。示例中的MotorControlMode::CASCADED_PID_CONTROL_MODE。关于不同的控制模式，以及运动的目标取值的含义等相关详细信息，请参阅灵巧手的用户手册和通讯协议手册。

- #### 获取灵巧手设备状态数据

前面提到过对于使用CANFD通讯的灵巧手设备，用户可以通过注册回调函数的方式，实时异步获取并处理灵巧手设备的状态数据。DexHand灵巧手SDK要求用于处理状态数据的回调函数，需要遵循以下声明形式：

```CPP
    std::function<void (const Dex021::DX21StatusRxData *)>
```

该声明形式在头文件DexHand.h中被定义为：

```CPP
    typedef std::function<void (const Dex021::DX21StatusRxData *)> DH21StatusRxCallBack;
```

其中DX21StatusRxData为携带有灵巧手所有状态数据的类，SDK通过将该类的对象指针传入回调函数供CANFD用户处理。假设有以下函数定义：

```CPP
    void CallbackFunc21S(const DX21StatusRxData * handStatus)
    {
        printf("BoradTemp = %d\n", handStatus->boardTemper);
        printf("Position1 = %d, Speed1 = %d, Current1=%d, MT Temp1=%d\n"
            , handStatus->MotorHallValue(1)
            , handStatus->MotorVelocity(1)
            , handStatus->MotorCurrent(1)
            , handStatus->MotorTemperature(1));

        printf("Position2 = %d, Speed2 = %d, Current2=%d, MT Temp2=%d\n"
            , handStatus->MotorHallValue(2)
            , handStatus->MotorVelocity(2)
            , handStatus->MotorCurrent(2)
            , handStatus->MotorTemperature(2));
    
        printf("Position3 = %d, Speed3 = %d, Current3=%d, MT Temp3=%d\n"
            , handStatus->MotorHallValue(3)
            , handStatus->MotorVelocity(3)
            , handStatus->MotorCurrent(3)
            , handStatus->MotorTemperature(3));
    
        printf("Position4 = %d, Speed4 = %d, Current4=%d, MT Temp4=%d\n"
            , handStatus->MotorHallValue(4)
            , handStatus->MotorVelocity(4)
            , handStatus->MotorCurrent(4)
            , handStatus->MotorTemperature(4));
    }
```

该函数注册为状态回调函数后，会将各手指对应电机的霍尔位置，速度，电流，温度打印在标准输出上。默认情况下，

而对于使用485串口通讯的灵巧手设备来说，因无法使用回调函数来异步实时获取设备的状态数据，则需要主动调用设备对象的handStatus485()成员函数来获取状态数据：

```CPP
   auto motorStatus = hand->HandStatus485();
```

该函数返回一个DX21StatusRxData的只能指针，获得该指针后，用户可以通过其接口获取舵机及其余传感器的状态数据。总体来说，使用485通讯控制的能力是弱于使用CANFD的，因而还是建议用户使用CANFD对灵巧智能的灵巧手设备进行控制。

- #### 断开设备

当使用完灵巧手之后，用户需要确保程序实例与灵巧手设备的连接断开：

```CPP
    gHandInstance->disconnect();
```

该调用会自动退出设备的实时状态数据采样模式。此后灵巧手设备将不再接受程序实例发出的任何控制指令。
