// DexHand021 S - xCore xPanel RS485 read-only diagnostic
// Read and print the basic data of Motor_1/Motor_2/Motor_3.
// Motor_4 is the rotation joint, so it is also read and printed separately.
// No 0x31 motor-control frame is sent by this program.

// xPRWRegister RL arrays use 1-based indexes.
VAR int finger_angle[3] = {-999,-999,-999}; // input register 0x00~0x02, angle * 100
VAR int rotation_angle[1] = {-999};         // input register 0x03, angle * 100

// Per-servo basic data:
// hall_position: int32; speed/current/torque: int16 x 3;
// temperature/voltage: int16 x 2.
VAR int hall_1[1] = {-999};
VAR int data_1[3] = {-999,-999,-999};
VAR int temp_voltage_1[2] = {-999,-999};
VAR int hall_2[1] = {-999};
VAR int data_2[3] = {-999,-999,-999};
VAR int temp_voltage_2[2] = {-999,-999};
VAR int hall_3[1] = {-999};
VAR int data_3[3] = {-999,-999,-999};
VAR int temp_voltage_3[2] = {-999,-999};
VAR int hall_4[1] = {-999};
VAR int data_4[3] = {-999,-999,-999};
VAR int temp_voltage_4[2] = {-999,-999};

GLOBAL PROC main()
    // 1) Read the three finger joint angles: register 0x00~0x02.
    // Returned value = angle(deg) * 100.
    XPRWRegister(1, 4, 0, "uint16", 3, finger_angle, false);
    Wait(0.02);

    // Motor_4 is the rotation joint: register 0x03.
    XPRWRegister(1, 4, 3, "uint16", 1, rotation_angle, false);
    Wait(0.02);

    // 2) Read Motor_1 basic servo data. Base address = 0x40 (decimal 64).
    // 0x40~0x41: Hall position; 0x42~0x44: speed/current/torque;
    // 0x45~0x46: temperature/voltage.
    XPRWRegister(1, 4, 64, "int32", 1, hall_1, false);
    Wait(0.02);
    XPRWRegister(1, 4, 66, "int16", 3, data_1, false);
    Wait(0.02);
    XPRWRegister(1, 4, 69, "int16", 2, temp_voltage_1, false);
    Wait(0.02);

    // Motor_2 base address = 0x70 (decimal 112).
    XPRWRegister(1, 4, 112, "int32", 1, hall_2, false);
    Wait(0.02);
    XPRWRegister(1, 4, 114, "int16", 3, data_2, false);
    Wait(0.02);
    XPRWRegister(1, 4, 117, "int16", 2, temp_voltage_2, false);
    Wait(0.02);

    // Motor_3 base address = 0xA0 (decimal 160).
    XPRWRegister(1, 4, 160, "int32", 1, hall_3, false);
    Wait(0.02);
    XPRWRegister(1, 4, 162, "int16", 3, data_3, false);
    Wait(0.02);
    XPRWRegister(1, 4, 165, "int16", 2, temp_voltage_3, false);
    Wait(0.02);

    // Motor_4 base address = 0xD0 (decimal 208).
    XPRWRegister(1, 4, 208, "int32", 1, hall_4, false);
    Wait(0.02);
    XPRWRegister(1, 4, 210, "int16", 3, data_4, false);
    Wait(0.02);
    XPRWRegister(1, 4, 213, "int16", 2, temp_voltage_4, false);

    // 3) Print the three finger angles and the rotation joint angle.
    Print("DexHand Motor_1 angle(deg):", finger_angle[1] / 100.0);
    Print("DexHand Motor_2 angle(deg):", finger_angle[2] / 100.0);
    Print("DexHand Motor_3 angle(deg):", finger_angle[3] / 100.0);
    Print("DexHand Motor_4 rotation angle(deg):", rotation_angle[1] / 100.0);

    // 4) Print Motor_1~Motor_4 basic data.
    Print("Motor_1 hall:", hall_1[1], "speed(deg/s):", data_1[1], "current(mA):", data_1[2], "torque(PWM):", data_1[3], "temp(C):", temp_voltage_1[1], "voltage(mV):", temp_voltage_1[2]);
    Print("Motor_2 hall:", hall_2[1], "speed(deg/s):", data_2[1], "current(mA):", data_2[2], "torque(PWM):", data_2[3], "temp(C):", temp_voltage_2[1], "voltage(mV):", temp_voltage_2[2]);
    Print("Motor_3 hall:", hall_3[1], "speed(deg/s):", data_3[1], "current(mA):", data_3[2], "torque(PWM):", data_3[3], "temp(C):", temp_voltage_3[1], "voltage(mV):", temp_voltage_3[2]);
    Print("Motor_4 hall:", hall_4[1], "speed(deg/s):", data_4[1], "current(mA):", data_4[2], "torque(PWM):", data_4[3], "temp(C):", temp_voltage_4[1], "voltage(mV):", temp_voltage_4[2]);
ENDPROC
