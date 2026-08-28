// DexHand021 S / ER7 末端 RS485 只读探测
// 仅验证控制器 RL 末端透传是否能够收到 021S 的 P1~P3 输入寄存器回复，
// 不发送任何 0x31 电机控制帧。
//
// XPRS485RWData 的第 2、4 个参数必须是 byte 数组，数组长度必须分别
// 等于第 1、3 个参数；校验和由 DexHand 帧本身携带，RL 不再补 CRC。

VAR byte tx[8] = {1,4,0,0,0,3,176,11};
VAR byte rx[11] = {0,0,0,0,0,0,0,0,0,0,0};
VAR int ret = -1;

GLOBAL PROC main()
    // ER7 末端工具 RS485：115200、无校验、1 位停止位。
    // 字符串必须加引号，不能写成未定义变量 none/stopbit1。
    XPRS485Init(115200, "none", "stopbit1");

    // 01 04 00 00 00 03 B0 0B：读取 P1/P2/P3 输入寄存器。
    XPRS485RWData(8, tx, 11, rx, ret);

    // 控制器日志只打印固定文本；回读数组保留在 rx 中供调试器查看。
    Print("DexHand021 S RL 485 read probe complete");
ENDPROC
