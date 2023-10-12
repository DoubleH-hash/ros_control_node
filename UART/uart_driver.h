#include <iostream>
#include <thread>
#include <functional>

using namespace std;


#define LORA_MODBUS_DATA_SEND_LEN 8
#define LORA_MODBUS_DATA_REC_LEN 25

#define LORA_MODBUS_DEVICE_ID 0x58

enum error_code{
    NO_START_CODE = 1,     //没收到数据
    NO_DATA,//无数据
    LEN_ERROR,  //反馈的数据长度错误
    CRC_ERROR,       //CRC校验失败

    FUNC_ERROR, //不支持的功能码
    WAIT_DROP_START,

    NO_RESPOND_ERROR,  //小车发送控制信号后，超时未收到plc回复的反馈信号

    WAIT_LOAD_START,
    WAIT_DROP_NOWNUM,//正在等待发送的卸货数量反馈信号

    SUCCESS,
};

//Lora模组的地址设置
typedef struct{
    unsigned int connection_addr = 0xf001;      //目标接驳台的地址

    unsigned int channel = 72;   //信道固定为72

    unsigned char feedback_a_flag = 0; // a
    unsigned char feedback_ok_flag = 0; //ok
    unsigned char feedback_1_flag = 0; //+ok
    unsigned char feedback_error_flag = 0; //ERROR-1


}Lora_set;

//向接驳台发送的数据
typedef struct{
    //unsigned char device_id = 0x58; //设备地址  默认为88，即0x58
    unsigned char func_code;  //功能码   03：读取寄存器值   06：设置寄存器值
    
    unsigned short register_addr;// 在06的情况下需要设置的寄存器地址
    unsigned short register_code; // 在06的情况下需要设置的寄存器值

    unsigned short send_crc = 0;
}Connection_send;

//从接驳台接收的数据 处理之后的数据
typedef struct{
    //unsigned char device_id = 0x58; //设备地址  默认为88，即0x58
    unsigned char func_code;  //功能码   03：读取寄存器值   06：设置寄存器值
    
    unsigned short register_addr;// 在06的情况下需要设置的寄存器地址
    unsigned short register_code; // 在06的情况下需要设置的寄存器值
    
    unsigned char register_len;  //寄存器长度 默认为 10个寄存器 每个寄存器反馈值16bit 即长度为0x14

    unsigned short Drop_start;   //卸货：开始卸货  addr：1000
    unsigned short Drop_startroll; //卸货：接驳台准备完毕 小车转动滚筒  addr：1001
    unsigned short Drop_nownum;//卸货：当前在卸第几个货 addr:1002
    unsigned short Reserved4;
    unsigned short Reserved5;
    
    unsigned short Reserved6;
    unsigned short Reserved7;
    unsigned short Reserved8;
    unsigned short Reserved9;
    unsigned short Reserved10;

    unsigned short recv_crc;

    unsigned char error_code;
}Connection_recv;

class ys_Uart
{
private:
    /* data */

    int uart_fd = 0;
    std::thread* th;
    int loopOn = 0;
    int run_loraset = 0;
    int run_calcu_flag = 0;

    bool addr1000_09_fd = false;
    bool addr1000_00_fd = false;
    bool addr1001_09_fd = false;
    bool addr1002_num_fd = false;
    bool drop_nownum = 0; //当前的接驳次数



public:

    int UartOpen(char* port);
    void UART_Close(int fd);
    int UART_Set(int fd, int speed, int flow_ctrl, int databits, int stopbits, int parity);
    int UART_Init(char *devname, int speed,int flow_ctrl,int databits,int stopbits);
    int UART_Recv(char *rcv_buf,int data_len);
    int UART_Send(char *send_buf,int data_len);
    int UARTReadLoop(std::function <void(Connection_recv *msg)>callback, uint64_t interval);
    void EndReadLoop();

    int Lora_Init(void);

    unsigned char Lora_SetAddress(unsigned short addr);

    unsigned int  CalculateRawModbus(unsigned char *raw_date, int len, unsigned char device_id);
    void LORA_ModbusSend(unsigned char func,unsigned short register_addr ,unsigned short num);
    unsigned char Modbus_SendAndCheckACK(unsigned char func,unsigned short register_addr ,unsigned short num);
    void Modbus_RecvCallBack(Connection_recv *msg);

    //接驳相关
    unsigned char Connect_StartDrop(unsigned char mode, unsigned char nownum);
    unsigned char Connect_WaitRoll();
    unsigned char Connect_ClearStartDrop();

    unsigned char Connect_StartLoad(unsigned char mode, unsigned char nownum);

    Lora_set lora_set;   //保存设置的lora参数
    Connection_send c_send;
    Connection_recv c_recv;

};





