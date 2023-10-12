
#include "ros/ros.h"
#include "ys_ros_msgs/vehicle_info.h"

#include <LoopThread.h>
#include "geometry_msgs/Twist.h"
#include "ys_ros_msgs/VehicleCMD.h"
#include "ys_ros_msgs/Location.h"
#include "ys_ros_msgs/VehicleStatus.h"
#include "uart_driver.h"

#include <stdlib.h>
#include <string.h>

ys_Uart ys_uart0;
ZBaseLoopThread* uart_thWrite;
unsigned int conv_mod = 1;

#define SRAM_buf_size 200 * 1024

uint32_t SRAM_jpeg_buf[SRAM_buf_size];


bool UART_WriteThread() {
    unsigned char nownum = 1, allnum = 3;
    unsigned char ret = 0;

    //当接收到其他节点的路线完成标志后就进行接驳

    //测试流程  接驳  ---->卸货
    while(nownum <= allnum){
        ros::Duration(0.5).sleep();
        ret = ys_uart0.Connect_StartDrop(conv_mod, nownum);
        if(ret == SUCCESS){
            ret = ys_uart0.Connect_WaitRoll();
            if(ret == SUCCESS){
                //此时应该开启滚筒,需要向控制节点发送开启滚筒的msg

                ret = ys_uart0.Connect_ClearStartDrop();
                if(ret == SUCCESS){
                    //此时应该检测传感器判断该剁货物是否转移完成，需要控制节点发送传感器的msg



                    //如果转移完成则开始下一个转移流程


                }else break;
            }else break;
        }else break;

        nownum ++;
    }

    if(ret != SUCCESS){
        //此处应该发布一个错误信息给决策节点
        printf("Error ret:%d \n",ret);
        conv_mod = 0;
    }

    //测试流程  接驳  ---->上货
    

    return true;
}

void UARTMessageCallback(Connection_recv *msg)
{
    if(msg->error_code != SUCCESS){    //传输出错
        printf("[UARTMessageCallback] Error: %d\n", msg->error_code);
    }
    else{   //传输成功
        ys_uart0.Modbus_RecvCallBack(msg);
    }
}

void UARTattach()
{
    ys_uart0.UART_Init("/dev/ttyUSB0", 115200, 0, 8, 1);
    ys_uart0.UARTReadLoop(std::bind(UARTMessageCallback, std::placeholders::_1), 10);

    uart_thWrite = new ZBaseLoopThread(std::bind(UART_WriteThread));
    uart_thWrite->SetLoopInterval(1000);
    uart_thWrite->Start();
}

void UARTdetach()
{
    ys_uart0.EndReadLoop();

    uart_thWrite->Stop();
    delete uart_thWrite;
    uart_thWrite = nullptr;
}

int main(int argc, char *argv[])
{
    //ys_ros_msgs::vehicle_info car_info;
    ros::init(argc,argv,"ys_ros_conveyor");
    
    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("cmd_vel",10,doControlCar);
    // ros::Publisher vehicle_info_pub = nh.advertise<ys_ros_msgs::vehicle_info>("carInfo_051",10);


    UARTattach();

    ros::Rate rate(10);
    
    ys_uart0.Lora_SetAddress(0xf001);   //设置lora地址为0xf001

    while(ros::ok()){


        
        rate.sleep();
        ros::spinOnce();
    }

    UARTdetach();
    return 0;
}



