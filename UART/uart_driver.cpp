#include<stdio.h>      /*标准输入输出定义*/
#include<stdlib.h>     /*标准函数库定义*/
#include<unistd.h>     /*Unix 标准函数定义*/
#include<sys/types.h> 
#include<sys/stat.h>   
#include<fcntl.h>      /*文件控制定义*/
#include<termios.h>    /*PPSIX 终端控制定义*/
#include<errno.h>      /*错误号定义*/
#include<string.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include "uart_driver.h"
#include <pthread.h>

#include <iostream>
#include "ros/ros.h"

//宏定义
#define FALSE  -1
#define TRUE   0

/* CRC 高位字节值表 */ 
const  uint8_t auchCRCHi[] = 
{ 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
	0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
	0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 
};

/* CRC低位字节值表*/ 
const  uint8_t auchCRCLo[] = 
{ 
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 
	0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 
	0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 
	0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 
	0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 
	0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
	0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 
	0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 
	0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 
	0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 
	0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
	0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 
	0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 
	0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 
	0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 
	0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 
	0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 
	0x43, 0x83, 0x41, 0x81, 0x80, 0x40 
};

/*
* 函数名 :CRC16
* 描述 : 计算CRC16
* 输入 : puchMsg---数据地址,usDataLen---数据长度
* 输出 : 校验值
*/
uint16_t CRC16(uint8_t *puchMsg, uint8_t usDataLen) 
{ 
	uint8_t uchCRCHi = 0xFF ;              // 高CRC字节初始化  
	uint8_t uchCRCLo = 0xFF ;              // 低CRC 字节初始化 

	uint8_t uIndex ;                      // CRC循环中的索引   
	
	while (usDataLen--)                  // 传输消息缓冲区  
	{ 
		uIndex = uchCRCHi ^ *puchMsg++ ; // 计算CRC         
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex]; 
		uchCRCLo = auchCRCLo[uIndex]; 
	}

	return (uchCRCHi << 8 | uchCRCLo);	// MODBUS 规定高位在前
}


int ys_Uart :: UartOpen( char* port)
{
    int fd;
    fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY);
    if (FALSE == fd)
    {
        perror("Can't Open Serial Port");
        return(FALSE);
    }
    //恢复串口为阻塞状态                               
    if(fcntl(fd, F_SETFL, 0) < 0)
    {
        printf("fcntl failed!\n");
        return(FALSE);
    } 
    else
    {
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
    }
    //测试是否为终端设备    
    if(0 == isatty(STDIN_FILENO))
    {
        printf("standard input is not a terminal device\n");
        return(FALSE);
    }
    else
    {
        printf("is a tty success!\n");
    }
    printf("fd->open=%d\n",fd);

    return fd;
}

void ys_Uart :: UART_Close(int fd)
{
    close(fd);
}

int ys_Uart :: UART_Set(int fd, int speed, int flow_ctrl, int databits, int stopbits, int parity)
{

    int   i;
    int   status;
    int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};
         
    struct termios options;
   
    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，
        并将它们保存于options,该函数还可以测试配置是否正确，
        该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
    */
    if  ( tcgetattr( fd, &options)  !=  0)
       {
          perror("SetupSerial 1");    
          return(FALSE); 
       }
  
    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
        if  (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }
   
    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;
  
    //设置数据流控制
    switch(flow_ctrl)
    {
       case 0 ://不使用流控制
            options.c_cflag &= ~CRTSCTS;
            break;   
      
       case 1 ://使用硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
       case 2 ://使用软件流控制
            options.c_cflag |= IXON | IXOFF | IXANY;
            break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
       case 5    :
                options.c_cflag |= CS5;
                break;
       case 6    :
                options.c_cflag |= CS6;
                break;
       case 7    :    
                options.c_cflag |= CS7;
                break;
       case 8:    
                options.c_cflag |= CS8;
                break;  
       default:   
                fprintf(stderr,"Unsupported data size\n");
                return (FALSE); 
    }
    //设置校验位
    switch (parity)
    {  
       case 'n':
       case 'N': //无奇偶校验位。
                 options.c_cflag &= ~PARENB; 
                 options.c_iflag &= ~INPCK;    
                 break; 
       case 'o':  
       case 'O'://设置为奇校验    
                 options.c_cflag |= (PARODD | PARENB); 
                 options.c_iflag |= INPCK;             
                 break; 
       case 'e': 
       case 'E'://设置为偶校验  
                 options.c_cflag |= PARENB;       
                 options.c_cflag &= ~PARODD;       
                 options.c_iflag |= INPCK;      
                 break;
       case 's':
       case 'S': //设置为空格 
                 options.c_cflag &= ~PARENB;
                 options.c_cflag &= ~CSTOPB;
                 break; 
        default:  
                 fprintf(stderr,"Unsupported parity\n");    
                 return (FALSE); 
    } 
    // 设置停止位 
    switch (stopbits)
    {  
       case 1:   
                 options.c_cflag &= ~CSTOPB; break; 
       case 2:   
                 options.c_cflag |= CSTOPB; break;
       default:   
                       fprintf(stderr,"Unsupported stop bits\n"); 
                       return (FALSE);
    }
   
    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;
  
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    //options.c_lflag &= ~(ISIG | ICANON);
   
    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */  
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */
   
    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);
   
    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("com set error!\n");  
        return (FALSE); 
    }

    return (TRUE);
}

int ys_Uart :: UART_Init(char *devname,int speed, int flow_ctrl, int databits, int stopbits)
{
    int err;
    uart_fd =  UartOpen(devname);

    //设置串口数据帧格式  例： (fd,115200,0,8,1,'N')
    if (UART_Set(uart_fd, speed, flow_ctrl, databits,stopbits, 'N') == FALSE)
    {
        printf("ttys init fail !! \n");
        return FALSE;
    }
    else
    {
        printf("ttys init success !! \n");
        loopOn = 1;
        return  TRUE;
    }
}

//设置本车地址及目标接驳台地址
unsigned char ys_Uart::Lora_SetAddress(unsigned short addr)
{
    char addr_c[15];
    unsigned char timeout = 0,retry = 0;

    run_loraset = 1;
rerun:

    printf("AT+ENTM \n");
    UART_Send("AT+ENTM\n", 8);
    ros::Duration(0.5).sleep();

    printf("+++ \n");
    UART_Send("+++", 3);
    ros::Duration(0.1).sleep();

    UART_Send("a", 1);
    while(lora_set.feedback_1_flag != 1 && timeout < 5){
        ros::Duration(0.1).sleep();
        timeout ++;
    }
    if(timeout >= 5 && retry < 5) {
        timeout = 0;
        retry ++;
        goto rerun;
    }
    if(retry >=5){
        run_loraset = 0;
        return FALSE;
    }

    lora_set.feedback_1_flag = 0;
    printf("finished +ok\n");

    sprintf(addr_c,"AT+ADDR=%d\n", addr);
    printf("len=%d,addr_c :%s", strlen(addr_c), addr_c);
    UART_Send(addr_c,strlen(addr_c));
    ros::Duration(0.5).sleep();

    UART_Send("AT+ENTM\n", 8);
    ros::Duration(0.5).sleep();

    UART_Send("AT+Z\n", 5);
    ros::Duration(0.5).sleep();

    printf("finished Set Addr\n");
    run_loraset = 0;
    return SUCCESS;
}

/*               以下为串口发送及处理相关代码                       */
int ys_Uart ::UART_Send(char *send_buf, int data_len)
{
    int len = 0;
   
    len = write(uart_fd, send_buf, data_len);
    tcflush(uart_fd, TCOFLUSH);
    if (len == data_len )
    {
        return TRUE;
    }
    else
    {
        tcflush(uart_fd, TCOFLUSH);
        return FALSE;
    }
}

void ys_Uart ::LORA_ModbusSend(unsigned char func,unsigned short register_addr ,unsigned short num)
{
    unsigned char buf[LORA_MODBUS_DATA_SEND_LEN];
    unsigned short crc_calc = 0;

    while(run_loraset == 1 || run_calcu_flag == 1){
        ros::Duration(0.1).sleep();
    }  //等待数据接收完成

    // if(run_loraset == 0 && run_calcu_flag == 0){
        buf[0] = LORA_MODBUS_DEVICE_ID;
        buf[1] = func;
        buf[2] = (register_addr >> 8) & 0xff;
        buf[3] = register_addr & 0xff;
        buf[4] = (num >> 8) & 0xff;
        buf[5] = num & 0xff;
        crc_calc = CRC16(buf,LORA_MODBUS_DATA_SEND_LEN - 2);
        buf[6] = (crc_calc >> 8) & 0xff;
        buf[7] = crc_calc & 0xff;

        printf("\n[LORA_ModbusSend] send:{%x %x %x %x %x %x %x %x}\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
        UART_Send((char*)buf,LORA_MODBUS_DATA_SEND_LEN);
        // ros::Duration(0.1).sleep();
        c_send.register_addr = register_addr;
        c_send.register_code = num;
        c_send.send_crc = crc_calc;
    // }
}

/*               以下为串口接收及处理相关代码                     */
int ys_Uart :: UART_Recv(char *rcv_buf, int data_len)
{
    int len, fs_sel;
    fd_set fs_read;
   
    struct timeval time;
   
    FD_ZERO(&fs_read);
    FD_SET(uart_fd,&fs_read);
   
    time.tv_sec = 10;
    time.tv_usec = 0;
   
    //使用select实现串口的多路通信
    // fs_sel = select(uart_fd + 1, &fs_read, NULL, NULL, &time);
    fs_sel = select(uart_fd + 1, &fs_read, NULL, NULL, NULL);
    if(fs_sel)
    {
        len = read(uart_fd, rcv_buf, data_len);
	    //printf("[UART_Recv] len = %d fs_sel = %d  \n", len, fs_sel);
        return len;
    }
    else
    {
	      //printf("[UART_Recv] erro  \n");
            return FALSE;
    }
}

unsigned int ys_Uart::CalculateRawModbus(unsigned char *raw_date, int len,unsigned char device_id)
{
    int i = 0;
    unsigned char flag_03 = 0,flag_06 = 0;
	unsigned short calcu_crc = 0x00;

    for(i = 0; i < len; i++){

        //printf("i:%d,%x,%x \n",i, raw_date[i], raw_date[i+1]);

        if(raw_date[i] == device_id && raw_date[i+1] == 0x03){
            flag_03 = 1;
            break;
        }
        if(raw_date[i] == device_id && raw_date[i+1] == 0x06){
            flag_06 = 1;
            break;
        }
    }

    if(i >= len) return NO_START_CODE;  //没收到起始头

	i++;
    if(flag_03 == 1){  //反馈寄存器值
        
        c_recv.func_code = 0x03;
        if(len < 25) return LEN_ERROR;

		c_recv.register_len = raw_date[i+1];
		c_recv.Drop_start = (raw_date[i+2]<<8) & 0xff00 | raw_date[i+3] & 0x00ff;
		c_recv.Drop_startroll = (raw_date[i+4]<<8) & 0xff00 | raw_date[i+5] & 0x00ff;
		c_recv.Drop_nownum = (raw_date[i+6]<<8) & 0xff00 | raw_date[i+7] & 0x00ff;
		c_recv.Reserved4 = (raw_date[i+8]<<8) & 0xff00 | raw_date[i+9] & 0x00ff;
		c_recv.Reserved5 = (raw_date[i+10]<<8) & 0xff00 | raw_date[i+11] & 0x00ff;
		c_recv.Reserved6 = (raw_date[i+12]<<8) & 0xff00 | raw_date[i+13] & 0x00ff;
		c_recv.Reserved7 = (raw_date[i+14]<<8) & 0xff00 | raw_date[i+15] & 0x00ff;
		c_recv.Reserved8 = (raw_date[i+16]<<8) & 0xff00 | raw_date[i+17] & 0x00ff;
		c_recv.Reserved9 = (raw_date[i+18]<<8) & 0xff00 | raw_date[i+19] & 0x00ff;
		c_recv.Reserved10 = (raw_date[i+20]<<8) & 0xff00 | raw_date[i+21] & 0x00ff;
        c_recv.recv_crc = (raw_date[i+22]<<8) & 0xff00 | raw_date[i+23] & 0x00ff;

		calcu_crc = CRC16(raw_date + i - 1, 25 - 2);

        printf("[CalculateRawModbus] 03--d--i:%d,len:%d,{%d %d %d %d %d %d %d %d %d %d 0x%x 0x%x}\n",i,c_recv.register_len,
                                                        c_recv.Drop_start,c_recv.Drop_startroll,c_recv.Drop_nownum,c_recv.Reserved4,c_recv.Reserved5,c_recv.Reserved6,c_recv.Reserved7,c_recv.Reserved8,c_recv.Reserved9,c_recv.Reserved10,c_recv.recv_crc,calcu_crc);
        printf("[CalculateRawModbus] 03--x--i:%d,len:%d,{0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x}\n",i,c_recv.register_len,
                                                        c_recv.Drop_start,c_recv.Drop_startroll,c_recv.Drop_nownum,c_recv.Reserved4,c_recv.Reserved5,c_recv.Reserved6,c_recv.Reserved7,c_recv.Reserved8,c_recv.Reserved9,c_recv.Reserved10,c_recv.recv_crc,calcu_crc);
        if(calcu_crc != c_recv.recv_crc){
            printf("[CalculateRawModbus] crc error!\n");
            return CRC_ERROR;
        }
        
        return SUCCESS;
    }
	if(flag_06 == 1){
        c_recv.func_code = 0x06;

        if(len < 8) return LEN_ERROR;

        c_recv.register_addr = (raw_date[i+1]<<8) & 0xff00 | raw_date[i+2] & 0x00ff;
        c_recv.register_code = (raw_date[i+3]<<8) & 0xff00 | raw_date[i+4] & 0x00ff;

        c_recv.recv_crc = (raw_date[i+5]<<8) & 0xff00 | raw_date[i+6] & 0x00ff;

        calcu_crc = CRC16(raw_date + i -1, 8 -2);

        printf("[CalculateRawModbus] 06--d--{%d %d 0x%x 0x%x}\n",c_recv.register_addr, c_recv.register_code, c_recv.recv_crc, calcu_crc);
        printf("[CalculateRawModbus] 06--x--{0x%x 0x%x 0x%x 0x%x}\n",c_recv.register_addr, c_recv.register_code, c_recv.recv_crc, calcu_crc);

        if(calcu_crc != c_recv.recv_crc){
            printf("[CalculateRawModbus] crc error!\n");
            return CRC_ERROR;
        }

        return SUCCESS;
	}

    return NO_DATA;
}

int ys_Uart::UARTReadLoop(std::function <void(Connection_recv *msg)>callback,uint64_t interval)
{
    th = new std::thread([&, callback, interval]() -> void {
    
        char buffer[LORA_MODBUS_DATA_REC_LEN * 2];
        int len = 0;
        unsigned int i = 0, ret = 0;

        printf("[UARTReadLoop] init !!\n");

        while(loopOn){

            len = UART_Recv(buffer, sizeof(buffer));
            
            if(len > 0 ){

                printf("\n[UARTReadLoop]  len = %d ", len);

                for(i = 0; i < len; i ++){
                    printf("[%d]:0x%x", i, buffer[i]);
                }printf("\n");

                if(len == 1 && buffer[0] == 'a' && run_loraset == 1){   //a
                    lora_set.feedback_a_flag = 1;
                }
                else if(strstr(buffer, "+OK") && run_loraset == 1){
                    lora_set.feedback_1_flag = 1;
                }
                else if(strstr(buffer, "ERR-1") && run_loraset == 1){
                    lora_set.feedback_error_flag = 1;
                }
                else{
                    run_calcu_flag = 1;
                    
                    ret = CalculateRawModbus((unsigned char *)buffer, len, LORA_MODBUS_DEVICE_ID);
                    c_recv.error_code = ret;
                    callback(&c_recv);
             
                    run_calcu_flag = 0;
                }
                
                memset(buffer, 0, sizeof(buffer));
            }
            else{
                printf("\n[readloop]no data recv \n");
            }
        }

    });

    return 0;
}

void ys_Uart::EndReadLoop() {
    
    loopOn = 0;

    close(uart_fd);
    th->join();
    delete th;
}


/*         发送和接收的整个流程             */
//发送数据  -->>  等待回复  -->> 检查回复是否正确
unsigned char ys_Uart::Modbus_SendAndCheckACK(unsigned char func,unsigned short register_addr ,unsigned short num)
{

    LORA_ModbusSend(func, register_addr, num);


    return 0;
}

//接收到的数据
void ys_Uart::Modbus_RecvCallBack(Connection_recv *msg)
{

    if(msg->func_code == 0x03){   //收到读取命令的反馈
            
    }
    else if(msg->func_code ==0x06){  //收到设置命令的反馈
        switch(msg->register_addr){
            //卸货的反馈
            case 1000: 
                if(msg->register_code == 0x09) addr1000_09_fd = 1; 
                else if (msg->register_code == 0x00) addr1000_00_fd = 1;
                break;
            case 1001: if(msg->register_code == 0x09) addr1001_09_fd = 1;break;
            case 1002: if(msg->register_code == drop_nownum) addr1002_num_fd = 1;break; //反馈的当前个数和发送的个数相同时

            default : break;
        }
    }
}

/*  接驳    相关流程 -----> 卸货  */
//开始接驳
unsigned char ys_Uart::Connect_StartDrop(unsigned char start, unsigned char nownum)
{
    unsigned char ret = 0;
    unsigned char timeout = 0,retry = 0;

    drop_nownum = nownum;

    if(start == 1){     //发送开始卸货接驳信号
send_start:
        LORA_ModbusSend(0x06, 1000, 0x09);
        ret = WAIT_DROP_START;
    
        while(timeout <= 10){
            ros::Duration(0.1).sleep();
            timeout ++;
            if(addr1000_09_fd == true) break;
        }
        if(timeout > 10 && retry <= 5){timeout = 0;retry ++;goto send_start;}   //超时未回复，重新发送
        else if(timeout > 10 && retry > 5){return NO_RESPOND_ERROR;}   //重复5次依旧未回复
        else if(timeout < 10){  //有回复
            addr1000_09_fd = false;
            retry = 0;  timeout = 0;
            ret = SUCCESS;
        }

send_nownum:
        LORA_ModbusSend(0x06, 1002, drop_nownum);  //发送当前为第几次接驳
        ret = WAIT_DROP_NOWNUM;
        //等待plc反馈
        while(timeout <= 10){
            ros::Duration(0.1).sleep();
            timeout ++;
            if(addr1002_num_fd == true) break;
        }
        if(timeout > 10 && retry <= 5){timeout = 0; retry ++; goto send_nownum;}   //超时未回复，重新发送
        else if(timeout > 10 && retry > 5){return NO_RESPOND_ERROR;}   //重复5次依旧未回复
        else if(timeout < 10){  //有回复
            addr1002_num_fd = false;
            retry = 0;  timeout = 0;
            ret = SUCCESS;
        }

    }

    return ret;
}

//等待接驳线反馈卸货准备好了 此时小车转动滚筒
unsigned char ys_Uart::Connect_WaitRoll()
{
    unsigned char timeout = 0,retry = 0, ret = 0;

send_0x03:
    c_recv.error_code = NO_DATA;
    c_recv.Drop_startroll = 0;
    LORA_ModbusSend(0x03, 1000, 10); //读取地址从1000开始的10个寄存器值

    while(timeout <= 10 ){  //等待plc反馈数据
        ros::Duration(0.2).sleep();
        timeout ++;
        if(c_recv.error_code != NO_DATA) break;  //收到plc反馈数据
    }

   if(timeout < 10){   //plc有反馈
        if(c_recv.Drop_startroll == 0x09){ret = SUCCESS;}
        else{
            ros::Duration(30).sleep();  //等待30秒后重发 等待PLC准备好
            goto send_0x03;
        }
   }
   else{   //plc无反馈，重新请求寄存器数据
    if(retry > 5){ return NO_RESPOND_ERROR;}
    if(retry < 5){
        timeout = 0;
        retry ++;
        goto send_0x03;
    }
   }

    return ret;
}

//收到卸货准备好信号后，需要清空开始接驳信号
unsigned char ys_Uart::Connect_ClearStartDrop()
{
    unsigned char ret = 0, retry = 0,timeout = 0;

clear_start:
    addr1000_00_fd = false;
    LORA_ModbusSend(0x06, 1000, 0x00);   //清空plc 的 addr:1000
    ret = WAIT_DROP_START;
    while(timeout <= 10){
        ros::Duration(0.1).sleep();
        timeout ++;
        if(addr1000_00_fd == true) break;
    }
    if(timeout > 10 && retry <= 5){timeout = 0;retry ++;goto clear_start;}   //超时未回复，重新发送
    else if(timeout > 10 && retry > 5){return NO_RESPOND_ERROR;}   //重复5次依旧未回复
    else if(timeout < 10){  //有回复
        addr1000_00_fd = false;
        retry = 0;  timeout = 0;
        ret = SUCCESS;
    }

    return ret;
}


/*接驳    相关流程 -------> 装货  */
unsigned char ys_Uart::Connect_StartLoad(unsigned char mode, unsigned char nownum)
{

}










