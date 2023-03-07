#include "linux_serial.h"

using namespace std;
using namespace boost::asio;
//串口相关对象
boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyUSB0");
boost::system::error_code err;
/********************************************************
            串口发送接收相关常量、变量、共用体对象
********************************************************/
const unsigned char ender[2] = {0x0d, 0x0a};
const unsigned char header[2] = {0x55, 0xaa};

typedef union FloatData
{
    float 	f_data;
    unsigned char	byte[4];
}UsartFloatData;


//发送数据共用体
UsartFloatData   joint1_pos, 			joint2_pos, 		joint3_pos,			joint4_pos,			joint5_pos,			joint6_pos,			joint7_pos;
//接收数据共用体
UsartFloatData joint1_pos_now, 	joint2_pos_now, joint3_pos_now,	joint4_pos_now,	joint5_pos_now,	joint6_pos_now,	joint7_pos_now;


/********************************************************
函数功能：串口参数初始化
入口参数：无
出口参数：
********************************************************/
void serialInit()
{
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));    
}

/********************************************************
函数功能：将对机器人的位置控制关节角度，打包发送给下位机
入口参数：关节角度值
出口参数：
********************************************************/
void  writePosition(float joint1_position, float joint2_position,float joint3_position,float 
									joint4_position,float joint5_position,float joint6_position,float joint7_position,unsigned char ctrlFlag)
{
    unsigned char buf[35] = {0};//
    int i, length = 0;

    joint1_pos.f_data  = joint1_position;
	joint2_pos.f_data  = joint2_position;
	joint3_pos.f_data  = joint3_position;
	joint4_pos.f_data  = joint4_position;
	joint5_pos.f_data  = joint5_position;
	joint6_pos.f_data  = joint6_position;
	joint7_pos.f_data  = joint7_position;
    
    // 设置消息头
    for(i = 0; i < 2; i++)
        buf[i] = header[i];             //buf[0]  buf[1]
    
    // 设置机器人7关节角度位置
   length = 29;
	buf[2] = length;                             // buf[2]
	for(i = 0; i < 4; i++)
	{
		buf[i + 3] = joint1_pos.byte[i];         // buf[3] buf[4] 5 6
		buf[i + 7] = joint2_pos.byte[i];        // 7 8 9 10
		buf[i + 11] = joint3_pos.byte[i];           // buf[11] buf[12] 13 14
		buf[i + 15] = joint4_pos.byte[i];
		buf[i + 19] = joint5_pos.byte[i];
		buf[i + 23] = joint6_pos.byte[i];
		buf[i + 27] = joint7_pos.byte[i];					// buf[27] buf[28] 29 30
		
	}
    // 预留控制指令
    buf[3 + length - 1] = ctrlFlag;       //buf[31]

    // 设置校验值、消息尾
    buf[3 + length] = getCrc8(buf, 3 + length);//buf[32]
    buf[3 + length + 1] = ender[0];     //buf[33]
    buf[3 + length + 2] = ender[1];     //buf[34]
    // for(int j = 0;   j<36;    j++)
    // {
    //     ROS_INFO("%u\n",buf[j]&0xff);
    // }


    // 通过串口下发数据
   boost::asio::write(sp, boost::asio::buffer(buf));
}
/********************************************************
函数功能：从下位机读取数据
入口参数：7自由度关节角度，预留控制位
出口参数：bool
********************************************************/
bool readPosition(float *p_joint_pos_now,unsigned char *ctrlFlag)
{
    char i, length = 0;
    unsigned char checkSum;
    unsigned char usart_rx_buff[150]={0};
    //=========================================================
    //此段代码可以读数据的结尾，进而来进行读取数据的头部
    try
    {
        boost::asio::streambuf response;
        boost::asio::read_until(sp, response, "\r\n",err);   
        copy(istream_iterator<unsigned char>(istream(&response)>>noskipws),
        istream_iterator<unsigned char>(),
        usart_rx_buff); 
    }  
    catch(boost::system::system_error &err)
    {
        ROS_INFO("read_until error");
    } 
    //=========================================================        

    // 检查信息头
    if (usart_rx_buff[0]!= header[0] || usart_rx_buff[1] != header[1])   //buf[0] buf[1]
    {
        ROS_ERROR("Received message header error!");
        return false;
    }
    // 数据长度
    length = usart_rx_buff[2];                                 //buf[2]

    // 检查信息校验值
    checkSum = getCrc8(usart_rx_buff, 3 + length);             //buf[10] 计算得出
    if (checkSum != usart_rx_buff[3 + length])                 //buf[10] 串口接收
    {
        ROS_ERROR("Received data check sum error!");
        return false;
    }    

    // 读取
     for(i = 0; i < 4; i++)
					{
						joint1_pos_now.byte[i]  = usart_rx_buff[i + 3]; //buf[3]  buf[4] 5 6 
						joint2_pos_now.byte[i]  = usart_rx_buff[i+ 7]; //buf[7]  buf[6] 9 10
						joint3_pos_now.byte[i]  = usart_rx_buff[i + 11];
						joint4_pos_now.byte[i]  = usart_rx_buff[i + 15];
						joint5_pos_now.byte[i]  = usart_rx_buff[i + 19];
						joint6_pos_now.byte[i]  = usart_rx_buff[i + 23];
						joint7_pos_now.byte[i]  = usart_rx_buff[i+ 27];//27 28 29 30
					}

    // 读取控制标志位
    *ctrlFlag = usart_rx_buff[9];
    
    p_joint_pos_now[0]  = (float)joint1_pos_now.f_data;
    p_joint_pos_now[1]  = (float)joint2_pos_now.f_data;
    p_joint_pos_now[2]  = (float)joint3_pos_now.f_data;
   p_joint_pos_now[3] = (float)joint4_pos_now.f_data;
   p_joint_pos_now[4]   = (float)joint5_pos_now.f_data;
   p_joint_pos_now[5]  = (float)joint6_pos_now.f_data;
   p_joint_pos_now[6]  = (float)joint7_pos_now.f_data;

    return true;
}
/********************************************************
函数功能：获得8位循环冗余校验值
入口参数：数组地址、长度
出口参数：校验值
********************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else 
                crc >>= 1;
        }
    }
    return crc;
}
