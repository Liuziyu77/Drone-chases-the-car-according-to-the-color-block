#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "cam/bias.h"
#include <vector>
#include <iostream>
#include <numeric>
#include <cstring>
#include <cstdio>


serial::Serial ser;

void write_callback(const cam::bias::ConstPtr& msg){
    if(msg->flage ==1){
        // 发送校验位 0x41,0x43
        int sumA = 0;
        int sumB = 0;
        char data[] = {0x41, 0x43};
        int len1 = sizeof(data)/sizeof(data[0]);
        // assume uart is already set up and initialized
        ser.write((const uint8_t*)&data[0], len1);

        // 发送校验位 0x02,8
        char data2[] = {0x02, 8};
        int len2 = sizeof(data2)/sizeof(data2[0]);
        for(int i = 0; i < len2; i++) {
            sumB += data2[i];
            sumA += sumB;
        }
        ser.write((const uint8_t*)&data2, len2);
        ROS_INFO_STREAM("  SumA:"<<sumA<<"  SumB:"<<sumB);

        // 将x的位置偏移转化为byte类型发送
        float float_value = msg->bias_x;
        std::vector<char> float_bytes(sizeof(float));
        memcpy(&float_bytes[0], &float_value, sizeof(float));
        int len3 = float_bytes.size();
        for(int i = 0; i < len3; i++) {
            sumB += float_bytes[i];
            sumA += sumB;
        }
        ser.write((const uint8_t*)&float_bytes[0], len3);
        ROS_INFO_STREAM("  SumA:"<<sumA<<"  SumB:"<<sumB);


        // 将y的位置偏移转化为byte类型发送
        float_value = msg->bias_y;
        memcpy(&float_bytes[0], &float_value, sizeof(float));
        for(int i = 0; i < len3; i++) {
            sumB += float_bytes[i];
            sumA += sumB;
        }
        ser.write((const uint8_t*)&float_bytes[0], len3);
        ROS_INFO_STREAM("  SumA:"<<sumA<<"  SumB:"<<sumB);

        // 发送sumB和sumA,校验
        char sumA_char = sumA;
        char sumB_char = sumB;
        char data3[] = {sumB_char,sumA_char};
        int len4 = sizeof(data3)/sizeof(data3[0]);
        ser.write((const uint8_t*)&data3, len4);

        ROS_INFO_STREAM("Writing to serial port  " <<"bias_x:"<< msg->bias_x <<"   bias_y:"<< msg->bias_y<<"  SumA:"<<sumA<<"  SumB:"<<sumB);

        // ser.write((const uint8_t*)&msg->bias_x, sizeof(msg->bias_x));
        // ser.write((const uint8_t*)&msg->bias_y, sizeof(msg->bias_y));
    }
    else{
        // 发送校验位 0x41,0x43
        int sumA = 0;
        int sumB = 0;
        char data[] = {0x41, 0x43};
        int len1 = sizeof(data)/sizeof(data[0]);
        // assume uart is already set up and initialized
        ser.write((const uint8_t*)&data[0], len1);

        // 发送校验位 0x02,8
        char data2[] = {0x01, 0};
        int len2 = sizeof(data2)/sizeof(data2[0]);
        for(int i = 0; i < len2; i++) {
            sumB += data2[i];
            sumA += sumB;
        }
        ser.write((const uint8_t*)&data2, len2);
        ROS_INFO_STREAM("  SumA:"<<sumA<<"  SumB:"<<sumB);

        // 发送sumB和sumA,校验
        char sumA_char = sumA;
        char sumB_char = sumB;
        char data3[] = {sumB_char,sumA_char};
        int len4 = sizeof(data3)/sizeof(data3[0]);
        ser.write((const uint8_t*)&data3, len4);
        ROS_INFO_STREAM("Writing to serial port  "<<"  SumA:"<<sumA<<"  SumB:"<<sumB);

    }
}

int main (int argc, char** argv){
    // 初始化结点
    ros::init(argc, argv, "uart_catch_car");
    // 设置节点句柄
    ros::NodeHandle nh;
    // 设置订阅者，订阅色块位置的偏置信息/color_block_bias，并在回调函数中通过UART发送出去
    ros::Subscriber write_sub = nh.subscribe("/color_block_bias", 10, write_callback);
    // ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    // 串口的初始化，以及检查串口是否配置完成
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    // // ros::Rate loop_rate(5);
    // while(ros::ok()){

    //     ros::spinOnce();

    //     if(ser.available()){
    //         // ROS_INFO_STREAM("Reading from serial port");
    //         // std_msgs::String result;
    //         // result.data = ser.read(ser.available());
    //         // ROS_INFO_STREAM("Read: " << result.data);
    //         // read_pub.publish(result);
    //     }
    // }


    // 循环等待回调函数
    ros::spin();

    return 0;
}

