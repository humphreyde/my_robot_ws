#ifndef LINUX_SERIAL_H
#define LINUX_SERIAL_H

#include <ros/ros.h>
#include <ros/time.h>
#include <boost/asio.hpp>

extern void serialInit();
extern void  writePosition(float joint1_position, float joint2_position,float joint3_position,
                                                        float joint4_position,float joint5_position,float joint6_position,float joint7_position,unsigned char ctrlFlag);
extern bool readPosition(float *p_joint_pos_now,unsigned char *ctrlFlag);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);

#endif
