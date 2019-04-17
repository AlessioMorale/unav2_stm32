#include "FreeRTOS.h"
#include "main.h"
#include "tim.h"
#include "timing.h"
#include <modules/rosmotormodule.h>
#include <ros.h>
#include <rosserial_msgs/Log.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

namespace unav::modules{

void RosMotorModule::initialize(){
    BaseRosModule::initialize(osPriority::osPriorityAboveNormal, 512);
}

void mot1_cb( const std_msgs::Int16& cmd_msg){
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmd_msg.data);
}

void mot2_cb( const std_msgs::Int16& cmd_msg){
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, cmd_msg.data);
}
void RosMotorModule::moduleThreadStart(){
    ros::Subscriber<std_msgs::Int16> subMot1("unav/mot1", mot1_cb);
    ros::Subscriber<std_msgs::Int16> subMot2("unav/mot2", mot2_cb);
    TickType_t c = xTaskGetTickCount();
    uint16_t lastreading1 = TIM2->CNT;
    uint16_t lastreading2 = TIM3->CNT;
    float lasttime = ((float)timing_getUs() * 0.000001);
    float speed1 = 0;
    float speed2 = 0;
    ros::Publisher pubEncoder1("unav/enc1", &encoder1);
    ros::Publisher pubEncoder2("unav/enc2", &encoder2);
    getNodeHandle().advertise(pubEncoder1);
    getNodeHandle().advertise(pubEncoder2);
    while (true) {
        float currenttime = ((float)timing_getUs() * 0.000001);
        float dt =currenttime - lasttime; 
        lasttime = currenttime;
        uint32_t current1 = TIM2->CNT;
        uint32_t current2 = TIM3->CNT;
        int32_t deltaenc1 = lastreading1 - current1; 
        int32_t deltaenc2 = lastreading2 - current2;
        speed1 = speed1 * 0.005f + 0.995f * ((float)deltaenc1) / dt;
        speed2 = speed2 * 0.005f + 0.995f * ((float)deltaenc2) / dt;
        lastreading1 = current1;
        lastreading2 = current2;
        encoder1.data = speed1;
        pubEncoder1.publish(&encoder1);
        encoder2.data = dt;
        pubEncoder2.publish(&encoder2);
        vTaskDelayUntil(&c, 1);  
    }
}
}