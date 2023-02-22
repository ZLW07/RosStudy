//
// Created by wei on 2022/1/20.
//
#include "Log/log.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"control");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);
    sensor_msgs::JointState oJointStates;
    oJointStates.header.stamp = ros::Time::now();
    oJointStates.name.resize(6);
    oJointStates.position.resize(6);
    double dTrans = 3.1415926/180;
    oJointStates.name[0] = "BaseToLink_1";
    oJointStates.name[1] = "Link_1ToLink_2";
    oJointStates.name[2] = "Link_2ToLink_3";
    oJointStates.name[3] = "Link_3ToLink_4";
    oJointStates.name[4] = "Link_4ToLink_5";
    oJointStates.name[5] = "Link_5ToLink_6";
    oJointStates.position[0] = 0;
    oJointStates.position[1] = 0;
    oJointStates.position[2] = 0;
    oJointStates.position[3] = 0;
    oJointStates.position[4] = 0;
    oJointStates.position[5] = 0;
    ros::Rate r(10);
    int ii = 150;
    bool bTemp = true;
    while(ros::ok())
    {
        if (bTemp)
        {
            oJointStates.position[2] = ii * dTrans;
            oJointStates.header.stamp = ros::Time::now();
            pub.publish(oJointStates);
            if (ii <=10)
            {
               bTemp = false;
            }
            ii--;
        }
        else
        {
            oJointStates.position[2] = ii * dTrans;
            oJointStates.header.stamp = ros::Time::now();
            pub.publish(oJointStates);
            if (ii >= 150)
            {
                bTemp = true;
            }
            ii++;
        }
        r.sleep();
        ros::spinOnce();
    }
    return 0;

}

