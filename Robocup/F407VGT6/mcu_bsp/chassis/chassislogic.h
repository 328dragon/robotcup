#ifndef __CHASSISLOGIC_H
#define __CHASSISLOGIC_H


#define SHOVOL 0.3
typedef enum
{
    IDLE_CHASSIS = 0, // 空闲状态
    LEAVE_HOME = 1,//离开家
    FIND_GRAY = 2,//等待灰度寻找十字
    FIND_THING = 3,//前往寻找物体
    WAITPICK =4,//等待上层机构拾取物体
    TOGOAL=5,//开环粗前往目标点
    FINDING_GOAL=6,//等待摄像头找到目标点
    FINDED_GOAL=7,//摄像头找到目标点
    TO_ADJUSTGOAL=8,//前往并调整目标点
    WAITGOAL=9//等待上层机构得分

}ChassisTaskFlag;


#endif
