/*************************************************
Copyright:wootion
Author: ZhouJinGang
Date:2018-12-25
Description: IAI Slave define
**************************************************/

#ifndef PROJECT_IAICONTROL_H
#define PROJECT_IAICONTROL_H

#define IAI_ORIGIN_RETURN 0x00000212
#define IAIStartMoveToPos(n) (0x0211+(n<<16))
#define IAICompleteMoveToPos(n) (0x0210+(n<<16))
#define IAI_SHAFT_SLAVE_ON 0x00000010 //伺服ON
#define IAI_SHAFT_INITIALISE 128

#define ON 1;
#define OFF 0;

typedef struct _tagIAIStatus_
{
//    IAI_ONLINE = 0, /* 从站在线 */
//    IAI_OFFLINE, /* 从站掉线 */
    int IAI_RUN; /* 网关正常动作 */
    int IAI_USE_SHAFT; /* 使用0-3 四个电缸轴 */
}IAIStatus;

typedef struct _tagIAIShaftStatus_
{
    int IAI_LocationNum;/* 位置点 */

    int IAI_Localization_Done; /* 定位完成 */
    int IAI_Origin_Return_Done; /* 原点复位完成 */
    int IAI_Moving; /* 移动中 */
    int IAI_Alerting; /* 报警 */
    int IAI_Servo_On; /* 运动准备结束(伺服ON) */
    int IAI_Push_Press_Done; /* 推压动作空转完成 */
    int IAI_Collision; /* 负载输出判定(检出碰撞是ON) */
    int IAI_Overload_Alerting; /* 发生过载警告或信息级别报警时 */

    int IAI_MV_Command_Done; /* 移动指令完成 */
    int IAI_Get_Position_Data_Done; /* 位置数据获取完成 */
    int IAI_Teach_Mode; /* 敷在输出判定(检出碰撞是ON) */
    int IAI_Ctrl_Ready; /* 控制器准备完成 */
    int IAI_Emergency_stop; /* 敷在输出判定(检出碰撞是ON) */

}IAIShaftStatus;

#endif //PROJECT_IAICONTROL_H
