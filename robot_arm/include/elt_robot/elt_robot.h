#ifndef ELT_ROBOT_H
#define ELT_ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "elt_rpc.h"

#define AXIS_COUNT 8
#define ROBOT_POSE_SIZE 6
#define ERR_MSG_MAX_LEN 256

// 机器人状态
enum EltRobotState {
    Stop = 0,                           // 停止状态
    Pause = 1,                          // 暂停状态
    EmeStop = 2,                        // 急停状态
    Running = 3,                        // 运行状态
    Error = 4,                          // 错误状态
};

// 机器人模式
enum EltRobotMode {
    Teach = 0,                          // 示教模式
    Play = 1,                           // 运行模式
    Remote = 2,                         // 远程模式
};

// 机器人坐标
enum EltRobotCoord {
    ROBOT_COORDINAT_JOINT = 0,
    ROBOT_COORDINAT_CART = 1,
    ROBOT_COORDINAT_TOOL = 2,
    ROBOT_COORDINAT_USER = 3,
    ROBOT_COORDINAT_CYLINDER = 4,
};

// 机器人循环模式
enum EltCycleMode {
    CYCLE_MODE_STEP = 0,
    CYCLE_MODE_ONE = 1,
    CYCLE_MODE_SERIES = 2,
};

// 机器人运动类型
enum EltMoveType {
    TRACK_MOVE_JOINT = 0,
    TRACK_MOVE_LINE = 1,
    TRACK_MOVE_ROTATE = 2,
    TRACK_MOVE_CIRCLE = 3,
};

// IO状态
enum EltIOStatus {
    IO_OFF = 0,        // 无效
    IO_ON = 1,          // 有效
};

typedef struct elt_error_t {
    int code;
    char err_msg[ERR_MSG_MAX_LEN];
} elt_error;


// 关节角信息
typedef double elt_robot_pos[AXIS_COUNT];
// 位姿信息(X,Y,Z,Rx,Ry,Rz)(X,Y,Z单位为毫米,Rx,Ry,Rz单位为弧度)
typedef double elt_robot_pose[ROBOT_POSE_SIZE];
// 马达速度
typedef double elt_motor_speed[AXIS_COUNT];
// 机器人编码器
typedef double elt_robot_encode[AXIS_COUNT];
// 机器人力矩
typedef double elt_robot_torques[AXIS_COUNT];

/**
 * 与机器人控制系统建立连接并获得授权
 * @ctx 登陆上下文
 * @return 是否登陆系统成功：true 登陆成功 false 登陆失败
 */
ELT_SDK_PUBLIC(int) elt_login(ELT_CTX ctx);


/**
 * 与机器人控制系统建立断开连接并取消授权
 * @ctx 登陆上下文
 * @return 是否成功登出
 */
ELT_SDK_PUBLIC(int) elt_logout(ELT_CTX ctx);


/**
 * 获取机器人当前位置信息
 * @ctx 登陆上下文
 * @pos_array 存储获取的机器人关节位置
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_get_robot_pos(ELT_CTX ctx, elt_robot_pos pos_array, elt_error *err);


/**
 * 获取机器人当前位姿信息
 * @ctx 登陆上下文
 * @pose_array 存储获取的机器人位姿信息
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_get_robot_pose(ELT_CTX ctx, elt_robot_pose pose_array, elt_error *err);


/**
 * 获取机器人状态
 * @ctx 登陆上下文
 * @state 存储获取的机器人状态 <EltRobotState>  
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_get_robot_state(ELT_CTX ctx, int *state, elt_error *err);


/**
 * 获取机器人模式
 * @ctx 登陆上下文
 * @mode 存储获取的机器人模式 <EltRobotMode>
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_get_robot_mode(ELT_CTX ctx, int *mode, elt_error *err);


/**
 * 获取机械臂伺服状态
 * @ctx 登陆上下文
 * @status 存储获取的机器人状态
            ELT_TRUE 启用 
            ELT_FALSE 未启用
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_get_servo_status(ELT_CTX ctx, int *status, elt_error *err);

/**
 * 设置伺服使能状态
 * @ctx 登陆上下文
 * @status 1 on 0 off
 * @err 错误信息
 * @return 成功或者失败
*/
ELT_SDK_PUBLIC(int) elt_set_servo_status(ELT_CTX ctx, int status, elt_error *err);

/**
 * 获取机械臂上下电状态
 * @ctx 登陆上下文
 * @status 存储获取的机器人上下电状态
            ELT_TRUE 上电 
            ELT_FALSE 下电
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_get_motor_status(ELT_CTX ctx, int *status, elt_error *err);

/**
 * 同步伺服编码器数据
 * @ctx 登陆上下文
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_sync_motor_status(ELT_CTX ctx, elt_error *err);

/**
 * 清除报警
 * @ctx 登陆上下文
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_clear_alarm(ELT_CTX ctx, int force, elt_error *err);

/**
 * 获取机器人马达速度
 * @ctx 登陆上下文
 * @speed_array 存储获取的机器人马达速度，转/分
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_get_motor_speed(ELT_CTX ctx, elt_motor_speed speed_array, elt_error *err);


/**
 * 获取机器人当前坐标
 * @ctx 登陆上下文
 * @coord 存储获取的机器人当前坐标 <EltRobotCoord>
 * @err 错误信息
 * @return 成功或者失败
*/
ELT_SDK_PUBLIC(int) elt_get_current_coord(ELT_CTX ctx, int *coord, elt_error *err);


/**
 * 获取机器人循环模式
 * @ctx 登陆上下文
 * @mode 存储获取的机器人循环模式 <EltCycleMode>
 * @err 错误信息
 * @return 成功或者失败
*/
ELT_SDK_PUBLIC(int) elt_get_cycle_mode(ELT_CTX ctx, int *mode, elt_error *err);

/**
 * 机器人自动运行
 * @ctx 登陆上下文
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_run(ELT_CTX ctx, elt_error *err);

/**
 * 机器人暂停
 * @ctx 登陆上下文
 * @err 错误信息
 * @return 成功或者失败
*/
ELT_SDK_PUBLIC(int) elt_pause(ELT_CTX ctx, elt_error *err);

/**
 * 获取机器人当前作业运行行号
 * @ctx 登陆上下文
 * @line_no 存储获取的机器人当前作业运行行号
 * @err 错误信息
 * @return 成功或者失败
*/
ELT_SDK_PUBLIC(int) elt_get_current_job_line(ELT_CTX ctx, int *line_no, elt_error *err);


/**
 * 获取机器人当前编码器值列表
 * @ctx 登陆上下文
 * @encode_array 存储获取的机器人当前编码器值列表
 * @err 错误信息
 * @return 成功或者失败
*/
ELT_SDK_PUBLIC(int) elt_get_current_encode(ELT_CTX ctx, elt_robot_encode encode_array, elt_error *err);


/**
 * 获取机器人当前工具号
 * @ctx 登陆上下文
 * @tool_num 存储获取的机器人当前工具号，范围:[0~7]
 * @err 错误信息
 * @return 成功或者失败
*/
ELT_SDK_PUBLIC(int) elt_get_tool_number(ELT_CTX ctx, int *tool_num, elt_error *err);


/**
 * 获取机器人当前用户工具号
 * @ctx 登陆上下文
 * @user_num 存储获取的机器人当前用户工具号，范围:[0~7]
 * @err 错误信息
 * @return 成功或者失败
*/
ELT_SDK_PUBLIC(int) elt_get_user_number(ELT_CTX ctx, int *user_num, elt_error *err);


/**
 * 关节运动
 * @ctx 登陆上下文
 * @target_pos_array 目标关节点
 * @speed 运行速度百分比，范围:[1~100]，单位：百分比
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_joint_move(ELT_CTX ctx, elt_robot_pos target_pos_array, double speed, elt_error *err);


/**
 * 直线运动
 * @ctx 登陆上下文
 * @target_pos_array 目标关节点
 * @speed 运行速度，范围[1~直线最大速度参数值]，单位：毫米/秒
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_line_move(ELT_CTX ctx, elt_robot_pos target_pos_array, double speed, elt_error *err);



/**
 * 圆弧运动
 * @ctx 登陆上下文
 * @middle_target_pos_array 中间关节点
 * @target_pos_array 结束关节点
 * @speed 运行速度，范围[1~直线最大速度参数值]，单位：毫米/秒
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_arc_move(ELT_CTX ctx, elt_robot_pos middle_target_pos_array, elt_robot_pos target_pos_array, double speed, elt_error *err);


/**
 * 旋转运动
 * @ctx 登陆上下文
 * @target_pos_array 目标关节点
 * @speed 运行速度，范围[1~旋转角最大速度参数值], 单位: 度/秒
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_rotate_move(ELT_CTX ctx, elt_robot_pos target_pos_array, double speed, elt_error *err);


/**
 * 设置路点运动时最大关节速度
 * @ctx 登陆上下文
 * @speed 关节速度，范围:[1~100], 单位：百分比
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_set_waypoint_max_joint_speed(ELT_CTX ctx, double speed, elt_error *err);


/**
 * 设置路点运动时最大直线速度
 * @ctx 登陆上下文
 * @speed 直线速度，范围:[1~直线最大速度参数值], 单位：毫米/秒
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_set_waypoint_max_line_speed(ELT_CTX ctx, double speed, elt_error *err);


/**
 * 设置路点运动时最大旋转速度
 * @ctx 登陆上下文
 * @speed 旋转速度，范围:[1~旋转角最大速度参数值], 单位：度/秒
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_set_waypoint_max_rotate_speed(ELT_CTX ctx, double speed, elt_error *err);


/**
 * 添加路点信息
 * @ctx 登陆上下文
 * @waypoint_array 目标位置（关节角）
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_add_waypoint(ELT_CTX ctx, elt_robot_pos waypoint_array, elt_error *err);


/**
 * 清除路点信息
 * @ctx 登陆上下文
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_clear_waypoint(ELT_CTX ctx, elt_error *err);


/**
 * 轨迹运动
 * @ctx 登陆上下文
 * @move_type 运动类型,<EltMoveType>
 * @pl 平滑度等级，范围[0~7]
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_track_move(ELT_CTX ctx,int move_type, int pl, elt_error *err);


/**
 * 停止机器人运行
 * @ctx 登陆上下文
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_stop(ELT_CTX ctx, elt_error *err);


/**
 * 逆解函数，根据位姿信息得到对应的机械臂关节角信息
 * @ctx 登陆上下文
 * @target_pose_array 目标位姿信息
 * @response_pos_array 存储获取的响应关节角信息
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_inverse_kinematic(ELT_CTX ctx, elt_robot_pose target_pose_array, elt_robot_pos response_pos_array, elt_error *err);


/**
 * 正解函数，根据机械臂关节角信息得到对应的位姿信息
 * @ctx 登陆上下文
 * @target_pos_array 目标关节角信息
 * @response_pose_array 存储获取的响应位姿信息
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_positive_kinematic(ELT_CTX ctx, elt_robot_pos target_pos_array, elt_robot_pose response_pose_array, elt_error *err);


/**
 * 基坐标到用户坐标位姿转化函数，当前用户坐标系下，根据基坐标的位姿信息得到对应用户坐标系下的位姿信息
 * @ctx 登陆上下文
 * @base_pose_array 基坐标系下的位姿信息
 * @user_no 用户坐标号，范围[0~7]
 * @user_pose_array 存储获取的用户标系下的位姿信息
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_base2user(ELT_CTX ctx, elt_robot_pose base_pose_array, int user_no, elt_robot_pose user_pose_array, elt_error *err);


/**
 * 用户坐标到基坐标位姿转化，当前用户坐标系下，根据用户坐标的位姿信息得到对应基坐标系下的位姿信息
 * @ctx 登陆上下文
 * @user_pose_array 用户标系下的位姿信息
 * @user_no 用户坐标号，范围[0~7]
 * @base_pose_array 存储获取的基坐标系下的位姿信息
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_user2base(ELT_CTX ctx, elt_robot_pose user_pose_array, int user_no, elt_robot_pose base_pose_array, elt_error *err);


/**
 * 获取输入IO状态
 * @ctx 登陆上下文
 * @addr 输入IO地址，范围[0~127]
 * @status 存储获取的输入IO状态 <EltIOStatus>
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_get_input(ELT_CTX ctx, int addr, int *status, elt_error *err);


/**
 * 获取输出IO状态
 * @ctx 登陆上下文
 * @addr 输出IO地址，范围[0~127]
 * @status 存储获取的输出IO状态 <EltIOStatus>
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_get_output(ELT_CTX ctx, int addr, int *status, elt_error *err);


/**
 * 设置输出IO状态
 * @ctx 登陆上下文
 * @addr 输入IO地址，范围[0~127]
 * @status IO状态 <EltIOStatus>
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_set_output(ELT_CTX ctx, int addr, int status, elt_error *err);


/**
 * 获取虚拟输入IO状态
 * @ctx 登陆上下文
 * @addr 虚拟IO地址，范围[0~399]
 * @status 存储获取的输入IO状态 <EltIOStatus>
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_get_virtual_input(ELT_CTX ctx, int addr, int *status, elt_error *err);


/**
 * 获取虚拟输出IO状态
 * @ctx 登陆上下文
 * @addr 虚拟IO地址，范围[400~1536]
 * @status 存储获取的输出IO状态 <EltIOStatus>
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_get_virtual_output(ELT_CTX ctx, int addr, int *status, elt_error *err);


/**
 * 设置虚拟输出IO状态
 * @ctx 登陆上下文
 * @addr 输出IO地址，范围[400~799]
 * @status IO状态 <EltIOStatus>
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_set_virtual_output(ELT_CTX ctx, int addr, int status, elt_error *err);


/**
 * 获取系统B变量值
 * @ctx 登陆上下文
 * @addr 变量地址，范围[0~255]
 * @value 存储获取的B变量值，范围[0~65535]
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_get_sysvar_b(ELT_CTX ctx, int addr, int *value, elt_error *err);


/**
 * 设置系统B变量值
 * @ctx 登陆上下文
 * @addr 变量地址，范围[0~255]
 * @value 变量值，范围[0~2^16-1]
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_set_sysvar_b(ELT_CTX ctx, int addr, int value, elt_error *err);


/**
 * 获取系统I变量值
 * @ctx 登陆上下文
 * @addr 变量地址，范围[0~255]
 * @value 存储获取的I变量值
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_get_sysvar_i(ELT_CTX ctx, int addr, int *value, elt_error *err);


/**
 * 设置系统I变量值
 * @ctx 登陆上下文
 * @addr 变量地址，范围[-2^31~2^31-1]
 * @value 变量值
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_set_sysvar_i(ELT_CTX ctx, int addr, int value, elt_error *err);


/**
 * 获取系统D变量值
 * @ctx 登陆上下文
 * @addr 变量地址，范围[0~255]
 * @value 存储获取的D变量值，范围[-3.4E+38~3.4E+38]
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_get_sysvar_d(ELT_CTX ctx, int addr, double *value, elt_error *err);


/**
 * 设置系统D变量值
 * @ctx 登陆上下文
 * @addr 变量地址，范围[0~255]
 * @value 变量值，范围[-3.4E+38~3.4E+38]
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_set_sysvar_d(ELT_CTX ctx, int addr, double value, elt_error *err);

/**
 * 获取机器人当前力矩信息
 * @ctx 登陆上下文
 * @torques 存储获取的机器人当前力矩信息（8个轴的力矩），单位%(额定扭矩百分比)
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_get_robot_torques(ELT_CTX ctx, elt_robot_torques torques, elt_error *err);


/**
 * 获取模拟量输入
 * @ctx 登陆上下文
 * @addr 模拟量地址，范围[0~1]
 * @value 存储获取的模拟量输入，范围[-10~10]
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_get_analog_input(ELT_CTX ctx, int addr, double *value, elt_error *err);


/**
 * 设置模拟量输出
 * @ctx 登陆上下文
 * @addr 模拟量地址，范围[0~3]
 * @value 模拟量值，范围[-10~10]
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_set_analog_output(ELT_CTX ctx, int addr, double value, elt_error *err);

/**
 * 拖动使能开关
 * @ctx 登陆上下文
 * @onoff 开关 [0~1]
 * @err 错误信息
 * @return 成功或者失败
 */
ELT_SDK_PUBLIC(int) elt_drag_teach(ELT_CTX ctx, int onoff, elt_error *err);

#ifdef __cplusplus
}
#endif

#endif // ELT_ROBOT_H
