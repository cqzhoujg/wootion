Header      header
string      sender
string      receiver
string      robot_ip
uint32      trans_id
uint8       status           #机器人状态：bit0：离线/正常， bit1：急停开关按下 bit2：电池故障 bit3：电机故障 bit4：充电中
int32       mode             #控制模式：0:任务模式 1:手柄模式 2:侵限点定位模式
float64     velocity_x       #当前速度 m/s
int32       direction        #运动方向  1：FORWARD, 2:BACKWARD, 3:FORWARD_RETURN, 4:BACKWARD_RETURN
float64     position_x       #当前位置 里程值m
int32       light_status     #指示灯状态：0：指示灯红 1：指示灯绿 2：指示灯蓝
uint16      battery_voltage  #电池电压 V
uint8       battery_quantity #电池电量 百分比
uint8       battery_status   #电池状态：bit0：欠压 bit1：过压 bit2：过流 bit3：低温 bit4：高温 bit5：充电中 bit6：放电中 bit7：充电fet损坏
string      motor_status     #电机状态
int32[]      temperature     #温度 0：电池温度 1：电机伺服器1温度 2：电机伺服器2温度
