# gary_msgs

gary_msgs包是gary_ros2定义的一组msg, service和action. 所有节点间通信应当尽可能使用本包定义的内容或ros内置消息类型. 


## Topics

* msg/DR16Receiver.msg: DR16接收机消息类型
* msg/GameStatus.msg: 比赛状态数据, 3Hz周期发送
* msg/GameResult.msg: 比赛结果数据, 比赛结束后发送
* msg/RobotHP.msg: 比赛机器人血量数据, 3Hz周期发送
* msg/ICRABuffDebuffZoneAndLurkStatus.msg: 人工智能挑战赛加成与惩罚状态, 1Hz周期发送
* msg/FieldEvents.msg: 场地事件数据, 3Hz周期发送
* msg/SupplyProjectileAction.msg: 场地补给站动作标识数据, 动作改变后发送
* msg/SupplyProjectileRequest.msg: 请求补给站补弹数据, 由参赛队发送, 上限10Hz, 对抗赛尚未开放
* msg/RefereeWarning.msg: 裁判警告数据, 警告发生后发送
* msg/DartRemainingTime.msg: 飞镖发射口倒计时, 3Hz周期发送
* msg/RobotStatus.msg: 机器人状态数据, 10Hz周期发送
* msg/PowerHeat.msg: 实时功率热量数据, 50Hz周期发送
* msg/RobotPosition.msg: 机器人位置数据, 10Hz周期发送
* msg/RobotBuff.msg: 机器人增益数据, 1Hz周期发送
* msg/AerialRobotEnergy.msg: 空中机器人能量状态数据, 10Hz周期发送, 只有空中机器人主控发送
* msg/RobotHurt.msg: 伤害状态数据, 伤害发生后发送
* msg/ShootData.msg 实时射击数据, 子弹发射后发送
* msg/BulletRemaining.msg: 子弹剩余发送数, 空中机器人以及哨兵机器人发送, 10Hz周期发送
* msg/RFIDStatus.msg: 机器人RFID状态, 3Hz周期发送
* msg/DartClientCmd.msg: 飞镖机器人客户端指令, 10Hz周期发送
* msg/InteractiveDataRecv.msg: 机器人间交互数据, 上限10Hz
* msg/InteractiveDataSend.msg: 机器人间交互数据发送方触发发送, 上限10Hz
* msg/CustomController.msg: 自定义控制器交互数据接口, 通过客户端触发发送, 上限30Hz
* msg/ClientCommand.msg: 客户端小地图交互数据, 触发发送
* msg/ClientReceive.msg: 客户端小地图接收信息
* msg/ImageTransmitter.msg: 键盘, 鼠标信息, 通过图传串口发送
* msg/AutoAIM.msg: 视觉自瞄指令
* msg/PID.msg: PID控制器状态数据
* msg/PIDwithFilter: 带滤波PID控制器状态数据
* msg/DualLoopPID.msg: 双环PID控制器状态数据
* msg/DualLoopPIDwithFilter.msg: 带滤波双环PID控制器状态数据

## Services

* srv/VisionModeSwitch.srv: 视觉模式切换

## Action

TBD
