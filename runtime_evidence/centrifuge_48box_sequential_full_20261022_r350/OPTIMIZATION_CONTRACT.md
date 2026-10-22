# r350：有界传输队列与源进程快速失效停持

继承r349的规划资源复用/两个装盒静止期提前规划，以及r347原物理配置。
不加载r348的新DART响应库；不改变分开抓姿态、速度、抓持、0.5ms步长和30秒稳转。

r349在装盒前发生相邻关节源戳1ms跳步，strict relay拒绝并触发launch关闭；
此次未进入规划优化验证。原ROS桥接publisher/subscriber queue都是20000，
但Gazebo Transport底层默认HWM是1000。本候选只为新launch进程树设置有界
RCVHWM/SNDHWM=20000，不用无限队列，不补帧、不插值、不取消原步长/源龄门。
这仅是可能改善突发积压的假设，不冒称已证明历史丢帧唯一发生层。
[官方说明](https://gazebosim.org/api/transport/13/envvars.html)。

执行前绑定精确owned relay/server的PID、创建时间和命令；原guard每1wall秒读检查，
身份消失即调用原SIGINT stop-and-hold，避免等待冻结的sim时钟和后续跟踪错误。
不增加动作指令。正式模型、历史源/视频/报告/备份保持；全新domain158与独占run。
