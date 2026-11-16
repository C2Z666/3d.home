D1导航 SDK桥接 d1-navigation-v6.5

楼梯控制与平地控制使用不同的话题。普通 Nav2 `/cmd_vel` 在实际 Stair 模式下一律忽略；楼梯段只接受 `/stair_cmd_vel` 的前向 `linear.x`，并要求 `/stair_motion_permit` 在桥接看门狗时限内持续为 true。许可撤销、心跳超时、SDK 状态过期、控制权丢失或急停都会拦截移动并清零速度。

`stair_max_lin_vel` 默认是 `0.0`，表示楼梯控制保持关闭。离线测试会单独设置测试值；该值不能直接当作真机安全速度。ROS 任务适配器尚未接入，因此当前版本只能验证门控，不能执行自动楼梯任务。

默认只前进及左右旋转，横移／头尾切换禁止。可选末端后退沿用V6规则：allow_terminal_reverse=true且0.2秒内后退授权有效，负向速度限制0.06 m/s；授权丢失或速度看门狗超时归零。键盘不提供后退授权，s/j/k/空格仍停车。

V6.5距离渐进减速与角度限制由导航层和DWB插件参数同步实现。桥接继续执行速度／加速度上限、SDK状态与控制权检查、零速度退出、不自动开启软急停。

重新编译本版本，不能复用V6已编译二进制。check.sh、check_state_rate.sh、check_terminal_reverse.sh使用假SDK，不连接真实机器人；本次Windows交付未执行ROS/C++集成检查。
