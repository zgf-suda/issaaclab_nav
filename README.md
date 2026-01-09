# issaaclab_nav
基于isaaclab训练go2的局部避障导航

### 只传了核心训练逻辑，添加自定义奖励在isaaclab_tasks/isaaclab_tasks/manager_based/navigation/mdp/rewards.py
### policy2dog 为 sim2real的代码，测试通过

# 训练设置：
    思路：强化学习的训练逻辑：目标点内20m、10m、2m距离奖励，目标点内0.2米很大奖励
    1.5米外启用航向对齐奖励、1.5米内启用目标对齐奖励

    奖励包括：距离目标点、航向对齐

    观测： 雷达平面扫描的障碍物costmap、base_link下的三轴速度（vx/vy/vz）、目标位置（x，y，z）、目标点基于base_link的偏航角

    episode结束：意外停止（如摔倒等）、碰撞障碍物、到达时间步

    效果：初期走向目标点，靠近时开始航向对齐，符合人类导航。
    
    class RewardsCfg:
        """Reward terms for the MDP."""
        termination_penalty = RewTerm(func=mdp.is_terminated, weight=-100.0) 
        distance = RewTerm(
            func=mdp.distance_reward,
            weight=-100.0
        )
        position_tracking = RewTerm(
            func=mdp.position_command_error_tanh,
            weight=60,
            params={"std": 2.0, "tr":10,"command_name": "pose_command"},
        )
        position_tracking_fine_grained = RewTerm(
            func=mdp.position_command_error_tanh,
            weight=60,
            params={"std": 0.5, "tr":5,"command_name": "pose_command"},
        )
        orientation_tracking = RewTerm(
            func=mdp.heading_command_error_abs,
            weight=30,
            params={"command_name": "pose_command"},
        )
