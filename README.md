1. 该文件下/assets打包了机器人、工厂物品、工厂场景资源，**未上传到远程仓库**；
2. 通过“python scripts/tutorials/magicbot/isaaclab_ctrl.py --enable_camera”指令启动场景（python的路径自行指定）；
3. 通过运行scripts/tutorials/magicbot/pub_action.py播放自己的动作序列数据（代码示例中将动作序列数据保存为.npy文件，如果文件格式不同可自行修改代码）
4. 启动后通过选中/World/ActionGraph/ActionGraph/isaac_create_render_product 指定相机prim path(Property->Isaac Create Render Product Node->Inputs->cameraPrim->Add Target)完成ROS图像消息的发布，相机路径/World/envs/env_0/Robot/p5_humanoid/link_hp/Camera_head，发布的图像消息话题名/camera/iamge_head
5. 机器人关节数据实时发布话题名/true_act_qpos

