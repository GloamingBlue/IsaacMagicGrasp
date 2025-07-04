import h5py
import numpy as np
import os
import cv2
import time

hdf5_path = "/mnt/new_space/data/task_74"
hdf5_file_name = f"episode_qpos_0.hdf5"
# with h5py.File(os.path.join(hdf5_path, hdf5_file_name), 'r') as root:
#     images = root[f'/observations/images/head']
#     # 逐张读取（避免一次性加载大文件）
#     for i in range(len(images)):
#         img = cv2.cvtColor(images[i], cv2.COLOR_RGB2BGR)  # 如果原始是RGB
#         # 处理图像（例如显示）
#         cv2.imshow('Image', img)
#         key = cv2.waitKey(0)  # 按任意键继续
#         if key == ord('q'):    # 按q退出
#             break

target_fps = 30  # 目标帧率
frame_delay = int(1000 / target_fps)  # 每帧延迟毫秒数

# 读取并播放图像
with h5py.File(os.path.join(hdf5_path, hdf5_file_name), 'r') as f:
    # 获取图像数据集
    images = f['/observations/images/head']
    print(f"总帧数: {len(images)}, 图像尺寸: {images.shape[1:]}")

    # 计算实际帧率
    start_time = time.time()
    
    for i in range(len(images)):
        # 读取当前帧
        frame = images[i]
        
        # 格式转换（根据实际存储格式调整）
        if frame.dtype == np.float32:
            frame = (frame * 255).astype(np.uint8)
        if len(frame.shape) == 3 and frame.shape[2] == 3:  # 如果是RGB
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # 显示帧
        cv2.imshow('30FPS Playback', frame)
        
        # 计算并保持帧率
        current_time = time.time()
        elapsed = current_time - start_time
        expected_time = i / target_fps
        
        # 动态调整延迟
        remaining_delay = max(1, int((expected_time - elapsed) * 1000))
        key = cv2.waitKey(remaining_delay) & 0xFF
        
        # 按q退出
        if key == ord('q'):
            break

cv2.destroyAllWindows()