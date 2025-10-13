# BVH转PKL详细指令指南

## 概述

本文档详细介绍了如何使用GMR (General Motion Retargeting) 将BVH格式的人体动作数据转换为PKL格式的机器人动作数据。

## 支持的机器人类型

GMR支持以下机器人类型的BVH重定向：

| 机器人ID | 机器人名称 | 自由度 | 状态 |
|---------|-----------|--------|------|
| `unitree_g1` | Unitree G1 | 29 | ✅ 完全支持 |
| `unitree_g1_with_hands` | Unitree G1 (带手部) | 43 | ✅ 完全支持 |
| `booster_t1` | Booster T1 | 23 | ✅ 完全支持 |
| `booster_t1_29dof` | Booster T1 (29自由度) | 29 | ✅ 完全支持 |
| `booster_k1` | Booster K1 | 22 | ✅ 完全支持 |
| `stanford_toddy` | Stanford ToddlerBot | - | ✅ 完全支持 |
| `fourier_n1` | Fourier N1 | - | ✅ 完全支持 |
| `engineai_pm01` | ENGINEAI PM01 | - | ✅ 完全支持 |
| `hightorque_hi` | HighTorque Hi | 25 | ✅ 完全支持 |
| `galaxea_r1pro` | Galaxea R1 Pro | 24 | ✅ 完全支持 |
| `kuavo_s45` | KUAVO S45 | 28 | ✅ 完全支持 |
| `berkeley_humanoid_lite` | Berkeley Humanoid Lite | 22 | ✅ 完全支持 |
| `pnd_adam_lite` | PND Adam Lite | 25 | ✅ 完全支持 |
| `tienkung` | Tienkung | 20 | ✅ 完全支持 |

## 环境准备

### 1. 安装GMR

```bash
# 创建conda环境
conda create -n gmr python=3.10 -y
conda activate gmr

# 安装GMR
pip install -e .

# 解决可能的渲染问题
conda install -c conda-forge libstdcxx-ng -y
```

### 2. 准备BVH数据

下载LAFAN1数据集：
```bash
# 从官方仓库下载LAFAN1数据
wget https://github.com/ubisoft/ubisoft-laforge-animation-dataset/blob/master/lafan1/lafan1.zip
unzip lafan1.zip
```

## 单文件转换指令

### 基本命令

```bash
python scripts/bvh_to_robot.py \
    --bvh_file <path_to_bvh_file> \
    --robot <robot_name> \
    --save_path <output_pkl_path> \
    --rate_limit
```

### 参数说明

- `--bvh_file`: BVH文件的完整路径
- `--robot`: 目标机器人名称（见上表）
- `--save_path`: 输出的PKL文件路径
- `--rate_limit`: 限制重定向速度以保持与人体动作相同的速率
- `--record_video`: 录制视频（可选）
- `--video_path`: 视频保存路径（可选）

### 实际示例

#### 示例1: 转换到Unitree G1
```bash
python scripts/bvh_to_robot.py \
    --bvh_file /path/to/lafan1/dance1_subject2.bvh \
    --robot unitree_g1 \
    --save_path ./output/dance1_subject2_g1.pkl \
    --rate_limit
```

#### 示例2: 转换到Booster T1并录制视频
```bash
python scripts/bvh_to_robot.py \
    --bvh_file /path/to/lafan1/walk1_subject1.bvh \
    --robot booster_t1 \
    --save_path ./output/walk1_subject1_t1.pkl \
    --rate_limit \
    --record_video \
    --video_path ./videos/walk1_subject1_t1.mp4
```

#### 示例3: 转换到带手部的Unitree G1
```bash
python scripts/bvh_to_robot.py \
    --bvh_file /path/to/lafan1/grab1_subject3.bvh \
    --robot unitree_g1_with_hands \
    --save_path ./output/grab1_subject3_g1_hands.pkl \
    --rate_limit
```

## 批量转换指令

### 基本批量转换命令

```bash
python scripts/bvh_to_robot_dataset.py \
    --src_folder <bvh_folder_path> \
    --tgt_folder <output_folder_path> \
    --robot <robot_name>
```

### 参数说明

- `--src_folder`: 包含BVH文件的源文件夹路径
- `--tgt_folder`: 输出PKL文件的目标文件夹路径
- `--robot`: 目标机器人名称
- `--override`: 覆盖已存在的文件（可选）
- `--target_fps`: 目标帧率（默认30）

### 实际示例

#### 示例1: 批量转换LAFAN1数据集到Unitree G1
```bash
python scripts/bvh_to_robot_dataset.py \
    --src_folder /path/to/lafan1/ \
    --tgt_folder ./output/lafan1_g1/ \
    --robot unitree_g1
```

#### 示例2: 批量转换并覆盖已存在文件
```bash
python scripts/bvh_to_robot_dataset.py \
    --src_folder /path/to/lafan1/ \
    --tgt_folder ./output/lafan1_t1/ \
    --robot booster_t1 \
    --override
```

#### 示例3: 转换到多个不同机器人
```bash
# 转换到Unitree G1
python scripts/bvh_to_robot_dataset.py \
    --src_folder /path/to/lafan1/ \
    --tgt_folder ./output/lafan1_g1/ \
    --robot unitree_g1

# 转换到Booster T1
python scripts/bvh_to_robot_dataset.py \
    --src_folder /path/to/lafan1/ \
    --tgt_folder ./output/lafan1_t1/ \
    --robot booster_t1

# 转换到Stanford ToddlerBot
python scripts/bvh_to_robot_dataset.py \
    --src_folder /path/to/lafan1/ \
    --tgt_folder ./output/lafan1_toddy/ \
    --robot stanford_toddy
```

## 输出文件格式

转换后的PKL文件包含以下数据结构：

```python
{
    "fps": 30,                    # 帧率
    "root_pos": numpy.array,      # 根部位置 (N, 3)
    "root_rot": numpy.array,      # 根部旋转 (N, 4) - xyzw格式
    "dof_pos": numpy.array,       # 关节角度 (N, num_dofs)
    "local_body_pos": numpy.array, # 局部身体位置 (N, num_bodies, 3)
    "link_body_list": list,       # 身体链接名称列表
}
```

## 可视化重定向结果

### 查看已保存的机器人动作
```bash
python scripts/vis_robot_motion.py \
    --robot <robot_name> \
    --robot_motion_path <path_to_pkl_file>
```

### 录制可视化视频
```bash
python scripts/vis_robot_motion.py \
    --robot unitree_g1 \
    --robot_motion_path ./output/dance1_subject2_g1.pkl \
    --record_video \
    --video_path ./videos/dance1_subject2_g1.mp4
```

## 性能优化

### 1. 速度限制选项
- 使用 `--rate_limit` 保持与原始动作相同的播放速度
- 不使用 `--rate_limit` 可以获得最快的转换速度

### 2. 批量处理优化
- 批量转换时不会显示可视化窗口，处理速度更快
- 使用 `--override` 可以跳过已存在的文件

## 常见问题解决

### 1. 导入错误
```bash
# 确保在正确的conda环境中
conda activate gmr

# 重新安装GMR
pip install -e .
```

### 2. BVH文件格式问题
- 确保BVH文件来自LAFAN1数据集
- 检查BVH文件是否包含标准的骨骼结构

### 3. 机器人模型问题
- 确保选择的机器人名称正确
- 检查机器人模型文件是否存在

### 4. 内存不足
- 对于大型数据集，分批处理
- 使用更少的并行进程

## 高级用法

### 自定义配置文件

可以通过修改IK配置文件来调整重定向参数：

```bash
# 配置文件位置
general_motion_retargeting/ik_configs/bvh_to_<robot_name>.json
```

主要配置参数：
- `human_height_assumption`: 假设的人体高度
- `human_scale_table`: 人体各部位缩放比例
- `ik_match_table1/2`: IK匹配表，定义人体关节与机器人关节的对应关系

### 编程接口使用

```python
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.lafan1 import load_lafan1_file

# 加载BVH数据
lafan1_data_frames, actual_human_height = load_lafan1_file("path/to/file.bvh")

# 初始化重定向系统
retargeter = GMR(
    src_human="bvh",
    tgt_robot="unitree_g1",
    actual_human_height=actual_human_height,
)

# 逐帧重定向
for frame_data in lafan1_data_frames:
    qpos = retargeter.retarget(frame_data)
    # qpos包含: [root_pos(3), root_rot(4), dof_pos(num_dofs)]
```

## 总结

本指南涵盖了BVH到PKL转换的所有主要方面：

1. **环境准备**: 安装GMR和相关依赖
2. **单文件转换**: 使用 `bvh_to_robot.py` 脚本
3. **批量转换**: 使用 `bvh_to_robot_dataset.py` 脚本
4. **结果可视化**: 查看和录制转换结果
5. **性能优化**: 提高转换效率的方法
6. **问题解决**: 常见问题的解决方案

通过这些指令，你可以将任何BVH格式的人体动作数据转换为各种机器人可以执行的PKL格式动作数据。
