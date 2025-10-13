# BVH文件身高和身材比例调整指南

## 概述

本文档详细介绍了当BVH文件来自不同身高和身材比例的人时，如何调整GMR重定向参数来获得最佳的重定向效果。不同的BVH文件可能来自儿童、成人、高个子、矮个子等不同体型的人，需要相应的参数调整。

## 影响重定向效果的关键因素

### 1. 身高差异
- **儿童 vs 成人**: 身高差异可达2-3倍
- **高个子 vs 矮个子**: 成人间身高差异可达30-50cm
- **不同种族**: 平均身高差异

### 2. 身材比例差异
- **四肢长度比例**: 不同人群的手臂/腿部长度比例不同
- **躯干长度**: 上半身与下半身的比例差异
- **肩宽和髋宽**: 影响整体姿态

### 3. 动作幅度差异
- **关节活动范围**: 不同人群的关节柔韧性不同
- **动作风格**: 文化背景影响动作表现

## 参数调整策略

### 1. 整体身高调整

#### 自动高度检测和调整

```python
# GMR会自动检测BVH中的人体高度
def calculate_human_height(bvh_file):
    """
    从BVH文件计算实际人体高度
    """
    # 加载BVH数据
    lafan1_data_frames, actual_human_height = load_lafan1_file(bvh_file)
    
    # 计算头部到脚部的高度
    sample_frame = lafan1_data_frames[0]
    head_height = sample_frame["Head"][0][2]
    foot_height = min(
        sample_frame["LeftFootMod"][0][2], 
        sample_frame["RightFootMod"][0][2]
    )
    calculated_height = head_height - foot_height
    
    return calculated_height, actual_human_height

# 使用示例
bvh_file = "path/to/child_motion.bvh"
calculated_height, detected_height = calculate_human_height(bvh_file)
print(f"Calculated height: {calculated_height:.2f}m")
print(f"Detected height: {detected_height:.2f}m")
```

#### 手动高度调整

如果自动检测不准确，可以手动调整：

```python
# 在重定向时指定实际身高
retargeter = GMR(
    src_human="bvh",
    tgt_robot="unitree_g1",
    actual_human_height=1.2,  # 手动指定身高（如儿童1.2米）
    verbose=True
)
```

### 2. 配置文件参数调整

#### 调整 human_height_assumption

```json
// 原配置 (成人)
{
    "human_height_assumption": 1.8,
    "human_scale_table": {
        "Hips": 0.9,
        "Spine2": 0.9,
        "LeftUpLeg": 0.9,
        "RightUpLeg": 0.9,
        "LeftLeg": 0.9,
        "RightLeg": 0.9,
        "LeftArm": 0.75,
        "RightArm": 0.75,
        "LeftForeArm": 0.75,
        "RightForeArm": 0.75,
        "LeftHand": 0.75,
        "RightHand": 0.75
    }
}

// 儿童配置
{
    "human_height_assumption": 1.2,  // 降低假设高度
    "human_scale_table": {
        "Hips": 0.6,        // 大幅缩小
        "Spine2": 0.6,
        "LeftUpLeg": 0.6,
        "RightUpLeg": 0.6,
        "LeftLeg": 0.6,
        "RightLeg": 0.6,
        "LeftArm": 0.5,     // 手臂更短
        "RightArm": 0.5,
        "LeftForeArm": 0.5,
        "RightForeArm": 0.5,
        "LeftHand": 0.5,
        "RightHand": 0.5
    }
}

// 高个子配置
{
    "human_height_assumption": 2.0,  // 提高假设高度
    "human_scale_table": {
        "Hips": 1.0,        // 接近原始尺寸
        "Spine2": 1.0,
        "LeftUpLeg": 1.0,
        "RightUpLeg": 1.0,
        "LeftLeg": 1.0,
        "RightLeg": 1.0,
        "LeftArm": 0.8,     // 手臂稍长
        "RightArm": 0.8,
        "LeftForeArm": 0.8,
        "RightForeArm": 0.8,
        "LeftHand": 0.8,
        "RightHand": 0.8
    }
}
```

### 3. 不同体型的具体调整方案

#### 儿童体型 (身高1.0-1.4米)

```json
{
    "robot_root_name": "pelvis",
    "human_root_name": "Hips",
    "ground_height": 0.0,
    "human_height_assumption": 1.2,
    "use_ik_match_table1": true,
    "use_ik_match_table2": true,
    
    "human_scale_table": {
        "Hips": 0.6,        // 躯干较短
        "Spine2": 0.6,
        "LeftUpLeg": 0.65,  // 腿部相对较长
        "RightUpLeg": 0.65,
        "LeftLeg": 0.65,
        "RightLeg": 0.65,
        "LeftFootMod": 0.6,
        "RightFootMod": 0.6,
        "LeftArm": 0.5,     // 手臂较短
        "RightArm": 0.5,
        "LeftForeArm": 0.5,
        "RightForeArm": 0.5,
        "LeftHand": 0.5,
        "RightHand": 0.5
    }
}
```

#### 矮个子成人 (身高1.5-1.6米)

```json
{
    "human_height_assumption": 1.55,
    "human_scale_table": {
        "Hips": 0.8,
        "Spine2": 0.8,
        "LeftUpLeg": 0.8,
        "RightUpLeg": 0.8,
        "LeftLeg": 0.8,
        "RightLeg": 0.8,
        "LeftFootMod": 0.8,
        "RightFootMod": 0.8,
        "LeftArm": 0.7,
        "RightArm": 0.7,
        "LeftForeArm": 0.7,
        "RightForeArm": 0.7,
        "LeftHand": 0.7,
        "RightHand": 0.7
    }
}
```

#### 高个子成人 (身高1.8-2.0米)

```json
{
    "human_height_assumption": 1.9,
    "human_scale_table": {
        "Hips": 1.0,
        "Spine2": 1.0,
        "LeftUpLeg": 1.0,
        "RightUpLeg": 1.0,
        "LeftLeg": 1.0,
        "RightLeg": 1.0,
        "LeftFootMod": 1.0,
        "RightFootMod": 1.0,
        "LeftArm": 0.8,
        "RightArm": 0.8,
        "LeftForeArm": 0.8,
        "RightForeArm": 0.8,
        "LeftHand": 0.8,
        "RightHand": 0.8
    }
}
```

#### 长臂型身材

```json
{
    "human_height_assumption": 1.8,
    "human_scale_table": {
        "Hips": 0.9,
        "Spine2": 0.9,
        "LeftUpLeg": 0.9,
        "RightUpLeg": 0.9,
        "LeftLeg": 0.9,
        "RightLeg": 0.9,
        "LeftFootMod": 0.9,
        "RightFootMod": 0.9,
        "LeftArm": 0.9,     // 手臂更长
        "RightArm": 0.9,
        "LeftForeArm": 0.9,
        "RightForeArm": 0.9,
        "LeftHand": 0.9,
        "RightHand": 0.9
    }
}
```

#### 长腿型身材

```json
{
    "human_height_assumption": 1.8,
    "human_scale_table": {
        "Hips": 0.9,
        "Spine2": 0.9,
        "LeftUpLeg": 1.0,   // 腿部更长
        "RightUpLeg": 1.0,
        "LeftLeg": 1.0,
        "RightLeg": 1.0,
        "LeftFootMod": 1.0,
        "RightFootMod": 1.0,
        "LeftArm": 0.75,    // 手臂相对较短
        "RightArm": 0.75,
        "LeftForeArm": 0.75,
        "RightForeArm": 0.75,
        "LeftHand": 0.75,
        "RightHand": 0.75
    }
}
```

## 实际操作步骤

### 1. 分析BVH文件特征

```python
#!/usr/bin/env python3
"""
BVH文件特征分析工具
文件路径: examples/analyze_bvh_characteristics.py
"""

import numpy as np
from general_motion_retargeting.utils.lafan1 import load_lafan1_file

def analyze_bvh_characteristics(bvh_file):
    """
    分析BVH文件的特征，包括身高、身材比例等
    """
    print(f"分析BVH文件: {bvh_file}")
    
    # 加载BVH数据
    lafan1_data_frames, detected_height = load_lafan1_file(bvh_file)
    sample_frame = lafan1_data_frames[0]
    
    # 计算关键尺寸
    measurements = {}
    
    # 1. 整体高度
    head_pos = sample_frame["Head"][0]
    left_foot_pos = sample_frame["LeftFootMod"][0]
    right_foot_pos = sample_frame["RightFootMod"][0]
    foot_height = min(left_foot_pos[2], right_foot_pos[2])
    total_height = head_pos[2] - foot_height
    measurements["total_height"] = total_height
    
    # 2. 躯干长度 (Hips到Spine2)
    hips_pos = sample_frame["Hips"][0]
    spine2_pos = sample_frame["Spine2"][0]
    torso_length = np.linalg.norm(spine2_pos - hips_pos)
    measurements["torso_length"] = torso_length
    
    # 3. 腿部长度 (Hips到脚部)
    left_leg_length = np.linalg.norm(left_foot_pos - hips_pos)
    right_leg_length = np.linalg.norm(right_foot_pos - hips_pos)
    leg_length = (left_leg_length + right_leg_length) / 2
    measurements["leg_length"] = leg_length
    
    # 4. 手臂长度 (肩膀到手腕)
    left_shoulder_pos = sample_frame["LeftArm"][0]
    left_hand_pos = sample_frame["LeftHand"][0]
    left_arm_length = np.linalg.norm(left_hand_pos - left_shoulder_pos)
    
    right_shoulder_pos = sample_frame["RightArm"][0]
    right_hand_pos = sample_frame["RightHand"][0]
    right_arm_length = np.linalg.norm(right_hand_pos - right_shoulder_pos)
    arm_length = (left_arm_length + right_arm_length) / 2
    measurements["arm_length"] = arm_length
    
    # 5. 计算比例
    measurements["leg_to_torso_ratio"] = leg_length / torso_length
    measurements["arm_to_torso_ratio"] = arm_length / torso_length
    
    # 输出分析结果
    print(f"\n=== BVH文件特征分析 ===")
    print(f"检测到的身高: {detected_height:.2f}m")
    print(f"计算的总高度: {total_height:.2f}m")
    print(f"躯干长度: {torso_length:.2f}m")
    print(f"腿部长度: {leg_length:.2f}m")
    print(f"手臂长度: {arm_length:.2f}m")
    print(f"腿部/躯干比例: {measurements['leg_to_torso_ratio']:.2f}")
    print(f"手臂/躯干比例: {measurements['arm_to_torso_ratio']:.2f}")
    
    # 判断体型类型
    if total_height < 1.4:
        print("体型判断: 儿童")
    elif total_height < 1.6:
        print("体型判断: 矮个子成人")
    elif total_height > 1.9:
        print("体型判断: 高个子成人")
    else:
        print("体型判断: 标准成人")
    
    if measurements["leg_to_torso_ratio"] > 1.5:
        print("身材特点: 长腿型")
    elif measurements["arm_to_torso_ratio"] > 0.8:
        print("身材特点: 长臂型")
    else:
        print("身材特点: 标准比例")
    
    return measurements

if __name__ == "__main__":
    # 分析示例BVH文件
    bvh_file = "path/to/your/bvh_file.bvh"
    measurements = analyze_bvh_characteristics(bvh_file)
```

### 2. 自动配置生成

```python
#!/usr/bin/env python3
"""
自动生成适合的配置文件
文件路径: examples/generate_config_for_height.py
"""

import json
import os
from general_motion_retargeting.utils.lafan1 import load_lafan1_file

def generate_config_for_height(bvh_file, robot_type="unitree_g1"):
    """
    根据BVH文件自动生成适合的配置文件
    """
    # 分析BVH特征
    measurements = analyze_bvh_characteristics(bvh_file)
    
    # 基于分析结果生成配置
    config = generate_height_based_config(measurements)
    
    # 保存配置文件
    config_filename = f"bvh_to_{robot_type}_custom.json"
    config_path = f"custom_configs/{config_filename}"
    
    os.makedirs(os.path.dirname(config_path), exist_ok=True)
    with open(config_path, 'w') as f:
        json.dump(config, f, indent=4)
    
    print(f"生成的配置文件保存到: {config_path}")
    return config_path

def generate_height_based_config(measurements):
    """
    基于测量结果生成配置
    """
    total_height = measurements["total_height"]
    leg_to_torso_ratio = measurements["leg_to_torso_ratio"]
    arm_to_torso_ratio = measurements["arm_to_torso_ratio"]
    
    # 基础配置
    config = {
        "robot_root_name": "pelvis",
        "human_root_name": "Hips",
        "ground_height": 0.0,
        "human_height_assumption": total_height,
        "use_ik_match_table1": True,
        "use_ik_match_table2": True,
        "human_scale_table": {}
    }
    
    # 根据身高调整整体缩放
    if total_height < 1.4:  # 儿童
        base_scale = 0.6
        arm_scale = 0.5
    elif total_height < 1.6:  # 矮个子
        base_scale = 0.8
        arm_scale = 0.7
    elif total_height > 1.9:  # 高个子
        base_scale = 1.0
        arm_scale = 0.8
    else:  # 标准成人
        base_scale = 0.9
        arm_scale = 0.75
    
    # 根据比例调整
    leg_scale = base_scale * (1.0 + (leg_to_torso_ratio - 1.2) * 0.1)
    arm_scale = arm_scale * (1.0 + (arm_to_torso_ratio - 0.7) * 0.2)
    
    # 生成缩放表
    config["human_scale_table"] = {
        "Hips": base_scale,
        "Spine2": base_scale,
        "LeftUpLeg": leg_scale,
        "RightUpLeg": leg_scale,
        "LeftLeg": leg_scale,
        "RightLeg": leg_scale,
        "LeftFootMod": leg_scale,
        "RightFootMod": leg_scale,
        "LeftArm": arm_scale,
        "RightArm": arm_scale,
        "LeftForeArm": arm_scale,
        "RightForeArm": arm_scale,
        "LeftHand": arm_scale,
        "RightHand": arm_scale
    }
    
    # 添加IK匹配表（使用默认值）
    config["ik_match_table1"] = get_default_ik_match_table()
    config["ik_match_table2"] = get_default_ik_match_table()
    
    return config

def get_default_ik_match_table():
    """
    获取默认的IK匹配表
    """
    return {
        "pelvis": ["Hips", 0, 10, [0.0, 0.0, 0.0], [0.5, 0.5, 0.5, 0.5]],
        "left_hip_yaw_link": ["LeftUpLeg", 0, 10, [0.0, 0.0, 0.0], [-0.5, 0.5, 0.5, -0.5]],
        "left_knee_link": ["LeftLeg", 0, 10, [0.0, 0.0, 0.0], [-0.5, 0.5, 0.5, -0.5]],
        "left_ankle_roll_link": ["LeftFootMod", 50, 10, [0.0, 0.0, 0.0], [-0.70710678, 0.70710678, 0.0, 0.0]],
        "right_hip_yaw_link": ["RightUpLeg", 0, 10, [0.0, 0.0, 0.0], [-0.5, 0.5, 0.5, -0.5]],
        "right_knee_link": ["RightLeg", 0, 10, [0.0, 0.0, 0.0], [-0.5, 0.5, 0.5, -0.5]],
        "right_ankle_roll_link": ["RightFootMod", 50, 10, [0.0, 0.0, 0.0], [-0.70710678, 0.70710678, 0.0, 0.0]],
        "torso_link": ["Spine2", 0, 100, [0.0, 0.0, 0.0], [0.5, 0.5, 0.5, 0.5]],
        "left_shoulder_yaw_link": ["LeftArm", 0, 100, [0.0, 0.0, 0.0], [0.5, 0.5, -0.5, -0.5]],
        "left_elbow_link": ["LeftForeArm", 0, 10, [0.0, 0.0, 0.0], [0.70710678, 0.70710678, 0.0, 0.0]],
        "left_wrist_yaw_link": ["LeftHand", 0, 10, [0.0, 0.0, 0.0], [0.70710678, 0.70710678, 0.0, 0.0]],
        "right_shoulder_yaw_link": ["RightArm", 0, 100, [0.0, 0.0, 0.0], [0.5, 0.5, -0.5, -0.5]],
        "right_elbow_link": ["RightForeArm", 0, 10, [0.0, 0.0, 0.0], [0.70710678, 0.70710678, 0.0, 0.0]],
        "right_wrist_yaw_link": ["RightHand", 0, 10, [0.0, 0.0, 0.0], [0.70710678, 0.70710678, 0.0, 0.0]]
    }

if __name__ == "__main__":
    bvh_file = "path/to/your/bvh_file.bvh"
    config_path = generate_config_for_height(bvh_file)
```

### 3. 手动调整指南

#### 步骤1: 运行重定向并观察结果

```bash
# 使用默认配置运行重定向
python scripts/bvh_to_robot.py \
    --bvh_file path/to/your/bvh_file.bvh \
    --robot unitree_g1 \
    --save_path output/test_result.pkl \
    --rate_limit
```

#### 步骤2: 识别问题

观察重定向结果中的问题：

1. **整体太小/太大**: 调整 `human_height_assumption`
2. **手臂太长/太短**: 调整手臂相关的缩放值
3. **腿部太长/太短**: 调整腿部相关的缩放值
4. **躯干比例不当**: 调整躯干相关的缩放值

#### 步骤3: 修改配置文件

```bash
# 复制默认配置文件
cp general_motion_retargeting/ik_configs/bvh_to_g1.json custom_configs/bvh_to_g1_custom.json

# 编辑配置文件
nano custom_configs/bvh_to_g1_custom.json
```

#### 步骤4: 使用自定义配置

```bash
# 使用自定义配置运行重定向
python scripts/bvh_to_robot.py \
    --bvh_file path/to/your/bvh_file.bvh \
    --robot unitree_g1 \
    --save_path output/test_result_custom.pkl \
    --rate_limit
```

## 常见问题诊断和解决方案

### 1. 问题: 重定向后机器人动作太小

**症状**: 机器人动作幅度明显小于原始动作

**解决方案**:
```json
{
    "human_height_assumption": 1.6,  // 降低假设高度
    "human_scale_table": {
        "Hips": 1.0,        // 增加缩放比例
        "Spine2": 1.0,
        "LeftUpLeg": 1.0,
        "RightUpLeg": 1.0,
        // ... 其他部位也相应增加
    }
}
```

### 2. 问题: 重定向后机器人动作太大

**症状**: 机器人动作幅度明显大于原始动作

**解决方案**:
```json
{
    "human_height_assumption": 2.0,  // 提高假设高度
    "human_scale_table": {
        "Hips": 0.7,        // 减少缩放比例
        "Spine2": 0.7,
        "LeftUpLeg": 0.7,
        "RightUpLeg": 0.7,
        // ... 其他部位也相应减少
    }
}
```

### 3. 问题: 手臂动作不自然

**症状**: 手臂位置过高或过低，动作不协调

**解决方案**:
```json
{
    "human_scale_table": {
        "LeftArm": 0.6,     // 调整手臂长度
        "RightArm": 0.6,
        "LeftForeArm": 0.6,
        "RightForeArm": 0.6,
    },
    "ik_match_table1": {
        "left_shoulder_yaw_link": ["LeftArm", 0, 50, [0.0, 0.0, 0.0], [0.5, 0.5, -0.5, -0.5]],  // 降低权重
        "right_shoulder_yaw_link": ["RightArm", 0, 50, [0.0, 0.0, 0.0], [0.5, 0.5, -0.5, -0.5]]
    }
}
```

### 4. 问题: 腿部动作不自然

**症状**: 机器人腿部弯曲过度或不足

**解决方案**:
```json
{
    "human_scale_table": {
        "LeftUpLeg": 0.8,   // 调整大腿长度
        "RightUpLeg": 0.8,
        "LeftLeg": 0.8,     // 调整小腿长度
        "RightLeg": 0.8,
    },
    "ik_match_table1": {
        "left_knee_link": ["LeftLeg", 0, 5, [0.0, 0.0, 0.0], [-0.5, 0.5, 0.5, -0.5]],  // 降低权重
        "right_knee_link": ["RightLeg", 0, 5, [0.0, 0.0, 0.0], [-0.5, 0.5, 0.5, -0.5]]
    }
}
```

### 5. 问题: 重定向后机器人姿态不稳定

**症状**: 机器人晃动或姿态异常

**解决方案**:
```json
{
    "ik_match_table1": {
        "pelvis": ["Hips", 0, 100, [0.0, 0.0, 0.0], [0.5, 0.5, 0.5, 0.5]],  // 增加根部权重
        "torso_link": ["Spine2", 0, 100, [0.0, 0.0, 0.0], [0.5, 0.5, 0.5, 0.5]]  // 增加躯干权重
    },
    "ik_match_table2": {
        "pelvis": ["Hips", 100, 10, [0.0, 0.0, 0.0], [0.5, 0.5, 0.5, 0.5]]  // 二次优化中增加位置权重
    }
}
```

## 批量处理不同体型的BVH文件

### 1. 自动分类和配置

```python
#!/usr/bin/env python3
"""
批量处理不同体型的BVH文件
文件路径: examples/batch_process_different_heights.py
"""

import os
import glob
from pathlib import Path

def batch_process_by_height(bvh_folder, output_folder, robot_type="unitree_g1"):
    """
    根据身高自动分类处理BVH文件
    """
    # 创建输出文件夹
    os.makedirs(output_folder, exist_ok=True)
    
    # 按身高分类
    height_categories = {
        "child": [],      # < 1.4m
        "short": [],      # 1.4-1.6m
        "normal": [],     # 1.6-1.8m
        "tall": []        # > 1.8m
    }
    
    # 分析所有BVH文件
    bvh_files = glob.glob(os.path.join(bvh_folder, "**/*.bvh"), recursive=True)
    
    for bvh_file in bvh_files:
        measurements = analyze_bvh_characteristics(bvh_file)
        height = measurements["total_height"]
        
        if height < 1.4:
            height_categories["child"].append(bvh_file)
        elif height < 1.6:
            height_categories["short"].append(bvh_file)
        elif height < 1.8:
            height_categories["normal"].append(bvh_file)
        else:
            height_categories["tall"].append(bvh_file)
    
    # 为每个类别生成配置并处理
    for category, files in height_categories.items():
        if not files:
            continue
            
        print(f"处理{category}类别: {len(files)}个文件")
        
        # 生成该类别的配置
        config = generate_config_for_category(category)
        config_path = f"custom_configs/bvh_to_{robot_type}_{category}.json"
        
        os.makedirs(os.path.dirname(config_path), exist_ok=True)
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=4)
        
        # 批量处理
        for bvh_file in files:
            output_file = os.path.join(
                output_folder, 
                category,
                os.path.basename(bvh_file).replace('.bvh', '.pkl')
            )
            
            os.makedirs(os.path.dirname(output_file), exist_ok=True)
            
            # 使用对应配置进行重定向
            process_single_file(bvh_file, output_file, config_path)

def generate_config_for_category(category):
    """
    为不同身高类别生成配置
    """
    configs = {
        "child": {
            "human_height_assumption": 1.2,
            "human_scale_table": {
                "Hips": 0.6, "Spine2": 0.6,
                "LeftUpLeg": 0.65, "RightUpLeg": 0.65,
                "LeftLeg": 0.65, "RightLeg": 0.65,
                "LeftArm": 0.5, "RightArm": 0.5,
                "LeftForeArm": 0.5, "RightForeArm": 0.5,
                "LeftHand": 0.5, "RightHand": 0.5
            }
        },
        "short": {
            "human_height_assumption": 1.55,
            "human_scale_table": {
                "Hips": 0.8, "Spine2": 0.8,
                "LeftUpLeg": 0.8, "RightUpLeg": 0.8,
                "LeftLeg": 0.8, "RightLeg": 0.8,
                "LeftArm": 0.7, "RightArm": 0.7,
                "LeftForeArm": 0.7, "RightForeArm": 0.7,
                "LeftHand": 0.7, "RightHand": 0.7
            }
        },
        "normal": {
            "human_height_assumption": 1.8,
            "human_scale_table": {
                "Hips": 0.9, "Spine2": 0.9,
                "LeftUpLeg": 0.9, "RightUpLeg": 0.9,
                "LeftLeg": 0.9, "RightLeg": 0.9,
                "LeftArm": 0.75, "RightArm": 0.75,
                "LeftForeArm": 0.75, "RightForeArm": 0.75,
                "LeftHand": 0.75, "RightHand": 0.75
            }
        },
        "tall": {
            "human_height_assumption": 1.9,
            "human_scale_table": {
                "Hips": 1.0, "Spine2": 1.0,
                "LeftUpLeg": 1.0, "RightUpLeg": 1.0,
                "LeftLeg": 1.0, "RightLeg": 1.0,
                "LeftArm": 0.8, "RightArm": 0.8,
                "LeftForeArm": 0.8, "RightForeArm": 0.8,
                "LeftHand": 0.8, "RightHand": 0.8
            }
        }
    }
    
    # 添加基础配置
    base_config = {
        "robot_root_name": "pelvis",
        "human_root_name": "Hips",
        "ground_height": 0.0,
        "use_ik_match_table1": True,
        "use_ik_match_table2": True
    }
    
    base_config.update(configs[category])
    return base_config

if __name__ == "__main__":
    batch_process_by_height(
        bvh_folder="data/mixed_height_bvh/",
        output_folder="output/processed_by_height/",
        robot_type="unitree_g1"
    )
```

## 总结

本文档详细介绍了如何调整GMR参数来适配不同身高和身材比例的BVH文件：

1. **身高调整**: 通过修改 `human_height_assumption` 和缩放比例
2. **身材比例调整**: 针对不同部位设置不同的缩放值
3. **自动化工具**: 提供分析工具和配置生成脚本
4. **问题诊断**: 常见问题的识别和解决方案
5. **批量处理**: 自动分类和处理不同体型的BVH文件

通过这些方法，你可以有效地处理来自不同身高和身材比例的人的BVH文件，获得最佳的重定向效果。
