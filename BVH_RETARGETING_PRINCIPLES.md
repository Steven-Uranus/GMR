# BVH重定向原理与示例代码详解

## 概述

本文档详细介绍了GMR (General Motion Retargeting) 中BVH重定向的技术原理、算法实现和示例代码。BVH重定向是将人体动作数据适配到不同机器人模型的核心技术。

## 技术架构

### 1. 整体流程

```
BVH文件 → 骨骼解析 → 人体动作数据 → 重定向算法 → 机器人动作数据 → PKL文件
```

### 2. 核心组件

- **BVH解析器**: 解析BVH文件格式
- **骨骼映射**: 人体骨骼到机器人关节的映射
- **逆运动学(IK)求解器**: 计算关节角度
- **运动重定向**: 适配不同机器人尺寸和结构

## BVH文件格式解析

### BVH文件结构

BVH (Biovision Hierarchy) 文件包含两部分：
1. **HIERARCHY**: 骨骼层次结构
2. **MOTION**: 动作数据

### 解析代码示例

```python
# 文件路径: general_motion_retargeting/utils/lafan1.py

import numpy as np
from scipy.spatial.transform import Rotation as R
import general_motion_retargeting.utils.lafan_vendor.utils as utils
from general_motion_retargeting.utils.lafan_vendor.extract import read_bvh

def load_lafan1_file(bvh_file):
    """
    加载LAFAN1格式的BVH文件
    
    Args:
        bvh_file: BVH文件路径
        
    Returns:
        frames: 每帧的骨骼数据
        human_height: 人体高度
    """
    # 1. 读取BVH文件
    data = read_bvh(bvh_file)
    
    # 2. 前向运动学计算全局位置和旋转
    global_data = utils.quat_fk(data.quats, data.pos, data.parents)
    
    # 3. 坐标系转换矩阵 (BVH到机器人坐标系)
    rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    rotation_quat = R.from_matrix(rotation_matrix).as_quat(scalar_first=True)
    
    # 4. 处理每一帧数据
    frames = []
    for frame in range(data.pos.shape[0]):
        result = {}
        for i, bone in enumerate(data.bones):
            # 应用坐标系旋转
            orientation = utils.quat_mul(rotation_quat, global_data[0][frame, i])
            # 转换单位: cm -> m
            position = global_data[1][frame, i] @ rotation_matrix.T / 100
            
            result[bone] = (position, orientation)
        
        # 5. 添加修改后的足部姿态
        result["LeftFootMod"] = (result["LeftFoot"][0], result["LeftToe"][1])
        result["RightFootMod"] = (result["RightFoot"][0], result["RightToe"][1])
        
        frames.append(result)
    
    # 6. 计算人体高度
    human_height = result["Head"][0][2] - min(
        result["LeftFootMod"][0][2], 
        result["RightFootMod"][0][2]
    )
    human_height = 1.75  # 固定高度假设
    
    return frames, human_height
```

## 骨骼映射原理

### 1. 人体骨骼结构

LAFAN1数据集使用标准人体骨骼结构：

```
Hips (根节点)
├── Spine
├── Spine1
├── Spine2 (胸部)
├── LeftUpLeg (左大腿)
│   └── LeftLeg (左小腿)
│       └── LeftFoot (左脚)
│           └── LeftToe (左脚趾)
├── RightUpLeg (右大腿)
│   └── RightLeg (右小腿)
│       └── RightFoot (右脚)
│           └── RightToe (右脚趾)
├── LeftArm (左上臂)
│   └── LeftForeArm (左前臂)
│       └── LeftHand (左手)
└── RightArm (右上臂)
    └── RightForeArm (右前臂)
        └── RightHand (右手)
```

### 2. 机器人关节映射

以Unitree G1为例的关节映射：

```python
# 配置文件: general_motion_retargeting/ik_configs/bvh_to_g1.json

{
    "robot_root_name": "pelvis",      # 机器人根关节
    "human_root_name": "Hips",        # 人体根关节
    "human_scale_table": {
        "Hips": 0.9,                  # 各部位缩放比例
        "Spine2": 0.9,
        "LeftUpLeg": 0.9,
        "RightUpLeg": 0.9,
        # ... 更多映射
    },
    "ik_match_table1": {
        "pelvis": ["Hips", 0, 10, [0,0,0], [0.5,0.5,0.5,0.5]],
        "left_hip_yaw_link": ["LeftUpLeg", 0, 10, [0,0,0], [-0.5,0.5,0.5,-0.5]],
        # ... 更多IK匹配
    }
}
```

### 3. IK匹配表结构

每个IK匹配条目包含5个元素：
```python
[human_bone_name, position_weight, rotation_weight, position_offset, rotation_offset]
```

- `human_bone_name`: 对应的人体骨骼名称
- `position_weight`: 位置权重 (0-100)
- `rotation_weight`: 旋转权重 (0-100)  
- `position_offset`: 位置偏移 [x, y, z]
- `rotation_offset`: 旋转偏移 [w, x, y, z] (四元数)

## 重定向算法实现

### 1. 核心重定向类

```python
# 文件路径: general_motion_retargeting/motion_retarget.py

class GeneralMotionRetargeting:
    def __init__(self, src_human="bvh", tgt_robot="unitree_g1", 
                 actual_human_height=None, solver="daqp", damping=5e-1):
        """
        初始化重定向系统
        
        Args:
            src_human: 源人体数据格式 ("bvh", "smplx")
            tgt_robot: 目标机器人类型
            actual_human_height: 实际人体高度
            solver: IK求解器类型 ("daqp", "quadprog")
            damping: 阻尼系数
        """
        # 1. 加载机器人模型
        self.xml_file = str(ROBOT_XML_DICT[tgt_robot])
        self.model = mj.MjModel.from_xml_path(self.xml_file)
        
        # 2. 加载IK配置
        config_path = IK_CONFIG_DICT[f"{src_human}_to_{tgt_robot}"]
        with open(config_path, 'r') as f:
            self.config = json.load(f)
        
        # 3. 初始化IK求解器
        self.ik_solver = mink.MinkIKSolver(
            self.xml_file,
            solver=solver,
            damping=damping,
            use_velocity_limit=use_velocity_limit
        )
        
        # 4. 设置人体缩放
        self.human_height_assumption = self.config.get("human_height_assumption", 1.8)
        if actual_human_height is not None:
            self.human_height_scale = actual_human_height / self.human_height_assumption
        else:
            self.human_height_scale = 1.0
```

### 2. 单帧重定向算法

```python
def retarget(self, human_data):
    """
    重定向单帧人体动作数据到机器人
    
    Args:
        human_data: 人体骨骼数据字典
        
    Returns:
        qpos: 机器人关节位置 [root_pos(3), root_rot(4), dof_pos(num_dofs)]
    """
    # 1. 提取目标位置和旋转
    target_positions = {}
    target_rotations = {}
    
    for robot_body, ik_match in self.config["ik_match_table1"].items():
        human_bone = ik_match[0]
        if human_bone in human_data:
            # 应用人体缩放
            scale = self.config["human_scale_table"].get(human_bone, 1.0)
            pos_offset = np.array(ik_match[3])
            rot_offset = np.array(ik_match[4])
            
            # 计算目标位置和旋转
            human_pos, human_rot = human_data[human_bone]
            target_pos = human_pos * scale * self.human_height_scale + pos_offset
            target_rot = utils.quat_mul(human_rot, rot_offset)
            
            target_positions[robot_body] = target_pos
            target_rotations[robot_body] = target_rot
    
    # 2. 设置IK约束
    self.ik_solver.clear_constraints()
    
    for robot_body, ik_match in self.config["ik_match_table1"].items():
        if robot_body in target_positions:
            pos_weight = ik_match[1]
            rot_weight = ik_match[2]
            
            # 添加位置约束
            if pos_weight > 0:
                self.ik_solver.add_position_constraint(
                    robot_body, 
                    target_positions[robot_body],
                    weight=pos_weight
                )
            
            # 添加旋转约束  
            if rot_weight > 0:
                self.ik_solver.add_rotation_constraint(
                    robot_body,
                    target_rotations[robot_body], 
                    weight=rot_weight
                )
    
    # 3. 求解IK
    qpos = self.ik_solver.solve()
    
    # 4. 二次优化 (使用ik_match_table2)
    if self.config.get("use_ik_match_table2", False):
        self._refine_with_table2(qpos, human_data)
    
    return qpos
```

### 3. 二次优化算法

```python
def _refine_with_table2(self, qpos, human_data):
    """
    使用第二张IK匹配表进行精细优化
    """
    # 重新设置更严格的约束
    self.ik_solver.clear_constraints()
    
    for robot_body, ik_match in self.config["ik_match_table2"].items():
        human_bone = ik_match[0]
        if human_bone in human_data:
            pos_weight = ik_match[1]
            rot_weight = ik_match[2]
            pos_offset = np.array(ik_match[3])
            rot_offset = np.array(ik_match[4])
            
            # 计算精确目标
            human_pos, human_rot = human_data[human_bone]
            scale = self.config["human_scale_table"].get(human_bone, 1.0)
            target_pos = human_pos * scale * self.human_height_scale + pos_offset
            target_rot = utils.quat_mul(human_rot, rot_offset)
            
            # 添加精确约束
            if pos_weight > 0:
                self.ik_solver.add_position_constraint(robot_body, target_pos, weight=pos_weight)
            if rot_weight > 0:
                self.ik_solver.add_rotation_constraint(robot_body, target_rot, weight=rot_weight)
    
    # 重新求解
    return self.ik_solver.solve()
```

## 坐标系转换

### 1. BVH坐标系到机器人坐标系

```python
# BVH使用右手坐标系，机器人通常使用右手坐标系但Y轴向上
rotation_matrix = np.array([
    [1, 0, 0],    # X轴不变
    [0, 0, -1],   # Y轴映射到-Z轴
    [0, 1, 0]     # Z轴映射到Y轴
])

# 四元数表示
rotation_quat = R.from_matrix(rotation_matrix).as_quat(scalar_first=True)
```

### 2. 单位转换

```python
# BVH通常使用厘米，机器人使用米
position_meters = position_centimeters / 100.0
```

## 示例代码：完整重定向流程

### 1. 单文件重定向示例

```python
#!/usr/bin/env python3
"""
完整的BVH重定向示例
文件路径: examples/bvh_retargeting_example.py
"""

import os
import numpy as np
import pickle
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.lafan1 import load_lafan1_file

def bvh_to_robot_example():
    """BVH到机器人重定向的完整示例"""
    
    # 1. 配置参数
    bvh_file = "path/to/your/motion.bvh"
    robot_type = "unitree_g1"
    output_file = "output/motion_g1.pkl"
    
    # 2. 加载BVH数据
    print("Loading BVH data...")
    lafan1_data_frames, actual_human_height = load_lafan1_file(bvh_file)
    print(f"Loaded {len(lafan1_data_frames)} frames")
    print(f"Human height: {actual_human_height:.2f}m")
    
    # 3. 初始化重定向系统
    print("Initializing retargeting system...")
    retargeter = GMR(
        src_human="bvh",
        tgt_robot=robot_type,
        actual_human_height=actual_human_height,
        solver="daqp",
        damping=5e-1,
        verbose=True
    )
    
    # 4. 重定向每一帧
    print("Retargeting motion...")
    qpos_list = []
    
    for i, frame_data in enumerate(lafan1_data_frames):
        if i % 30 == 0:  # 每30帧打印一次进度
            print(f"Processing frame {i}/{len(lafan1_data_frames)}")
        
        # 执行重定向
        qpos = retargeter.retarget(frame_data)
        qpos_list.append(qpos.copy())
    
    # 5. 保存结果
    print("Saving results...")
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    
    # 转换数据格式
    qpos_array = np.array(qpos_list)
    root_pos = qpos_array[:, :3]
    root_rot = qpos_array[:, 3:7]
    # 转换四元数格式: wxyz -> xyzw
    root_rot = root_rot[:, [1, 2, 3, 0]]
    dof_pos = qpos_array[:, 7:]
    
    motion_data = {
        "fps": 30,
        "root_pos": root_pos,
        "root_rot": root_rot, 
        "dof_pos": dof_pos,
        "local_body_pos": None,
        "link_body_list": None,
    }
    
    with open(output_file, "wb") as f:
        pickle.dump(motion_data, f)
    
    print(f"Saved motion data to {output_file}")
    print(f"Motion duration: {len(qpos_list)/30:.2f} seconds")
    print(f"Total frames: {len(qpos_list)}")

if __name__ == "__main__":
    bvh_to_robot_example()
```

### 2. 批量处理示例

```python
#!/usr/bin/env python3
"""
批量BVH重定向示例
文件路径: examples/batch_bvh_retargeting.py
"""

import os
import glob
from tqdm import tqdm
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.lafan1 import load_lafan1_file

def batch_retarget_bvh_folder(src_folder, tgt_folder, robot_type):
    """
    批量重定向文件夹中的所有BVH文件
    
    Args:
        src_folder: 源BVH文件夹
        tgt_folder: 目标PKL文件夹  
        robot_type: 机器人类型
    """
    
    # 1. 查找所有BVH文件
    bvh_files = glob.glob(os.path.join(src_folder, "**/*.bvh"), recursive=True)
    print(f"Found {len(bvh_files)} BVH files")
    
    # 2. 初始化重定向系统 (复用同一个实例提高效率)
    retargeter = None
    
    # 3. 处理每个文件
    for bvh_file in tqdm(bvh_files, desc="Retargeting"):
        try:
            # 计算输出文件路径
            rel_path = os.path.relpath(bvh_file, src_folder)
            pkl_file = os.path.join(tgt_folder, rel_path.replace('.bvh', '.pkl'))
            
            # 跳过已存在的文件
            if os.path.exists(pkl_file):
                print(f"Skipping {bvh_file} (already exists)")
                continue
            
            # 加载BVH数据
            lafan1_data_frames, actual_human_height = load_lafan1_file(bvh_file)
            
            # 初始化或重用重定向系统
            if retargeter is None:
                retargeter = GMR(
                    src_human="bvh",
                    tgt_robot=robot_type,
                    actual_human_height=actual_human_height,
                    verbose=False  # 批量处理时关闭详细输出
                )
            
            # 重定向所有帧
            qpos_list = []
            for frame_data in lafan1_data_frames:
                qpos = retargeter.retarget(frame_data)
                qpos_list.append(qpos.copy())
            
            # 保存结果
            os.makedirs(os.path.dirname(pkl_file), exist_ok=True)
            save_motion_data(qpos_list, pkl_file)
            
        except Exception as e:
            print(f"Error processing {bvh_file}: {e}")
            continue
    
    print(f"Batch processing completed. Results saved to {tgt_folder}")

def save_motion_data(qpos_list, output_file):
    """保存动作数据到PKL文件"""
    import pickle
    import numpy as np
    
    qpos_array = np.array(qpos_list)
    root_pos = qpos_array[:, :3]
    root_rot = qpos_array[:, 3:7]
    root_rot = root_rot[:, [1, 2, 3, 0]]  # wxyz -> xyzw
    dof_pos = qpos_array[:, 7:]
    
    motion_data = {
        "fps": 30,
        "root_pos": root_pos,
        "root_rot": root_rot,
        "dof_pos": dof_pos,
        "local_body_pos": None,
        "link_body_list": None,
    }
    
    with open(output_file, "wb") as f:
        pickle.dump(motion_data, f)

if __name__ == "__main__":
    # 示例用法
    batch_retarget_bvh_folder(
        src_folder="data/lafan1/",
        tgt_folder="output/lafan1_g1/",
        robot_type="unitree_g1"
    )
```

### 3. 自定义重定向参数示例

```python
#!/usr/bin/env python3
"""
自定义重定向参数示例
文件路径: examples/custom_retargeting.py
"""

import json
import numpy as np
from general_motion_retargeting import GeneralMotionRetargeting as GMR

def custom_retargeting_example():
    """演示如何自定义重定向参数"""
    
    # 1. 加载默认配置
    config_path = "general_motion_retargeting/ik_configs/bvh_to_g1.json"
    with open(config_path, 'r') as f:
        config = json.load(f)
    
    # 2. 修改配置参数
    config["human_height_assumption"] = 1.75  # 修改假设高度
    config["human_scale_table"]["LeftArm"] = 0.8  # 调整左臂缩放
    config["human_scale_table"]["RightArm"] = 0.8  # 调整右臂缩放
    
    # 3. 保存自定义配置
    custom_config_path = "custom_configs/bvh_to_g1_custom.json"
    os.makedirs(os.path.dirname(custom_config_path), exist_ok=True)
    with open(custom_config_path, 'w') as f:
        json.dump(config, f, indent=4)
    
    # 4. 使用自定义配置进行重定向
    # (注意: 需要修改GMR类以支持自定义配置文件路径)
    
    print(f"Custom configuration saved to {custom_config_path}")

def analyze_retargeting_quality(bvh_file, robot_type):
    """分析重定向质量"""
    
    # 加载数据
    from general_motion_retargeting.utils.lafan1 import load_lafan1_file
    lafan1_data_frames, actual_human_height = load_lafan1_file(bvh_file)
    
    # 初始化重定向系统
    retargeter = GMR(
        src_human="bvh",
        tgt_robot=robot_type,
        actual_human_height=actual_human_height,
        verbose=True
    )
    
    # 分析重定向误差
    position_errors = []
    rotation_errors = []
    
    for frame_data in lafan1_data_frames:
        qpos = retargeter.retarget(frame_data)
        
        # 计算位置和旋转误差
        # (这里需要根据具体需求实现误差计算)
        
        pass
    
    # 输出分析结果
    print(f"Average position error: {np.mean(position_errors):.4f}m")
    print(f"Average rotation error: {np.mean(rotation_errors):.4f}rad")

if __name__ == "__main__":
    custom_retargeting_example()
```

## 性能优化技巧

### 1. 内存优化

```python
# 对于大型数据集，使用生成器避免内存溢出
def process_bvh_generator(bvh_file, batch_size=100):
    """使用生成器处理大型BVH文件"""
    lafan1_data_frames, actual_human_height = load_lafan1_file(bvh_file)
    
    for i in range(0, len(lafan1_data_frames), batch_size):
        batch = lafan1_data_frames[i:i+batch_size]
        yield batch, actual_human_height
```

### 2. 并行处理

```python
from multiprocessing import Pool
import functools

def parallel_batch_retarget(bvh_files, robot_type, num_processes=4):
    """并行处理多个BVH文件"""
    
    retarget_func = functools.partial(retarget_single_file, robot_type=robot_type)
    
    with Pool(num_processes) as pool:
        results = pool.map(retarget_func, bvh_files)
    
    return results
```

### 3. 缓存优化

```python
# 缓存重定向系统实例
retargeter_cache = {}

def get_retargeter(robot_type, human_height):
    """获取或创建重定向系统实例"""
    cache_key = f"{robot_type}_{human_height:.2f}"
    
    if cache_key not in retargeter_cache:
        retargeter_cache[cache_key] = GMR(
            src_human="bvh",
            tgt_robot=robot_type,
            actual_human_height=human_height,
            verbose=False
        )
    
    return retargeter_cache[cache_key]
```

## 调试和故障排除

### 1. 常见问题诊断

```python
def diagnose_retargeting_issues(bvh_file, robot_type):
    """诊断重定向问题"""
    
    try:
        # 1. 检查BVH文件格式
        lafan1_data_frames, actual_human_height = load_lafan1_file(bvh_file)
        print(f"✓ BVH file loaded successfully")
        print(f"  - Frames: {len(lafan1_data_frames)}")
        print(f"  - Human height: {actual_human_height:.2f}m")
        
        # 2. 检查骨骼结构
        sample_frame = lafan1_data_frames[0]
        required_bones = ["Hips", "Spine2", "LeftUpLeg", "RightUpLeg", 
                         "LeftArm", "RightArm", "LeftHand", "RightHand"]
        
        missing_bones = [bone for bone in required_bones if bone not in sample_frame]
        if missing_bones:
            print(f"✗ Missing bones: {missing_bones}")
        else:
            print(f"✓ All required bones present")
        
        # 3. 检查重定向系统
        retargeter = GMR(
            src_human="bvh",
            tgt_robot=robot_type,
            actual_human_height=actual_human_height,
            verbose=False
        )
        print(f"✓ Retargeting system initialized")
        
        # 4. 测试单帧重定向
        test_qpos = retargeter.retarget(sample_frame)
        print(f"✓ Single frame retargeting successful")
        print(f"  - Output shape: {test_qpos.shape}")
        
        return True
        
    except Exception as e:
        print(f"✗ Error: {e}")
        return False
```

### 2. 可视化调试

```python
def visualize_retargeting_debug(bvh_file, robot_type):
    """可视化调试重定向过程"""
    
    from general_motion_retargeting import RobotMotionViewer
    
    # 加载数据
    lafan1_data_frames, actual_human_height = load_lafan1_file(bvh_file)
    retargeter = GMR(
        src_human="bvh",
        tgt_robot=robot_type,
        actual_human_height=actual_human_height
    )
    
    # 创建可视化器
    viewer = RobotMotionViewer(
        robot_type=robot_type,
        motion_fps=30,
        transparent_robot=0.5  # 半透明显示
    )
    
    # 逐帧显示重定向过程
    for i, frame_data in enumerate(lafan1_data_frames[:100]):  # 只显示前100帧
        qpos = retargeter.retarget(frame_data)
        
        viewer.step(
            root_pos=qpos[:3],
            root_rot=qpos[3:7],
            dof_pos=qpos[7:],
            human_motion_data=retargeter.scaled_human_data,
            rate_limit=True
        )
        
        if i % 30 == 0:
            print(f"Debug frame {i}/{min(100, len(lafan1_data_frames))}")
    
    viewer.close()
```

## 总结

本文档详细介绍了BVH重定向的技术原理和实现细节：

1. **BVH解析**: 如何解析BVH文件格式并转换为标准数据结构
2. **骨骼映射**: 人体骨骼与机器人关节的对应关系
3. **重定向算法**: 基于逆运动学的动作重定向核心算法
4. **坐标系转换**: 不同坐标系之间的转换方法
5. **示例代码**: 完整的重定向流程实现
6. **性能优化**: 提高处理效率的各种技巧
7. **调试方法**: 问题诊断和可视化调试

通过这些原理和示例代码，你可以深入理解BVH重定向的工作原理，并根据需要进行自定义修改和优化。
