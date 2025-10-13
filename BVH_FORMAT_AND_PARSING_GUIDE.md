# BVH文件格式详解与解析过程指南

## 概述

本文档详细介绍了BVH (Biovision Hierarchy) 文件格式的结构、组成，以及GMR项目如何解析BVH文件并进行骨骼映射的完整过程。

## BVH文件格式详解

### 1. BVH文件结构

BVH文件由两个主要部分组成：

```
HIERARCHY部分: 定义骨骼层次结构
MOTION部分: 包含动作数据
```

### 2. HIERARCHY部分详解

#### 基本语法结构

```bvh
HIERARCHY
ROOT Hips
{
    OFFSET 0.00 0.00 0.00
    CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation
    JOINT Spine
    {
        OFFSET 0.00 5.21 0.00
        CHANNELS 3 Zrotation Xrotation Yrotation
        JOINT Spine1
        {
            OFFSET 0.00 5.19 0.00
            CHANNELS 3 Zrotation Xrotation Yrotation
            JOINT Spine2
            {
                OFFSET 0.00 5.19 0.00
                CHANNELS 3 Zrotation Xrotation Yrotation
                JOINT LeftShoulder
                {
                    OFFSET 5.00 0.00 0.00
                    CHANNELS 3 Zrotation Xrotation Yrotation
                    JOINT LeftArm
                    {
                        OFFSET 0.00 -3.00 0.00
                        CHANNELS 3 Zrotation Xrotation Yrotation
                        JOINT LeftForeArm
                        {
                            OFFSET 0.00 -2.50 0.00
                            CHANNELS 3 Zrotation Xrotation Yrotation
                            JOINT LeftHand
                            {
                                OFFSET 0.00 -2.50 0.00
                                CHANNELS 3 Zrotation Xrotation Yrotation
                                End Site
                                {
                                    OFFSET 0.00 -2.50 0.00
                                }
                            }
                        }
                    }
                }
                JOINT RightShoulder
                {
                    OFFSET -5.00 0.00 0.00
                    CHANNELS 3 Zrotation Xrotation Yrotation
                    JOINT RightArm
                    {
                        OFFSET 0.00 -3.00 0.00
                        CHANNELS 3 Zrotation Xrotation Yrotation
                        JOINT RightForeArm
                        {
                            OFFSET 0.00 -2.50 0.00
                            CHANNELS 3 Zrotation Xrotation Yrotation
                            JOINT RightHand
                            {
                                OFFSET 0.00 -2.50 0.00
                                CHANNELS 3 Zrotation Xrotation Yrotation
                                End Site
                                {
                                    OFFSET 0.00 -2.50 0.00
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    JOINT LeftUpLeg
    {
        OFFSET 3.91 0.00 0.00
        CHANNELS 3 Zrotation Xrotation Yrotation
        JOINT LeftLeg
        {
            OFFSET 0.00 -4.04 0.00
            CHANNELS 3 Zrotation Xrotation Yrotation
            JOINT LeftFoot
            {
                OFFSET 0.00 -4.04 0.00
                CHANNELS 3 Zrotation Xrotation Yrotation
                JOINT LeftToe
                {
                    OFFSET 0.00 -1.00 0.00
                    CHANNELS 3 Zrotation Xrotation Yrotation
                    End Site
                    {
                        OFFSET 0.00 0.00 0.00
                    }
                }
            }
        }
    }
    JOINT RightUpLeg
    {
        OFFSET -3.91 0.00 0.00
        CHANNELS 3 Zrotation Xrotation Yrotation
        JOINT RightLeg
        {
            OFFSET 0.00 -4.04 0.00
            CHANNELS 3 Zrotation Xrotation Yrotation
            JOINT RightFoot
            {
                OFFSET 0.00 -4.04 0.00
                CHANNELS 3 Zrotation Xrotation Yrotation
                JOINT RightToe
                {
                    OFFSET 0.00 -1.00 0.00
                    CHANNELS 3 Zrotation Xrotation Yrotation
                    End Site
                    {
                        OFFSET 0.00 0.00 0.00
                    }
                }
            }
        }
    }
}
```

#### 关键元素说明

1. **ROOT**: 根关节，通常为Hips
2. **JOINT**: 普通关节
3. **End Site**: 末端关节（如手指尖、脚趾尖）
4. **OFFSET**: 相对于父关节的偏移量 (X, Y, Z)
5. **CHANNELS**: 数据通道数量和类型

#### 通道类型详解

```bvh
CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation
```

- **位置通道**: Xposition, Yposition, Zposition
- **旋转通道**: Xrotation, Yrotation, Zrotation
- **通道数量**: 3 (仅旋转) 或 6 (位置+旋转)

### 3. MOTION部分详解

```bvh
MOTION
Frames: 100
Frame Time: 0.033333
0.00 0.00 0.00 0.00 0.00 0.00 15.00 0.00 0.00 0.00 0.00 0.00 ...
```

#### 数据格式说明

- **Frames**: 总帧数
- **Frame Time**: 每帧时间间隔（秒）
- **数据行**: 每行包含一帧所有关节的通道数据

#### 数据排列顺序

数据按照HIERARCHY中关节出现的顺序排列，每个关节的数据按CHANNELS定义的顺序排列。

## GMR项目中的BVH解析过程

### 1. 解析器核心代码

#### 主要解析函数

```python
# 文件路径: general_motion_retargeting/utils/lafan_vendor/extract.py

def read_bvh(filename, start=None, end=None, order=None):
    """
    读取BVH文件并提取动画信息
    
    Args:
        filename: BVH文件路径
        start: 开始帧
        end: 结束帧  
        order: 欧拉角旋转顺序
        
    Returns:
        Anim对象，包含提取的信息
    """
    
    f = open(filename, "r")
    
    # 初始化变量
    i = 0
    active = -1
    end_site = False
    
    names = []                    # 关节名称列表
    orients = np.array([]).reshape((0, 4))  # 初始方向（四元数）
    offsets = np.array([]).reshape((0, 3))  # 关节偏移量
    parents = np.array([], dtype=int)       # 父关节索引
    
    # 逐行解析文件
    for line in f:
        # 跳过标题行
        if "HIERARCHY" in line: continue
        if "MOTION" in line: continue
        
        # 解析ROOT关节
        rmatch = re.match(r"ROOT (\w+)", line)
        if rmatch:
            names.append(rmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients = np.append(orients, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue
        
        # 解析JOINT关节
        jmatch = re.match("\s*JOINT\s+(\w+)", line)
        if jmatch:
            names.append(jmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients = np.append(orients, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue
        
        # 解析OFFSET
        offmatch = re.match(r"\s*OFFSET\s+([\-\d\.e]+)\s+([\-\d\.e]+)\s+([\-\d\.e]+)", line)
        if offmatch:
            if not end_site:
                offsets[active] = np.array([list(map(float, offmatch.groups()))])
            continue
        
        # 解析CHANNELS
        chanmatch = re.match(r"\s*CHANNELS\s+(\d+)", line)
        if chanmatch:
            channels = int(chanmatch.group(1))
            # 自动检测旋转顺序
            if order is None:
                channelis = 0 if channels == 3 else 3
                channelie = 3 if channels == 3 else 6
                parts = line.split()[2 + channelis:2 + channelie]
                if any([p not in channelmap for p in parts]):
                    continue
                order = "".join([channelmap[p] for p in parts])
            continue
        
        # 解析End Site
        if "End Site" in line:
            end_site = True
            continue
        
        # 解析帧数
        fmatch = re.match("\s*Frames:\s+(\d+)", line)
        if fmatch:
            if start and end:
                fnum = (end - start) - 1
            else:
                fnum = int(fmatch.group(1))
            positions = offsets[np.newaxis].repeat(fnum, axis=0)
            rotations = np.zeros((fnum, len(orients), 3))
            continue
        
        # 解析帧时间
        fmatch = re.match("\s*Frame Time:\s+([\d\.]+)", line)
        if fmatch:
            frametime = float(fmatch.group(1))
            continue
        
        # 解析动作数据
        dmatch = line.strip().split(' ')
        if dmatch:
            data_block = np.array(list(map(float, dmatch)))
            N = len(parents)
            fi = i - start if start else i
            
            if channels == 3:
                # 只有根关节有位置数据
                positions[fi, 0:1] = data_block[0:3]
                rotations[fi, :] = data_block[3:].reshape(N, 3)
            elif channels == 6:
                # 所有关节都有位置和旋转数据
                data_block = data_block.reshape(N, 6)
                positions[fi, :] = data_block[:, 0:3]
                rotations[fi, :] = data_block[:, 3:6]
            elif channels == 9:
                # 复杂格式
                positions[fi, 0] = data_block[0:3]
                data_block = data_block[3:].reshape(N - 1, 9)
                rotations[fi, 1:] = data_block[:, 3:6]
                positions[fi, 1:] += data_block[:, 0:3] * data_block[:, 6:9]
            else:
                raise Exception("Too many channels! %i" % channels)
            
            i += 1
    
    f.close()
    
    # 转换欧拉角为四元数
    rotations = utils.euler_to_quat(np.radians(rotations), order=order)
    rotations = utils.remove_quat_discontinuities(rotations)
    
    return Anim(rotations, positions, offsets, parents, names)
```

#### Anim类结构

```python
class Anim(object):
    """
    基本的动画对象
    """
    def __init__(self, quats, pos, offsets, parents, bones):
        """
        Args:
            quats: 局部四元数张量 (frames, joints, 4)
            pos: 局部位置张量 (frames, joints, 3)  
            offsets: 局部关节偏移量 (joints, 3)
            parents: 骨骼层次结构 (joints,)
            bones: 骨骼名称列表 (joints,)
        """
        self.quats = quats    # 旋转四元数
        self.pos = pos        # 位置数据
        self.offsets = offsets # 关节偏移
        self.parents = parents # 父关节索引
        self.bones = bones    # 骨骼名称
```

### 2. 坐标系转换

#### BVH到机器人坐标系转换

```python
# 文件路径: general_motion_retargeting/utils/lafan1.py

def load_lafan1_file(bvh_file):
    """
    加载LAFAN1格式的BVH文件并转换为标准格式
    """
    # 1. 读取BVH文件
    data = read_bvh(bvh_file)
    
    # 2. 前向运动学计算全局位置和旋转
    global_data = utils.quat_fk(data.quats, data.pos, data.parents)
    
    # 3. 坐标系转换矩阵
    # BVH使用右手坐标系，机器人使用右手坐标系但Y轴向上
    rotation_matrix = np.array([
        [1, 0, 0],    # X轴不变
        [0, 0, -1],   # Y轴映射到-Z轴  
        [0, 1, 0]     # Z轴映射到Y轴
    ])
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

## 骨骼映射和JSON配置详解

### 1. 配置文件结构

#### 配置文件位置

```python
# 文件路径: general_motion_retargeting/params.py

IK_CONFIG_DICT = {
    "bvh": {
        "unitree_g1": IK_CONFIG_ROOT / "bvh_to_g1.json",
        "booster_t1_29dof": IK_CONFIG_ROOT / "bvh_to_t1_29dof.json",
        "fourier_n1": IK_CONFIG_ROOT / "bvh_to_n1.json",
        "stanford_toddy": IK_CONFIG_ROOT / "bvh_to_toddy.json",
        "engineai_pm01": IK_CONFIG_ROOT / "bvh_to_pm01.json",
    }
}
```

#### 配置文件示例 (bvh_to_g1.json)

```json
{
    "robot_root_name": "pelvis",
    "human_root_name": "Hips",
    "ground_height": 0.0,
    "human_height_assumption": 1.8,
    "use_ik_match_table1": true,
    "use_ik_match_table2": true,
    
    "human_scale_table": {
        "Hips": 0.9,
        "Spine2": 0.9,
        "LeftUpLeg": 0.9,
        "RightUpLeg": 0.9,
        "LeftLeg": 0.9,
        "RightLeg": 0.9,
        "LeftFootMod": 0.9,
        "RightFootMod": 0.9,
        "LeftArm": 0.75,
        "RightArm": 0.75,
        "LeftForeArm": 0.75,
        "RightForeArm": 0.75,
        "LeftHand": 0.75,
        "RightHand": 0.75
    },
    
    "ik_match_table1": {
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
    },
    
    "ik_match_table2": {
        // 与ik_match_table1结构相同，但权重不同
        // 用于二次优化
    }
}
```

### 2. 配置参数详解

#### 基础参数

```json
{
    "robot_root_name": "pelvis",           // 机器人根关节名称
    "human_root_name": "Hips",             // 人体根关节名称
    "ground_height": 0.0,                  // 地面高度
    "human_height_assumption": 1.8,        // 假设的人体高度
    "use_ik_match_table1": true,           // 是否使用第一张IK匹配表
    "use_ik_match_table2": true            // 是否使用第二张IK匹配表
}
```

#### 人体缩放表 (human_scale_table)

```json
"human_scale_table": {
    "Hips": 0.9,        // 髋部缩放比例
    "Spine2": 0.9,      // 胸部缩放比例
    "LeftUpLeg": 0.9,   // 左大腿缩放比例
    "RightUpLeg": 0.9,  // 右大腿缩放比例
    "LeftLeg": 0.9,     // 左小腿缩放比例
    "RightLeg": 0.9,    // 右小腿缩放比例
    "LeftFootMod": 0.9, // 左脚缩放比例
    "RightFootMod": 0.9,// 右脚缩放比例
    "LeftArm": 0.75,    // 左上臂缩放比例
    "RightArm": 0.75,   // 右上臂缩放比例
    "LeftForeArm": 0.75,// 左前臂缩放比例
    "RightForeArm": 0.75,// 右前臂缩放比例
    "LeftHand": 0.75,   // 左手缩放比例
    "RightHand": 0.75   // 右手缩放比例
}
```

**缩放比例说明**:
- 值越小，该部位相对于机器人越短
- 腿部和躯干通常使用较大的值 (0.9)
- 手臂通常使用较小的值 (0.75)

#### IK匹配表 (ik_match_table1/2)

每个条目包含5个元素：

```json
"robot_body_name": [
    "human_bone_name",    // 对应的人体骨骼名称
    position_weight,      // 位置权重 (0-100)
    rotation_weight,      // 旋转权重 (0-100)
    [x_offset, y_offset, z_offset],  // 位置偏移
    [w, x, y, z]          // 旋转偏移 (四元数)
]
```

**权重说明**:
- `position_weight`: 控制位置约束的重要性
  - 0: 不约束位置
  - 10: 弱约束
  - 50: 中等约束
  - 100: 强约束
- `rotation_weight`: 控制旋转约束的重要性
  - 0: 不约束旋转
  - 10: 弱约束
  - 100: 强约束

**偏移说明**:
- `position_offset`: 在机器人坐标系中的位置偏移
- `rotation_offset`: 四元数旋转偏移，用于调整坐标系差异

### 3. 骨骼映射过程

#### 映射流程

```python
def retarget(self, human_data):
    """
    重定向单帧人体动作数据到机器人
    """
    # 1. 提取目标位置和旋转
    target_positions = {}
    target_rotations = {}
    
    for robot_body, ik_match in self.config["ik_match_table1"].items():
        human_bone = ik_match[0]  # 人体骨骼名称
        
        if human_bone in human_data:
            # 2. 获取缩放比例
            scale = self.config["human_scale_table"].get(human_bone, 1.0)
            
            # 3. 获取偏移量
            pos_offset = np.array(ik_match[3])  # 位置偏移
            rot_offset = np.array(ik_match[4])  # 旋转偏移
            
            # 4. 计算目标位置和旋转
            human_pos, human_rot = human_data[human_bone]
            
            # 应用缩放和高度调整
            target_pos = (human_pos * scale * self.human_height_scale + 
                         pos_offset)
            
            # 应用旋转偏移
            target_rot = utils.quat_mul(human_rot, rot_offset)
            
            target_positions[robot_body] = target_pos
            target_rotations[robot_body] = target_rot
    
    # 5. 设置IK约束并求解
    # ... IK求解过程
```

#### 映射示例

以Unitree G1为例：

```python
# 人体骨骼 -> 机器人关节映射
mapping_examples = {
    "Hips": "pelvis",                    # 髋部 -> 骨盆
    "LeftUpLeg": "left_hip_yaw_link",    # 左大腿 -> 左髋关节
    "LeftLeg": "left_knee_link",         # 左小腿 -> 左膝关节
    "LeftFootMod": "left_ankle_roll_link", # 左脚 -> 左踝关节
    "RightUpLeg": "right_hip_yaw_link",  # 右大腿 -> 右髋关节
    "RightLeg": "right_knee_link",       # 右小腿 -> 右膝关节
    "RightFootMod": "right_ankle_roll_link", # 右脚 -> 右踝关节
    "Spine2": "torso_link",              # 胸部 -> 躯干
    "LeftArm": "left_shoulder_yaw_link", # 左上臂 -> 左肩关节
    "LeftForeArm": "left_elbow_link",    # 左前臂 -> 左肘关节
    "LeftHand": "left_wrist_yaw_link",   # 左手 -> 左腕关节
    "RightArm": "right_shoulder_yaw_link", # 右上臂 -> 右肩关节
    "RightForeArm": "right_elbow_link",  # 右前臂 -> 右肘关节
    "RightHand": "right_wrist_yaw_link"  # 右手 -> 右腕关节
}
```

## 比例缩放详解

### 1. 缩放计算过程

```python
def calculate_scaled_position(human_pos, scale, human_height_scale):
    """
    计算缩放后的位置
    
    Args:
        human_pos: 人体关节位置
        scale: 部位缩放比例
        human_height_scale: 整体高度缩放比例
        
    Returns:
        scaled_pos: 缩放后的位置
    """
    # 1. 应用部位缩放
    scaled_pos = human_pos * scale
    
    # 2. 应用整体高度缩放
    scaled_pos = scaled_pos * human_height_scale
    
    return scaled_pos

# 实际计算示例
human_height_assumption = 1.8  # 配置中的假设高度
actual_human_height = 1.75     # 实际人体高度

# 计算整体缩放比例
human_height_scale = actual_human_height / human_height_assumption
# human_height_scale = 1.75 / 1.8 = 0.972

# 计算最终位置
human_pos = np.array([0.5, 1.2, 0.1])  # 人体关节位置
scale = 0.9                             # 部位缩放比例

final_pos = human_pos * scale * human_height_scale
# final_pos = [0.5, 1.2, 0.1] * 0.9 * 0.972 = [0.437, 1.049, 0.087]
```

### 2. 不同机器人的缩放策略

#### Unitree G1 (成人尺寸机器人)
```json
"human_scale_table": {
    "Hips": 0.9,        // 接近原始尺寸
    "Spine2": 0.9,
    "LeftUpLeg": 0.9,
    "RightUpLeg": 0.9,
    "LeftLeg": 0.9,
    "RightLeg": 0.9,
    "LeftArm": 0.75,    // 手臂较短
    "RightArm": 0.75,
    "LeftForeArm": 0.75,
    "RightForeArm": 0.75,
    "LeftHand": 0.75,
    "RightHand": 0.75
}
```

#### Stanford ToddlerBot (儿童尺寸机器人)
```json
"human_scale_table": {
    "Hips": 0.35,       // 大幅缩小
    "Spine2": 0.35,
    "LeftUpLeg": 0.35,
    "RightUpLeg": 0.35,
    "LeftLeg": 0.35,
    "RightLeg": 0.35,
    "LeftArm": 0.35,
    "RightArm": 0.35,
    "LeftForeArm": 0.35,
    "RightForeArm": 0.35,
    "LeftHand": 0.35,
    "RightHand": 0.35
}
```

### 3. 缩放调整指南

#### 如何调整缩放比例

1. **观察重定向结果**:
   ```bash
   python scripts/bvh_to_robot.py --bvh_file test.bvh --robot unitree_g1
   ```

2. **识别问题**:
   - 手臂太长/太短
   - 腿部比例不当
   - 躯干长度不合适

3. **修改配置文件**:
   ```json
   "human_scale_table": {
       "LeftArm": 0.8,      // 从0.75调整到0.8
       "RightArm": 0.8,
       "LeftForeArm": 0.8,
       "RightForeArm": 0.8
   }
   ```

4. **重新测试**:
   ```bash
   python scripts/bvh_to_robot.py --bvh_file test.bvh --robot unitree_g1
   ```

## 总结

本文档详细介绍了：

1. **BVH文件格式**: 完整的文件结构和语法
2. **解析过程**: GMR项目如何解析BVH文件
3. **骨骼映射**: 人体骨骼到机器人关节的映射关系
4. **JSON配置**: 详细的配置文件结构和参数说明
5. **比例缩放**: 如何调整不同部位和整体的缩放比例

通过这些知识，你可以：
- 理解BVH文件的内部结构
- 自定义骨骼映射关系
- 调整缩放比例以适配不同机器人
- 创建新的机器人配置文件
- 调试和优化重定向效果
