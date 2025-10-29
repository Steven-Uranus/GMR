# 使用GMR实现通用人形机器人动捕数据重定向

## 一、为什么需要动作重定向

动作重定向（Motion Retargeting）是将人体的运动映射到机器人或虚拟角色身上的过程。

由于人体与机器人的形态差异显著（例如关节拓扑、自由度数量、肢体比例），人体捕捉到的运动数据无法直接驱动机器人执行。

因此，需要通过一种重定向方法，将人体捕捉数据转化为机器人可执行的关节角度和根部位姿。

常见的重定向方法包括：

- **PHC（Perpetual Humanoid Control）**：依赖优化式的离线处理，通过最小化人体与机器人关键点差距获得高质量匹配。
- **GMR（General Motion Retargeting）**：通过明确的几何映射规则与逆运动学（IK）解算，实现高效实时的动作转换。

本文以宇树 **Unitree G1** 机器人为例，介绍如何使用 **GMR 框架** 实现从动捕数据到机器人动作的重定向。

---

## 二、总体流程概览

GMR 重定向的完整流程包括五个主要步骤：

1. **Human–Robot KeyBody Matching**
2. **Human–Robot Cartesian Space Alignment**
3. **Human Data Non-Uniform Local Scaling**
4. **IK with Rotation Constraint**
5. **IK with Rotation & Translation Constraint**

下图展示了整个流程的逻辑结构：

![image.png](image.png)

![3b75e7709c1e4f7c981c738d17bc635a_778x639.png](3b75e7709c1e4f7c981c738d17bc635a_778x639.png)

---

## 三、关键流程详解

### 1. 人体–机器人关键体匹配（KeyBody Matching）

KeyBody Matching 的目标是建立人体骨骼与机器人关节的一一对应关系，确保语义一致。每个关节的旋转和位置权重通过配置表定义，用于后续 IK 求解。

在项目配置文件 `bvh_to_g1.json` 中，`ik_match_table1` 与 `ik_match_table2` 就是关键体匹配表。第一阶段主要约束旋转，第二阶段同时约束位置。参数分别对应人体骨骼名称、平移权重、旋转权重、平移偏移量 pos_off、旋转偏移量 rot_off (四元数)。

```json
"ik_match_table1": {    "pelvis": ["Hips", 0, 10, [0.0, 0.0, 0.0], [0.5, -0.5, -0.5, -0.5]],    "left_hip_yaw_link": ["LeftUpLeg", 0, 10, [0.0, 0.0, 0.0], [0.5, -0.5, -0.5, -0.5]],    "left_shoulder_yaw_link": ["LeftArm", 0, 100, [0.0, 0.0, 0.0], [0.70710678, 0.0, -0.70710678, 0.0]]}

```

这些表由 `GeneralMotionRetargeting` 类在初始化时自动读取并存入 `self.ik_match_table1` 与 `self.ik_match_table2`。加载逻辑在：

```python
retargeter = GMR(
    src_human=f"bvh_{args.format}",
    tgt_robot=args.robot,
    actual_human_height=actual_human_height,
)
```

此时，机器人每个 link 将与 BVH 动捕文件的骨骼节点绑定，实现一一对应。

---

### 2. 坐标系对齐（Cartesian Space Alignment）

人体动捕系统与机器人往往使用不同的世界坐标系，因此必须对齐参考坐标。

通常选取 **骨盆（pelvis）** 作为参考原点，调整旋转和平移以保持全局一致。若未对齐机器人动作会出现错位、旋转畸形。

- **对齐方法**：根据三轴方向（X、Y、Z）建立旋转矩阵，将人体坐标变换到机器人坐标系下。
- 动捕系统通常定义为：
    - X → 右，Y → 上，Z → 前
- 而 G1 机器人定义为：
    - X → 前，Y → 右，Z → 上

两者存在 120° 旋转差异，等价于绕 (1,1,1) 旋转 120°（下文会解释），其四元数表示为：

**$q = [0.5, -0.5, -0.5, -0.5]$，**该四元数即为 BVH→G1 坐标系的旋转，对应的`rot_off` 字段。

> 🧭 图 坐标对齐效果示意图
> 

---

### 3. 局部比例缩放（Non-Uniform Local Scaling）

由于人体和机器人的肢体比例不同（如手臂、腿长度），直接映射会造成“手太长”或“跨步太大”的问题。

GMR通过**非均匀缩放**修正这种比例差异：
$*Lrobot, i = si ⋅ Lhuman, i*$
其中：

- $(L_{robot,i}$)：机器人第 i 段肢体长度

- ($L_{human,i}$)：人体第 i 段肢体长度

- ($s_i$)：局部缩放因子

在 `bvh_to_g1.json` 中的 `human_scale_table` 字段：

```json
"human_scale_table": {    "Hips": 0.9,    "Spine2": 0.9,    "LeftUpLeg": 0.9,    "RightUpLeg": 0.9,    "LeftArm": 0.75,    "RightArm": 0.75,    "LeftForeArm": 0.75,    "RightForeArm": 0.75}
```

GMR 初始化时读取该表，对人体模型的每个对应关节长度乘以系数 s_i，从而得到 `scaled_human_data`。

在主程序 `bvh_to_robot.py` 中，缩放后的人体数据被传入渲染器：

```python
robot_motion_viewer.step(
    root_pos=qpos[:3],
    root_rot=qpos[3:7],
    dof_pos=qpos[7:],
    human_motion_data=retargeter.scaled_human_data,
)
```

这一步确保了在重定向后，人体与机器人姿态比例保持一致，不再出现穿模或过度拉伸。

---

### 4. 逆运动学解算（IK with Rotation Constraint）

当所有关节坐标系统一后，我们进入动作重定向的核心阶段——**逆运动学（Inverse Kinematics, IK）求解**。

GMR 框架的关键特性之一，就是采用了 **两阶段 IK 求解机制**，即「先对齐旋转姿态，再逼近空间位置」。这种方式可以在保持姿态自然的同时，保证末端位置精度。

---

### 4.1 阶段一：Rotation Constraint（旋转约束）

在第一阶段，我们只约束关节的旋转姿态，使机器人姿态与人体方向一致。

目标函数可表示为：
$\min_{q} \; \sum_i \; w_r^i \cdot \| R_i(q) - R_i^{\text{target}} \|^2$

其中：

- $q$ ：机器人所有关节角度向量；
- $R_i(q)$ ：由当前关节角计算得到的旋转矩阵；
- $R_i^{\text{target}}$：目标关节的旋转；
- $w_r^i$：旋转误差的权重。

此时不考虑末端位置（Translation），仅保证各关节的**朝向正确**。

这样做的好处是：

- 姿态不会“翻转”或“扭结”；
- 关节间保持连贯的动作方向。

代码实现中，调用 IK 求解器时，`ik_match_table1` 用于此阶段：

```json
"ik_match_table1": {    "left_knee_link": ["LeftLeg", 0, 10, [0.0, 0.0, 0.0], [0.5, -0.5, -0.5, -0.5]]}
```

每个条目的 **第二个数字对**（`0,10`）对应 **位置权重=0，旋转权重=10**。

这说明此阶段优化器仅根据旋转误差最小化，位置误差被忽略。

在主循环中：

```python
# bvh_to_robot.pyqpos = retargeter.retarget(smplx_data)
```

此调用触发内部 GMR 的第一阶段 IK 过程。

```python
if self.use_ik_match_table1:
    curr_error = self.error1()
    dt = self.configuration.model.opt.timestep
    vel1 = mink.solve_ik(
        self.configuration, self.tasks1, dt, self.solver, self.damping, self.ik_limits
    )
    self.configuration.integrate_inplace(vel1, dt)
    next_error = self.error1()
    num_iter = 0    while curr_error - next_error > 0.001 and num_iter < self.max_iter:
        curr_error = next_error
        dt = self.configuration.model.opt.timestep
        vel1 = mink.solve_ik(
            self.configuration, self.tasks1, dt, self.solver, self.damping, self.ik_limits
        )
        self.configuration.integrate_inplace(vel1, dt)
        next_error = self.error1()
        num_iter += 1
```

- `self.tasks1`：由 `ik_match_table1` 定义的旋转任务集合。
- `mink.solve_ik()`：调用 **Mink IK 求解器**，最小化旋转误差。
- `integrate_inplace()`：将求解出的角速度 `vel1` 积分为新的机器人姿态。
- `error1()`：计算当前姿态误差并进行迭代收敛。

---

### 4.2 阶段二：Rotation + Translation Constraint（双约束优化）

完成旋转一致性后，我们进一步优化末端位置误差。

在第二阶段的目标函数中，同时引入旋转与平移项，

对机器人所有关节角向量$q$求解，使得每个被约束关节的旋转误差和位置误差加权和最小。
$\min_{q} \sum_i \left( w_r^i \| R_i(q) - R_i^{\text{target}} \|^2 + w_t^i \| p_i(q) - p_i^{\text{target}} \|^2 \right)$

其中：

- $p_i(q)$：由当前关节角求出的末端位置；
- $p_i^{\text{target}}$：目标末端位置；
- $w_t^i$：位置误差权重。

这一步的作用是让机器人“更贴近人体动作”，

确保末端（手、脚）到达目标位置，同时姿态保持稳定。

`ik_match_table2` 为此阶段配置：

```json
"ik_match_table2": {    "left_ankle_roll_link": ["LeftFootMod", 50, 10, [0.0, 0.0, 0.0], [0.5, -0.5, -0.5, -0.5]]}
```

此处位置权重=50，旋转权重=10，即第二阶段开始补偿末端空间偏差。

```python
if self.use_ik_match_table2:
    curr_error = self.error2()
    dt = self.configuration.model.opt.timestep
    vel2 = mink.solve_ik(
        self.configuration, self.tasks2, dt, self.solver, self.damping, self.ik_limits
    )
    self.configuration.integrate_inplace(vel2, dt)
    next_error = self.error2()
    num_iter = 0    while curr_error - next_error > 0.001 and num_iter < self.max_iter:
        curr_error = next_error
        # Solve the IK problem with the second task        dt = self.configuration.model.opt.timestep
        vel2 = mink.solve_ik(
            self.configuration, self.tasks2, dt, self.solver, self.damping, self.ik_limits
        )
        self.configuration.integrate_inplace(vel2, dt)
        next_error = self.error2()
        num_iter += 1
```

- `self.tasks2`：由 `ik_match_table2` 定义，包含平移与旋转的双权重任务。
- `error2()`：计算位置与姿态的综合误差。
- `self.max_iter`：最大迭代次数，防止陷入无限优化循环。

📌 **第二阶段是在第一阶段收敛后执行的**，相当于“精修动作”，保证手脚落点正确，同时保持姿态自然。

---

## 四、坐标系统一与四元数变换

在动作重定向过程中，**坐标系对齐**是最关键的环节之一。如果动捕系统与机器人坐标定义不一致，那么即使角度数据完全正确，机器人动作也会表现为“扭曲”、“反向”甚至“倒立”。

以本项目中使用的通用动捕系统为例，其坐标定义为：

- **X 轴朝右   Y 轴朝上   Z 轴朝前**

而对于仿人机器人（以 Unitree G1 为例），其全身关节遵循右手法则：

- **X 轴朝前   Y 轴朝右   Z 轴朝上**

### 1. 双约束逆运动学（IK with Rotation & Translation Constraint）

显然，这两套坐标系并不一致。我们需要求出一个旋转，使得动捕数据的坐标系能够与机器人保持一致。

---

### 1.1 坐标轴映射关系

通过比较发现，动捕坐标系与机器人坐标系的关系为：

| 动捕坐标 | → | 机器人坐标 |
| --- | --- | --- |
| X（右） | → | Y（右） |
| Y（上） | → | Z（上） |
| Z（前） | → | X（前） |

换句话说，这个变换是一个**坐标轴循环置换**，对应的旋转是：
$X_{\text{new}} = Y_{\text{old}},\quad
Y_{\text{new}} = Z_{\text{old}},\quad
Z_{\text{new}} = X_{\text{old}}$

---

### 1.2 构造旋转矩阵

根据上述对应关系，可以得到该变换的旋转矩阵：
$R =
\begin{bmatrix}
0 & 1 & 0\\
0 & 0 & 1\\
1 & 0 & 0
\end{bmatrix}$

该矩阵将旧坐标系的向量变换为新坐标系下的表示。

---

### 1.3 从旋转矩阵到轴角表示

根据旋转矩阵的基本性质：
$\cos\theta = \frac{\text{trace}(R) - 1}{2}$

其中 $\text{trace}(R)=0$，可得：
$\cos\theta = -\frac{1}{2} \Rightarrow \theta = 120^\circ$

进一步求旋转轴方向：

$\mathbf{u} =
\frac{1}{2\sin\theta}
\begin{bmatrix}
R_{32}-R_{23}\\
R_{13}-R_{31}\\
R_{21}-R_{12}
\end{bmatrix}=

-\frac{1}{\sqrt{3}}(1,1,1)$

这说明：**该坐标变换等价于绕 (1,1,1) 方向旋转 120°。**

---

### 1.4 四元数表示

将上述轴角形式转为四元数（采用 $[w, x, y, z]$形式）：

$w = \cos\left(\frac{\theta}{2}\right) = \frac{1}{2}$

$(x,y,z) = \mathbf{u} \sin\left(\frac{\theta}{2}\right) = -\frac{1}{2}(1,1,1)$

因此四元数为：
$\boxed{q = [0.5,\; -0.5,\; -0.5,\; -0.5]}$

需要注意：四元数与其相反数表示同一旋转，

即 $[-0.5, 0.5, 0.5, 0.5]$ 与上式等价。

---

### 1.5 验证

该旋转的几何意义是 **沿主对角线方向 (1,1,1) 顺时针旋转 120°**，从而使得：
$(X,Y,Z){\text{motion}} \Rightarrow (Y,Z,X){\text{robot}}$

这一结论可直接验证：

- 原动捕的 X 轴（右）旋转后对齐机器人 Y 轴（右）；
- 原动捕的 Y 轴（上）旋转后对齐机器人 Z 轴（上）；
- 原动捕的 Z 轴（前）旋转后对齐机器人 X 轴（前）。

因此，**当对所有腿部关节统一乘以此四元数旋转后**，动捕数据与机器人坐标系完全一致。

---

### 2. 特殊关节调整

- **腿部关节**：使用统一的旋转修正，保持与右手法则一致。
- **手臂关节**：左右对称，但符号方向相反。
- **肩部关节（shoulder_yaw）**：存在约30°的倾角，需额外绕Y轴旋转90°修正。

在完成了全局坐标系对齐之后，我们需要进一步解决**各个关节自身的局部坐标系方向差异**。

虽然腿部关节、手臂关节、肩关节都遵循右手坐标系，但它们的零位定义并不完全相同，因此需要针对性地调整。

---

### 2.1 腿部关节：完全统一的 120° 旋转

对于腿部部分（hips、knees、ankles），其定义较为标准，且机器人所有腿部关节均遵循：

- X 轴指向前方
- Y 轴指向右方
- Z 轴指向上方

动捕系统的坐标方向为：

- X 朝右
- Y 朝上
- Z 朝前

经过前文分析，这两者的关系正好是**绕 (1,1,1) 方向旋转 120°**。

因此，所有腿部关节只需统一使用相同的旋转四元数进行对齐：
$q_{\text{leg}} = [0.5, -0.5, -0.5, -0.5]$

---

### 2.2 手臂关节：左右对称（符号差异）

手臂部分则稍微复杂一些。

原因在于：人体的左右臂在动捕数据中是完全镜像定义的，即左臂坐标系的 **Y 轴指向左侧**，右臂的 **Y 轴指向右侧**。而机器人的定义中，**左右两侧手臂的坐标系方向相同**（右手法则始终一致）。

这就意味着：

当我们将动捕数据摆成机器人“零位”姿势时，左臂与右臂的坐标系在 Y 轴上正好是反向的。

因此，对手臂部分的旋转四元数，需要在左右两侧分别取正负号：
$q_{\text{left arm}} = [0.5, -0.5, -0.5, -0.5]$
~~$q_{\text{right arm}} = [-0.5, +0.5, +0.5, +0.5]$~~

这种做法可以保证左右手臂的摆动方向一致——否则，当人抬左手时，机器人会“抬右手”或手臂扭转方向错误。

---

### 2.3 肘关节：坐标系一致，无需旋转

肘关节（`elbow`）的特殊之处在于：

其局部坐标定义中，**X 轴恰好沿着前臂指向小臂方向**，

Y、Z 分别指向垂直和平面方向。

当动捕数据摆成与机器人零位相同的姿态时，

**肘关节的坐标系与机器人完全一致**。

因此，在此处不需要任何旋转修正，直接使用单位四元数即可：
$q_{\text{elbow}} = [1.0, 0.0, 0.0, 0.0]$

---

### 2.4 肩关节：存在轻微倾角需额外补偿

在实际校对过程中，我们发现机器人的 `shoulder_yaw` 关节的零位坐标系

并非与世界坐标完全重合，而是存在约 **30° 的倾角**。

同时，当手臂抬至机器人零位（水平）时，关节坐标轴方向为：

- X 轴朝下
- Y 轴朝右
- Z 轴朝前

要将其旋回到与世界坐标一致的位置，

只需**沿 Y 轴逆时针旋转 90°**（或顺时针旋转 -90°）。

对应四元数为：
$q_{\text{shoulder yaw}} = [0.7071, 0.0, 0.7071, 0.0]$

其中：
$0.7071 = \cos(45^\circ)$

---

> 当所有局部坐标系对齐后，GMR 框架即可在统一空间下进行动作重定向，
> 
> 
> 确保“旋转方向一致、肢体姿态自然”。
> 

---

~~是否需要Beyondmimic相关内容~~

---

下图展示了**动捕人物动作**与**Unitree G1 机器人动作**的对比：

- 左侧为原始动捕数据动画；
- 右侧为经 GMR 重定向后的机器人动作。

重定向后的机器人动作能够平滑复现人体姿态，

整体动作自然流畅，无明显卡顿或姿态跳变。