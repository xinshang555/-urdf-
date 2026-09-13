# 轮腿机器人 URDF / MJCF

由 `car.STEP` + SolidWorks 质量报告重建的**权威模型**。
几何与质量属性均按 CAD 真值重算并逐项验证。

---

## 该用哪个文件

| 文件 | 用途 |
|---|---|
| **`mjcf/car.xml`** | **MuJoCo 首选**。含地面、光照、4 个电机、`home` 关键帧。 |
| **`urdf/car.urdf`** | 权威几何。基座是 `body`，无 world。适合建模/转换、RViz。 |
| **`urdf/car_floating.urdf`** | 带 floating 基座，`body` 是真正 6 自由度刚体。 |
| **`urdf/car_zup.urdf`** | Z-up URDF（`car.xml` 由它生成）。 |

> ⚠️ **别直接用 `car.urdf` 跑 MuJoCo**
> MuJoCo 会把 URDF 的**根 link 合并进 `world`**，其质量与惯量**被丢弃**。
> 实测：`car.urdf` 载入后总质量只剩 **4.00 g**（21.53 g 的车身没了）。
> 用 `car_floating.urdf` / `car_zup.urdf` / `car.xml` → **25.53000 g**，与 CAD 一致。

> ⚠️ **`car.xml` 里的网格是相对路径**（`meshes/*.stl`），
> 所以 `urdf/` 和 `mjcf/` **各自带一份 `meshes/`**。别把 `car.xml` 单独挪走。

---

## 打开

**Windows**

```
双击 view_mujoco.bat
```

**WSL / Linux**

```bash
cd urdf
python3 -c "import mujoco,glfw; ..."   # 或直接用你的查看器
python3 -m mujoco.viewer --mjcf=mjcf/car.xml
```

**MuJoCo python**

```python
import mujoco
m = mujoco.MjModel.from_xml_path("mjcf/car.xml")
d = mujoco.MjData(m)
mujoco.viewer.launch(m, d)
```

操作：`Space` 暂停 · `Backspace` 回到 home · `Tab` 面板（电机滑块）· `Esc` 退出

---

## 验证过的事实

```
每件位置误差        0.000000 mm（vs CAD）
配合间隙            0.264890 / 0.189206 / 0.264890 / 0.189206 mm
                    （与源 CAD 网格完全一致）
两轮横向间距        39.0000 mm
车身居中            −17.5000 mm（正好在两轮中点）
两轮静止高度        +7.9731 mm（等高，不歪）
总质量              25.53000 g = 21.53(车体) + 2×1.14(腿) + 2×0.86(轮)
各件密度            1000.1 / 1001.2 / 1006.8 kg/m³（一致 → 单位正确）
渲染连通块          1（正面/侧面/俯视）
```

**CAD 真值**

```
髋轴→轮轴节距      40.000000 mm
轮外半径           8.000 mm      轮宽 7.000 mm
车身               40 × 25 × 35 mm
髋轴 Z             0.0 / 31.5
轮心 Z             −2.0 / +37.0
```

**关节**

| 关节 | 父→子 | 类型 | 范围 |
|---|---|---|---|
| `hip_lower_joint` | body → leg_lower | revolute | ±0.8 rad |
| `hip_upper_joint` | body → leg_upper | revolute | ±0.8 rad |
| `wheel_lower_joint` | leg_lower → wheel_lower | continuous | — |
| `wheel_upper_joint` | leg_upper → wheel_upper | continuous | — |

轴向均为 `0 0 1`。**关节无任何 `<dynamics>` 阻尼** —— 加了会静默破坏平衡控制器
（实测 `damping=0.01` 会让 20 rad/s 在 0.05 s 内归零）。

**MuJoCo 特有**

```
nbody=6  njnt=5  nu=4  nq=11
电机 gear = 0.004473 N·m（按腿的重力矩算出，不是 1）
髋限位实测停在 +0.80021 rad（限位 +0.8000）
```

---

## 坐标系

沿用 **CAD 装配轴**：X = 前后，Y = 竖直，Z = 侧向。
CAD 的"下"是 **−Y**。

`car_zup.urdf` 用 **+90° 绕 X**（`rpy="1.5707963267948966 0 0"`）把 −Y 映射到 −Z。
（−90° 会让轮子跑到车身上面，已实测。）

---

## 注意事项

**这是个倒立摆。** 总质心在轮轴上方约 33 mm，不上电会在约 1 秒内倒下 ——
**这是正确物理，不是缺陷**。闭环控制是控制器的事。
`home` 关键帧（轮子落在 Z=0、关节全零、车身竖直）就是控制器该起步的姿态。

**质量报告里没有的项（我推断的，欢迎纠正）**

| 项 | 来源 | 把握 |
|---|---|---|
| 质量 / 质心 / 主惯量 | 报告 | 高 |
| 零件几何与位置 | STEP | 高 |
| 关节轴向 = Z | 几何上唯一合理 | 中，未确认 |
| **关节限位 ±0.8 rad** | **取的"看起来合理"值** | **低，需确认** |

**质量报告的两个坑**（如果你要重新导出，注意）

1. `leg1/leg2`、`wheel1/wheel2` 的**编号与上下相反** ——
   必须按质心坐标匹配，不能按文件名。
2. 轮子报告体积比网格大 0.97%（文件名 `wheel_defeature`，去特征版）——
   质量可用，但别用体积反推验证。

**轮子间距以 STEP 为准。** 报告质心差是 41.52 mm（那是含轮毂的质心，
且两轮不是镜像摆放），**轴线实际间距是 39.00 mm**，以 STEP 网格测量为准。

---

## 目录

```
urdf/
├── car.urdf              权威几何（基座 body）
├── car_floating.urdf     floating base
├── car_zup.urdf          Z-up（car.xml 来源）
├── meshes/               link-local 网格（米制）
├── CAD到URDF标准流程.md   从 STEP 生成 URDF 的规范步骤
└── 转化纪实.md            这次转化怎么成功的：判断依据与排查手法
mjcf/
├── car.xml               MuJoCo 模型
└── meshes/               （car.xml 的相对引用）
```

> **改了模型怎么验证?** 先读 `urdf/CAD到URDF标准流程.md`。
> 一句话：**逐零件检查(包围盒/质量/密度)发现不了"零件之间脱开"**，
> 必须单独验证装配连接性。
