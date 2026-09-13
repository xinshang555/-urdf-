# CAD 装配体 → URDF/MJCF 标准流程

> 从 `car.STEP` + 质量报告生成可用 URDF/MJCF 的规范步骤。
> 依据 `轮腿训练/newstart` 的实际踩坑记录整理,每一步都标注了**为什么**和**怎么验证**。
>
> 核心教训:**逐零件检查(包围盒/质量/密度)永远发现不了"零件之间脱开"。必须单独验证装配连接性。**

---

## 0. 心法:三类检查,缺一不可

| 层级 | 检查什么 | 能发现 | **发现不了** |
|---|---|---|---|
| **单零件** | 包围盒、体积、质量、密度、质心、惯量 | 网格损坏、单位错、质量错 | 零件间错位 |
| **坐标系** | 正运动学 vs 手算 | 旋转符号错、轴向错 | 零件间错位 |
| **装配连接性** | 相邻零件的**最近顶点距离** | **零件散落** | — |

> ⚠️ 本项目就是栽在这里:前两层全部通过(包围盒误差 0.000000 mm),但轮子离腿 **13 mm**。
> 包围盒对得上**纯属巧合** —— 因为包围盒是稀疏描述,零件平移一段仍可能不改变它。

---

## 1. 读 STEP 装配结构(先搞清有几个零件)

**不要假设。** 从 STEP 里数出来:

```python
# PRODUCT_DEFINITION + NEXT_ASSEMBLY_USAGE_OCCURRENCE
```

本项目实测:

```
PRODUCT 'car'              ← 装配体
PRODUCT 'body'             ← 零件 1
PRODUCT 'leg'              ← 零件 2
PRODUCT 'wheel_defeature'  ← 零件 3
MANIFOLD_SOLID_BREP = 3    ← 只有 3 个实体

NEXT_ASSEMBLY_USAGE_OCCURRENCE 5 条 = 5 个装配实例:
    NAUO1 → body
    NAUO2 → leg
    NAUO3 → wheel
    NAUO4 → leg      ← 同一零件的第 2 个实例
    NAUO5 → wheel    ← 同一零件的第 2 个实例
```

**结论:5 个实例 = 3 种零件。** 导出成 5 个 STL 是**正确**的(URDF 的 `<mesh>` 不支持镜像复用),
不要误以为"把模型切成了 5 块"。

**验证**:`grep -c 'MANIFOLD_SOLID_BREP' car.STEP` 应等于实体数。

---

## 2. 导出零件网格(保持装配坐标!)

**关键:导出后必须仍是 CAD 装配坐标,不要各自居中/归零。**

原因:装配体的全部信息就是**相对位置**。一旦归零就丢了。

```python
# 正确:直接用装配坐标
# 错误:mesh.center() 或 v -= v.mean(0)
```

**验证**:五个网格摆在一起,配合面必须贴合:

```python
body      <-> leg_lower      0.264890 mm
leg_lower <-> wheel_lower    0.189207 mm
body      <-> leg_upper      0.264890 mm
leg_upper <-> wheel_upper    0.189207 mm
```

> 这 4 个数就是**基准值**,后面每一步都要复现它们。

---

## 3. 定零件坐标系 —— 用纯平移,不加旋转

**这是本项目最大的坑。**

```python
# ✅ 正确:每个 link 定义一个锚点,网格 = 装配坐标 - 锚点
loc = v_assembly - anchor

# ❌ 错误:自作聪明加旋转
if link == "wheel_upper":
    v = (R @ v.T).T + T      # ← 把上轮推开了 30 mm
```

**为什么不能加旋转**:源网格已经是装配坐标,**原样用就是对的**。
我当初"发现"两轮有 180° 关系,就加了旋转 —— 反而破坏了装配。

> 判别方法:如果加了变换后配合间隙变大,变换就是错的。
> **任何变换都必须通过第 6 步的连接性检查。**

### 锚点怎么选

选**物理上有意义的点**,并让关节位置自然:

| 零件 | 锚点 | 理由 |
|---|---|---|
| body | CAD (0,0,0) | 天然基准 |
| leg_* | 髋关节轴承孔中心 | 关节转轴 |
| wheel_* | 轮轴中心 | 关节转轴 |

**注意**:轮子是"轮盘 + 轮毂"结构,轮毂(5 mm 凸台)才是装配面。
轴线位置要**量出来**,不要猜:

```
part_003 (下轮)  轮盘 Z -5.50..-0.25   轮毂 Z -0.25..+1.50
part_005 (上轮)  轮盘 Z 36.50..40.50   轮毂 Z 33.50..+36.50
```

> ⚠️ 上轮锚点 Z=**35.0**,不是 31.5(腿的间距)。写成 31.5 会让轮子挂在轴外 3.5 mm。

**镜像约束自检**:两轮中心之和应等于车身中平面 × 2:
```
下轮中心 Z = -2.0,  上轮中心 Z = +37.0,  和 = +35.0 = 2 × 17.5  ✅
```

---

## 4. 生成 URDF

### 4.1 三个变体,各有用处

| 文件 | 基座 | 用途 |
|---|---|---|
| `car.urdf` | `body`,无 world | 权威几何、RViz、转换输入 |
| `car_floating.urdf` | `<world>` + floating 关节 | MuJoCo/PyBullet |
| `car_zup.urdf` | world → mount(floating) → body(fixed, rpy=+90°X) | Z-up 仿真器 |

> ⚠️ **MuJoCo 会把 URDF 根 link 合并进 `world` 并丢弃其质量。**
> 实测:直接用 `car.urdf` 总质量只有 **4.00 g**(21.53 g 的车身消失了)。
> `car_floating.urdf` 才是 **25.53000 g**。

### 4.2 Z-up 旋转方向

CAD 的"下"是 −Y,Z-up 世界要映射成 −Z:

```
+90° 绕 X:  (x,y,z) → (x,-z,y)
     CAD down (0,-1,0) → (0,0,-1)   ✅
     CAD lat  (0,0,1)  → (0,-1,0)   两轮沿世界 Y 并排
```
URDF rpy 是 `Rz·Ry·Rx`,纯 roll 即 `rpy="1.5707963267948966 0 0"`。

> **符号必须实测。** 我先写成 −90°,轮子跑到车身上面去了。

### 4.3 绝对不要写关节阻尼

```xml
<!-- ❌ 千万别加 -->
<dynamics damping="0.01" friction="0.001"/>
```

MuJoCo 会当 DOF 阻尼导入,轮子 20 rad/s 在 **0.05 s 内衰减到 0** —— **会静默毁掉平衡控制器**。
摩擦/阻尼属于仿真器建模,不写进运动学文件。

---

## 5. 质量与惯量

**只取质量报告的"由重心决定"块,忽略"由输出座标系决定"块。**

单位换算:
```
g      → kg     × 1e-3
g·mm²  → kg·m²  × 1e-9
mm     → m      × 1e-3
```

**交叉验证**(必做):网格积分出的密度应接近材料密度:
```
body 1000.9 | leg 1001.2 | wheel 1006.8  kg/m³   ← 全部 ~1000,一致
```
若某个零件密度明显不同 → 单位或体积算错了。

> ⚠️ 别用 `user_model.json`,它比报告高约 20%。

---

## 6. 【必做】装配连接性验证

**这一步是本流程的核心,前五步都替代不了它。**

### 6.1 两条检查互补,缺一不可

| 检查 | 抓什么 | 盲区 |
|---|---|---|
| **相对间隙** | 零件之间脱开 | 整体平移(腿长 48 mm,平移 5 mm 仍与车身重叠) |
| **绝对包围盒** | 整体平移 | — |

> 本项目实测:只做相对间隙检查时,"整条腿偏移 5 mm"**漏检**。
> 所以两条都要做。`tools/test_assembly_gate.py` 用注入 bug 的方式证明了这一点。

### 6.2 现成工具

```bash
cd urdf/tools
./run.sh assembly_gate.py          # 用内置 CONFIG
./run.sh assembly_gate.py my.json  # 用外部配置(换项目只改 json)
./run.sh test_assembly_gate.py     # 门禁自测:注入 4 种 bug,必须全部报警
```

配置项(`CONFIG` 或 json):

```python
{
  "mesh_dir": "../meshes",
  "mesh_scale_to_mm": 1000.0,              # STL 里是米 → ×1000
  "links": {                                # 每个零件的锚点 + 质量
      "wheel_upper": {"mesh": "wheel_upper_local.stl",
                      "anchor": [-11.202069, -41.040180, 35.0],  # ← 实际轴线
                      "mass_g": 0.86},
  },
  "mates": [["leg_upper", "wheel_upper"]],  # 只写真正有配合的对
  "max_gap_mm": 0.5,
  "expected_density": 1000.0,
  "expect_bbox": {                          # ← 锁死整体平移,必需
      "wheel_upper": [[-19.196, -49.013, 33.5], [-3.208, -33.067, 40.5]],
  },
  "bbox_tol_mm": 0.01,
  "symmetry": [{"links": ["wheel_lower", "wheel_upper"], "axis": 2,
                "expect_sum": 35.0, "tol": 0.01}],
}
```

**第一次用不知道 `expect_bbox` 填什么?** 把 `expect_bbox` 留空跑一次,
工具会把实测包围盒**打印成可直接粘贴的格式**。

### 6.3 要点

1. **只比较真正有配合关系的对**。两腿之间、两轮之间**故意分开**(车身两侧),不是缺陷。
2. **必须用顶点级距离**,不能用包围盒 —— 包围盒会骗你。
3. **在静止姿态(关节全零)下测**,不要在摔倒过程中测。

> ⚠️ 我曾在机器人**正在倾倒**时测量,量到 7.5 mm 的"轮高差",误判成几何缺陷。
> 静止后再测:**0.0001 mm**。

### 6.4 生成脚本里做成硬门禁

```python
if worst >= 0.5:
    raise SystemExit(f"ASSEMBLY GATE FAILED: gap {worst:.4f} mm")
```

不要只 `print` 警告 —— 散架的 URDF 毫无用处,而且**所有单零件检查都会通过**。

### 6.5 渲染层验证(补充)

把机器人单独渲染出来,数**连通块个数**:

```
正确:front 1 块 | side 1 块 | top 1 块
错误:front 5 块 | side 3 块 | top 2 块      ← 散落
```

> 小间隙(0.2 mm)在 620 px 画面里约 0.6 px,可能把一块切成两块。
> 所以**膨胀 1~2 px 后仍应为 1 块**,否则是真脱开。


---

## 7. URDF → MJCF

### 7.1 geom 的旋转**不要抄编译值**

```python
# ❌ 错误
gattr.append(f'quat="{mp.geom_quat[gid]}"')

# ✅ 正确:URDF 里 <visual><origin rpy="0 0 0">,geom 继承 body 坐标系
```

**为什么**:MuJoCo 编译 URDF 时会**重新表达 geom 坐标系**,好让惯量主轴落位 ——
那些值**不是** URDF 的 `<origin>`。抄回去会让腿带上 `quat=[0.511,0.489,-0.511,0.489]`
(**118° 误差**),腿被放倒。

### 7.2 但固定关节的旋转要补回来

MuJoCo 会**折叠 fixed 关节**:把旋转推到**子 body**上,父 body 保持不转,却仍持有子的网格。

所以 `mount` 拿到 `body_local` 网格但**没拿到 +90° 旋转** → 车身躺倒。

**从 URDF 读出来补上**:
```python
for j in urdf_root.findall("joint"):
    if j.get("type") == "fixed":
        rpy = j.find("origin").get("rpy")     # 转成四元数
        # 该四元数要写到 mount 的 geom 上
```

### 7.3 电机 gear 必须按惯量算

本项目髋关节等效惯量仅 **1.94e-6 kg·m²**。
`gear="1"` 意味着 1 N·m → **5.15e5 rad/s²** → 每步 257 rad/s → **第一步就炸**。

```
腿质量 1.14 g × g × 节距 40 mm = 4.473e-4 N·m
gear = 10 × 该值 = 0.004473
```

### 7.4 关节限位要加 solreflimit

软限位在 20 倍力矩下会被推过 4.25 rad。加上:
```xml
<joint solreflimit="0.002 1" solimplimit="0.99 0.999 0.001"/>
```
实测髋关节停在 **+0.80021 rad**(限位 0.8000)。

---

## 8. 等价性验证(URDF vs MJCF)

**必须对齐积分器设置,否则是在比较两个积分器。**

URDF 没有 `<option>`,MuJoCo 用默认 `dt=2ms / Euler`;
MJCF 若写 `dt=0.5ms / implicitfast`,直接比较**毫无意义**。

> 我第一版就这么干了,报出假的 **1743 m/s²** 失配。

### 正确的比较方式

**在相同状态下比**,而不是比漫长轨迹:

```python
# 关掉所有接触(地面 + 自碰撞)
mp.geom_contype[:] = 0; mp.geom_conaffinity[:] = 0
mq.geom_contype[:] = 0; mq.geom_conaffinity[:] = 0

# 给两个模型设同一个随机状态,比 M / qfrc_bias / qacc
```

本项目实测(5 个随机状态):
```
|dM|          ≤ 1.5e-16
|dqfrc_bias|  ≤ 4.6e-15
|dqacc|       ≤ 2.0e-09
```

### 长轨迹比较的陷阱

| 陷阱 | 现象 | 处理 |
|---|---|---|
| 只 MJCF 有地面 | 1000 步后差 19 | 关掉地面接触 |
| **自碰撞**在不同时刻触发 | 690 步突然跳变 | 关掉所有 contype |
| 撞到关节限位 | 1322 步从 2.5e-12 跳到 8.3e-4 | 只比前 1000 步,或改比状态等价 |

> 判别是不是真差异:**用同一个模型跑两个相差 1e-12 的初值**。
> 若 1e-12 也放大 → 混沌,是精度极限;若 1e-12 保持不动而两模型发散 → 有真差异。
>
> 本项目实测:1e-12 扰动 2000 步后仅 1e-11,而 MJCF 差 2.6e-2
> → **不是混沌**,查下去才发现是关节限位(第 1322 步 `qpos[7]` 撞 ±0.8)。

---

## 9. 验收清单

```
□ STEP 实体数 = 实际零件数,grep 确认
□ 网格保持装配坐标(导出后配合面间隙 = 源值)
□ 零件坐标系只用平移,无自加旋转
□ 两轮/两腿镜像约束:中心和对 = 中平面 × 2
□ 质量:总质量 = 报告值;各零件密度 ≈ 1000 kg/m³
□ 无 <dynamics> 阻尼
□ 【装配连接性】所有配合对间隙 < 0.5 mm  ← 最重要
□ 渲染连通块 = 1(膨胀 1px 后仍为 1)
□ MuJoCo 载入总质量正确(别被根 link 丢质量坑了)
□ gear 按惯量缩放,不是 1
□ 限位有 solreflimit
□ URDF vs MJCF:对齐 option 后 |dM| < 1e-14
□ 轮子自由空间中转速不衰减(无隐藏阻尼)
□ 关节全零时两轮等高(不歪)
```

---

## 10. 常见症状 → 病因对照

| 症状 | 病因 |
|---|---|
| 模型"散架",零件分离 | 坐标系加了多余旋转;或关节原点算错 |
| 包围盒全对但看起来散落 | **只做了单零件检查** —— 补做第 6 步 |
| 车身躺倒 / 腿横着 | MJCF geom 抄了编译后的 `geom_quat` |
| 轮子偏在轴外 | 锚点取了腿间距而非实际轴线 |
| 总质量少一大截 | 根 link 被 MuJoCo 吞了,要用 floating 变体 |
| 第一步就数值爆炸 | gear 太大,按惯量缩 |
| 轮子自己停转 | 有隐藏 DOF 阻尼 |
| 限位轻易被推过 | 缺 `solreflimit` |
| 仿真器里"下"是歪的 | Z-up 旋转方向错,要实测符号 |

---

## 附:工具脚本

```bash
cd urdf/tools

# 生成(自带装配硬门禁,散架会直接失败)
./run.sh build_final.py        # 3 个 URDF + 5 个网格
./run.sh make_mjcf.py          # car.xml + 逐步对比 URDF

# 装配门禁(可独立复用,换项目只改 CONFIG / json)
./run.sh assembly_gate.py       # 检查:密度 + 配合间隙 + 绝对位置 + 对称
./run.sh test_assembly_gate.py  # 门禁自测:注入 4 种 bug,必须全部报警

# 验证(建议全部跑)
./run.sh test_core.py          # STL 读写器 + 惯量数学回归
./run.sh validate_urdf.py      # 结构/物理/运动学
./run.sh mjcf_functional.py    # MJCF 功能 + 装配连接
./run.sh mujoco_test.py        # URDF 载入
./run.sh acceptance_test.py    # 动力学验收
./run.sh zup_check.py          # Z-up 变体
./run.sh stance_audit.py       # 站姿几何
./run.sh silhouette2.py        # 渲染连通块(必须 = 1)

# 看
python3 tools/view.py
```

> `run.sh` 会 `cd` 到脚本目录再执行,避免路径问题。
> PowerShell 里跑内联 Python 引号会被吃掉 —— **一律写成文件再跑**。

---

## 附:换到新项目的 5 步

1. **数零件**:`grep -c MANIFOLD_SOLID_BREP 新.step`
2. **导网格**:保持装配坐标,不要居中
3. **填 CONFIG**:`assembly_gate.py` 里改 `links`(锚点+质量)和 `mates`
4. **跑门禁**:`expect_bbox` 先留空 → 回填打印出来的值 → 再跑一次
5. **接进生成脚本**:`if worst >= 0.5: raise SystemExit(...)`
