# Arm Teach Studio · 六轴机械臂示教上位机

简体中文 · [English](README.md)

基于浏览器的六轴机械臂示教与轨迹规划上位机。通过串口 UART 文本协议与 ESP32 控制器通信，
支持 **拖动示教 → 记录轨迹 → 平滑去噪 → 一键复现（可调速度）**，也支持在 3D 视图中
摆放航点、由 jerk 受限的 S 曲线规划器经逆运动学驱动机械臂走出规划路径。

无需 Flask / Qt / 打包构建。后端是 Python 标准库 + `pyserial` + `numpy`；
前端是原生 ES 模块 + 本地化的 three.js，**完全离线可用**。

> **本仓库不附带机械臂模型**，需要你自备 URDF + STL 目录，详见 [机械臂模型](#机械臂模型)。

## 功能

- **实时位姿** —— 通过 SSE 以约 20 Hz 推送真机状态，3D 视图实时跟随；右侧表格显示每轴的
  位置 / 速度 / 电流 / 力矩 / 温度 / 在线状态。
- **拖动示教** —— 电机进入零力矩模式，手掰机械臂走轨迹；后端持续采集关节角，之后用中值滤波
  去除手抖尖峰、滑动平均得到平滑曲线。
- **一键复现** —— 以 0.1×～3.0× 倍速回放平滑轨迹，带进度条与随时停止。
- **航点规划** —— 直线 / Catmull-Rom 曲线航点经 IK 求解，速度采用闭式七段非对称 S 曲线
  （加速度连续、jerk 有界、收尾干脆），拐角由 junction-deviation 主动限速。
- **点动与标定** —— 每轴独立滑条；「校准零点」对齐 URDF 零位与电机绝对位置；可按轴翻转方向。
- **安全** —— 红色急停按钮随时可用；使能 / 失能对应固件的 `CLEAR` / `ESTOP`。
- **离线分析** —— `analyze_plan.py` 分析导出的规划数据，对比末端笛卡尔速度与关节空间速度的
  起伏，输出每轴峰值速度 / 加速度，可导出 CSV。

## 架构

```
浏览器前端 (three.js + URDF)            Python 后端 (server.py)
  ├─ 3D 实时位姿 / 末端轨迹曲线   <──SSE──   ├─ 串口收发 (pyserial)
  ├─ 关节位置/速度/力矩/温度表    ──HTTP──>  ├─ 记录 / numpy 平滑去噪
  ├─ 航点规划 + IK                          ├─ 轨迹复现 (可调速)
  └─ 控制面板                                └─ 静态文件 / 模型资源服务
```

## 环境要求

- Python 3.9+
- `pyserial >= 3.5`、`numpy >= 1.20`
- Chromium 内核浏览器（Chrome / Edge），需支持 ES 模块与 SSE
- 机械臂侧固件需实现 `$ARM,...*XX` 串口协议，见 [协议](#协议)

```bash
git clone https://github.com/wandou-000/yuying_arm_studio.git
cd yuying_arm_studio
pip install -r requirements.txt
```

## 机械臂模型

3D 视图需要 URDF 及其 STL 网格。这部分**不随仓库分发**（与具体臂型强绑定，且是大体积二进制）。
请自备模型目录并在启动时指定：

```
your_robot_model/
├── urdf/
│   └── robot.urdf        # 任意单个 .urdf 文件，程序自动探测
└── meshes/
    └── Link1.STL ...     # URDF 中以 package://<pkg>/meshes/... 引用
```

URDF 里的 `package://<任意名>/` 前缀会被改写为 `/assets/`，因此
`package://myarm/meshes/Link1.STL` 会从 `your_robot_model/meshes/Link1.STL` 提供。

启动方式二选一：

```bash
python server.py --assets /path/to/your_robot_model
# 或
ARM_TEACH_ASSETS=/path/to/your_robot_model python server.py
```

没有模型时服务仍可启动，串口与遥测功能正常，只是 3D 视图会提示 URDF 加载失败。

## 启动

```bash
python server.py                              # 默认 http://127.0.0.1:8000
python server.py --port 8080
python server.py --assets ../my_model --port 8080
```

然后在浏览器打开终端里提示的地址。

## 使用流程

1. **连接** —— 顶栏选择端口（如 `COM3`、`/dev/ttyUSB0`）与波特率（默认 `1000000`），点「连接」。
2. **使能 / 失能** —— 使能发送 `ARM,CLEAR`（清除急停、恢复位置速度模式）；失能发送
   `ARM,ESTOP`（急停，停止所有轴）。
3. **示教记录** —— 点「进入示教模式」发送 `ARM,TEACH,1`，电机零力矩，可用手自由拖动；
   点「开始记录」后端持续采集关节角；点「停止记录」，末端轨迹以橙色曲线显示。
4. **平滑处理** —— 调好「去尖峰窗口 / 平滑窗口」后点「平滑处理」，结果以青色曲线叠加。
5. **一键复现** —— 拖动「复现速度 ×」滑条（0.1～3.0），点「▶ 一键 Display」，
   机械臂按平滑轨迹复现，可随时「■ 停止」。
6. **轨迹规划** —— 在 3D 视图中添加直线 / 曲线航点，规划器求解 IK 并生成 jerk 受限的关节轨迹，
   以 `ARM,ALL` 帧下发。
7. **单轴点动** —— 右下角每轴滑条单独发送 `ARM,SET`。

### 关节角标定

电机上报的是绝对位置（0～360°），与 URDF 关节零位不一定一致。可在某个参考姿态下点
「校准零点」，把当前姿态设为可视化 0 位。若某轴可视化方向与真机相反，可在 `web/app.js`
顶部修改 `calib.sign[j] = -1`。

## 协议

帧格式为 `$...*XX`，带 XOR 校验，与 ESP32 固件一致。

| 功能 | 帧 |
| --- | --- |
| 查询反馈 | `$ARM,GET*..` |
| 单轴控制 | `$ARM,SET,<axis>,<angle_cdeg>,<speed_dps>*..` |
| 六轴联动 | `$ARM,ALL,...*..` |
| 急停 / 失能 | `$ARM,ESTOP*..` |
| 清除 / 使能 | `$ARM,CLEAR*..` |
| 进入 / 退出示教 | `$ARM,TEACH,1` / `$ARM,TEACH,0` |
| 反馈帧 | `$ARM,FB,<estop>,<moving>,<轴1..6 各 8 字段>*..` |

角度单位为百分之一度（cdeg），速度为整数 °/s。

## 文件结构

```
arm_teach_studio/
├── server.py          # 后端：串口 / HTTP + SSE / 记录 / 平滑 / 复现
├── analyze_plan.py    # 规划数据离线分析
├── requirements.txt
└── web/
    ├── index.html     # 界面布局
    ├── style.css
    ├── app.js         # 3D 场景 / SSE / UI 控制
    ├── robot.js       # URDF 解析 + STL 加载 + 正运动学
    ├── planner.js     # 航点 → S 曲线关节轨迹
    ├── ik.js          # 逆运动学
    └── vendor/        # 本地化的 three.js / OrbitControls / STLLoader
```

## 安全提示

本软件直接控制真实电机，机械臂动作快且有夹伤风险。请远离工作空间、保持急停可及、
新轨迹先低速试跑。软件不提供任何担保，详见 [LICENSE](LICENSE)。

## 参与贡献

欢迎提 Issue 和 PR，见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 许可证

MIT，见 [LICENSE](LICENSE)。

`web/vendor/` 中的第三方代码（three.js、OrbitControls、STLLoader）为 MIT 许可，
版权归各自作者所有。
