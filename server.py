#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Arm Teach Studio - 上位机后端服务

功能:
  - 通过串口与 ESP32 六轴机械臂通信 (UART 文本协议, 见 project3/HOST_INTERFACE_USAGE.md)
  - 读取各关节实时位置 / 速度 / 电流 / 力矩 / 温度
  - 使能 / 失能 / 急停 / 清除急停
  - 进入 / 退出拖动示教模式 (零力矩)
  - 记录手掰轨迹, numpy 平滑去噪, 一键复现 (可调速度)
  - 通过本地 HTTP + SSE 给前端 (three.js + URDF) 推送实时状态

仅依赖: 标准库 + pyserial + numpy (均已安装)。
本程序不修改 project3 工程, 只复用其串口协议并读取 apps/v2.5 的 URDF/网格。

启动:
    python server.py            # 默认 http://127.0.0.1:8000
    python server.py --port 8080
"""

import argparse
import json
import math
import os
import queue
import re
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import numpy as np

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None


# --------------------------------------------------------------------------
# 路径配置
# --------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
WEB_DIR = os.path.join(HERE, "web")

# 模型资源 (URDF + STL) 目录。本仓库不附带机械臂模型, 请通过以下任一方式指定:
#   1) 命令行:  python server.py --assets /path/to/robot_model
#   2) 环境变量: ARM_TEACH_ASSETS=/path/to/robot_model
#   3) 默认值:  同级目录 ../v2.5 (作者本地的 URDF 工程布局)
# 该目录内应有 urdf/*.urdf 与 meshes/*.STL。
DEFAULT_ASSET_DIR = os.path.normpath(os.path.join(HERE, "..", "v2.5"))
ASSET_DIR = os.path.normpath(os.environ.get("ARM_TEACH_ASSETS") or DEFAULT_ASSET_DIR)
# 前端加载的 URDF 相对路径 (相对 ASSET_DIR), 启动时自动探测。
URDF_REL = "urdf/v2.5.urdf"


def _detect_urdf(asset_dir):
    """在 asset_dir 中查找 URDF 文件, 返回相对路径 (找不到时返回 None)。"""
    for sub in ("urdf", "."):
        d = os.path.join(asset_dir, sub)
        if not os.path.isdir(d):
            continue
        names = sorted(n for n in os.listdir(d) if n.lower().endswith(".urdf"))
        if names:
            return os.path.join(sub, names[0]).replace("\\", "/").lstrip("./")
    return None

AXIS_COUNT = 6
DEFAULT_BAUD = 1000000

# 回零点: 六轴移动到 0 的速度。0.175 rad/s ≈ 10 °/s (固件速度字段为 °/s 整数)。
HOME_SPEED_DPS = max(1, int(round(0.175 * 180.0 / math.pi)))

# 轨迹收尾"落位"速度下限 (°/s)。
# END_SETTLE_DPS: 录制回放用 —— 录制轨迹末端仍带 ~15% 速度, 用它利落停住、消除随动误差。
END_SETTLE_DPS = 10

# 规划器轨迹收尾"利落落位"(开环, 无反馈 → 不受遥测滞后影响, 不会抖/不会突跳):
# 实机固件随动有滞后, 轨迹放完时真机仍落后端点一小段(亚度~几度)。读一次真机实际位置, 用一小段
# smootherstep 主动减速走到端点 —— 收尾减速到 0(不冲过端点)、主动到点(不拖尾)、速度平滑爬升(不突跳)。
SETTLE_PEAK_DPS = 8      # 收尾段峰值速度 (°/s): 调大→更利落(过大→末端像"小冲一下"); 调小→更柔和(过小→略拖尾)
SETTLE_TOL_DEG = 0.1     # 残余容差 (度): 小于它直接保持端点 (仿真无滞后→走这条)
SETTLE_MIN_S = 0.08      # 收尾段最短时长 (s)
SETTLE_MAX_S = 0.5       # 收尾段最长时长 (s), 防呆上限

# 流式回放的"目标前瞻"拍数: 目标点取 STREAM_LEAD 拍之后的位置(速度仍按真实节奏),
# 使固件始终够不到目标、不会"到点即停" → 过点不顿。值越大越不会停顿但拐角抄近一点点。
STREAM_LEAD = 2.5


# Windows 默认定时器粒度约 15.6ms, 会让 50Hz(20ms) 的 time.sleep 严重抖动 → 下发节拍不稳、
# 观感卡顿。复现期间临时把系统定时器分辨率提到 1ms, 使 sleep 准确、节拍平稳; 结束后还原。
try:
    import ctypes
    _WINMM = ctypes.WinDLL("winmm") if os.name == "nt" else None
except Exception:  # pragma: no cover
    _WINMM = None


def _hi_res_timer(on: bool):
    if _WINMM is None:
        return
    try:
        _WINMM.timeBeginPeriod(1) if on else _WINMM.timeEndPeriod(1)
    except Exception:
        pass


# --------------------------------------------------------------------------
# 协议: 帧封装与解析 (与 project3 固件一致的 XOR 校验)
# --------------------------------------------------------------------------
def xor_checksum(text: str) -> int:
    value = 0
    for ch in text:
        value ^= ord(ch)
    return value & 0xFF


def build_frame(body: str) -> bytes:
    return f"${body}*{xor_checksum(body):02X}\r\n".encode("ascii")


def split_frame(line: str):
    line = line.strip()
    if not line.startswith("$"):
        return None
    payload = line[1:]
    if "*" in payload:
        body, crc_text = payload.rsplit("*", 1)
        try:
            expected = int(crc_text[:2], 16)
        except ValueError:
            return None
        if xor_checksum(body) != expected:
            return None
    else:
        body = payload
    return body.split(",")


# --------------------------------------------------------------------------
# 全局状态
# --------------------------------------------------------------------------
class ArmState:
    """线程安全的机械臂状态快照, 供 SSE 推送。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.connected = False
        self.port = ""
        self.baud = DEFAULT_BAUD
        self.estop = 0
        self.arm_moving = 0
        self.teach = False
        self.recording = False
        self.replaying = False
        self.replay_progress = 0.0
        self.record_count = 0
        self.last_ack = ""
        self.last_err = ""
        self.status = "未连接"
        self.axes = [self._empty_axis(i + 1) for i in range(AXIS_COUNT)]
        self.fb_ts = 0.0
        self.device_log = deque(maxlen=400)

    @staticmethod
    def _empty_axis(axis):
        return {
            "axis": axis,
            "online": 0,
            "pos_cdeg": 0,
            "vel_dps": 0,
            "current_a": 0.0,
            "torque_nm": 0.0,
            "temp_c": 0,
            "moving": 0,
        }

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "connected": self.connected,
                "port": self.port,
                "baud": self.baud,
                "estop": self.estop,
                "arm_moving": self.arm_moving,
                "teach": self.teach,
                "recording": self.recording,
                "replaying": self.replaying,
                "replay_progress": round(self.replay_progress, 3),
                "record_count": self.record_count,
                "last_ack": self.last_ack,
                "last_err": self.last_err,
                "status": self.status,
                "axes": [dict(a) for a in self.axes],
                "fb_ts": self.fb_ts,
                "log": list(self.device_log)[-80:],
                "ts": time.time(),
            }

    _ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")

    def add_log(self, line: str):
        """记录固件的非协议输出 (ESP_LOG, 与 ARM 协议共用 UART0)。"""
        line = self._ANSI_RE.sub("", line).rstrip()
        if not line:
            return
        entry = time.strftime("%H:%M:%S  ") + line
        with self.lock:
            self.device_log.append(entry)
        print("[ESP]", line, flush=True)


STATE = ArmState()


# --------------------------------------------------------------------------
# 串口管理
# --------------------------------------------------------------------------
class ArmSerial:
    def __init__(self):
        self.port = None
        self.thread = None
        self.running = False
        self.write_lock = threading.Lock()
        # 记录缓冲 (在记录模式下追加 FB 关节角)
        self.record_lock = threading.Lock()
        self.recording = False
        self.record_frames = []  # list[(t, [6] pos_cdeg)]
        self.record_t0 = 0.0

    # ---- 连接 ----
    def connect(self, port_name: str, baud: int):
        if serial is None:
            raise RuntimeError("未安装 pyserial, 请运行: pip install pyserial")
        self.disconnect()
        self.port = serial.Serial(port_name, baudrate=baud, timeout=0.05)
        self.running = True
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()
        with STATE.lock:
            STATE.connected = True
            STATE.port = port_name
            STATE.baud = baud
            STATE.status = f"已连接 {port_name} @ {baud}"
        self.send_body("ARM,GET")

    def disconnect(self):
        self.running = False
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
        self.port = None
        with STATE.lock:
            STATE.connected = False
            STATE.status = "未连接"

    def is_connected(self) -> bool:
        return self.port is not None and self.port.is_open

    # ---- 发送 ----
    def send_body(self, body: str) -> bool:
        if not self.is_connected():
            return False
        try:
            with self.write_lock:
                self.port.write(build_frame(body))
            return True
        except Exception as exc:
            with STATE.lock:
                STATE.last_err = f"写串口失败: {exc}"
            return False

    # ---- 记录控制 ----
    def start_recording(self):
        with self.record_lock:
            self.record_frames = []
            self.record_t0 = time.monotonic()
            self.recording = True
        with STATE.lock:
            STATE.recording = True
            STATE.record_count = 0

    def stop_recording(self):
        with self.record_lock:
            self.recording = False
            frames = list(self.record_frames)
        with STATE.lock:
            STATE.recording = False
            STATE.record_count = len(frames)
        return frames

    def get_recorded(self):
        with self.record_lock:
            return list(self.record_frames)

    # ---- 读取线程 ----
    def _reader_loop(self):
        buffer = bytearray()
        err_count = 0
        while self.running and self.port is not None:
            try:
                data = self.port.read(128)
                err_count = 0
            except Exception as exc:
                # Windows 下 USB 转串口驱动偶发让 ClearCommError 抛 PermissionError(13)，
                # 多为设备/驱动瞬时抖动，下一次读即恢复，不应一次就断开。
                # 仅在端口真的关闭、或连续多次失败(约1秒)时才判定掉线。
                err_count += 1
                port_open = self.port is not None and self.port.is_open
                with STATE.lock:
                    STATE.last_err = f"串口读取异常(已忽略x{err_count}): {exc}"
                if not port_open or err_count >= 20:
                    with STATE.lock:
                        STATE.connected = False
                        STATE.status = "串口断开"
                    break
                time.sleep(0.05)
                continue
            if not data:
                continue
            for byte in data:
                if byte in (10, 13):
                    if buffer:
                        self._handle_line(buffer.decode("ascii", errors="ignore"))
                        buffer.clear()
                elif len(buffer) < 512:
                    buffer.append(byte)
                else:
                    buffer.clear()

    def _handle_line(self, line: str):
        tokens = split_frame(line)
        if not tokens or len(tokens) < 2 or tokens[0] != "ARM":
            # 非 ARM 协议行: 多为固件 ESP_LOG 输出 (与协议共用 UART0), 采集以便排查
            STATE.add_log(line)
            return
        kind = tokens[1]
        if kind == "FB":
            self._handle_feedback(tokens)
        elif kind == "ACK":
            with STATE.lock:
                STATE.last_ack = ",".join(tokens[1:])
        elif kind == "ERR":
            with STATE.lock:
                STATE.last_err = "ERR:" + ",".join(tokens[2:])

    def _handle_feedback(self, tokens):
        if len(tokens) < 4 + AXIS_COUNT * 8:
            return
        try:
            estop = int(tokens[2])
            arm_moving = int(tokens[3])
        except ValueError:
            return
        axes = []
        pos_vec = [0] * AXIS_COUNT
        index = 4
        for _ in range(AXIS_COUNT):
            try:
                axis = int(tokens[index])
                a = {
                    "axis": axis,
                    "online": int(tokens[index + 1]),
                    "pos_cdeg": int(tokens[index + 2]),
                    "vel_dps": int(tokens[index + 3]),
                    "current_a": float(tokens[index + 4]),
                    "torque_nm": float(tokens[index + 5]),
                    "temp_c": int(tokens[index + 6]),
                    "moving": int(tokens[index + 7]),
                }
            except (ValueError, IndexError):
                return
            axes.append(a)
            if 1 <= axis <= AXIS_COUNT:
                pos_vec[axis - 1] = a["pos_cdeg"]
            index += 8

        axes.sort(key=lambda x: x["axis"])
        with STATE.lock:
            STATE.estop = estop
            STATE.arm_moving = arm_moving
            STATE.axes = axes
            STATE.fb_ts = time.time()

        # 记录
        with self.record_lock:
            if self.recording:
                t = time.monotonic() - self.record_t0
                self.record_frames.append((t, list(pos_vec)))
                count = len(self.record_frames)
            else:
                count = None
        if count is not None:
            with STATE.lock:
                STATE.record_count = count


LINK = ArmSerial()


# --------------------------------------------------------------------------
# 轨迹平滑 (numpy, 无需 scipy)
# --------------------------------------------------------------------------
def _hampel(a: np.ndarray, win: int, n_sigma: float = 3.0) -> np.ndarray:
    """逐列 Hampel 滤波: 用滑动中值 + MAD 鲁棒地识别并替换离群点(手抖突变)。"""
    if win < 3:
        return a
    if win % 2 == 0:
        win += 1
    h = win // 2
    n = a.shape[0]
    out = a.copy()
    for i in range(n):
        lo = max(0, i - h)
        hi = min(n, i + h + 1)
        seg = a[lo:hi]
        med = np.median(seg, axis=0)
        mad = np.median(np.abs(seg - med), axis=0)
        thresh = n_sigma * 1.4826 * mad
        dev = np.abs(a[i] - med)
        out[i] = np.where(dev > np.maximum(thresh, 1e-9), med, a[i])
    return out


def _savgol(a: np.ndarray, win: int, poly: int = 3) -> np.ndarray:
    """逐列 Savitzky-Golay 平滑: 滑动窗口内最小二乘多项式拟合, 比滑动平均更贴合
    曲线形状且一阶/二阶导更平滑(速度更连续)。numpy 实现, 无需 scipy。"""
    if win < 3:
        return a
    if win % 2 == 0:
        win += 1
    if win <= poly:
        poly = max(1, win - 1)
    half = win // 2
    x = np.arange(-half, half + 1, dtype=float)
    A = np.vander(x, poly + 1, increasing=True)        # (win, poly+1)
    kernel = np.linalg.pinv(A)[0]                       # 中心点拟合值的权重 (win,)
    pad = np.pad(a, ((half, half), (0, 0)), mode="edge")
    k = kernel[::-1]
    out = np.empty_like(a)
    for j in range(a.shape[1]):
        out[:, j] = np.convolve(pad[:, j], k, "valid")
    return out


def _cubic_spline(x: np.ndarray, y: np.ndarray, xq: np.ndarray) -> np.ndarray:
    """自然三次样条插值(Thomas 算法解三对角), 用于把平滑后的轨迹加密成更细的曲线。"""
    n = len(x)
    if n < 3:
        return np.interp(xq, x, y)
    h = np.diff(x)
    lower = np.zeros(n)
    diag = np.ones(n)
    upper = np.zeros(n)
    rhs = np.zeros(n)
    for i in range(1, n - 1):
        lower[i] = h[i - 1]
        diag[i] = 2.0 * (h[i - 1] + h[i])
        upper[i] = h[i]
        rhs[i] = 6.0 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1])
    # Thomas 前向消元 + 回代 (自然边界: M0 = M_{n-1} = 0)
    cp = np.zeros(n)
    dp = np.zeros(n)
    cp[0] = 0.0
    dp[0] = 0.0
    for i in range(1, n):
        m = diag[i] - lower[i] * cp[i - 1]
        cp[i] = upper[i] / m if m != 0 else 0.0
        dp[i] = (rhs[i] - lower[i] * dp[i - 1]) / m if m != 0 else 0.0
    M = np.zeros(n)
    M[-1] = dp[-1]
    for i in range(n - 2, -1, -1):
        M[i] = dp[i] - cp[i] * M[i + 1]
    idx = np.clip(np.searchsorted(x, xq) - 1, 0, n - 2)
    yq = np.empty(len(xq), dtype=float)
    for k2 in range(len(xq)):
        i = idx[k2]
        hi = h[i]
        xr = x[i + 1] - xq[k2]
        xl = xq[k2] - x[i]
        yq[k2] = (M[i] * xr ** 3 / (6 * hi) + M[i + 1] * xl ** 3 / (6 * hi)
                  + (y[i] / hi - M[i] * hi / 6.0) * xr
                  + (y[i + 1] / hi - M[i + 1] * hi / 6.0) * xl)
    return yq


def smooth_trajectory(frames, median_win=7, smooth_win=21, resample_dt=0.05):
    """
    frames: list[(t_seconds, [6] pos_cdeg)]
    流程: 解缠绕 -> 均匀重采样 -> Hampel 去离群(手抖/突变) -> Savitzky-Golay 曲线拟合
          -> 三次样条加密插值。
    返回: { t, pos(=wrapped 兼容前端), wrapped(0..35999 整数), cont(连续未回绕 cdeg 浮点) }
    cont 供复现端连续插值(避免 0/360 跳变); wrapped 供前端画曲线。
    median_win 作 Hampel 窗口, smooth_win 作 Savitzky-Golay 窗口。
    """
    if len(frames) < 2:
        pos = [f[1] for f in frames]
        t = [f[0] for f in frames]
        cont = [[float(v) for v in row] for row in pos]
        return {"t": t, "pos": pos, "wrapped": pos, "cont": cont}

    times = np.array([f[0] for f in frames], dtype=float)
    pos_cdeg = np.array([f[1] for f in frames], dtype=float)  # (N,6)

    order = np.argsort(times)
    times = times[order]
    pos_cdeg = pos_cdeg[order]
    keep = np.concatenate(([True], np.diff(times) > 1e-6))
    times = times[keep]
    pos_cdeg = pos_cdeg[keep]

    # 连续未回绕 cdeg (处理 0/360 跨越)
    rad = np.unwrap(np.deg2rad(pos_cdeg / 100.0), axis=0)
    cont_in = np.rad2deg(rad) * 100.0

    t0, t1 = float(times[0]), float(times[-1])
    if resample_dt <= 0:
        resample_dt = 0.05
    n_in = max(3, int(round((t1 - t0) / resample_dt)) + 1)
    ut = np.linspace(t0, t1, n_in)
    res = np.empty((n_in, AXIS_COUNT))
    for j in range(AXIS_COUNT):
        res[:, j] = np.interp(ut, times, cont_in[:, j])

    res = _hampel(res, int(median_win))
    res = _savgol(res, int(smooth_win))

    # 三次样条加密到细网格 (~40Hz), 让曲线与运动更平滑
    out_dt = 0.025
    n_out = max(n_in, int(round((t1 - t0) / out_dt)) + 1)
    ft = np.linspace(t0, t1, n_out)
    fine = np.empty((n_out, AXIS_COUNT))
    for j in range(AXIS_COUNT):
        fine[:, j] = _cubic_spline(ut, res[:, j], ft)

    wrapped = np.mod(np.round(fine), 36000).astype(int)
    t_rel = [round(float(x - ft[0]), 4) for x in ft]
    return {
        "t": t_rel,
        "pos": wrapped.tolist(),       # 前端画曲线用
        "wrapped": wrapped.tolist(),
        "cont": fine.tolist(),         # 复现连续插值用
    }


# 保存当前已平滑的轨迹, 供复现使用
SMOOTHED = {"t": [], "cont": [], "wrapped": []}
SMOOTH_LOCK = threading.Lock()


def _smootherstep(s: float) -> float:
    """5 阶平滑插值: 起止处一阶/二阶导均为 0, 速度无突变。"""
    if s <= 0.0:
        return 0.0
    if s >= 1.0:
        return 1.0
    return s * s * s * (s * (s * 6.0 - 15.0) + 10.0)


def _unwrap_cdeg(pos):
    """把 [[6]cdeg] 解缠绕成连续 cdeg (用于复现端连续插值, 避免 0/360 跳变)。"""
    arr = np.array(pos, dtype=float)
    rad = np.unwrap(np.deg2rad(arr / 100.0), axis=0)
    return np.rad2deg(rad) * 100.0


# --------------------------------------------------------------------------
# 轨迹复现 (回放)
# --------------------------------------------------------------------------
class Replayer:
    def __init__(self):
        self.thread = None
        self.stop_flag = threading.Event()
        self.actual_log = []   # [[t_rel(s), [6]pos_cdeg], ...] 本次运行各电机实际关节角
        self.plan_meta = None  # 自动运行时的规划数据(航点+稠密轨迹), 收尾时与 actual 一并落盘

    def is_running(self):
        return self.thread is not None and self.thread.is_alive()

    def _read_actual_cdeg(self):
        """读当前各电机实际关节角 (单圈 cdeg 0..35999), 与规划 pos 同口径。"""
        out = [0.0] * AXIS_COUNT
        with STATE.lock:
            for a in STATE.axes:
                j = a["axis"] - 1
                if 0 <= j < AXIS_COUNT:
                    out[j] = float(a["pos_cdeg"])
        return out

    def start(self, speed_factor=1.0):
        if self.is_running():
            return False
        with SMOOTH_LOCK:
            t = list(SMOOTHED["t"])
            cont = [list(p) for p in SMOOTHED["cont"]]
        if len(cont) < 2:
            return False
        if not LINK.is_connected():
            return False
        self.plan_meta = None       # 录制回放不落盘规划
        self.actual_log = []
        self.stop_flag.clear()
        self.thread = threading.Thread(
            target=self._run, args=(t, cont, max(0.05, float(speed_factor))), daemon=True
        )
        self.thread.start()
        return True

    def stop(self):
        self.stop_flag.set()

    def start_explicit(self, t, pos, speed_factor=1.0, do_approach=True, plan=None):
        """运行外部(规划器)给定的关节轨迹: t 秒, pos 为 [[6]cdeg]。
        plan: 可选规划元数据(航点+稠密轨迹), 运行结束后与实际反馈一并落盘。"""
        if self.is_running():
            return False
        if not LINK.is_connected():
            return False
        if not pos or len(pos) < 2:
            return False
        cont = _unwrap_cdeg(pos).tolist()
        self.plan_meta = plan
        self.actual_log = []
        self.stop_flag.clear()
        self.thread = threading.Thread(
            target=self._run, args=(list(t), cont, max(0.05, float(speed_factor)), bool(do_approach)), daemon=True
        )
        self.thread.start()
        return True

    DT = 0.02  # 下发节拍 (s) ≈ 50Hz: 比 20Hz 更细, 拐点处固件始终有"下一拍目标"可追, 不到点空等

    def _send(self, target, spd_from, spd_to, dt, min_spd=1):
        """下发一帧 ARM,ALL: 各轴目标=target(连续 cdeg), 速度=|spd_to-spd_from|/dt。
        把"目标"与"速度参考"解耦: 目标可比速度参考点更靠前(前瞻), 使固件始终在朝前追、
        到不了目标就不会停 → 过点(轨迹瞬时变慢处)不再因"到点即停"而顿一下。
        min_spd 抬高速度下限(收尾落位用), 让固件迅速消除随动误差而非低速爬行。"""
        fields = []
        for j in range(AXIS_COUNT):
            ang = int(round(target[j])) % 36000
            spd = abs(spd_to[j] - spd_from[j]) / 100.0 / dt    # deg/s
            spd = int(max(min_spd, min(2000, round(spd))))
            fields.append(str(ang))
            fields.append(str(spd))
        LINK.send_body("ARM,ALL," + ",".join(fields))

    def _send_step(self, posv, prev, dt, min_spd=1):
        """目标即速度参考点(无前瞻)的常规下发, 用于接近起点/录制回放/收尾落位。"""
        self._send(posv, prev, posv, dt, min_spd)

    def _run(self, ts, cont, speed_factor, do_approach=True):
        ts = np.array(ts, dtype=float)
        cont = np.array(cont, dtype=float)            # (N,6) 连续未回绕 cdeg
        dur = float(ts[-1])
        DT = self.DT
        with STATE.lock:
            STATE.replaying = True
            STATE.replay_progress = 0.0
            STATE.status = f"复现: 移动到轨迹起点 (x{speed_factor:.2f})"
        # 仅在确有需要时才退出示教/清急停, 避免无谓的"切模式(下电→使能)"造成下坠抖动
        with STATE.lock:
            was_teach = STATE.teach
            was_estop = STATE.estop
        if was_teach:
            # 退出示教前先读当前位姿, 退出后立刻下发"保持当前位姿", 让电机在
            # 模式切换瞬间主动锁位, 减少重力造成的"突然下垂"。
            hold = [0.0] * AXIS_COUNT
            with STATE.lock:
                for a in STATE.axes:
                    j = a["axis"] - 1
                    if 0 <= j < AXIS_COUNT:
                        hold[j] = float(a["pos_cdeg"])
            hold = np.array(hold, dtype=float)
            LINK.send_body("ARM,TEACH,0")
            self._send_step(hold, hold, DT)   # 立即锁定当前位姿
            time.sleep(0.25)   # 等退出示教并保持当前位姿稳定
        if was_estop:
            LINK.send_body("ARM,CLEAR")
            time.sleep(0.1)
        _hi_res_timer(True)   # 复现期间提高定时器分辨率, 稳住下发节拍
        try:
            # 当前真实位置作为起点 (单圈 cdeg)
            cur = [0.0] * AXIS_COUNT
            with STATE.lock:
                for a in STATE.axes:
                    j = a["axis"] - 1
                    if 0 <= j < AXIS_COUNT:
                        cur[j] = float(a["pos_cdeg"])
            start = np.array(cur, dtype=float)
            # 轨迹起点对齐到当前位置附近(每轴取最短等价角), 保证插值连续
            target0 = cont[0].copy()
            for j in range(AXIS_COUNT):
                while target0[j] - start[j] > 18000:
                    target0[j] -= 36000
                while target0[j] - start[j] < -18000:
                    target0[j] += 36000
            offset = target0 - cont[0]
            prev = target0.copy()

            # do_approach=False 时(规划器已把"当前位姿→第0点"作为笛卡尔直线放进轨迹里),
            # 跳过这套关节空间的"回起点", 避免甩/坠, 直接走笛卡尔轨迹。
            if do_approach:
                # ---- 移动到轨迹起点: 从当前真实位姿利落直达第0点 (smootherstep, 起止速度为 0) ----
                # 不做"回零点", 也不再用阻塞式到位校验 —— 该校验在硬件死区/遥测
                # 延迟下几乎必然超时误报, 且其期间会以最低速(1°/s)爬行。抵达起点后
                # 直接进入复现, 由复现阶段的 ramp-in 平滑起步, 残余小误差自然被吸收。
                max_delta_deg = float(np.max(np.abs(target0 - start))) / 100.0
                Ta = min(3.0, max(0.4, max_delta_deg / 120.0))     # 约 120 deg/s 的利落接近
                steps = max(1, int(round(Ta / DT)))
                prev = start.copy()
                for i in range(1, steps + 1):
                    if self.stop_flag.is_set():
                        return
                    s = _smootherstep(i / steps)
                    posv = start + (target0 - start) * s
                    self._send_step(posv, prev, DT)
                    prev = posv
                    time.sleep(DT)
                # 抵达起点后短暂稳定 (不做带超时的位置校验, 避免误报)
                self._send_step(target0, prev, DT)
                prev = target0.copy()
                time.sleep(0.15)
                if self.stop_flag.is_set():
                    return

            # ---- 阶段1: 复现轨迹 ----
            with STATE.lock:
                STATE.status = f"复现轨迹中 (x{speed_factor:.2f})"
            rec_t0 = time.monotonic()   # 实际反馈记录的时间基准 (与规划 t 同起点)
            rec = self.plan_meta is not None   # 仅自动运行(带规划)时记录实际关节角
            if do_approach:
                # 录制轨迹: 时间轴来自手动示教(未经速度规划), 用线性(梯形)首尾加减速包络
                # 柔化起步/收尾。线性减速是恒定减速度, 不像 S 曲线那样长时间贴最低速爬行;
                # 末点再由下方落位精确停住。
                Tr_in = min(0.5, dur * 0.15)                   # 起步加速段 (轨迹时间)
                Tr_out = min(0.3, dur * 0.1)                   # 收尾减速段 (短, 避免拖沓)
                R_MIN = 0.15                                   # 端点最低速度比例 (配合末尾落位)
                tt = 0.0
                while tt < dur and not self.stop_flag.is_set():
                    ri = R_MIN + (1.0 - R_MIN) * min(1.0, tt / Tr_in) if Tr_in > 1e-6 else 1.0
                    ro = R_MIN + (1.0 - R_MIN) * min(1.0, (dur - tt) / Tr_out) if Tr_out > 1e-6 else 1.0
                    r = min(ri, ro)
                    tt += r * speed_factor * DT
                    ttc = min(tt, dur)
                    posv = np.array([np.interp(ttc, ts, cont[:, j]) for j in range(AXIS_COUNT)]) + offset
                    self._send_step(posv, prev, DT)
                    prev = posv
                    if rec:
                        self.actual_log.append([round(time.monotonic() - rec_t0, 4), self._read_actual_cdeg()])
                    with STATE.lock:
                        STATE.replay_progress = ttc / dur if dur > 0 else 1.0
                    time.sleep(DT)
            else:
                # 规划器轨迹: 前端已把整条路径(到下一个停顿点为止)规划成一条连续轨迹,
                # 速度/加速度受限、过点速度连续。这里按其时间轴回放(仅 speed_factor 缩放):
                #   1) 目标前瞻、速度按真实一拍节奏 → 固件始终够不到目标、不会"到点即停",
                #      过点(轨迹瞬时变慢处)不再顿一下;
                #   2) 前瞻量临近终点平滑收敛到 0 → 目标精确落到端点、配合柔和落位, 按规划减速
                #      平滑停住, 不会因末端残余或落位速度突跳而"冲一下再停"(收尾顿挫);
                #   3) 实时时钟驱动 → 不受 Windows sleep 抖动影响, 节拍稳。
                def sample(tq):
                    return np.array([np.interp(min(dur, tq), ts, cont[:, j]) for j in range(AXIS_COUNT)]) + offset
                one = speed_factor * DT
                tt = 0.0
                last = time.monotonic()
                deadline = last
                while tt < dur and not self.stop_flag.is_set():
                    now = time.monotonic()
                    dt_real = now - last
                    last = now
                    tt += speed_factor * dt_real
                    pace = sample(tt + one)                        # 速度参考: 真实一拍后应到的位置
                    # 前瞻量: 中途领先 STREAM_LEAD 拍(过点不停); 临近终点平滑收敛到 0(目标精确
                    # 落到端点, 不再留 1 拍残余让收尾去"追") → 配合下方柔和落位, 收尾不冲。
                    lead = min(STREAM_LEAD * one, 0.5 * max(0.0, dur - tt))
                    target = sample(tt + lead)
                    self._send(target, prev, pace, DT)
                    prev = pace
                    if rec:
                        self.actual_log.append([round(time.monotonic() - rec_t0, 4), self._read_actual_cdeg()])
                    with STATE.lock:
                        STATE.replay_progress = min(tt, dur) / dur if dur > 0 else 1.0
                    deadline += DT
                    sl = deadline - time.monotonic()
                    if sl > 0:
                        time.sleep(sl)
                    else:
                        deadline = time.monotonic()                # 落后了, 重置节拍基准

            # ---- 收尾落位 ----
            if not self.stop_flag.is_set():
                end = cont[-1] + offset
                if do_approach:
                    # 录制回放: 末端仍有 ~15% 速度 → 用利落落位精确停住、消除随动误差。
                    self._send_step(end, prev, DT, min_spd=END_SETTLE_DPS)
                else:
                    # 规划器轨迹: 真机随动滞后 → 轨迹放完时仍落后端点一小段。读一次真机实际位置, 用一小段
                    # smootherstep 开环减速走到端点: 无反馈环(不抖)、收尾减速到 0(不冲过端点)、主动到点(不拖尾)、
                    # 速度从小平滑爬到峰值再回零(不突跳)。仿真无滞后 → 残余在容差内 → 跳过, 直接保持端点。
                    act = [0.0] * AXIS_COUNT
                    with STATE.lock:
                        for a in STATE.axes:
                            j = a["axis"] - 1
                            if 0 <= j < AXIS_COUNT:
                                act[j] = float(a["pos_cdeg"])
                    act = np.array(act, dtype=float)
                    for j in range(AXIS_COUNT):              # 单圈反馈对齐到 end 附近 (每轴取最短等价角)
                        while end[j] - act[j] > 18000:
                            act[j] += 36000
                        while end[j] - act[j] < -18000:
                            act[j] -= 36000
                    resid_deg = float(np.max(np.abs(end - act))) / 100.0
                    if resid_deg > SETTLE_TOL_DEG:
                        # 选时长使 smootherstep 峰值速度 ≈ SETTLE_PEAK_DPS (峰值 = 1.875·距离/时长)
                        Tf = min(SETTLE_MAX_S, max(SETTLE_MIN_S, 1.875 * resid_deg / SETTLE_PEAK_DPS))
                        steps = max(1, int(round(Tf / DT)))
                        base = act.copy()
                        prev = base.copy()   # 速度从实际位置算起 → 首拍速度≈0(smootherstep 起点), 不突跳
                        for i in range(1, steps + 1):
                            if self.stop_flag.is_set():
                                break
                            s = _smootherstep(i / steps)
                            posv = base + (end - base) * s
                            self._send_step(posv, prev, DT)
                            prev = posv
                            if rec:
                                self.actual_log.append([round(time.monotonic() - rec_t0, 4), self._read_actual_cdeg()])
                            time.sleep(DT)
                    self._send_step(end, prev, DT, min_spd=1)        # 精确保持端点
                    prev = end
                    if rec:
                        self.actual_log.append([round(time.monotonic() - rec_t0, 4), self._read_actual_cdeg()])
                with STATE.lock:
                    STATE.replay_progress = 1.0
        finally:
            _hi_res_timer(False)
            with STATE.lock:
                STATE.replaying = False
                STATE.status = "复现结束" if not self.stop_flag.is_set() else "复现已停止"
            # 落盘: 规划数据 + 各电机实际关节角 (同一文件, 同一时间轴, 便于对齐对比)
            if self.plan_meta is not None:
                try:
                    out = dict(self.plan_meta)
                    out["actual"] = self.actual_log
                    out["stopped"] = bool(self.stop_flag.is_set())
                    path = _write_plan_dump(out)
                    print(f"[dump] 规划+实际已写入: {path}  (实际采样 {len(self.actual_log)} 拍)")
                except Exception as exc:
                    print(f"[dump] 写入失败: {exc}")


REPLAYER = Replayer()


# --------------------------------------------------------------------------
# HTTP 处理
# --------------------------------------------------------------------------
MIME = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".urdf": "application/xml; charset=utf-8",
    ".xml": "application/xml; charset=utf-8",
    ".stl": "application/octet-stream",
    ".png": "image/png",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
}


def _safe_join(base, rel):
    rel = rel.lstrip("/")
    path = os.path.normpath(os.path.join(base, rel))
    if not path.startswith(os.path.normpath(base)):
        return None
    return path


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass  # 静默

    # ---- 工具 ----
    def _send_json(self, obj, code=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, path):
        if path is None or not os.path.isfile(path):
            self.send_error(404, "Not Found")
            return
        ext = os.path.splitext(path)[1].lower()
        ctype = MIME.get(ext, "application/octet-stream")
        try:
            with open(path, "rb") as f:
                data = f.read()
        except OSError:
            self.send_error(404, "Not Found")
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _read_body(self):
        length = int(self.headers.get("Content-Length", 0) or 0)
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode("utf-8"))
        except Exception:
            return {}

    # ---- GET ----
    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path == "/" or path == "":
            self._send_file(os.path.join(WEB_DIR, "index.html"))
            return
        if path == "/events":
            self._serve_sse()
            return
        if path == "/api/ports":
            self._send_json({"ports": list_serial_ports()})
            return
        if path == "/api/config":
            self._send_json({
                "urdf": "/assets/" + URDF_REL,
                "assets_ok": os.path.isfile(os.path.join(ASSET_DIR, URDF_REL)),
            })
            return
        if path.startswith("/assets/"):
            self._send_file(_safe_join(ASSET_DIR, path[len("/assets/"):]))
            return
        # 静态文件
        self._send_file(_safe_join(WEB_DIR, path))

    # ---- POST ----
    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        body = self._read_body()
        try:
            result = dispatch_api(path, body)
        except Exception as exc:
            self._send_json({"ok": False, "error": str(exc)}, 500)
            return
        if result is None:
            self.send_error(404, "Unknown API")
            return
        self._send_json(result)

    # ---- SSE ----
    def _serve_sse(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            while True:
                snap = STATE.snapshot()
                msg = "data: " + json.dumps(snap) + "\n\n"
                self.wfile.write(msg.encode("utf-8"))
                self.wfile.flush()
                time.sleep(0.05)  # ~20 Hz
        except (BrokenPipeError, ConnectionResetError, OSError):
            return


# --------------------------------------------------------------------------
# API 分发
# --------------------------------------------------------------------------
def list_serial_ports():
    if list_ports is None:
        return []
    return [{"device": p.device, "desc": p.description} for p in list_ports.comports()]


def _clamp_speed(v):
    try:
        return max(1, min(2000, int(v)))
    except (TypeError, ValueError):
        return 120


def _write_plan_dump(obj):
    """把一次运行的数据写入 plan_dumps/plan_<时间戳>.json, 返回完整路径。"""
    out_dir = os.path.join(HERE, "plan_dumps")
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S") + f"_{int(time.time() * 1000) % 1000:03d}"
    fpath = os.path.join(out_dir, f"plan_{stamp}.json")
    with open(fpath, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)
    return fpath


def dispatch_api(path: str, body: dict):
    if path == "/api/ports":
        return {"ok": True, "ports": list_serial_ports()}

    if path == "/api/connect":
        port = body.get("port", "")
        baud = int(body.get("baud", DEFAULT_BAUD))
        if not port:
            return {"ok": False, "error": "未选择串口"}
        try:
            LINK.connect(port, baud)
        except Exception as exc:
            return {"ok": False, "error": str(exc)}
        return {"ok": True}

    if path == "/api/disconnect":
        REPLAYER.stop()
        LINK.disconnect()
        return {"ok": True}

    if path == "/api/enable":
        # 使能: 清除急停并恢复位置速度模式
        ok = LINK.send_body("ARM,CLEAR")
        with STATE.lock:
            STATE.teach = False
        return {"ok": ok}

    if path == "/api/disable":
        # 失能: 急停 (停止所有轴, 拒绝运动命令)
        ok = LINK.send_body("ARM,ESTOP")
        return {"ok": ok}

    if path == "/api/estop":
        REPLAYER.stop()
        ok = LINK.send_body("ARM,ESTOP")
        return {"ok": ok}

    if path == "/api/clear":
        ok = LINK.send_body("ARM,CLEAR")
        with STATE.lock:
            STATE.teach = False
        return {"ok": ok}

    if path == "/api/home":
        # 回零点: 仅在确有需要时才退出示教/清急停(避免无谓切模式下坠), 再六轴移动到 0
        REPLAYER.stop()
        with STATE.lock:
            was_teach = STATE.teach
            was_estop = STATE.estop
        if was_teach:
            LINK.send_body("ARM,TEACH,0")
            time.sleep(0.25)
        if was_estop:
            LINK.send_body("ARM,CLEAR")
            time.sleep(0.1)
        with STATE.lock:
            STATE.teach = False
        fields = []
        for _ in range(AXIS_COUNT):
            fields.append("0")
            fields.append(str(HOME_SPEED_DPS))
        ok = LINK.send_body("ARM,ALL," + ",".join(fields))
        return {"ok": ok, "speed_dps": HOME_SPEED_DPS}

    if path == "/api/get":
        ok = LINK.send_body("ARM,GET")
        return {"ok": ok}

    if path == "/api/teach":
        on = bool(body.get("on", True))
        if on:
            REPLAYER.stop()
        ok = LINK.send_body("ARM,TEACH,1" if on else "ARM,TEACH,0")
        with STATE.lock:
            STATE.teach = on
        return {"ok": ok}

    if path == "/api/set_axis":
        axis = int(body.get("axis", 1))
        angle_cdeg = int(body.get("angle_cdeg", 0)) % 36000
        speed = _clamp_speed(body.get("speed", 120))
        if not (1 <= axis <= AXIS_COUNT):
            return {"ok": False, "error": "轴号超范围"}
        ok = LINK.send_body(f"ARM,SET,{axis},{angle_cdeg},{speed}")
        return {"ok": ok}

    if path == "/api/set_all":
        angles = body.get("angles_cdeg", [])
        speed = _clamp_speed(body.get("speed", 120))
        if len(angles) != AXIS_COUNT:
            return {"ok": False, "error": "需要 6 个角度"}
        fields = []
        for a in angles:
            fields.append(str(int(a) % 36000))
            fields.append(str(speed))
        ok = LINK.send_body("ARM,ALL," + ",".join(fields))
        return {"ok": ok}

    if path == "/api/stop_axis":
        axis = int(body.get("axis", 1))
        ok = LINK.send_body(f"ARM,STOP,{axis}")
        return {"ok": ok}

    # ---- 记录 / 平滑 / 复现 ----
    if path == "/api/record/start":
        LINK.start_recording()
        return {"ok": True}

    if path == "/api/record/stop":
        frames = LINK.stop_recording()
        return {
            "ok": True,
            "raw": {
                "t": [round(f[0], 4) for f in frames],
                "pos": [f[1] for f in frames],
            },
        }

    if path == "/api/record/clear":
        LINK.start_recording()
        LINK.stop_recording()
        with SMOOTH_LOCK:
            SMOOTHED["t"] = []
            SMOOTHED["cont"] = []
            SMOOTHED["wrapped"] = []
        with STATE.lock:
            STATE.record_count = 0
        return {"ok": True}

    if path == "/api/smooth":
        median_win = int(body.get("median_win", 7))
        smooth_win = int(body.get("smooth_win", 21))
        resample_dt = float(body.get("resample_dt", 0.05))
        frames = LINK.get_recorded()
        if len(frames) < 2:
            return {"ok": False, "error": "没有可平滑的轨迹, 请先记录"}
        smoothed = smooth_trajectory(frames, median_win, smooth_win, resample_dt)
        with SMOOTH_LOCK:
            SMOOTHED["t"] = smoothed["t"]
            SMOOTHED["cont"] = smoothed["cont"]
            SMOOTHED["wrapped"] = smoothed["wrapped"]
        return {
            "ok": True,
            "raw": {
                "t": [round(f[0], 4) for f in frames],
                "pos": [f[1] for f in frames],
            },
            # 只回传画曲线所需的 t/pos(wrapped), 不回传体积大的 cont
            "smoothed": {"t": smoothed["t"], "pos": smoothed["pos"]},
        }

    if path == "/api/replay/start":
        speed = float(body.get("speed_factor", 1.0))
        ok = REPLAYER.start(speed)
        if not ok:
            return {"ok": False, "error": "无法复现: 未连接 / 无平滑轨迹 / 正在复现"}
        return {"ok": True}

    if path == "/api/replay/stop":
        REPLAYER.stop()
        return {"ok": True}

    if path == "/api/run_traj":
        # 运行规划器生成的关节轨迹 (含缓慢回到起点 + 平滑加减速)
        t = body.get("t", [])
        pos = body.get("pos", [])
        speed = float(body.get("speed_factor", 1.0))
        do_approach = bool(body.get("approach", True))
        plan = body.get("plan")    # 规划元数据: 运行结束后与实际反馈一并落盘
        ok = REPLAYER.start_explicit(t, pos, speed, do_approach, plan=plan)
        if not ok:
            return {"ok": False, "error": "无法运行: 未连接 / 航点不足 / 正在运行"}
        return {"ok": True}

    if path == "/api/dump_plan":
        # 仅落盘规划数据(不含实际反馈)。自动运行已在运行结束后自动写"规划+实际",
        # 此接口保留作手动/无硬件时的备用导出。
        return {"ok": True, "file": _write_plan_dump(body)}

    return None


# --------------------------------------------------------------------------
# 入口
# --------------------------------------------------------------------------
def main():
    global ASSET_DIR, URDF_REL

    parser = argparse.ArgumentParser(description="Arm Teach Studio 后端")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument(
        "--assets", default=None,
        help="URDF/STL 模型目录 (含 urdf/ 与 meshes/)。"
             "也可用环境变量 ARM_TEACH_ASSETS 指定。",
    )
    args = parser.parse_args()

    if args.assets:
        ASSET_DIR = os.path.normpath(os.path.abspath(args.assets))

    found = _detect_urdf(ASSET_DIR)
    if found:
        URDF_REL = found
    else:
        print(f"[警告] 在模型目录中未找到 URDF 文件: {ASSET_DIR}")
        print("       本仓库不附带机械臂模型。请准备一个包含 urdf/ 与 meshes/ 的目录, 然后:")
        print("         python server.py --assets /path/to/robot_model")
        print("       或设置环境变量 ARM_TEACH_ASSETS。3D 视图在此之前无法加载。")

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    url = f"http://{args.host}:{args.port}"
    print("=" * 56)
    print("  Arm Teach Studio 已启动")
    print(f"  请在浏览器打开: {url}")
    print(f"  模型资源目录:   {ASSET_DIR}")
    print(f"  pyserial: {'OK' if serial else '缺失 (pip install pyserial)'}")
    print("  按 Ctrl+C 退出")
    print("=" * 56)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n正在退出...")
    finally:
        REPLAYER.stop()
        LINK.disconnect()
        server.shutdown()


if __name__ == "__main__":
    main()
