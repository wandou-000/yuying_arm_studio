#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_plan.py — 离线分析一次自动运行导出的规划数据 (plan_dumps/plan_*.json)。
只读, 不改任何算法。目的: 判定"顿挫"到底来自末端笛卡尔速度起伏, 还是仅关节空间起伏。

用法:
    python analyze_plan.py                      # 自动取 plan_dumps 里最新一份
    python analyze_plan.py plan_dumps/plan_xxx.json
    python analyze_plan.py xxx.json --csv out.csv

输出:
    · 末端笛卡尔速度 |dee|/dt 的均值/峰谷/起伏(局部极小)次数与发生时刻
    · 关节路径速度 |dq|/dt 的同上 (与笛卡尔对照: 若关节起伏而笛卡尔平 → 起伏无害)
    · 每轴峰值速度/加速度 (看是否有轴接近限幅)
    · 可选导出逐拍 CSV, 丢进 Excel/任意工具画曲线
"""
import os
import sys
import json
import glob
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def _latest_dump():
    files = glob.glob(os.path.join(HERE, "plan_dumps", "plan_*.json"))
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def _ripples(v, t, rel=0.04):
    """统计速度曲线 v 上的"起伏": 显著的局部极小 (谷比相邻峰低 > rel*量程)。
    返回 [(index, t, v_at_min)] 列表。rel 是相对量程的阈值, 滤掉数值毛刺。"""
    if len(v) < 3:
        return []
    span = float(np.max(v) - np.min(v)) or 1e-9
    out = []
    for i in range(1, len(v) - 1):
        if v[i] < v[i - 1] and v[i] <= v[i + 1]:
            # 谷深: 与左右最近的局部峰之差
            l = i
            while l > 0 and v[l - 1] >= v[l]:
                l -= 1
            r = i
            while r < len(v) - 1 and v[r + 1] >= v[r]:
                r += 1
            depth = min(v[l], v[r]) - v[i]
            if depth > rel * span:
                out.append((i, float(t[i]), float(v[i])))
    return out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    csv_path = None
    if "--csv" in sys.argv:
        k = sys.argv.index("--csv")
        csv_path = sys.argv[k + 1] if k + 1 < len(sys.argv) else "plan_analysis.csv"

    path = args[0] if args else _latest_dump()
    if not path or not os.path.isfile(path):
        print("找不到 dump 文件。先在前端点一次'自动运行'生成 plan_dumps/plan_*.json")
        return 1

    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    print(f"== 分析: {path}")

    t = np.asarray(d.get("t", []), dtype=float)
    q = np.asarray(d.get("jointsRad", []), dtype=float)   # (N,6) rad
    ee = np.asarray(d.get("ee", []), dtype=float)         # (N,3) m
    n = len(t)
    if n < 3:
        print("点数太少, 无法分析。")
        return 1
    print(f"   点数 N={n}  时长={t[-1]:.3f}s  航点数={len(d.get('waypoints', []))}")

    dt = np.diff(t)
    dt[dt <= 0] = 1e-6

    # 末端笛卡尔速度 (m/s) —— 眼睛真正看到的
    dee = np.linalg.norm(np.diff(ee, axis=0), axis=1)
    v_cart = dee / dt
    tc = 0.5 * (t[1:] + t[:-1])

    # 关节路径速度 (rad/s) —— 你测的 VC·Lj/Lc 对应的量
    dq = np.diff(q, axis=0)
    v_jarc = np.linalg.norm(dq, axis=1) / dt

    def report(name, v, unit):
        vmin, vmax, vmean = float(np.min(v)), float(np.max(v)), float(np.mean(v))
        rip = _ripples(v, tc)
        print(f"\n-- {name} ({unit})")
        print(f"   均值={vmean:.4f}  峰={vmax:.4f}  谷={vmin:.4f}  峰谷差={vmax - vmin:.4f}"
              f"  起伏比={(vmax - vmin) / (vmean or 1e-9) * 100:.1f}%")
        print(f"   显著起伏(局部极小)次数 = {len(rip)}")
        for i, ti, vi in rip:
            print(f"      idx={i:5d}  t={ti:6.3f}s  v={vi:.4f}")
        return rip

    rc = report("末端笛卡尔速度", v_cart, "m/s")
    rj = report("关节路径速度", v_jarc, "rad/s")

    # 每轴峰值速度/加速度 (看是否撞限幅 JOINT_VMAX=1 rad/s)
    print("\n-- 每轴峰值 (rad/s, rad/s²)")
    vj = dq / dt[:, None]
    aj = np.diff(vj, axis=0) / (0.5 * (dt[1:] + dt[:-1]))[:, None]
    for j in range(q.shape[1]):
        print(f"   J{j + 1}: |v|max={np.max(np.abs(vj[:, j])):.4f}  |a|max={np.max(np.abs(aj[:, j])):.4f}")

    # 各电机实际关节角 (反馈) —— 真机上眼睛真正看到的顿挫在这里
    actual = d.get("actual", [])
    if actual and len(actual) > 3:
        ta = np.asarray([r[0] for r in actual], dtype=float)
        qa = np.asarray([r[1] for r in actual], dtype=float)        # (Na,6) cdeg 单圈
        qa_deg = np.unwrap(np.deg2rad(qa / 100.0), axis=0) * 180.0 / np.pi
        dta = np.diff(ta); dta[dta <= 0] = 1e-6
        va = np.diff(qa_deg, axis=0) / dta[:, None]                 # deg/s per axis
        tca = 0.5 * (ta[1:] + ta[:-1])
        print(f"\n-- 实际电机关节角 (反馈, Na={len(ta)}, 覆盖 {ta[0]:.3f}..{ta[-1]:.3f}s)")
        for j in range(qa.shape[1]):
            sp = np.abs(va[:, j])
            rip = _ripples(sp, tca)
            print(f"   J{j + 1}: |v|max={np.max(sp):7.2f} deg/s  实际速度起伏次数={len(rip)}")

        # 规划 ↔ 实际 跟踪误差: 把规划 pos(cdeg) 重采样到实际时刻, 各轴去整圈偏移后比较
        pos = np.asarray(d.get("pos", []), dtype=float)             # (N,6) cdeg
        if len(pos) == n and n > 1:
            pos_deg = np.unwrap(np.deg2rad(pos / 100.0), axis=0) * 180.0 / np.pi
            errs = []
            for j in range(pos.shape[1]):
                pj = np.interp(ta, t, pos_deg[:, j])
                aj = qa_deg[:, j] + round((pj[0] - qa_deg[0, j]) / 360.0) * 360.0
                errs.append(float(np.max(np.abs(pj - aj))))
            print("   规划↔实际 各轴最大跟踪误差(deg): " +
                  ", ".join(f"J{j + 1}={e:.2f}" for j, e in enumerate(errs)))
    else:
        print("\n-- (本份无 actual 实际反馈: 未连硬件 / 仅规划导出)")

    # 结论提示
    print("\n== 判读")
    cart_rel = (np.max(v_cart) - np.min(v_cart)) / (np.mean(v_cart) or 1e-9)
    if len(rc) >= 2 and cart_rel > 0.15:
        print(f"   末端笛卡尔速度有 {len(rc)} 次明显起伏 (峰谷 {cart_rel*100:.0f}%) → 顿挫是真实的,")
        print("   且在末端就能看见 → 应在规划器压平巡航/按笛卡尔弧长重做时间参数化。")
    elif len(rj) >= 2 and len(rc) < 2:
        print("   关节速度起伏但末端笛卡尔速度基本平 → 起伏对观感无害, 顿挫另有来源(查折返点/收尾)。")
    else:
        print("   末端速度较平。把折返点(局部极小≈0)和收尾段单独看, 顿挫可能集中在那里。")

    if csv_path:
        rows = ["t,v_cart_mps,v_jarc_radps," + ",".join(f"v_j{j+1}" for j in range(q.shape[1]))]
        for i in range(len(tc)):
            rows.append(f"{tc[i]:.4f},{v_cart[i]:.6f},{v_jarc[i]:.6f}," +
                        ",".join(f"{vj[i, j]:.6f}" for j in range(q.shape[1])))
        with open(csv_path, "w", encoding="utf-8") as f:
            f.write("\n".join(rows))
        print(f"\n   已写 CSV: {csv_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
