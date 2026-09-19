#!/usr/bin/env python3
"""阶段 2 离线分析：时序统计、噪声辨识候选、温度关系。

在 WSL 分析环境运行（.venv，含 numpy/scipy/pandas/matplotlib）：

  .venv/bin/python Tests/ImuStream/stage2_analysis.py analyze <会话目录> [--plots]
  .venv/bin/python Tests/ImuStream/stage2_analysis.py all --root <数据根目录>

约定（对应 .docs/Stage2CampaignPlan_20260919.md §6）：
- 只做离线统计与辨识，不滤波、不改数据、不产生固件参数文件；
- 陀螺 1 LSB = 2000/32768 度/s，加速度 1 LSB = 3*9.80665/32768 m/s^2；
- 所有"方差/均值"仅在稳态窗（帧温度进入 50±0.25 度C 并保持 60 s 之后）计算；
- 窗口均值稳定性直接测量（滚动均值标准差），禁止用 sigma/sqrt(N) 推断
  ——阶段 1 已确认样本强相关（一阶自相关约 -0.6）。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

GYRO_LSB_DPS = 2000.0 / 32768.0          # 陀螺 LSB -> 度/s
ACCEL_LSB_MPS2 = 3.0 * 9.80665 / 32768.0  # 加速度 LSB -> m/s^2
TICK_HZ = 480_000_000.0
TICK_WRAP = 1 << 32
TEMP_TARGET_C = 50.0
TEMP_TOL_C = 0.25
TEMP_HOLD_S = 60.0
ROLLING_WINDOWS_S = (1, 5, 30, 60, 300)
ALLAN_MIN_SAMPLES_PER_BIN = 16


def unwrap_ticks(ticks: np.ndarray) -> np.ndarray:
    """32 位 tick 回绕展开：长采集每 8.95 s 绕一圈，必须展开。"""
    ticks = ticks.astype(np.int64)
    deltas = np.diff(ticks)
    deltas[deltas > (1 << 31)] -= 1 << 32
    deltas[deltas < -(1 << 31)] += 1 << 32
    return np.concatenate(([ticks[0]], ticks[0] + np.cumsum(deltas)))


def load_session(session: Path):
    """读入样本与帧级 CSV。保留流式顺序（即时间顺序）：drdyTick 每 8.95 s
    回绕一次，按其原始值排序会打乱时序，绝对不能排序。"""
    samples = pd.read_csv(session / "raw.csv")
    frames = pd.read_csv(session / "frames.csv")
    return samples, frames


def steady_start_tick(frames: pd.DataFrame) -> float | None:
    """帧温度进入 50±0.25 度C 且持续 60 s 的起始 tick（找不到返回 None）。"""
    temps = frames["temperature_celsius"].to_numpy()
    ticks = unwrap_ticks(frames["temperature_tick"].to_numpy())
    ok = np.abs(temps - TEMP_TARGET_C) <= TEMP_TOL_C
    hold_ticks = int(TEMP_HOLD_S * TICK_HZ)
    run = 0
    for index, good in enumerate(ok):
        run = run + 1 if good else 0
        if run >= 2 and ticks[index] - ticks[index - run + 1] >= hold_ticks:
            return float(ticks[index - run + 1])
    return None


def autocorr_fft(x: np.ndarray, max_lag: int) -> np.ndarray:
    """FFT 自相关（有偏估计，返回滞后 0..max_lag 的相关系数）。"""
    x = x - x.mean()
    size = 1
    while size < 2 * len(x):
        size <<= 1
    spectrum = np.fft.rfft(x, size)
    corr = np.fft.irfft(spectrum * np.conj(spectrum), size)[:max_lag + 1]
    return corr / corr[0]


def cross_corr_note() -> dict:
    """两路互相关 v1 未实现：陀螺/加速度样本时间基不同，按样本索引直接
    相关无意义；需先重采样到共同时间基，推迟到阶段 3 参数选择前补充。"""
    return {"implemented": False,
            "reason": "需共同时间基重采样，v1 未实现"}


def overlapping_allan(rate: np.ndarray, fs: float):
    """重叠 Allan 偏差（倍频 τ 网格，O(N log N) 的逐倍频实现）。"""
    n = len(rate)
    theta = np.cumsum(rate) / fs          # 角度积分
    taus, adevs = [], []
    m = 1
    while m * ALLAN_MIN_SAMPLES_PER_BIN < n:
        d = theta[2 * m:] - 2 * theta[m: n - m] + theta[: n - 2 * m]
        adev = float(np.sqrt(np.mean(d * d) / 2.0))
        taus.append(m / fs)
        adevs.append(adev)
        m *= 2
    return np.array(taus), np.array(adevs)


def rolling_mean_std(values: np.ndarray, fs: float, window_s: float) -> float:
    """滚动均值序列的标准差（直接测量，不做独立性假设）。"""
    window = max(1, int(round(window_s * fs)))
    if window < 2 or len(values) < window:
        return float("nan")
    cumulative = np.concatenate(([0.0], np.cumsum(values)))
    means = (cumulative[window:] - cumulative[:-window]) / window
    return float(np.std(means))


def analyze_sensor(samples_s: pd.DataFrame, lsb: float, unit: str,
                   steady_tick: float | None) -> dict:
    """单个传感器的全部统计（稳态窗优先，缺失时退回全程并标注）。"""
    ticks = unwrap_ticks(samples_s["drdy_tick"].to_numpy())
    si = samples_s[["x", "y", "z"]].to_numpy() * lsb
    if steady_tick is not None:
        keep = ticks >= steady_tick
    else:
        keep = np.ones(len(ticks), dtype=bool)
    used = "steady" if keep.sum() > 0.5 * len(ticks) else "full"
    if used == "steady":
        ticks_u, si_u = ticks[keep], si[keep]
    else:
        ticks_u, si_u = ticks, si

    fs = (len(ticks_u) - 1) / ((ticks_u[-1] - ticks_u[0]) / TICK_HZ)
    dt_ms = np.diff(ticks_u) / TICK_HZ * 1e3
    result = {
        "unit": unit, "window_used": used, "samples": int(len(ticks_u)),
        "duration_s": float((ticks_u[-1] - ticks_u[0]) / TICK_HZ),
        "rate_hz": float(fs),
        "dt_ms": {"median": float(np.median(dt_ms)),
                  "p001": float(np.percentile(dt_ms, 0.1)),
                  "p999": float(np.percentile(dt_ms, 99.9)),
                  "min": float(dt_ms.min()), "max": float(dt_ms.max())},
        "mean": [float(v) for v in si_u.mean(axis=0)],
        "std": [float(v) for v in si_u.std(axis=0)],
        "kurtosis": [float(v) for v in
                     (si_u ** 4).mean(axis=0) / (si_u.std(axis=0) ** 4)],
        "autocorr_lag1_20": [float(v) for v in autocorr_fft(si_u[:, 0], 20)[1:]],
    }

    # Welch 功率谱（各轴，仅取 z 轴密度做 JSON 摘要，曲线供绘图）
    from scipy.signal import welch
    freqs, psd_x = welch(si_u[:, 0], fs=fs, nperseg=min(4096, len(si_u) // 4))
    _, psd_y = welch(si_u[:, 1], fs=fs, nperseg=min(4096, len(si_u) // 4))
    _, psd_z = welch(si_u[:, 2], fs=fs, nperseg=min(4096, len(si_u) // 4))
    band = (freqs > fs * 0.1) & (freqs < fs * 0.4)
    result["psd_high_band_rms"] = [
        float(np.sqrt(np.trapezoid(psd[band], freqs[band])))
        for psd in (psd_x, psd_y, psd_z)]

    # 重叠 Allan 偏差（x 轴为代表，三轴曲线见绘图）
    taus, adev_x = overlapping_allan(si_u[:, 0], fs)
    _, adev_y = overlapping_allan(si_u[:, 1], fs)
    _, adev_z = overlapping_allan(si_u[:, 2], fs)
    # 样本强负相关时最短 tau 的 Allan 远低于白噪声底，不能直接当 ARW；
    # ARW 候选取白噪声区（tau^-1/2 斜率段，去相关之后、零偏不稳定性之前）
    # adev*sqrt(tau) 的中位数，最终拟合由阶段 3 完成。
    def white_region_arw(adev: np.ndarray) -> float:
        lo = 5.0 / fs
        hi = min(2.0, taus[-1] / 4.0)
        mask = (taus >= lo) & (taus <= hi)
        if not mask.any():
            mask = taus <= taus[-1] / 4.0
        return float(np.median(adev[mask] * np.sqrt(taus[mask])) * 60.0)
    result["allan"] = {
        "taus_s": [float(t) for t in taus],
        "adev_deg_per_s": [[float(a) for a in axis]
                            for axis in (adev_x, adev_y, adev_z)],
        "arw_white_region_deg_per_sqrt_hr": [
            white_region_arw(a) for a in (adev_x, adev_y, adev_z)],
        "bias_instability_candidate_deg_per_s": [
            float(np.min(a)) for a in (adev_x, adev_y, adev_z)],
        "note": "ARW 取白噪声区中位数；RRW/零偏不稳定性用全曲线拟合在阶段 3 完成",
    }

    # 窗口均值稳定性（x 轴）
    result["rolling_mean_std"] = {
        f"{w}s": rolling_mean_std(si_u[:, 0], fs, w) for w in ROLLING_WINDOWS_S}

    # 滚动均值曲线与谱曲线留给绘图
    result["_curves"] = {
        "ticks": ticks_u, "si": si_u, "freqs": freqs,
        "psd": (psd_x, psd_y, psd_z), "taus": taus,
        "adev": (adev_x, adev_y, adev_z), "fs": fs,
    }
    return result


def temperature_bias_curve(samples: pd.DataFrame, frames: pd.DataFrame,
                           steady_tick: float | None) -> dict:
    """陀螺零偏-温度曲线（0.25 度C 分箱，利用升温段；需要帧温度对齐）。"""
    gyro = samples[samples["sensor"] == "gyro"].copy()
    temp_by_frame = frames.set_index("frame_sequence")["temperature_celsius"]
    gyro["temperature"] = gyro["frame_sequence"].map(temp_by_frame)
    gyro = gyro.dropna(subset=["temperature"])
    if len(gyro) < 1000:
        return {"available": False}
    bins = np.arange(20.0, 60.0 + 0.25, 0.25)
    gyro["bin"] = np.digitize(gyro["temperature"].to_numpy(), bins)
    rows = []
    for bin_id, group in gyro.groupby("bin"):
        if len(group) < 200:
            continue
        rows.append({
            "temperature_c": float(bins[bin_id - 1]),
            "count": int(len(group)),
            "gyro_bias_dps": [float(v * GYRO_LSB_DPS) for v in
                              group[["x", "y", "z"]].mean().to_numpy()],
        })
    return {"available": bool(rows), "bins": rows,
            "steady_only": steady_tick is not None}


def accel_direction_noise(si: np.ndarray) -> dict:
    """加速度方向噪声：样本偏离平均重力方向（切平面内）的逐样本标准差。"""
    mean_norm = np.linalg.norm(si.mean(axis=0))
    unit = si.mean(axis=0) / mean_norm
    proj = si - np.outer(si @ unit, unit)   # 去掉沿重力方向分量
    return {"per_sample_direction_std_mps2": [
        float(v) for v in proj.std(axis=0)]}


def analyze_session(session: Path, plots: bool) -> dict:
    samples, frames = load_session(session)
    steady = steady_start_tick(frames)
    report = {
        "schema": "stage2-analysis/1",
        "session": session.name,
        "steady_start_found": steady is not None,
        "gyro": analyze_sensor(samples[samples["sensor"] == "gyro"],
                               GYRO_LSB_DPS, "dps", steady),
        "accel": analyze_sensor(samples[samples["sensor"] == "accel"],
                                ACCEL_LSB_MPS2, "mps2", steady),
        "temperature_bias": temperature_bias_curve(samples, frames, steady),
    }
    report["accel"]["direction_noise"] = accel_direction_noise(
        report["accel"]["_curves"]["si"])
    report["gyro_accel_crosscorr"] = cross_corr_note()

    curves = {"gyro": report["gyro"].pop("_curves"),
              "accel": report["accel"].pop("_curves")}
    (session / "analysis.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    if plots:
        make_plots(session, report, curves)
    return report


def make_plots(session: Path, report: dict, curves: dict) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out = session / "plots"
    out.mkdir(exist_ok=True)
    for sensor in ("gyro", "accel"):
        curve = curves[sensor]
        fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
        t = (curve["ticks"] - curve["ticks"][0]) / TICK_HZ
        for axis_index, name in enumerate("xyz"):
            axes[axis_index].plot(t, curve["si"][:, axis_index],
                                  lw=0.3, rasterized=True)
            axes[axis_index].set_ylabel(f"{name} ({report[sensor]['unit']})")
        axes[-1].set_xlabel("time since window start (s)")
        fig.suptitle(f"{session.name} {sensor} raw (unfiltered)")
        fig.tight_layout()
        fig.savefig(out / f"{sensor}-raw.png", dpi=120)
        plt.close(fig)

        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        for axis_index, name in enumerate("xyz"):
            axes[0].semilogx(curve["freqs"][1:], curve["psd"][axis_index][1:],
                             lw=0.6, label=name)
            axes[1].loglog(curve["taus"], curve["adev"][axis_index],
                           lw=1.0, label=name)
        axes[0].set_xlabel("Hz"); axes[0].set_ylabel("PSD")
        axes[0].legend(); axes[1].legend()
        axes[1].set_xlabel("tau (s)"); axes[1].set_ylabel("ADEV (deg/s)")
        fig.suptitle(f"{session.name} {sensor} PSD / Allan")
        fig.tight_layout()
        fig.savefig(out / f"{sensor}-psd-allan.png", dpi=120)
        plt.close(fig)
    print(f"[s2] 图输出 {out}")


def cmd_analyze(args: argparse.Namespace) -> int:
    report = analyze_session(Path(args.session), args.plots)
    gyro = report["gyro"]
    arw = gyro["allan"]["arw_white_region_deg_per_sqrt_hr"]
    print(f"会话 {report['session']}：稳态窗 {'找到' if report['steady_start_found'] else '未找到（用全程）'}")
    print(f"gyro 速率 {gyro['rate_hz']:.1f} Hz，均值 {gyro['mean']}")
    print(f"gyro std {gyro['std']}，ARW(度/√h) {arw}")
    print(f"滚动均值 std {gyro['rolling_mean_std']}")
    return 0


def cmd_all(args: argparse.Namespace) -> int:
    root = Path(args.root)
    sessions = sorted(p for p in root.glob("s2-*") if (p / "raw.csv").exists())
    if not sessions:
        print(f"{root} 下没有含 raw.csv 的会话")
        return 1
    rows = []
    for session in sessions:
        report = analyze_session(session, args.plots)
        rows.append({
            "session": session.name,
            "action": session.name.split("-", 4)[4] if session.name.count("-") >= 4 else "",
            "gyro_rate": report["gyro"]["rate_hz"],
            "gyro_mean_x": report["gyro"]["mean"][0],
            "gyro_arw_x": report["gyro"]["allan"][
                "arw_white_region_deg_per_sqrt_hr"][0],
        })
        print(f"[s2] 完成 {session.name}")
    summary = pd.DataFrame(rows)
    summary.to_csv(root / "analysis-index.csv", index=False)
    print(summary.to_string(index=False))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="阶段 2 离线分析")
    subparsers = parser.add_subparsers(dest="command", required=True)
    analyze = subparsers.add_parser("analyze", help="分析单个会话目录")
    analyze.add_argument("session")
    analyze.add_argument("--plots", action="store_true")
    analyze.set_defaults(func=cmd_analyze)
    all_parser = subparsers.add_parser("all", help="分析根目录下全部会话")
    all_parser.add_argument("--root", required=True)
    all_parser.add_argument("--plots", action="store_true")
    all_parser.set_defaults(func=cmd_all)
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
