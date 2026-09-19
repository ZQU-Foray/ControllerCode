#!/usr/bin/env python3
"""阶段 2 采集会话编排：建档、归档固件、采集、解码、验收、索引。

用法（Git Bash，Windows Python 执行采集）：
  python.exe Tests/ImuStream/stage2_campaign.py start \\
      --action cold-static --boot cold --rep 1 --duration 2400 \\
      --port COM4 --note "E1-r1 冷启动"
  python.exe Tests/ImuStream/stage2_campaign.py verify <会话目录>
  python.exe Tests/ImuStream/stage2_campaign.py status

设计依据：.docs/Stage2CampaignPlan_20260919.md。本脚本不滤波、不改固件，
只调用 imu_stream_tool 的采集与解码能力。
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import shutil
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

# 使脚本在任意工作目录下都能导入同目录的 imu_stream_tool
sys.path.insert(0, str(Path(__file__).resolve().parent))
import imu_stream_tool  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DATA_ROOT = REPO_ROOT / "Platform" / "STM32H7" / "DM_MC02" / "build" / "stage2-data"
FIRMWARE_DIR = REPO_ROOT / "Platform" / "STM32H7" / "DM_MC02" / "build" / "WSL-Release"
FIRMWARE_STEM = "mc02_cpp"


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_head() -> str | None:
    """记录当前 git 快照；失败返回 None（不阻断采集）。"""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, capture_output=True,
            text=True, timeout=10)
        if result.returncode == 0:
            return result.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        pass
    return None


def git_dirty() -> bool | None:
    try:
        result = subprocess.run(
            ["git", "status", "--short"], cwd=REPO_ROOT, capture_output=True,
            text=True, timeout=10)
        if result.returncode == 0:
            return bool(result.stdout.strip())
    except (OSError, subprocess.SubprocessError):
        pass
    return None


def find_st_port(explicit: str | None) -> str:
    """自动发现板载 ST CDC 端口（Windows 用 WMI，Linux 扫 ttyACM）。"""
    if explicit:
        return explicit
    if platform.system() == "Windows":
        query = ("Get-CimInstance Win32_PnPEntity | Where-Object { "
                 "$_.PNPDeviceID -like '*VID_0483&PID_5740*' -and "
                 "$_.Name -match 'COM' } | ForEach-Object { "
                 "if ($_.Name -match '\\((COM[0-9]+)\\)') { $Matches[1] } }")
        try:
            result = subprocess.run(
                ["powershell", "-NoProfile", "-Command", query],
                capture_output=True, text=True, timeout=30)
            for line in result.stdout.splitlines():
                line = line.strip()
                if line.startswith("COM"):
                    return line
        except (OSError, subprocess.SubprocessError):
            pass
        raise SystemExit("未找到 ST CDC 端口，请用 --port 显式指定")
    # Linux：扫 /dev/ttyACM* 的 udev 标识
    for candidate in sorted(Path("/dev").glob("ttyACM*")):
        try:
            udev = subprocess.run(
                ["udevadm", "info", "-q", "property", "-n", str(candidate)],
                capture_output=True, text=True, timeout=10)
            if "ID_VENDOR_ID=0483" in udev.stdout:
                return str(candidate)
        except (OSError, subprocess.SubprocessError):
            continue
    raise SystemExit("未找到 ST CDC 端口（/dev/ttyACM*），请用 --port 显式指定")


def write_manifest(session: Path, args: argparse.Namespace,
                   firmware: dict) -> None:
    manifest = {
        "schema": "stage2-session/1",
        "created_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "created_epoch": int(time.time()),
        "action": args.action,
        "boot": args.boot,
        "rep": args.rep,
        "nominal_duration_s": args.duration,
        "operator_note": args.note,
        "campaign": "stage2-20260919",
        "firmware": firmware,
        "git_head": git_head(),
        "git_dirty_at_start": git_dirty(),
        "host": {
            "system": platform.system(),
            "machine": platform.machine(),
            "python": platform.python_version(),
        },
    }
    (session / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")


OPERATOR_TEMPLATE = """# 操作员记录：{action} r{rep}

- 环境：环境温度 ___ °C，桌面/夹具描述 ___
- 供电：___（电池/线性电源，是否共地）
- 异常与触碰：无 / ___（发生时刻与描述）
- 动作时间线（E5/E6/E7 必填）：___
- 安装朝向（E3/E4 必填）：___

补充：___
"""


def archive_firmware(session: Path, copy: bool = True) -> dict:
    """归档固件身份（可选复制），并返回散列信息。"""
    info = {}
    for ext in ("elf", "bin"):
        source = FIRMWARE_DIR / f"{FIRMWARE_STEM}.{ext}"
        if not source.exists():
            raise SystemExit(f"固件不存在：{source}（阶段 2 禁止重新编译）")
        digest = sha256_of(source)
        entry = {"source": str(source.relative_to(REPO_ROOT)),
                 "sha256": digest, "bytes": source.stat().st_size}
        if copy:
            target = session / f"firmware.{ext}"
            shutil.copy2(source, target)
            entry["path"] = target.name
        info[ext] = entry
    if copy:
        (session / "firmware.sha256").write_text(
            f"{info['elf']['sha256']}  firmware.elf\n"
            f"{info['bin']['sha256']}  firmware.bin\n", encoding="utf-8")
    return info


def update_index(data_root: Path, session: Path, manifest: dict,
                 acceptance: str, verdict: str, raw_bytes: int,
                 raw_sha: str) -> None:
    index_path = data_root / "index.csv"
    new_row = [
        session.name, manifest["created_local"], manifest["action"],
        manifest["boot"], manifest["rep"], manifest["nominal_duration_s"],
        verdict, raw_bytes, raw_sha,
        manifest["firmware"]["bin"]["sha256"][:8],
    ]
    exists = index_path.exists()
    with index_path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        if not exists:
            writer.writerow(["session", "created", "action", "boot", "rep",
                             "nominal_duration_s", "verdict", "raw_bytes",
                             "raw_sha256", "fw8"])
        writer.writerow(new_row)
    _ = acceptance  # 验收全文在会话目录 acceptance.txt 内，索引只存结论


def cmd_start(args: argparse.Namespace) -> int:
    data_root = Path(args.root) if args.root else DEFAULT_DATA_ROOT
    data_root.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M")
    firmware_probe = archive_firmware(None, copy=False)
    fw8 = firmware_probe["bin"]["sha256"][:8]
    session_name = f"s2-{stamp}-{fw8}-{args.boot}-{args.action}-r{args.rep}"
    session = data_root / session_name
    if session.exists():
        raise SystemExit(f"会话目录已存在：{session}（勿覆盖，请增加重复序号）")
    session.mkdir(parents=True)

    firmware = archive_firmware(session)
    write_manifest(session, args, firmware)
    (session / "operator-notes.md").write_text(
        OPERATOR_TEMPLATE.format(action=args.action, rep=args.rep),
        encoding="utf-8")

    if args.dry_run:
        print(f"[dry-run] 会话目录 {session} 已建档，跳过采集")
        return 0

    port = find_st_port(args.port)
    print(f"[s2] 会话 {session_name}")
    print(f"[s2] 端口 {port}，采集 {args.duration} s（含前置 'R' 开出口）")

    capture_ns = SimpleNamespace(
        port=port, baud=args.baud, out=str(session / "raw.bin"),
        seconds=float(args.duration), pause_first=False)
    rc = imu_stream_tool.capture(capture_ns)
    if rc != 0:
        print(f"[s2] 采集异常退出 rc={rc}，保留现场，请排查后重采（rep+1）",
              file=sys.stderr)
        return rc

    raw_bin = session / "raw.bin"
    decode_ns = SimpleNamespace(
        bin=str(raw_bin), csv=str(session / "raw.csv"),
        frames_csv=str(session / "frames.csv"))
    decode_stdout = []
    verdict = "FAIL"
    try:
        # imu_stream_tool.decode 直接返回 0/1，输出已打印；重定向捕获留档
        import contextlib
        import io
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            rc = imu_stream_tool.decode(decode_ns)
        decode_stdout = buffer.getvalue().splitlines()
        verdict = "PASS" if rc == 0 else "FAIL"
    except Exception as error:  # noqa: BLE001 —— 验收失败也要落盘原因
        decode_stdout = [f"decode 异常：{error!r}"]
    (session / "acceptance.txt").write_text(
        "\n".join(decode_stdout) + f"\nverdict: {verdict}\n", encoding="utf-8")

    update_index(data_root, session, json.loads(
        (session / "manifest.json").read_text(encoding="utf-8")),
        "\n".join(decode_stdout), verdict, raw_bin.stat().st_size,
        sha256_of(raw_bin))

    print(f"[s2] 验收 {verdict}；数据与报告见 {session}")
    if verdict == "PASS":
        print("[s2] 提醒：填写 operator-notes.md，E5/E6/E7 记录动作时间线")
    return 0 if verdict == "PASS" else 1


def cmd_verify(args: argparse.Namespace) -> int:
    session = Path(args.session)
    raw = session / "raw.bin"
    if not raw.exists():
        raise SystemExit(f"缺 {raw}")
    decode_ns = SimpleNamespace(
        bin=str(raw), csv=str(session / "raw.csv"),
        frames_csv=str(session / "frames.csv"))
    import contextlib
    import io
    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        rc = imu_stream_tool.decode(decode_ns)
    print(buffer.getvalue(), end="")
    print(f"verdict: {'PASS' if rc == 0 else 'FAIL'}")
    return rc


def cmd_status(args: argparse.Namespace) -> int:
    data_root = Path(args.root) if args.root else DEFAULT_DATA_ROOT
    index = data_root / "index.csv"
    if not index.exists():
        print(f"尚无会话（{index} 不存在）")
        return 0
    rows = list(csv.reader(index.open(encoding="utf-8")))
    for row in rows:
        print("  ".join(row))
    total = len(rows) - 1
    passed = sum(1 for row in rows[1:] if len(row) > 6 and row[6] == "PASS")
    print(f"共 {total} 个会话，{passed} 个 PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="阶段 2 采集会话编排（详见 .docs/Stage2CampaignPlan）")
    subparsers = parser.add_subparsers(dest="command", required=True)

    start = subparsers.add_parser("start", help="建档并执行一次采集会话")
    start.add_argument("--action", required=True,
                       help="cold-static/hot-static/sixface-X+/tilt-…/rot-…/dynamic/linacc")
    start.add_argument("--boot", choices=["cold", "hot"], required=True)
    start.add_argument("--rep", type=int, required=True, help="重复序号，从 1 起")
    start.add_argument("--duration", type=float, required=True,
                       help="采集秒数（E1 建议 2400 含升温，其余按矩阵）")
    start.add_argument("--port", default=None,
                       help="ST CDC 端口（缺省自动发现）")
    start.add_argument("--baud", type=int, default=115200)
    start.add_argument("--note", default="", help="操作备注")
    start.add_argument("--root", default=None, help="数据根目录（缺省工程 build 下）")
    start.add_argument("--dry-run", action="store_true", help="只建档不采集")
    start.set_defaults(func=cmd_start)

    verify = subparsers.add_parser("verify", help="对既有会话重跑解码验收")
    verify.add_argument("session", help="会话目录")
    verify.set_defaults(func=cmd_verify)

    status = subparsers.add_parser("status", help="列出全部会话与结论")
    status.add_argument("--root", default=None)
    status.set_defaults(func=cmd_status)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
