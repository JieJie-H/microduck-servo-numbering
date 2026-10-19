#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MicroDuck 舵机编号工具（飞特 SCS/STS 协议，HD-1910）

用途：把 15 颗飞特 HD-1910 从「当前 ID」改成 MicroDuck 的关节 ID，
每改一颗就断电重启、复核、打印要贴的标签。只改 ID 和两项保护，
不碰波特率 / 运行模式 / PID / 中位。

典型流程（一次只接一颗舵机）：

    python servo_numbering.py ports                 # 看串口
    python servo_numbering.py scan                  # 扫描当前这颗的 ID
    python servo_numbering.py assign --to 10 --joint 右髋yaw
    python servo_numbering.py verify                # 15 颗全接好后的总核对

也可以一条条带着做：

    python servo_numbering.py wizard

安全约定：
- 写 ID 前要解锁 EEPROM（寄存器 55=0），写完加锁（55=1），再 0x08 重启复核；
- 默认打开过流 / 过压保护（寄存器 19 置 bit0 和 bit3），与《调试记录》第 5 步一致；
- 总线上发现不止一颗舵机就拒绝改 ID，避免把同 ID 的几颗一起改掉；
- 每次操作都追加到 servo_ledger.jsonl，wizard 用它防止把两个关节写成同一个 ID。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

# 允许把 pyserial 装在脚本旁边的 .pylibs（不污染系统 Python）
_HERE = Path(__file__).resolve().parent
# 统一用 UTF-8 输出，避免中文在管道/重定向时变成乱码
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

_LOCAL_LIBS = _HERE.parent / ".pylibs"
if _LOCAL_LIBS.is_dir():
    sys.path.insert(0, str(_LOCAL_LIBS))

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    print("缺少 pyserial。先运行：")
    print(f'  python -m pip install --target "{_LOCAL_LIBS}" pyserial')
    raise SystemExit(2)

# ---------------------------------------------------------------- 协议常量

PING, READ, WRITE, REBOOT = 0x01, 0x02, 0x03, 0x08
BROADCAST = 0xFE

REG_ID = 5
REG_BAUD = 6
REG_UNLOAD = 19          # 卸载条件：bit0 电压、bit1 磁编码、bit2 过热、bit3 过流、bit5 过载
REG_TORQUE = 40
REG_LOCK = 55

UNLOAD_OVERVOLT = 0x01
UNLOAD_OVERHEAT = 0x04
UNLOAD_OVERCURRENT = 0x08
PROTECT_BITS = UNLOAD_OVERVOLT | UNLOAD_OVERCURRENT   # 4 → 13

BAUD_INDEX = {1_000_000: 0, 500_000: 1, 250_000: 2, 128_000: 3,
              115_200: 4, 76_800: 5, 57_600: 6, 38_400: 7, 19_200: 8}
BAUD_CANDIDATES = list(BAUD_INDEX)
DEFAULT_BAUD = 1_000_000
# 先扫这些：出厂 1、MicroDuck 的 10–14 / 20–24 / 30–34，以及常见 XGO 编号 1–15
LIKELY_IDS = list(range(0, 16)) + list(range(20, 35))

# 设计 ID 表：来自主仓 duck-control/src/model.rs，十位=部位，个位=从躯干往末端
JOINTS = [
    (10, "right_hip_yaw",   "右髋 yaw"),
    (11, "right_hip_roll",  "右髋 roll"),
    (12, "right_hip_pitch", "右髋 pitch"),
    (13, "right_knee",      "右膝"),
    (14, "right_ankle",     "右踝"),
    (20, "left_hip_yaw",    "左髋 yaw"),
    (21, "left_hip_roll",   "左髋 roll"),
    (22, "left_hip_pitch",  "左髋 pitch"),
    (23, "left_knee",       "左膝"),
    (24, "left_ankle",      "左踝"),
    (30, "neck_pitch",      "颈俯仰"),
    (31, "head_pitch",      "头俯仰"),
    (32, "head_yaw",        "头 yaw"),
    (33, "head_roll",       "头 roll"),
    (34, "mouth",           "嘴"),
]
JOINT_BY_ID = {j[0]: j for j in JOINTS}

LEDGER = _HERE / "servo_ledger.jsonl"
STATUS_BITS = {0: "电压", 1: "磁编码", 2: "温度", 3: "电流", 5: "负载"}


# ---------------------------------------------------------------- 小工具

def now():
    return datetime.now().astimezone().isoformat(timespec="seconds")


def hx(data):
    return " ".join(f"{b:02x}" for b in data)


def sign15(v):
    return -(v & 0x7FFF) if v & 0x8000 else v


def join_name(sid):
    j = JOINT_BY_ID.get(sid)
    return f"{j[2]}" if j else "?"


def plan_for(mouth_id):
    """mouth_id=0 表示不要嘴；29 兼容第一台机器的历史改法。"""
    plan = [j for j in JOINTS if j[0] != 34]
    if mouth_id:
        plan.append((mouth_id, "mouth", "嘴"))
    return sorted(plan)


# ---------------------------------------------------------------- 串口 / 协议

class BusError(Exception):
    pass


class BusTimeout(BusError):
    pass


class Bus:
    """最小飞特总线实现。ser 参数给模拟器用（sim_bus.SimSerial）。"""

    def __init__(self, port=None, baud=DEFAULT_BAUD, timeout=0.02, ser=None):
        self.port = port
        self.baud = baud
        self.ser = ser if ser is not None else serial.Serial(
            port=port, baudrate=baud, timeout=timeout, write_timeout=0.2)
        self.ser.timeout = timeout

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    @staticmethod
    def _packet(sid, instr, params=b""):
        length = len(params) + 2
        chk = (~(sid + length + instr + sum(params))) & 0xFF
        return bytes([0xFF, 0xFF, sid, length, instr]) + bytes(params) + bytes([chk])

    def _send(self, sid, instr, params=b""):
        self.ser.reset_input_buffer()
        self.ser.write(self._packet(sid, instr, params))

    def _read_status(self, expect_id=None):
        # 找 FF FF
        deadline = time.monotonic() + max(self.ser.timeout * 3, 0.03)
        hdr = b""
        while time.monotonic() < deadline:
            b = self.ser.read(1)
            if not b:
                break
            hdr = (hdr + b)[-2:]
            if hdr == b"\xff\xff":
                break
        if hdr != b"\xff\xff":
            raise BusTimeout("没应答（没收到帧头）")
        rest = self.ser.read(2)
        if len(rest) < 2:
            raise BusTimeout(f"应答断在 ID/长度（ff ff {hx(rest)}）")
        sid, length = rest[0], rest[1]
        body = self.ser.read(length)
        if len(body) < length:
            raise BusTimeout(f"应答不完整（要 {length} 字节，收到 {len(body)}）")
        err, params, chk = body[0], body[1:-1], body[-1]
        if ((~(sid + length + err + sum(params))) & 0xFF) != chk:
            raise BusError(f"校验和不对（ff ff {hx(rest + body)}）")
        if expect_id is not None and sid != expect_id:
            raise BusError(f"等 id {expect_id}，来的是 id {sid}")
        return sid, err, bytes(params)

    def ping(self, sid):
        try:
            self._send(sid, PING)
            _, err, _ = self._read_status(sid)
            return err
        except BusError:
            return None

    def read(self, sid, addr, n):
        self._send(sid, READ, bytes([addr, n]))
        _, err, params = self._read_status(sid)
        if len(params) != n:
            raise BusError(f"id {sid} 读地址 {addr} 要 {n} 字节，回 {len(params)}")
        return err, params

    def write(self, sid, addr, data, wait=True):
        self._send(sid, WRITE, bytes([addr]) + bytes(data))
        if not wait or sid == BROADCAST:
            return None
        try:
            _, err, _ = self._read_status(sid)
            return err
        except BusTimeout:
            return None

    def read_u8(self, sid, addr):
        return self.read(sid, addr, 1)[1][0]

    def read_u16(self, sid, addr):
        p = self.read(sid, addr, 2)[1]
        return p[0] | (p[1] << 8)

    def write_u8(self, sid, addr, value, wait=True):
        return self.write(sid, addr, [value & 0xFF], wait=wait)

    def reboot(self, sid):
        self._send(sid, REBOOT)      # 0x08 不回包，约 0.8 s 后回来


# ---------------------------------------------------------------- 端口 / 扫描

def list_serial_ports():
    ports = sorted(list_ports.comports(), key=lambda p: p.device)
    if not ports:
        print("没有发现串口。检查：URT-2 的 USB 是否插好；设备管理器「端口」里有没有 CH343。")
    for p in ports:
        print(f"  {p.device:10s}  {p.description}")
    return ports


def resolve_port(port=None):
    if port:
        return port
    ports = list_serial_ports()
    if len(ports) == 1:
        print(f"自动使用 {ports[0].device}")
        return ports[0].device
    if not ports:
        raise SystemExit("没有串口可用。")
    raise SystemExit("有多个串口，请用 --port 指定（比如 --port COM5）。")


def open_bus(port, baud, timeout=0.02):
    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=timeout, write_timeout=0.2)
    except serial.SerialException as e:
        raise SystemExit(f"打不开 {port} @ {baud}：{e}\n"
                         "常见原因：FD / 调试台正占着这个口；口选错；驱动没装。")
    time.sleep(0.05)
    return Bus(port, baud, timeout, ser=ser)


def scan_ids(bus, ids, quiet=False):
    found = {}
    t0 = time.monotonic()
    for i, sid in enumerate(ids):
        err = bus.ping(sid)
        if err is not None:
            found[sid] = err
            if not quiet:
                print(f"  发现 ID {sid}（状态 0x{err:02X}）")
        if not quiet and (i + 1) % 64 == 0:
            print(f"  ...已扫 {i + 1}/{len(ids)}，{time.monotonic() - t0:.1f}s")
    return found


def _scan_at(port, baud, ids, timeout, quiet, label):
    if not quiet:
        print(f"扫描 {port} @ {baud}（{label}）...")
    bus = open_bus(port, baud, timeout=timeout)
    try:
        return scan_ids(bus, ids, quiet=quiet)
    finally:
        bus.close()


def find_servos(port, bauds=None, ids=None, timeout=0.008, quiet=False,
                full_other_bauds=False):
    """找总线上的舵机。

    分阶段，避免无舵机时把 9 个波特率 × 254 个 ID 全扫一遍（Windows 每个空
    应答约 15 ms，全扫要 40 s）：
      1. 第一个波特率（默认 1M）扫常用 ID；
      2. 第一个波特率扫全 ID；
      3. 其余波特率只扫常用 ID；
      4. 仍没有，用更长超时重试第一个波特率的常用 ID。
    指定了 ids 或 bauds 时按指定的逐项扫。
    """
    bauds = bauds or BAUD_CANDIDATES
    if ids is not None:
        for baud in bauds:
            found = _scan_at(port, baud, ids, timeout, quiet, "指定 ID")
            if found:
                return baud, found
        return None, {}

    first = bauds[0]
    found = _scan_at(port, first, LIKELY_IDS, timeout, quiet, "常用 ID")
    if found:
        return first, found
    found = _scan_at(port, first, range(0, 254), timeout, quiet, "全 ID")
    if found:
        return first, found
    for baud in bauds[1:]:
        found = _scan_at(port, baud, LIKELY_IDS, timeout, quiet, "常用 ID")
        if found:
            return baud, found
        if full_other_bauds:
            found = _scan_at(port, baud, range(0, 254), max(timeout, 0.02),
                             quiet, "全 ID")
            if found:
                return baud, found
    found = _scan_at(port, first, LIKELY_IDS, max(timeout * 3, 0.02),
                     quiet, "常用 ID · 长超时")
    if found:
        return first, found
    return None, {}


def identify(bus, sid):
    """读一颗舵机的关键寄存器；读不到的项为 None。"""
    info = {"id": sid}
    try:
        fw = bus.read(sid, 0, 5)[1]
        info["fw"] = f"{fw[0]}.{fw[1]}"
        info["model"] = f"{fw[3]}.{fw[4]}"
    except BusError:
        info["fw"] = info["model"] = None
    for key, addr, fn in [
        ("id_reg", REG_ID, bus.read_u8),
        ("baud_idx", REG_BAUD, bus.read_u8),
        ("pos", 56, lambda s, a: sign15(bus.read_u16(s, a))),
        ("volt", 62, lambda s, a: bus.read_u8(s, a) / 10.0),
        ("temp", 63, bus.read_u8),
        ("status", 65, bus.read_u8),
        ("unload", REG_UNLOAD, bus.read_u8),
        ("torque", REG_TORQUE, bus.read_u8),
        ("lock", REG_LOCK, bus.read_u8),
    ]:
        try:
            info[key] = fn(sid, addr)
        except BusError:
            info[key] = None
    return info


def status_text(v):
    if v is None:
        return "?"
    bits = [name for bit, name in STATUS_BITS.items() if v >> bit & 1]
    return "0x%02X%s" % (v, "（" + "、".join(bits) + "）" if bits else "")


def print_info(info):
    print(f"  ID        : {info['id']}"
          + (f"（寄存器里也是 {info['id_reg']}）" if info.get("id_reg") is not None else ""))
    print(f"  型号/固件 : {info.get('model') or '?'} / {info.get('fw') or '?'}")
    if info.get("pos") is not None:
        print(f"  位置      : {info['pos']}")
    if info.get("volt") is not None:
        print(f"  电压/温度 : {info['volt']:.1f} V / {info['temp']} °C")
    print(f"  状态位    : {status_text(info.get('status'))}")
    if info.get("unload") is not None:
        print(f"  卸载条件  : 0x{info['unload']:02X}"
              f"（过压{'开' if info['unload'] & 1 else '关'}，"
              f"过流{'开' if info['unload'] & 8 else '关'}）")
    if info.get("lock") is not None:
        print(f"  EEPROM 锁 : {info['lock']}")


# ---------------------------------------------------------------- 台账

def load_ledger():
    if not LEDGER.exists():
        return []
    rows = []
    with LEDGER.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return rows


def append_ledger(entry):
    with LEDGER.open("a", encoding="utf-8") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")


def assigned_targets():
    out = {}
    for row in load_ledger():
        if row.get("ok") and row.get("new") is not None:
            out[row["new"]] = row
    return out


# ---------------------------------------------------------------- 改 ID

def unlock(bus, sid):
    bus.write_u8(sid, REG_LOCK, 0)


def lock(bus, sid):
    bus.write_u8(sid, REG_LOCK, 1, wait=False)


def set_protection(bus, sid):
    """打开过压 + 过流，返回 (before, after, ok)。"""
    unlock(bus, sid)
    try:
        before = bus.read_u8(sid, REG_UNLOAD)
        want = before | PROTECT_BITS
        if want == before:
            return before, before, True
        bus.write_u8(sid, REG_UNLOAD, want)
        time.sleep(0.03)
        after = bus.read_u8(sid, REG_UNLOAD)
        return before, after, after == want
    finally:
        lock(bus, sid)


def assign_id(bus, old, new, joint="", protect=True, reboot_wait=1.3):
    """把一颗舵机从 old 改成 new，断电重启后复核。返回结果 dict。"""
    result = {"old": old, "new": new, "joint": joint, "ok": False,
              "protect": None, "error": None}
    if not (0 <= new <= 253):
        result["error"] = "新 ID 必须在 0–253；200 留给 imu_to_dxl"
        return result
    if new == 200:
        result["error"] = "200 是 imu_to_dxl 的 ID，不能给舵机"
        return result

    # 复核当前 ID，防止扫到的东西跟实际不符
    try:
        cur = bus.read_u8(old, REG_ID)
    except BusError as e:
        result["error"] = f"改之前读 ID 失败：{e}"
        return result
    if cur != old:
        result["error"] = f"总线上 id {old} 读回来是 {cur}，不动"
        return result

    print(f"  解锁 EEPROM ...")
    unlock(bus, old)
    time.sleep(0.03)
    try:
        lock_state = bus.read_u8(old, REG_LOCK)
        if lock_state != 0:
            print(f"  警告：锁标志回读是 {lock_state}，继续尝试")
    except BusError:
        pass

    # 关扭矩，避免改 ID 时舵机受力
    try:
        bus.write_u8(old, REG_TORQUE, 0)
        time.sleep(0.02)
    except BusError:
        pass

    print(f"  写 ID {old} → {new} ...")
    bus.write(old, REG_ID, [new], wait=False)
    time.sleep(0.15)

    # 加锁：新 ID 回包可能用新 ID，也可能还用旧 ID
    locked = False
    for sid in (new, old):
        try:
            bus.write_u8(sid, REG_LOCK, 1, wait=False)
            locked = True
        except Exception:
            pass
    if not locked:
        print("  警告：加锁指令没发出去（不影响 ID，但 EEPROM 可能还开着）")

    time.sleep(0.05)          # 给 EEPROM 锁一点落地时间，再重启
    print(f"  重启（0x08），等 {reboot_wait:.1f}s ...")
    try:
        bus.reboot(new)
    except Exception:
        pass
    time.sleep(reboot_wait)

    # 复核：新 ID 要能 ping 通、寄存器 5 要等于 new；旧 ID 要消失
    ok_new = False
    for _ in range(10):
        if bus.ping(new) is not None:
            ok_new = True
            break
        time.sleep(0.15)
    if not ok_new:
        # 也许 ID 没写进去，旧 ID 还在
        if old != new and bus.ping(old) is not None:
            result["error"] = f"新 ID {new} 不应答，旧 ID {old} 还在 —— ID 没写进去"
        else:
            result["error"] = f"新 ID {new} 不应答（可能掉电/线松/写失败）"
        return result

    try:
        got = bus.read_u8(new, REG_ID)
        if got != new:
            result["error"] = f"id {new} 应答，但 ID 寄存器是 {got}"
            return result
    except BusError as e:
        result["error"] = f"复核 ID 寄存器失败：{e}"
        return result

    if old != new and bus.ping(old) is not None:
        result["error"] = f"新 ID 已生效，但旧 ID {old} 仍应答（总线上还有别的同 ID 舵机？）"
        return result

    if protect:
        try:
            before, after, pok = set_protection(bus, new)
            result["protect"] = {"before": before, "after": after, "ok": pok}
        except BusError as e:
            result["protect"] = {"error": str(e)}

    result["ok"] = True
    return result


# ---------------------------------------------------------------- 命令

def cmd_ports(args):
    list_serial_ports()


def cmd_scan(args):
    port = resolve_port(args.port)
    baud, found = find_servos(port, [args.baud] if args.baud else None,
                              quiet=args.quiet)
    if not found:
        print("\n一颗都没扫到。按顺序查：")
        print("  1. 先插 URT-2 的 USB，再给舵机上电（反了总线可能被压在 2.1 V）")
        print("  2. 舵机线：脚 1 Signal → S，脚 2 Vcc → V1，脚 3 GND → G；S/G 反了不烧但搜不到")
        print("  3. 波特率：本工具已自动试 1M / 500k / 250k / 128k / 115200 ...")
        print("  4. COM 口：FD / 调试台是不是还占着？换一根 USB 线试试")
        print("  5. 供电：配 ID 用 USB 5 V 就够；URT-2 拨码拨 5 V 侧")
        raise SystemExit(1)

    print(f"\n在 {port} @ {baud} 找到 {len(found)} 颗：")
    bus = open_bus(port, baud, timeout=0.02)
    try:
        rows = []
        for sid in sorted(found):
            info = identify(bus, sid)
            print(f"\n[ID {sid}]")
            print_info(info)
            rows.append(info)
    finally:
        bus.close()

    if args.json:
        out = _HERE / f"scan-{datetime.now():%Y%m%d-%H%M%S}.json"
        out.write_text(json.dumps({"port": port, "baud": baud, "servos": rows},
                                  ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"\n已存 {out}")


def cmd_assign(args):
    port = resolve_port(args.port)
    target = args.to
    if target not in {j[0] for j in plan_for(args.mouth_id)}:
        raise SystemExit(f"{target} 不在当前 ID 表里；用 --mouth-id 改嘴的 ID，或先改脚本里的表。")
    assigned = assigned_targets()
    if target in assigned and not args.force:
        row = assigned[target]
        raise SystemExit(f"台账里 {target} 已经分给「{row.get('joint')}」了（{row.get('ts')}）。"
                         "确认要重写就加 --force。")

    baud, found = find_servos(port, [args.baud] if args.baud else None)
    if not found:
        raise SystemExit("一颗都没扫到，先跑 `python servo_numbering.py scan` 排查接线/供电/COM。")
    if len(found) > 1:
        raise SystemExit(f"总线上发现多个 ID：{sorted(found)}。改 ID 时只能接一颗，拔掉其余的再试。")
    old = next(iter(found))
    print(f"\n当前这颗：ID {old}；目标：ID {target}（{args.joint or join_name(target)}）")

    bus = open_bus(port, baud, timeout=0.02)
    try:
        info = identify(bus, old)
        print_info(info)
        if old == target:
            print(f"\n已经是 ID {target}，不用改。")
            if not args.protect:
                return
        if not args.yes:
            ans = input(f"\n把 ID {old} 改成 {target}"
                        f"{'，并打开过压/过流保护' if args.protect else ''}？[y/N] ").strip().lower()
            if ans not in ("y", "yes"):
                print("已取消。")
                return
        result = assign_id(bus, old, target, joint=args.joint or join_name(target),
                           protect=args.protect)
    finally:
        bus.close()

    entry = {"ts": now(), "port": port, "baud": baud,
             "old": old, "new": target, "joint": args.joint or join_name(target),
             "ok": result["ok"], "protect": result.get("protect"),
             "error": result.get("error")}
    append_ledger(entry)

    if not result["ok"]:
        print(f"\n失败：{result['error']}")
        raise SystemExit(1)
    print(f"\n成功：ID {old} → {target}")
    if result.get("protect"):
        p = result["protect"]
        if p.get("ok"):
            print(f"保护已打开：寄存器 19 = 0x{p['after']:02X}")
        else:
            print(f"保护写入未确认：{p}")
    print(f"\n贴标签：{args.joint or join_name(target)} {target}")
    print("拔掉这颗，换下一颗；每颗都先断电再上电。")


def cmd_verify(args):
    port = resolve_port(args.port)
    plan = plan_for(args.mouth_id)
    expected = {j[0]: j for j in plan}
    baud, found = find_servos(port, [args.baud] if args.baud else None, quiet=True)
    if not found:
        raise SystemExit("一颗都没扫到。")
    got = set(found)
    missing = sorted(set(expected) - got)
    extra = sorted(got - set(expected))
    print(f"\n{port} @ {baud}：在线 {len(got)} 颗")
    print(f"  应有 {len(expected)} 颗：{sorted(expected)}")
    print(f"  实有 {len(got)} 颗：{sorted(got)}")
    if missing:
        print(f"  缺：{[(i, expected[i][2]) for i in missing]}")
    if extra:
        print(f"  多/不在表里：{extra}")
    ok = not missing and not extra
    print("\n" + ("通过：15 个关节 ID 齐了。" if ok else "不通过，按上面的缺/多重查。"))
    if args.json:
        out = _HERE / f"verify-{datetime.now():%Y%m%d-%H%M%S}.json"
        out.write_text(json.dumps({"expected": sorted(expected), "found": sorted(got),
                                   "missing": missing, "extra": extra, "ok": ok},
                                  ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"已存 {out}")
    raise SystemExit(0 if ok else 1)


def cmd_doctor(args):
    print("== 串口 ==")
    ports = list_serial_ports()
    if not ports:
        return
    port = args.port or ports[0].device
    print(f"\n== 探测 {port} ==")
    for baud in (args.baud and [args.baud] or BAUD_CANDIDATES):
        try:
            bus = open_bus(port, baud, timeout=0.01)
        except SystemExit as e:
            print(e)
            return
        try:
            # 只扫最可能的 ID 段，先快后慢
            found = scan_ids(bus, list(range(0, 16)) + list(range(20, 35)), quiet=True)
        finally:
            bus.close()
        print(f"  {baud:>9}: {'发现 ' + str(sorted(found)) if found else '无应答'}")

    # 回显测试：发一个非法短包，若收到原样字节，说明 TX/RX 被短接或处于 echo 模式
    try:
        bus = open_bus(port, args.baud or DEFAULT_BAUD, timeout=0.1)
        try:
            bus.ser.reset_input_buffer()
            bus.ser.write(b"\x55")
            time.sleep(0.05)
            echo = bus.ser.read(8)
            print(f"\n回显测试：{'收到 ' + hx(echo) + '（可能是回显/短接，不是舵机应答）' if echo else '无回显（正常）'}")
        finally:
            bus.close()
    except SystemExit:
        pass

    print("\n软件查不出来的三件事，只能上手：")
    print("  - 总线电压是不是被压在 2.1 V（上电顺序反了）")
    print("  - S/G 有没有接反（不烧，只是搜不到）")
    print("  - 舵机线脚 2 的 Vcc 有没有真的通到舵机")


def cmd_labels(args):
    plan = plan_for(args.mouth_id)
    out_md = _HERE / "舵机标签.md"
    out_csv = _HERE / "舵机标签.csv"
    lines = ["# MicroDuck 舵机标签", "",
             "| ID | 关节 | 贴纸写 |", "|---|---|---|"]
    for sid, key, cn in plan:
        lines.append(f"| {sid} | {key} | {cn} {sid} |")
    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    with out_csv.open("w", encoding="utf-8-sig", newline="") as f:
        f.write("ID,关节,贴纸写\n")
        for sid, key, cn in plan:
            f.write(f"{sid},{key},{cn} {sid}\n")
    print(f"已生成 {out_md}")
    print(f"已生成 {out_csv}（Excel 可直接打开）")
    for sid, key, cn in plan:
        print(f"  {cn:8s}  {sid}")


def cmd_ledger(args):
    rows = load_ledger()
    if not rows:
        print("台账还是空的。")
        return
    print(f"{'时间':20s} {'旧':>4s} {'新':>4s}  {'关节':10s} 结果")
    for r in rows:
        print(f"{r.get('ts','')[:19]:20s} {r.get('old',''):>4} {r.get('new',''):>4}  "
              f"{r.get('joint',''):10s} {'OK' if r.get('ok') else 'FAIL'}"
              + (f"  {r.get('error')}" if r.get("error") else ""))


def cmd_wizard(args):
    port = resolve_port(args.port)
    plan = plan_for(args.mouth_id)
    assigned = {} if args.fresh else assigned_targets()
    remaining = [j for j in plan if j[0] not in assigned]

    print("MicroDuck 舵机编号向导")
    print("规则：一次只接一颗；改完断电重启；贴标签（关节名 + ID）。")
    print(f"当前待编 {len(remaining)} 颗：")
    for sid, key, cn in remaining:
        print(f"  {sid:3d}  {cn}")
    print("\n每次接好一颗后按 Enter 扫描；输入 q 退出。\n")

    while remaining:
        try:
            cmd = input(f"[剩余 {len(remaining)}] 接好一颗，按 Enter 扫描（q 退出）：").strip().lower()
        except EOFError:
            break
        if cmd == "q":
            break

        baud, found = find_servos(port, [args.baud] if args.baud else None, quiet=True)
        if not found:
            print("  没扫到。检查供电/线序/COM，或先跑 doctor。\n")
            continue
        if len(found) > 1:
            print(f"  发现多个 ID：{sorted(found)}。只能接一颗，拔掉其余的再试。\n")
            continue

        old = next(iter(found))
        bus = open_bus(port, baud, timeout=0.02)
        try:
            print(f"\n  当前 ID {old}：")
            print_info(identify(bus, old))
        finally:
            bus.close()

        if old in assigned:
            print(f"  注意：ID {old} 在台账里已经是「{assigned[old].get('joint')}」。"
                  "如果这颗是另一颗舵机，改目标 ID 即可。")
        if old in {j[0] for j in remaining}:
            print(f"  提示：它现在的 ID 正好是待编关节「{join_name(old)}」；"
                  "如果就是这颗，可以直接回车保留，只做复核。")

        print("\n  可选目标：")
        for sid, key, cn in remaining:
            print(f"    {sid:3d}  {cn}")
        try:
            raw = input("  目标 ID（回车=保留当前 ID，q=退出）：").strip().lower()
        except EOFError:
            break
        if raw == "q":
            break
        if raw == "":
            if old not in {j[0] for j in remaining}:
                print("  当前 ID 不在待编表里，必须输入目标。\n")
                continue
            target = old
        else:
            try:
                target = int(raw)
            except ValueError:
                print("  请输入数字 ID。\n")
                continue
        if target not in {j[0] for j in remaining}:
            print(f"  {target} 不在待编表里（可能已经编过）。\n")
            continue

        joint = join_name(target)
        try:
            ans = input(f"  把 {old} 改成 {target}（{joint}）"
                        f"{'，并开保护' if args.protect else ''}？[Y/n] ").strip().lower()
        except EOFError:
            break
        if ans in ("n", "no"):
            print("  跳过。\n")
            continue

        bus = open_bus(port, baud, timeout=0.02)
        try:
            result = assign_id(bus, old, target, joint=joint, protect=args.protect)
        finally:
            bus.close()
        append_ledger({"ts": now(), "port": port, "baud": baud, "old": old,
                       "new": target, "joint": joint, "ok": result["ok"],
                       "protect": result.get("protect"), "error": result.get("error")})
        if not result["ok"]:
            print(f"  失败：{result['error']}\n")
            continue

        assigned[target] = {"joint": joint, "ts": now()}
        remaining = [j for j in remaining if j[0] != target]
        print(f"\n  成功。贴标签：{joint} {target}")
        print("  断电 → 拔掉这颗 → 换下一颗 → 重新上电。\n")

    if remaining:
        print("\n还剩：")
        for sid, key, cn in remaining:
            print(f"  {sid:3d}  {cn}")
    else:
        print("\n15 颗都编完了。把总线恢复成全部接上，跑 `verify` 复核。")


# ---------------------------------------------------------------- 入口

def build_parser():
    p = argparse.ArgumentParser(
        description="MicroDuck 飞特 HD-1910 舵机编号工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument("--port", help="串口，比如 COM5；只有一个串口时可省略")
    p.add_argument("--baud", type=int, help="只试这个波特率（默认自动试 1M/500k/...）")
    p.add_argument("--mouth-id", type=int, default=34,
                   help="嘴的 ID，默认 34；第一台机器历史值是 29；0=不编嘴")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ports", help="列出串口").set_defaults(func=cmd_ports)

    sp = sub.add_parser("scan", help="扫描当前接的舵机并读关键寄存器")
    sp.add_argument("--json", action="store_true", help="把结果存成 JSON")
    sp.add_argument("--quiet", action="store_true")
    sp.set_defaults(func=cmd_scan)

    sp = sub.add_parser("assign", help="把当前这一颗改成目标 ID")
    sp.add_argument("--to", type=int, required=True, help="目标 ID，比如 10")
    sp.add_argument("--joint", help="关节名，比如 右髋yaw；不填按 ID 表补")
    sp.add_argument("--protect", action="store_true", default=True,
                    help="打开过压/过流保护（默认开）")
    sp.add_argument("--no-protect", dest="protect", action="store_false")
    sp.add_argument("--yes", action="store_true", help="不再确认")
    sp.add_argument("--force", action="store_true", help="忽略台账里的重复目标")
    sp.set_defaults(func=cmd_assign)

    sp = sub.add_parser("verify", help="15 颗全接好后核对 ID 是否齐全")
    sp.add_argument("--json", action="store_true")
    sp.set_defaults(func=cmd_verify)

    sub.add_parser("doctor", help="排查「一颗都搜不到」").set_defaults(func=cmd_doctor)

    sp = sub.add_parser("labels", help="生成要贴的标签表")
    sp.set_defaults(func=cmd_labels)

    sub.add_parser("ledger", help="看已经改过哪些").set_defaults(func=cmd_ledger)

    sp = sub.add_parser("wizard", help="交互式，一颗一颗带着做")
    sp.add_argument("--protect", action="store_true", default=True)
    sp.add_argument("--no-protect", dest="protect", action="store_false")
    sp.add_argument("--fresh", action="store_true", help="忽略已有台账，从零开始")
    sp.set_defaults(func=cmd_wizard)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        args.func(args)
    except KeyboardInterrupt:
        print("\n已中断。")
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
