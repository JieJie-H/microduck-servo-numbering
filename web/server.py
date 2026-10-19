#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MicroDuck 舵机编号向导 · 本地服务

只监听 127.0.0.1。静态页面 + 几个 JSON 接口：
  GET  /api/config      关节表、说明、串口
  GET  /api/state       台账 + 手动标记
  GET  /api/ports       串口列表
  POST /api/scan        扫描当前总线上的舵机
  POST /api/assign      把当前这一颗改成目标 ID
  POST /api/verify      总核对
  POST /api/mark        手动标记/取消某个 ID 已完成

串口同一时刻只能被一个程序占用；本服务每次请求现开现关，不长期占用。
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import webbrowser
from datetime import datetime
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

WEB = Path(__file__).resolve().parent
ROOT = WEB.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "test_support"))
sys.path.insert(0, str(WEB))

import servo_numbering as sn  # noqa: E402

STATE_FILE = WEB / "web_state.json"
SERIAL_LOCK = threading.Lock()
DEFAULT_SERIAL = None
FAKE = False
FAKE_SERIAL = None

# 每个关节在实物上的位置 / 接线提示。ID 与 sn.JOINTS 对齐。
JOINT_INFO = {
    10: ("right", "右腿最上端、连接躯干", "整条腿绕竖直轴内外转。右腿最靠前、最靠上的那颗。"),
    11: ("right", "右腿髋部侧面", "腿向外张 / 向内收。在 10 号下面一点、腿的外侧。"),
    12: ("right", "右腿大腿根", "大腿前后摆。侧看时在髋部、大腿和躯干交界处。"),
    13: ("right", "右腿膝盖", "大腿和小腿之间，弯膝的那颗。"),
    14: ("right", "右腿脚踝", "小腿和脚板之间，脚板上下翘。"),
    20: ("left", "左腿最上端、连接躯干", "整条腿绕竖直轴内外转。左腿最靠前、最靠上的那颗。"),
    21: ("left", "左腿髋部侧面", "腿向外张 / 向内收。在 20 号下面一点、腿的外侧。"),
    22: ("left", "左腿大腿根", "大腿前后摆。侧看时在髋部、大腿和躯干交界处。"),
    23: ("left", "左腿膝盖", "大腿和小腿之间，弯膝的那颗。"),
    24: ("left", "左腿脚踝", "小腿和脚板之间，脚板上下翘。"),
    30: ("head", "脖子根部", "头整体前后点。躯干和脖子连接处。"),
    31: ("head", "头与脖子连接处", "头单独前后点。30 号上面、头壳下方。"),
    32: ("head", "头部中轴", "头左右转。头壳中间偏后。"),
    33: ("head", "头部侧面", "头左右歪。头壳侧面、靠近耳朵的位置。"),
    34: ("head", "嘴 / 下颌", "嘴开合。头壳前下方、下巴处；这个关节不在策略输出里，但 ID 要留。"),
}


def plan_payload():
    out = []
    for sid, key, cn in sn.plan_for(34):
        side, location, hint = JOINT_INFO.get(sid, ("head", "", ""))
        out.append({"id": sid, "key": key, "cn": cn, "side": side,
                    "location": location, "hint": hint})
    return out


def load_web_state():
    if not STATE_FILE.exists():
        return {"manual_done": [], "notes": {}}
    try:
        return json.loads(STATE_FILE.read_text(encoding="utf-8"))
    except Exception:
        return {"manual_done": [], "notes": {}}


def save_web_state(state):
    tmp = STATE_FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(STATE_FILE)


def done_ids():
    ids = {row["new"] for row in sn.load_ledger()
           if row.get("ok") and row.get("new") is not None}
    ids |= {int(x) for x in load_web_state().get("manual_done", [])}
    return sorted(ids)


def state_payload():
    entries = sn.load_ledger()
    return {
        "plan": plan_payload(),
        "done": done_ids(),
        "entries": entries,
        "manual_done": load_web_state().get("manual_done", []),
        "ledger_path": str(sn.LEDGER),
    }


def ports_payload():
    if FAKE:
        return [{"device": "FAKE", "description": "模拟总线（不接硬件，仅演示）"}]
    from serial.tools import list_ports
    ports = [{"device": p.device, "description": p.description}
             for p in sorted(list_ports.comports(), key=lambda x: x.device)]
    return ports


def pick_port(body):
    if FAKE:
        return "FAKE"
    port = (body or {}).get("port") or DEFAULT_SERIAL
    if port:
        return port
    ports = ports_payload()
    if len(ports) == 1:
        return ports[0]["device"]
    if not ports:
        raise RuntimeError("没有发现串口。检查 URT-2 的 USB 和 CH343 驱动。")
    raise RuntimeError("有多个串口，请先在页面顶栏选一个。")


def fake_servo_ids():
    return sorted({s.id for s in FAKE_SERIAL.servos.values()})


def fake_bus():
    return sn.Bus(ser=FAKE_SERIAL)


def scan_current(port):
    if FAKE:
        ids = fake_servo_ids()
        bus = fake_bus()
        servos = [sn.identify(bus, sid) for sid in ids]
        return {"ok": bool(servos), "port": "FAKE", "baud": 1_000_000,
                "servos": servos, "fake": True}
    baud, found = sn.find_servos(port, quiet=True)
    if not found:
        return {"ok": False, "port": port, "error": "总线上没有舵机应答",
                "hint": ["先插 URT-2 的 USB，再给舵机上电",
                         "脚 1→S、脚 2→V1、脚 3→G；S/G 反了不烧但搜不到",
                         "只接一颗；USB 5 V 就够配 ID",
                         "FD / 调试台是不是还占着 COM 口"]}
    bus = sn.open_bus(port, baud, timeout=0.02)
    try:
        servos = [sn.identify(bus, sid) for sid in sorted(found)]
    finally:
        bus.close()
    return {"ok": True, "port": port, "baud": baud, "servos": servos}


def log(msg):
    print(f"[{datetime.now():%H:%M:%S}] {msg}", flush=True)


class Handler(SimpleHTTPRequestHandler):
    server_version = "MicroDuckNumbering/1.0"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(WEB), **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, fmt, *args):
        # 静态文件不刷屏；只打 API
        if "/api/" in (self.path or ""):
            log(fmt % args)

    def _json(self, data, code=200):
        raw = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        if not n:
            return {}
        return json.loads(self.rfile.read(n).decode("utf-8") or "{}")

    # ---- GET ----
    def do_GET(self):
        path = self.path.split("?", 1)[0]
        try:
            if path == "/api/config":
                return self._json({"ok": True, "plan": plan_payload(),
                                   "ports": ports_payload(),
                                   "default_port": "FAKE" if FAKE else DEFAULT_SERIAL,
                                   "fake": FAKE,
                                   "ledger_path": str(sn.LEDGER)})
            if path == "/api/state":
                return self._json({"ok": True, **state_payload()})
            if path == "/api/ports":
                return self._json({"ok": True, "ports": ports_payload()})
            if path == "/api/verify":
                port = self.headers.get("X-Port") or DEFAULT_SERIAL
                with SERIAL_LOCK:
                    port = pick_port({"port": port})
                    if FAKE:
                        baud = 1_000_000
                        found = {sid: 0 for sid in fake_servo_ids()}
                    else:
                        baud, found = sn.find_servos(port, quiet=True)
                    got = set(found)
                    expected = {j["id"] for j in plan_payload()}
                    missing = sorted(expected - got)
                    extra = sorted(got - expected)
                    return self._json({"ok": True, "port": port, "baud": baud,
                                       "found": sorted(got), "missing": missing,
                                       "extra": extra,
                                       "pass": not missing and not extra})
        except (Exception, SystemExit) as e:
            return self._json({"ok": False, "error": str(e)}, 500)
        return super().do_GET()

    # ---- POST ----
    def do_POST(self):
        path = self.path.split("?", 1)[0]
        try:
            body = self._body()
        except Exception as e:
            return self._json({"ok": False, "error": f"请求体不是 JSON：{e}"}, 400)

        try:
            if path == "/api/scan":
                with SERIAL_LOCK:
                    port = pick_port(body)
                    return self._json(scan_current(port))

            if path == "/api/assign":
                target = int(body["target"])
                if target not in {j["id"] for j in plan_payload()}:
                    return self._json({"ok": False, "error": f"{target} 不在 ID 表里"}, 400)
                if target in done_ids() and not body.get("force"):
                    return self._json({"ok": False, "error": "already_done",
                                       "message": f"台账里 {target} 已经标记完成；要重写请加 force"}, 409)
                with SERIAL_LOCK:
                    port = pick_port(body)
                    if FAKE:
                        baud = 1_000_000
                        found = {sid: 0 for sid in fake_servo_ids()}
                    else:
                        baud, found = sn.find_servos(port, quiet=True)
                    if not found:
                        return self._json({"ok": False, "error": "总线上没有舵机应答"})
                    if len(found) > 1:
                        return self._json({"ok": False,
                                           "error": f"总线上发现多个 ID：{sorted(found)}；只接一颗再改"})
                    old = next(iter(found))
                    joint = body.get("joint") or sn.join_name(target)
                    bus = fake_bus() if FAKE else sn.open_bus(port, baud, timeout=0.02)
                    try:
                        result = sn.assign_id(bus, old, target, joint=joint,
                                              protect=bool(body.get("protect", True)))
                    finally:
                        bus.close()
                    sn.append_ledger({"ts": sn.now(), "port": port, "baud": baud,
                                      "old": old, "new": target, "joint": joint,
                                      "ok": result["ok"], "protect": result.get("protect"),
                                      "error": result.get("error"), "source": "web"})
                    log(f"改 ID {old}→{target} {'成功' if result['ok'] else '失败：' + str(result.get('error'))}")
                    return self._json({"ok": result["ok"], "old": old, "target": target,
                                       "joint": joint, **result, "state": state_payload()})

            if path == "/api/verify":
                with SERIAL_LOCK:
                    port = pick_port(body)
                    if FAKE:
                        baud = 1_000_000
                        found = {sid: 0 for sid in fake_servo_ids()}
                    else:
                        baud, found = sn.find_servos(port, quiet=True)
                    got = set(found)
                    expected = {j["id"] for j in plan_payload()}
                    missing = sorted(expected - got)
                    extra = sorted(got - expected)
                    return self._json({"ok": True, "port": port, "baud": baud,
                                       "found": sorted(got), "missing": missing,
                                       "extra": extra, "pass": not missing and not extra})

            if path == "/api/mark":
                target = int(body["target"])
                done = bool(body.get("done", True))
                st = load_web_state()
                cur = {int(x) for x in st.get("manual_done", [])}
                if done:
                    cur.add(target)
                else:
                    cur.discard(target)
                st["manual_done"] = sorted(cur)
                save_web_state(st)
                return self._json({"ok": True, "state": state_payload()})

            return self._json({"ok": False, "error": "未知接口"}, 404)
        except (Exception, SystemExit) as e:
            log(f"API 出错：{e}")
            return self._json({"ok": False, "error": str(e)}, 500)


def main(argv=None):
    global DEFAULT_SERIAL, FAKE, FAKE_SERIAL
    ap = argparse.ArgumentParser(description="MicroDuck 舵机编号向导")
    ap.add_argument("--http-port", type=int, default=8090)
    ap.add_argument("--serial", help="默认串口，比如 COM3；不填则自动选")
    ap.add_argument("--fake", action="store_true",
                    help="模拟总线（不接硬件，只演示页面和改 ID 流程）")
    ap.add_argument("--open", action="store_true", help="启动后自动打开浏览器")
    args = ap.parse_args(argv)
    DEFAULT_SERIAL = args.serial
    FAKE = args.fake
    if FAKE:
        try:
            import sim_bus
        except ImportError as e:
            print(f"模拟模式需要 test_support/ 里的 sim_bus：{e}")
            return 2
        FAKE_SERIAL = sim_bus.SimSerial([1])
        # 模拟模式用独立台账，别把真机台账弄脏
        sn.LEDGER = WEB / "fake_servo_ledger.jsonl"

    httpd = ThreadingHTTPServer(("127.0.0.1", args.http_port), Handler)
    httpd.daemon_threads = True
    url = f"http://127.0.0.1:{args.http_port}/"
    print(f"MicroDuck 舵机编号向导已启动：{url}")
    print(f"台账：{sn.LEDGER}")
    if FAKE:
        print("⚠ 模拟模式：总线上是一颗虚拟舵机（ID 1），不会碰真硬件。")
    print("按 Ctrl+C 停止。串口同一时刻只能被一个程序占用（FD / 调试台先关）。")
    if args.open:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n已停止。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
