# -*- coding: utf-8 -*-
"""不用硬件的自检：让真实的 servo_numbering.Bus 对着主仓的协议级模拟器收发。"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "test_support"))

import servo_numbering as sn
import sim_bus


def make_bus(ids):
    return sn.Bus(ser=sim_bus.SimSerial(ids))


def test_scan_and_identify():
    bus = make_bus([7])
    assert bus.ping(7) == 0
    assert bus.ping(8) is None
    info = sn.identify(bus, 7)
    assert info["id_reg"] == 7, info
    assert info["model"] == "3.10", info
    assert info["pos"] == 2048, info
    print("scan/identify ok:", info)


def test_assign_1_to_10():
    bus = make_bus([1])
    result = sn.assign_id(bus, 1, 10, "右髋yaw", protect=True, reboot_wait=0.05)
    assert result["ok"], result
    assert bus.ping(10) == 0
    assert bus.ping(1) is None
    assert bus.read_u8(10, 5) == 10
    assert bus.read_u8(10, sn.REG_LOCK) == 1
    # 模拟器出厂 19=0，所以置位后是 0x09；真机出厂 4，会变成 0x0D
    assert bus.read_u8(10, sn.REG_UNLOAD) == 9, hex(bus.read_u8(10, sn.REG_UNLOAD))
    print("assign 1->10 ok:", result)


def test_assign_34_to_21():
    # 第一台机器的历史改法：原嘴 34 挪作 21
    bus = make_bus([34])
    result = sn.assign_id(bus, 34, 21, "左髋roll", protect=True, reboot_wait=0.05)
    assert result["ok"], result
    assert bus.ping(21) == 0 and bus.ping(34) is None
    print("assign 34->21 ok")


def test_refuse_200():
    bus = make_bus([1])
    result = sn.assign_id(bus, 1, 200, protect=False, reboot_wait=0.01)
    assert not result["ok"] and "200" in result["error"]
    assert bus.ping(1) == 0
    print("refuse 200 ok")


def test_no_servo():
    bus = make_bus([])
    result = sn.assign_id(bus, 1, 10, protect=False, reboot_wait=0.01)
    assert not result["ok"]
    print("no servo handled:", result["error"])


def test_plan():
    plan = sn.plan_for(34)
    assert [j[0] for j in plan] == [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34]
    plan29 = sn.plan_for(29)
    assert [j[0] for j in plan29] == [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 29, 30, 31, 32, 33]
    print("plan ok")


if __name__ == "__main__":
    test_plan()
    test_scan_and_identify()
    test_assign_1_to_10()
    test_assign_34_to_21()
    test_refuse_200()
    test_no_servo()
    print("\nALL TESTS PASSED")
