# 署名与来源 / Attribution

本工具是 **MicroDuck 的第三方社区工具**，与 Pollen Robotics 无隶属关系，也未获其背书。

## 许可证范围

| 内容 | 许可证 | 说明 |
|---|---|---|
| `servo_numbering.py`、`web/server.py`、`web/index.html`、`web/app.js`、`web/style.css`、`selftest.py` | **MIT** | 本工具原创代码，见 `LICENSE` |
| `test_support/feetech.py`、`test_support/sim_bus.py` | **Apache-2.0** | 来自 [fanhao375/microduck-replica](https://github.com/fanhao375/microduck-replica)，全文见 `LICENSES/Apache-2.0.txt` |
| `web/model/model.json`、`web/model/meshes.bin` | **CC BY-NC-SA 4.0** | Microduck 上游 3D 模型的衍生作品；非商业、署名、相同方式共享。全文见 `LICENSES/CC-BY-NC-SA-4.0.txt` |
| `preview.png`、`labels/`、`README.md` | 随工具 | 预览图含 3D 模型画面，按 CC BY-NC-SA 4.0 对待 |

## 上游来源

| 来源 | 作者 | 许可证 |
|---|---|---|
| [pollen-robotics/microduck_rl](https://github.com/pollen-robotics/microduck_rl) | Pollen Robotics | 代码 Apache-2.0；3D 模型 CC BY-NC-SA |
| [apirrone/microduck_rl](https://github.com/apirrone/microduck_rl) | apirrone | Apache-2.0；本工具使用的 MJCF/STL 来源 |
| [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck) | Pollen Robotics | Apache-2.0 |
| [fanhao375/microduck-replica](https://github.com/fanhao375/microduck-replica) | fanhao375 | 代码 Apache-2.0；`tools/servo-web/model/` 为 CC BY-NC-SA 4.0 |

## 3D 模型

`web/model/model.json` 和 `meshes.bin` 由 `microduck-replica` 的 `tools/servo-web/build_model.py`
从 apirrone/microduck_rl 的 `robot_allcollisions.xml` 及 `assets/*.stl` 转换而来；网格本身是
Pollen Robotics 官方 Microduck CAD 导出的可见外形，顶点做了 int16 量化。

依据上游与 `microduck-replica` 的许可证说明，本工具中的这份转换模型以
**CC BY-NC-SA 4.0** 发布：

- **署名**：Pollen Robotics（Microduck 3D 模型）、apirrone（MJCF/STL 来源）、fanhao375（转换与网页模型）；
- **非商业**：不得用于商业目的、付费售卖或闭源商用；
- **相同方式共享**：修改后的模型/衍生作品必须沿用 CC BY-NC-SA 4.0。

## 协议实现

`test_support/feetech.py` 和 `test_support/sim_bus.py` 来自 `microduck-replica` 的
`tools/servo-web/`，原项目以 Apache-2.0 发布。为了让这套代码在 Python 3.11 上运行，
`feetech.py` 中一行 f-string（原第 143 行，表达式里带反斜杠）被改成先取变量再拼接；
协议、寄存器表和行为未改。

## 非隶属声明

Microduck 是 Pollen Robotics 的产品。本工具不是官方工具，不包含 Pollen Robotics 的
硬件设计文件、固件或商业支持。使用时请遵守当地法律和上游许可证。

## 链接

- Apache-2.0: https://www.apache.org/licenses/LICENSE-2.0
- CC BY-NC-SA 4.0: https://creativecommons.org/licenses/by-nc-sa/4.0/
- Microduck: https://pollen-robotics.com/microduck/
