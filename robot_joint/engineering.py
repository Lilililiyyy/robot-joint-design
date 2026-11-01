"""新写的工程参数、参数级BOM及参考布局导出。

仅把当前计算输入转换为CAD可接入的尺寸参数和布局，不创建制造实体、
真实齿廓或采购型号。SI参数原样保留，SVG中显示的mm尺寸另行标注。
"""
from __future__ import annotations

from copy import deepcopy
import csv
from html import escape
import json
import math
from pathlib import Path
from typing import TYPE_CHECKING

from gear_contact.io import json_value

if TYPE_CHECKING:
    from .analysis import JointResult


def _gear_radii(stage: dict) -> dict:
    """对应零变位Gear默认根顶圆；内齿的齿顶朝内、齿根朝外。"""
    module = stage["module_m"]
    cosine = math.cos(stage.get("pressure_angle_rad", math.radians(20)))
    radii = {}
    for name, field in (("sun", "sun_teeth"), ("planet", "planet_teeth"), ("ring", "ring_teeth")):
        teeth = stage[field]
        internal = name == "ring"
        reference = module * teeth / 2
        radii[name] = {
            "reference_radius_m": reference,
            "base_radius_m": reference * cosine,
            "tip_radius_m": module * (teeth - 2 if internal else teeth + 2) / 2,
            "root_radius_m": module * (teeth + 2.5 if internal else teeth - 2.5) / 2,
        }
    return radii


def _parameters(result: "JointResult") -> dict:
    """保留输入业务字段和已计算布局，未知结构尺寸显式登记为未构造。"""
    inputs = result.inputs
    stage = deepcopy(inputs["stage"])
    spacing = inputs["bearings"]["spacing_m"]
    return {
        "schema_version": 1,
        "case_id": inputs["case_id"],
        "units": {"length": "m", "angle": "rad", "force": "N", "torque": "N m",
                  "pressure": "Pa", "stiffness": "N/m"},
        "stage": stage,
        "geometry": json_value(result.geometry),
        "gear_radii_m": _gear_radii(stage),
        "face_width_m": stage["face_width_m"],
        "bearings": {
            "spacing_m": spacing,
            "centre_z_m": [-spacing / 2, spacing / 2],
            "load_offset_m": inputs["bearings"].get("load_offset_m", 0.0),
            "geometry_status": "仅定义双支承轴向中心位置，未提供轴承直径、宽度或安装配合",
        },
        "coordinate_system": "x/y为齿轮平面，z为旋转轴；原点为双支承轴向中点",
        "solid_constructed": False,
        "manufacturing_tooth_profile_constructed": False,
        "scope": "参数级布局与CAD接入参数；未构造实体、真实齿面、根部过渡、孔位公差或装配工艺",
    }


def _bom_rows(result: "JointResult") -> list[dict]:
    """数量来自拓扑，规格只使用输入参数；不推测型号或未知外形。"""
    inputs = result.inputs
    stage = inputs["stage"]
    material = deepcopy(inputs.get("mesh", {}).get("material", {}))
    common = {key: stage[key] for key in ("module_m", "face_width_m")}
    common["pressure_angle_rad"] = stage.get("pressure_angle_rad", math.radians(20))
    common["elastic_material_parameters"] = material
    count = stage["planet_count"]
    rows = []

    def append(item_id, name, quantity, specification, note):
        rows.append({
            "item_id": item_id,
            "part_name": name,
            "quantity": quantity,
            "specification": json.dumps(json_value(specification), ensure_ascii=False,
                                        separators=(",", ":"), allow_nan=False),
            "procurement_model": "",
            "note": note,
        })

    append("sun", "太阳轮", 1, {**common, "teeth": stage["sun_teeth"], "internal": False},
           "弹性材料参数不等同完整材料牌号、热处理及制造规范")
    append("planet", "行星轮", count, {**common, "teeth": stage["planet_teeth"], "internal": False},
           "各轮同名义几何，制造误差由载荷分配输入另行描述")
    append("ring", "内齿圈", 1, {**common, "teeth": stage["ring_teeth"], "internal": True},
           "固定内齿圈，未提供轮缘和壳体实体尺寸")
    append("carrier", "行星架", 1, {"planet_count": count},
           "仅已知行星数量与计算布局，未提供架板厚度、销孔尺寸及连接结构")
    append("planet_pin", "行星销", count,
           {"tangential_stiffness_n_per_m": deepcopy(inputs["load_sharing"]["pin_stiffness_n_per_m"])},
           "仅有等效支承刚度，未提供销径、长度或材料牌号")
    append("output_bearing", "输出支承", 2, deepcopy(inputs["bearings"]),
           "两组评级参数按负z、正z支承顺序给定；未指定采购型号")
    append("motor", "电机", 1, deepcopy(inputs["motor"]),
           "仅有计算输入的机电参数，未指定采购型号或安装包络")
    append("brake", "制动器", 1,
           {key: deepcopy(inputs["drive"][key]) for key in ("brake_torque_n_m", "brake_safety_factor")
            if key in inputs["drive"]},
           "输入侧静态制动能力参数，未指定采购型号或动态制动认证")
    append("encoder", "编码器", 1,
           {key: deepcopy(inputs["precision"][key]) for key in
            ("encoder_counts_per_revolution", "encoder_on_output") if key in inputs["precision"]},
           "每转计数及安装侧来自输入，未指定精度等级、接口或采购型号")
    return rows


def _layout_svg(parameters: dict) -> str:
    """绘制参考圆、根顶圆和销中心圆；无需脚本、外链或外部字体文件。"""
    stage = parameters["stage"]
    geometry = parameters["geometry"]
    radii = parameters["gear_radii_m"]
    outer = radii["ring"]["root_radius_m"]
    scale = 235 / outer
    centre = (315.0, 330.0)
    title = escape(str(parameters["case_id"]))
    output = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="650" viewBox="0 0 1000 650" role="img" aria-labelledby="title description">',
        f'<title id="title">{title}：NGW参数布局</title>',
        '<desc id="description">参考圆、齿顶圆、齿根圆和行星销中心圆构成的参数布局；不是制造齿廓或实体模型。</desc>',
        '<rect width="1000" height="650" fill="#f7f9fb"/>',
        '<g font-family="sans-serif" fill="#243241">',
        f'<text x="36" y="42" font-size="24">{title}</text>',
        '<text x="36" y="70" font-size="14">单级固定内齿圈 · 太阳轮输入 · 行星架输出 · 零变位直齿</text>',
        '<g id="ngw_layout" data-length-unit="m" fill="none">',
    ]

    def circle(identifier, x_m, y_m, radius_m, color, *, dash=None, width=1.3):
        cx = centre[0] + scale * x_m
        cy = centre[1] - scale * y_m
        dash_attribute = f' stroke-dasharray="{dash}"' if dash else ""
        output.append(f'<circle id="{identifier}" cx="{cx:.12g}" cy="{cy:.12g}" r="{scale*radius_m:.12g}" '
                      f'data-centre-x-m="{x_m:.17g}" data-centre-y-m="{y_m:.17g}" '
                      f'data-radius-m="{radius_m:.17g}" stroke="{color}" stroke-width="{width}"{dash_attribute}/>')

    for kind in ("root", "tip", "reference"):
        circle(f"ring_{kind}", 0, 0, radii["ring"][kind + "_radius_m"],
               "#657383" if kind != "reference" else "#273d4d",
               dash="7 5" if kind == "reference" else None, width=1.8)
    circle("carrier_pitch", 0, 0, geometry["carrier_radius_m"], "#23856b", dash="4 6")
    for kind in ("root", "tip", "reference"):
        circle(f"sun_{kind}", 0, 0, radii["sun"][kind + "_radius_m"], "#bb6a24",
               dash="6 4" if kind == "reference" else None)
    for index, (x, y) in enumerate(geometry["planet_centres_xy_m"], 1):
        output.append(f'<line x1="{centre[0]}" y1="{centre[1]}" '
                      f'x2="{centre[0]+scale*x:.12g}" y2="{centre[1]-scale*y:.12g}" '
                      'stroke="#b9c9c3" stroke-width="1" stroke-dasharray="3 6"/>')
        for kind in ("root", "tip", "reference"):
            circle(f"planet_{index}_{kind}", x, y, radii["planet"][kind + "_radius_m"], "#3a77a4",
                   dash="6 4" if kind == "reference" else None)
        # 十字仅标销中心，不用缺失的销直径伪造圆形安装包络。
        px, py = centre[0] + scale * x, centre[1] - scale * y
        output.append(f'<path d="M {px-4:.12g} {py:.12g} h 8 M {px:.12g} {py-4:.12g} v 8" '
                      'stroke="#23856b" stroke-width="1.5"/>')
    output.extend(['</g>', '<g font-size="15">',
                   '<text x="620" y="145" font-size="20">参数与图义</text>'])
    notes = [
        f'齿数 zs / zp / zr：{stage["sun_teeth"]} / {stage["planet_teeth"]} / {stage["ring_teeth"]}',
        f'行星数量：{stage["planet_count"]}；速比：{geometry["ratio"]:.6g}',
        f'模数：{stage["module_m"]*1000:.6g} mm',
        f'齿宽：{stage["face_width_m"]*1000:.6g} mm',
        f'销中心圆半径：{geometry["carrier_radius_m"]*1000:.6g} mm',
        f'双支承间距：{parameters["bearings"]["spacing_m"]*1000:.6g} mm',
        '虚线：齿轮参考圆；绿色点划圆：销中心圆',
        '细实线：默认齿顶圆与齿根圆',
        '十字：销中心；辐射线仅辅助表示方位',
        '参数文件统一采用 m / rad 等SI单位',
    ]
    for index, note in enumerate(notes):
        output.append(f'<text x="620" y="{182+31*index}">{escape(note)}</text>')
    output.extend(['</g>',
                   '<text x="36" y="605" font-size="15">参数布局；圆线不代表真实制造齿廓，未构造三维实体。</text>',
                   '<text x="36" y="629" font-size="13">未提供电机、轴承和制动器包络，图中不绘制其外形；孔位公差、配合与工艺需另行设计。</text>',
                   '</g>', '</svg>', ''])
    return '\n'.join(output)


def export_engineering_parameters(result: "JointResult", output_dir) -> list[str]:
    """导出CAD参数、参数级BOM与布局SVG，返回三个实际输出路径。

    所有数量和规格来自拓扑与当前输入。采购型号留空，未知实体尺寸
    不补默认值。输出目录由调用者决定，不依赖REA目录或当前工作目录。
    """
    root = Path(output_dir)
    root.mkdir(parents=True, exist_ok=True)
    parameters = _parameters(result)
    parameter_path = root / "cad_parameters.json"
    parameter_path.write_text(json.dumps(json_value(parameters), ensure_ascii=False, indent=2,
                                         allow_nan=False) + '\n', encoding="utf-8")
    bom_path = root / "bom.csv"
    with bom_path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("item_id", "part_name", "quantity", "specification",
                                                   "procurement_model", "note"))
        writer.writeheader()
        writer.writerows(_bom_rows(result))
    svg_path = root / "layout.svg"
    svg_path.write_text(_layout_svg(parameters), encoding="utf-8")
    return [str(parameter_path), str(bom_path), str(svg_path)]
