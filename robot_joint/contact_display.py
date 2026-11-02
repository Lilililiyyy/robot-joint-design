"""齿面接触后处理：材料坐标压力显示与瞬时局部接触带。

本模块不求解载荷、修改压力或补造独立端点。中心模型只将首尾单元的
常值显示支撑扩到该单元真实边界；实际啮合域、原始采样与显示网格分开保存。
"""
from pathlib import Path

import numpy as np


def contact_pressure_colormap():
    """本项目统一采用蓝红色带；采用材料坐标和局部峰压显示。"""
    from matplotlib.colors import LinearSegmentedColormap
    cmap = LinearSegmentedColormap.from_list('pressure_blue_to_red',
        ['#2166ac', '#92c5de', '#f4d3d2', '#b2182b'], N=256)
    cmap.set_bad('#e5e9ee')
    return cmap


def project_contact_pass_samples(result):
    """保留独立端点压力与延伸标记；坐标为小轮左端起齿宽及小轮展长，单位m。"""
    samples = []
    for rotation in result.rotations:
        geometry, pressure = rotation.contact_geometry_data, rotation.endpoint_pressure
        if geometry is None or pressure is None:
            raise ValueError('端点通过程需要完整接触线几何和独立端点压力')
        ends = geometry.endpoints
        p = np.asarray(pressure['pressure_pa'])
        if p.shape != ends.face_coordinate_m.shape or np.any(~np.isfinite(p)) or np.any(p < 0):
            raise ValueError('端点压力与几何布局不一致')
        samples.append(np.column_stack((ends.face_coordinate_m+result.pair.pinion.face_width_m/2,
            ends.roll_pinion_m, p, ends.extended_pinion, ends.extended_wheel)))
    return np.concatenate(samples)


def contact_pass_data(result, *, samples=(201, 241), face_coordinate_m=None, roll_distance_m=None):
    """输出实际啮合区的显示数据；指定坐标时可复用已绑定参照的显示网格。

    两轮接触展长交集不等于小轮完整渐开线。中心模型的边界支撑为原积分
    单元内常值显示延拓，不能解释为新增求解点；凸包外继续保留NaN。
    """
    boundary = []
    endpoints = [r.endpoint_pressure is not None for r in result.rotations]
    if all(endpoints):
        points = project_contact_pass_samples(result)
        definition = '独立端点压力'
        support_definition = '原独立端点，无额外边界显示延拓'
    elif any(endpoints) or any(r.contact_geometry_data is not None for r in result.rotations):
        raise ValueError('不能混合缺少端点压力或接触几何的相位')
    else:
        points = np.concatenate([np.column_stack((
            r.grid.face_coordinate_m+result.pair.pinion.face_width_m/2, r.grid.roll_pinion_m,
            r.pressure_pa, np.zeros((len(r.pressure_pa), 2)))) for r in result.rotations])
        definition = '条带中心压力（当前模型无独立端点）'
        cosine = np.cos(result.pair.base_helix_angle_rad)
        slope = np.tan(result.pair.base_helix_angle_rad)
        for rotation in result.rotations:
            grid = rotation.grid
            for pair_index in np.unique(grid.tooth_pair_index):
                indices = np.flatnonzero(grid.tooth_pair_index == pair_index)
                indices = indices[np.argsort(grid.face_coordinate_m[indices])]
                for index, sign in ((indices[0], -1), (indices[-1], 1)):
                    # widths_m是接触线长；先投影到轴向，再沿真实接触线到边界。
                    delta = sign*grid.widths_m[index]*cosine/2
                    boundary.append((grid.face_coordinate_m[index]+delta+result.pair.pinion.face_width_m/2,
                        grid.roll_pinion_m[index]+delta*slope, rotation.pressure_pa[index], 0., 0.))
        support_definition = '首尾中心值仅延拓到各自原积分单元边界；不是独立端点解'
    boundary = np.asarray(boundary, dtype=float).reshape(-1, 5)
    ordinary = points[~np.any(points[:, 3:5].astype(bool), axis=1)]
    display_samples = np.concatenate((ordinary, boundary))
    unique, inverse = np.unique(np.round(display_samples[:, :2], 12), axis=0, return_inverse=True)
    values = np.full(len(unique), -np.inf)
    np.maximum.at(values, inverse, display_samples[:, 2])
    left, right = result.pair.face_bounds_m
    x = (np.linspace(left+result.pair.pinion.face_width_m/2, right+result.pair.pinion.face_width_m/2, samples[0])
         if face_coordinate_m is None else np.asarray(face_coordinate_m, dtype=float))
    y = (np.linspace(*result.pair.contact_roll_bounds_m, samples[1])
         if roll_distance_m is None else np.asarray(roll_distance_m, dtype=float))
    for axis in (x, y):
        if axis.ndim != 1 or len(axis) < 2 or np.any(~np.isfinite(axis)) or np.any(np.diff(axis) <= 0):
            raise ValueError('显示坐标须为至少两个有限且严格递增的SI位置')
    field = np.full((len(x), len(y)), np.nan)
    interpolated = len(unique) >= 3 and np.linalg.matrix_rank(unique-unique.mean(axis=0), tol=1e-12) == 2
    if interpolated:
        import matplotlib.tri as tri
        triangulation = tri.Triangulation(unique[:, 0], unique[:, 1])
        xx, yy = np.meshgrid(x, y, indexing='ij')
        field = np.asarray(tri.LinearTriInterpolator(triangulation, values)(
            np.round(xx, 12), np.round(yy, 12)).filled(np.nan))
    return {'face_coordinate_m': x, 'roll_distance_m': y, 'pressure_pa_x_y': field,
            'samples': points, 'ordinary_samples': ordinary, 'boundary_display_samples': boundary,
            'contact_roll_bounds_m': np.asarray(result.pair.contact_roll_bounds_m),
            'pinion_involute_roll_bounds_m': np.asarray(result.pair.pinion.involute_roll_bounds_m),
            'definition': definition, 'support_definition': support_definition, 'interpolated': bool(interpolated)}


def draw_contact_pass(ax, data, *, maximum_pa, title=None, language="zh", member="pinion"):
    """接触分析单图、参照对比和候选比较共用的填色、等压线、坐标和背景。"""
    cmap = contact_pressure_colormap()
    maximum = max(float(maximum_pa)*1e-6, 1.)
    if data['interpolated']:
        field = np.ma.masked_invalid(data['pressure_pa_x_y'].T)*1e-6
        image = ax.pcolormesh(data['face_coordinate_m']*1e3, data['roll_distance_m']*1e3, field,
                             shading='gouraud', cmap=cmap, vmin=0., vmax=maximum, rasterized=True)
        if field.count() and float(field.max()) > float(field.min()):
            ax.contour(data['face_coordinate_m']*1e3, data['roll_distance_m']*1e3, field,
                       levels=np.linspace(0., maximum, 31)[1:-1], colors='black', linewidths=.35)
    else:
        own = data['ordinary_samples']
        image = ax.scatter(own[:, 0]*1e3, own[:, 1]*1e3, c=own[:, 2]*1e-6, cmap=cmap, vmin=0., vmax=maximum)
    ax.set(xlim=data['face_coordinate_m'][[0, -1]]*1e3, ylim=data['roll_distance_m'][[0, -1]]*1e3,
           xlabel='齿宽坐标 / mm' if language=='zh' else 'Face coordinate / mm',
           ylabel=('小轮展长 / mm' if member=='pinion' else '大轮展长 / mm') if language=='zh' else ('Pinion roll / mm' if member=='pinion' else 'Wheel roll / mm'))
    if title is not None:
        ax.set_title(title)
    ax.set_facecolor('#e5e9ee')
    return image


def plot_contact_pass_comparison(results, output):
    """同一齿轮副的候选通过程图：统一坐标及压力色标，指标仍取原数组。"""
    from gear_contact.io import write_json
    from gear_contact.plotting import _pyplot, _save
    if not results:
        raise ValueError('接触比较至少需要一个方案')
    first = next(iter(results.values()))
    if any(r.pair != first.pair for r in results.values()):
        raise ValueError('统一材料坐标比较要求相同齿轮副几何')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    plt = _pyplot()
    data = {name: contact_pass_data(result) for name, result in results.items()}
    maximum = max(float(d['samples'][:, 2].max()) for d in data.values())
    fig, axes = plt.subplots(1, len(results), figsize=(max(8., len(results)*4.4), 5.6), squeeze=False,
                             sharex=True, sharey=True)
    fig.subplots_adjust(left=.08, right=.86, bottom=.24, top=.84, wspace=.25)
    records, arrays = [], {}
    for index, ((name, result), ax) in enumerate(zip(results.items(), axes.flat)):
        d = data[name]
        peak_name = '端点峰压' if d['definition'] == '独立端点压力' else '中心峰压'
        image = draw_contact_pass(ax, d, maximum_pa=maximum,
            title=f"{name}\n{peak_name} {d['samples'][:, 2].max()*1e-6:.1f} MPa；PPTE {result.summary['te_peak_to_peak_m']*1e6:.3f} μm")
        for key, value in d.items():
            if isinstance(value, np.ndarray):
                arrays[f'case_{index+1:03d}_{key}'] = value
        records.append({'design': name, 'pressure_definition': d['definition'],
                        'support_definition': d['support_definition'], 'summary': result.summary,
                        'uncovered_display_count': int(np.count_nonzero(~np.isfinite(d['pressure_pa_x_y'])))})
    fig.colorbar(image, cax=fig.add_axes((.90, .30, .018, .48)), label='接触压力 / MPa')
    fig.suptitle('小轮整齿通过程接触压力比较', fontsize=13)
    fig.text(.5, .065, '坐标限于两轮实际啮合区；重复材料点取周期最大值，灰色为剩余未覆盖区。\n'
             '中心模型仅在原积分单元内提供边界显示支撑；本图不是瞬时接触面积。', ha='center', fontsize=8.2)
    files = _save(fig, output, '07_contact_pass_comparison')
    plt.close(fig)
    np.savez_compressed(output/'contact_pass_comparison_data.npz', **arrays)
    record = {'implementation': '接触分析统一接触后处理', 'shared_maximum_pa': maximum,
              'raw_results_modified': False, 'records': records, 'files': [Path(p).name for p in files]}
    write_json(output/'contact_pass_comparison.json', record)
    return record


def load_plot_result(directory):
    """校验已导出的文件并构造只供重绘的视图；不重新求解，不作为计算响应。

    保留每相位中心／端点及接触线offsets布局。返回的是SimpleNamespace
    绘图视图，不冒充AnalysisResult或新增数值验证。
    """
    import hashlib
    import json
    from types import SimpleNamespace
    from gear_contact.geometry import Gear, GearPair
    from gear_contact.elasticity import Material
    from gear_contact.config import modification_from_config
    directory = Path(directory).resolve()
    read = lambda name: json.loads((directory/name).read_text(encoding='utf-8-sig'))
    manifest = read('manifest.json')
    required = {'arrays.npz', 'input.json', 'summary.json', 'diagnostics.json'}
    if manifest.get('module') != 'gear_contact' or not required <= set(manifest.get('files', {})):
        raise ValueError('缺少有效的内核导出文件清单')
    for name, digest in manifest['files'].items():
        path = (directory/name).resolve()
        if not path.is_relative_to(directory) or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('绘图来源文件身份不一致：'+name)
    inputs = read('input.json')
    pair_data = dict(inputs.pop('pair'))
    pair = GearPair(pinion=Gear(**pair_data.pop('pinion')), wheel=Gear(**pair_data.pop('wheel')), **pair_data)
    for role in ('pinion', 'wheel'):
        inputs['material_'+role] = Material(**inputs['material_'+role])
        inputs['modification_'+role] = modification_from_config(inputs['modification_'+role])
    diagnostics = read('diagnostics.json')
    with np.load(directory/'arrays.npz', allow_pickle=False) as source:
        arrays = {key: source[key].copy() for key in source.files}
    offsets, phases = arrays['cell_offsets'], arrays['phase']
    if (offsets.dtype.kind not in 'iu' or offsets.shape != (len(phases)+1,) or offsets[0] != 0
            or offsets[-1] != len(arrays['pressure_pa']) or np.any(np.diff(offsets) <= 0)):
        raise ValueError('原数组的相位／单元offsets无效')
    rotations = []
    for index, (start, end) in enumerate(zip(offsets[:-1], offsets[1:])):
        section = slice(start, end)
        grid_names = ('face_coordinate_m', 'roll_pinion_m', 'roll_wheel_m', 'widths_m', 'tooth_pair_index',
                      'curvature_per_m', 'profile_factor_pinion', 'profile_factor_wheel',
                      'lead_factor_pinion', 'lead_factor_wheel')
        grid = SimpleNamespace(**{key: arrays[key][section] for key in grid_names})
        solution = SimpleNamespace(**{key: arrays[key][section] for key in ('forces_n', 'residual_gaps_m', 'active_mask')},
                                   diagnostics=diagnostics['contact'][index])
        geometry, endpoint = None, None
        if 'rotation_endpoint_offsets' in arrays:
            e0, e1 = arrays['rotation_endpoint_offsets'][index:index+2]
            l0, l1 = arrays['rotation_line_offsets'][index:index+2]
            ends = SimpleNamespace(**{key: arrays['endpoint_'+key][e0:e1] for key in (
                'face_coordinate_m', 'roll_pinion_m', 'roll_wheel_m', 'tooth_pair_index',
                'extended_pinion', 'extended_wheel')},
                line_offsets=arrays['line_endpoint_offsets'][l0:l1+1]-e0)
            geometry = SimpleNamespace(endpoints=ends, line_offsets=arrays['line_cell_offsets'][l0:l1+1]-start,
                centre_extended_pinion=arrays['extended_pinion'][section], centre_extended_wheel=arrays['extended_wheel'][section])
            endpoint = {key: arrays['endpoint_'+key][e0:e1] for key in ('pressure_pa', 'half_width_m', 'line_load_n_per_m')}
        rotations.append(SimpleNamespace(grid=grid, solution=solution, contact_geometry_data=geometry,
            endpoint_pressure=endpoint, elastic_diagnostics=diagnostics['elasticity'][index],
            **{key: arrays[key][section] for key in ('pressure_pa', 'half_width_m', 'line_load_n_per_m', 'gaps_m')}))
    return SimpleNamespace(pair=pair, inputs=inputs, summary=read('summary.json'), rotations=rotations,
        plot_only=True, **{key: arrays[key] for key in ('phase', 'normal_approach_m', 'angular_te_rad', 'mesh_stiffness_n_per_m')})


def _project_line_samples(rotation, pair_index, cosine):
    """整理本项目同齿中心与可选端点；延伸曲线按真实累计线长定位。

    普通域仍使用中心face/cos坐标。延伸域只用普通端点锚定累计width，
    不把整条非直接触曲线误当作face/cos，也不从参照压力反标长度。
    """
    grid = rotation.grid
    mask = np.asarray(grid.tooth_pair_index) == pair_index
    indices = np.flatnonzero(mask)
    if not len(indices):
        raise ValueError("本项目在相同物理位置没有所选齿对，不能替换为其他齿对")
    geometry = getattr(rotation, "contact_geometry_data", None)
    endpoint_values = getattr(rotation, "endpoint_pressure", None)
    if geometry is None:
        order = np.argsort(np.asarray(grid.face_coordinate_m)[indices])
        width = np.asarray(grid.widths_m)[indices][order]
        centres = np.asarray(grid.face_coordinate_m)[indices][order]/cosine
        intervals = np.column_stack((centres-width/2, centres+width/2))
        edges = np.r_[intervals[:, 0], intervals[-1, 1]]
        anchor_spread = 0.
        endpoint_slice = None
        if endpoint_values is not None:
            raise ValueError("本项目端点压力缺少对应接触线几何，不能猜测端点布局")
    else:
        offsets = np.asarray(geometry.line_offsets, dtype=int)
        matches = [i for i, (start, end) in enumerate(zip(offsets[:-1], offsets[1:]))
                   if end > start and np.all(np.asarray(grid.tooth_pair_index)[start:end] == pair_index)]
        if len(matches) != 1:
            raise ValueError("本项目齿对没有唯一的中心接触线布局")
        line = matches[0]
        start, end = offsets[line:line+2]
        if not np.array_equal(indices, np.arange(start, end)):
            raise ValueError("本项目中心齿对索引与接触线offsets不一致")
        order = np.arange(len(indices))
        width = np.asarray(grid.widths_m)[indices]
        if np.any(width <= 0) or not np.all(np.isfinite(width)):
            raise ValueError("本项目真实接触线条带长度必须正且有限")
        edges = np.r_[0., np.cumsum(width)]
        endpoint_offsets = np.asarray(geometry.endpoints.line_offsets, dtype=int)
        if len(endpoint_offsets) != len(offsets):
            raise ValueError("本项目中心与端点接触线数量不一致")
        first, last = endpoint_offsets[line:line+2]
        if last-first != len(width)+1:
            raise ValueError("本项目端点布局必须保持每线N+1个位置")
        endpoint_slice = slice(first, last)
        ordinary = ~(np.asarray(geometry.endpoints.extended_pinion)[endpoint_slice]
                     | np.asarray(geometry.endpoints.extended_wheel)[endpoint_slice])
        if not np.any(ordinary):
            raise ValueError("本项目所选延伸接触线没有普通端点，不能锚定局部沿线坐标")
        # 粗网格的单元可能横跨普通/延伸边界；中心坐标未必位于真实弧长中点。
        # 使用独立端点锚定累计实际线长，不能用中心flag替代端点几何。
        anchors = edges[ordinary]-np.asarray(geometry.endpoints.face_coordinate_m)[endpoint_slice][ordinary]/cosine
        anchor_spread = float(np.ptp(anchors))
        if anchor_spread > 1e-11:
            raise ValueError("本项目普通端点累计线长与几何坐标不相容")
        edges -= np.median(anchors)
        intervals = np.column_stack((edges[:-1], edges[1:]))
    sample = {"mask": mask, "order": order, "widths_m": width, "intervals_m": intervals,
        "edges_m": edges, "peak_pa": np.asarray(rotation.pressure_pa)[indices][order],
        "half_width_m": np.asarray(rotation.half_width_m)[indices][order],
        "force_n": np.asarray(rotation.solution.forces_n)[indices][order],
        "line_load_n_per_m": np.asarray(rotation.line_load_n_per_m)[indices][order],
        "ordinary_anchor_spread_m": anchor_spread, "endpoints": None}
    if endpoint_values is not None:
        count = int(geometry.endpoints.line_offsets[-1])
        data = {}
        for name in ("pressure_pa", "half_width_m", "line_load_n_per_m"):
            values = np.asarray(endpoint_values[name], dtype=float)
            if values.shape != (count,) or np.any(~np.isfinite(values)) or np.any(values < 0):
                raise ValueError("本项目端点压力、半宽与线载荷必须保持有效全局端点布局")
            data[name] = values[endpoint_slice].copy()
        sample["endpoints"] = data
    return sample


def instantaneous_contact_data(result, *, phase_index=None, tooth_pair_index=None, normal_points=301):
    """同相位同齿对局部Hertz带：沿接触线s与法向n均为m，压力矩阵[s,n]。

    中心模型严格保持每个原积分单元的p0、b常数；独立端点模型仅将端点
    分段线性细分用于显示。原条带合力不变，但端点显示面积积分可能偏离
    中心条带力；差值另报，不通过归一化压力消除。不是二维接触求解。
    """
    if isinstance(normal_points, bool) or not isinstance(normal_points, (int, np.integer)) or normal_points < 21:
        raise ValueError('局部垂线采样数必须为不小于21的整数')
    if phase_index is None:
        from .contact_pattern import interior_snapshot
        phase_index, selected = interior_snapshot(result)
        if tooth_pair_index is None:
            tooth_pair_index = selected
    if isinstance(phase_index, bool) or not isinstance(phase_index, (int, np.integer)) or not 0 <= phase_index < len(result.rotations):
        raise ValueError('瞬时相位序号无效')
    rotation = result.rotations[phase_index]
    if tooth_pair_index is None:
        tooth_pair_index = int(rotation.grid.tooth_pair_index[np.argmax(rotation.pressure_pa)])
    cosine = np.cos(result.pair.base_helix_angle_rad)
    sample = _project_line_samples(rotation, tooth_pair_index, cosine)
    edges, peak, half = sample['edges_m'], sample['peak_pa'], sample['half_width_m']
    definition = '原条带中心常值Hertz带'
    if sample['endpoints'] is not None:
        endpoint = sample['endpoints']
        display_edges = np.r_[np.concatenate([np.linspace(a, b, 5)[:-1]
            for a, b in zip(edges[:-1], edges[1:])]), edges[-1]]
        centres = (display_edges[:-1]+display_edges[1:])/2
        peak = np.interp(centres, edges, endpoint['pressure_pa'])
        half = np.interp(centres, edges, endpoint['half_width_m'])
        edges = display_edges
        definition = '独立端点分段线性显示Hertz带'
    normal_edges = np.linspace(-1.08*max(float(np.max(half)), 1e-9),
                                1.08*max(float(np.max(half)), 1e-9), normal_points+1)
    centres = (normal_edges[:-1]+normal_edges[1:])/2
    ratio = np.divide(centres[None, :], half[:, None],
        out=np.full((len(half), len(centres)), np.inf), where=half[:, None] > 0)
    pressure = peak[:, None]*np.sqrt(np.maximum(0., 1-ratio**2))
    integrated = float(np.sum(pressure*np.diff(edges)[:, None]*np.diff(normal_edges)[None, :]))
    total = float(np.sum(sample['force_n']))
    data = {'s_edges_m': edges, 'normal_edges_m': normal_edges, 'pressure_pa_s_n': pressure,
        'source_s_edges_m': sample['edges_m'], 'source_pressure_pa': sample['peak_pa'],
        'source_half_width_m': sample['half_width_m'], 'source_force_n': sample['force_n'],
        'source_widths_m': sample['widths_m'], 'phase_index': int(phase_index),
        'phase': float(result.phase[phase_index]), 'tooth_pair_index': int(tooth_pair_index),
        'definition': definition, 'independent_2d_contact': False,
        'source_total_force_n': total, 'display_integrated_force_n': integrated,
        'display_relative_force_error': (integrated-total)/total if total > 0 else 0.}
    if sample['endpoints'] is not None:
        for key, value in sample['endpoints'].items():
            data['source_endpoint_'+key] = value
    return data


def member_contact_pass_data(result, *, member='pinion', **kwargs):
    """用精确共轭关系换到大轮材料坐标；外啮合反向，内啮合同向，均不镜像工作面。"""
    if member not in ('pinion', 'wheel'):
        raise ValueError('member必须为pinion或wheel')
    data = contact_pass_data(result, **kwargs)
    if member == 'pinion':
        return data
    import math
    pair = result.pair
    span = pair.center_distance_m*math.sin(pair.working_pressure_angle_rad)
    # 全局face以小轮中面为原点；换到大轮左端须扣除大轮轴向偏置。
    shift = (pair.wheel.face_width_m-pair.pinion.face_width_m)/2-pair.axial_offset_m
    data['face_coordinate_m'] = data['face_coordinate_m']+shift
    sign = 1 if pair.wheel.internal else -1
    for key in ('samples', 'ordinary_samples', 'boundary_display_samples'):
        data[key] = data[key].copy()
        data[key][:, 0] += shift
        data[key][:, 1] = span+sign*data[key][:, 1]
    roll = span+sign*data['roll_distance_m']
    order = np.argsort(roll)
    data['roll_distance_m'] = roll[order]
    data['pressure_pa_x_y'] = data['pressure_pa_x_y'][:, order]
    data['contact_roll_bounds_m'] = np.sort(span+sign*data['contact_roll_bounds_m'])
    return data
