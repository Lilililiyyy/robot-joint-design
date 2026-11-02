"""论文式蓝红图表规范；只处理显示与导出，不修改计算数组。"""
from pathlib import Path
import textwrap

BLUE = '#2166ac'
RED = '#b2182b'
COLORS = [BLUE, RED, '#67a9cf', '#ef8a62']


def publication_settings(language='zh'):
    """采用可编辑字体、细坐标轴和白底；中文依当前环境选择字体。"""
    from matplotlib import font_manager
    names = {f.name for f in font_manager.fontManager.ttflist}
    candidates = ('Microsoft YaHei', 'SimHei', 'Noto Sans CJK SC', 'WenQuanYi Zen Hei') if language == 'zh' else ('Arial', 'Helvetica', 'DejaVu Sans')
    if language == 'zh' and not any(name in names for name in candidates):
        raise ValueError('中文绘图缺少CJK字体；请安装Noto Sans CJK SC/微软雅黑/黑体，或选择--language en')
    font = next((name for name in candidates if name in names), 'DejaVu Sans')
    return {'font.family': font, 'font.size': 10, 'axes.unicode_minus': False,
            'svg.fonttype': 'none', 'pdf.fonttype': 42, 'ps.fonttype': 42,
            'figure.facecolor': 'white', 'axes.facecolor': 'white',
            'axes.linewidth': .65, 'axes.spines.top': False, 'axes.spines.right': False,
            'axes.titlepad': 9, 'axes.labelpad': 5, 'legend.frameon': False,
            'xtick.direction': 'out', 'ytick.direction': 'out',
            'xtick.major.width': .6, 'ytick.major.width': .6,
            'grid.linewidth': .4, 'lines.linewidth': 1.2}


def save_publication_figure(fig, stem, *, title='', footnote='', language='zh'):
    """按183 mm双栏宽导出300 dpi PNG与可编辑SVG/PDF，保留原数据采样。"""
    from matplotlib.text import Text
    path = Path(stem)
    path.parent.mkdir(parents=True, exist_ok=True)
    width, height = fig.get_size_inches()
    fig.set_size_inches(183/25.4, min(170/25.4, 183/25.4*height/width))
    # 原布局以大画布编排，统一缩至论文尺寸，同时保留可读的5—7 pt字级。
    for text in fig.findobj(match=Text):
        text.set_fontsize(max(5.5, min(7, text.get_fontsize()*.66)))
    for index, axis in enumerate(fig.axes):
        if axis.get_label() == '<colorbar>':
            continue
        axis.text(-.075, 1.045, chr(97+index), transform=axis.transAxes,
                  fontsize=8, fontweight='bold', va='bottom')
        for line in axis.lines:
            line.set_linewidth(min(line.get_linewidth(), 1.25))
            line.set_markersize(min(line.get_markersize(), 2.6))
        for collection in axis.collections:
            if hasattr(collection, 'get_sizes') and len(collection.get_sizes()):
                collection.set_sizes(collection.get_sizes()*.45)
    if title:
        fig.suptitle(textwrap.fill(title, 105 if language == 'en' else 49),
                     fontsize=8, fontweight='bold', y=.995)
    if fig.get_layout_engine() is not None:
        fig.get_layout_engine().set(rect=(.015, .055 if footnote else .015, .97, .89))
    if footnote:
        fig.text(.02, .012, footnote, fontsize=5.8, color='#444444')
    fig.savefig(path.with_suffix('.png'), dpi=300, facecolor='white')
    fig.savefig(path.with_suffix('.svg'), facecolor='white',
                metadata={'Title': title, 'Description': 'language='+language})
    fig.savefig(path.with_suffix('.pdf'), facecolor='white', metadata={'Title': title})
