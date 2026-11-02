"""从正式对标数据绘制双语蓝红结果图。"""
import argparse
import csv
from pathlib import Path
import numpy as np
from robot_joint.figure_style import COLORS,publication_settings,save_publication_figure


def draw_validation(source,output,languages=('zh','en')):
    """在同一几何相位下比较法向TE与切线刚度，原响应不作拟合。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    source=Path(source);output=Path(output)
    with (source/'cycle.csv').open(encoding='utf-8-sig',newline='') as stream:
        cycle=list(csv.DictReader(stream))
    phase=np.asarray([float(row['phase_cycles']) for row in cycle])
    order=np.argsort(phase)
    for language in languages:
        L=lambda zh,en:zh if language=='zh' else en
        with plt.rc_context(publication_settings(language)):
            fig,axes=plt.subplots(1,2,figsize=(12,5.8),layout='constrained')
            for ax,field,factor,label in (
                (axes[0],'te_m',1e6,L('法向TE / μm','Normal TE / μm')),
                (axes[1],'stiffness_n_per_m',1e-6,L('法向切线刚度 / N/μm','Normal tangent stiffness / N/μm'))):
                ax.plot(phase[order],np.asarray([float(row['project_'+field])*factor for row in cycle])[order],
                    color=COLORS[0],label=L('本项目','This project'))
                ax.plot(phase[order],np.asarray([float(row['reference_'+field])*factor for row in cycle])[order],
                    'o',color=COLORS[1],mfc='none',ms=4,label='MASTA')
                ax.set(xlabel=L('啮合周期相位','Mesh-cycle phase'),ylabel=label)
                ax.legend();ax.grid(alpha=.15)
            save_publication_figure(fig,output/language/'01_masta_comparison',language=language,
                title=L('18/77斜齿轮副：传动误差与刚度对标','18/77 helical mesh: transmission error and stiffness benchmark'),
                footnote=L('相同几何、材料、载荷与相位；对标范围为指定单副工况。',
                    'Matching geometry, materials, load and phase; comparison covers the stated single mesh.'))
            plt.close(fig)


def main():
    root=next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file())
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=root/'benchmarks/results')
    parser.add_argument('--output',type=Path,default=root/'benchmarks/results')
    parser.add_argument('--language',choices=('zh','en','both'),default='both')
    args=parser.parse_args()
    draw_validation(args.source,args.output,('zh','en') if args.language=='both' else (args.language,))


if __name__=='__main__':main()
