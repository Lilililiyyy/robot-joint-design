# 复用源码与许可证

本项目以AGPL-3.0-only发布，完整许可证见根目录LICENSE。

`gear_contact/`保留[Cylindrical-Gear-LTCA](https://github.com/Lilililiyyy/Cylindrical-Gear-LTCA)的16个Python源文件原字节快照，来源提交为`c63030dc4314a67ba282a3899f7ec8a5f6950fe1`，版本0.2.0，原许可证AGPL-3.0-only。原始模块说明、接口名与注释保留。

`robot_joint/`的NGW降阶系统、支承、疲劳、热、机电、精度与工程交付为新写实现；不是商业软件反编译源码。运行代码不包含厂商二进制、授权文件或MATLAB部署载荷。

`robot_joint/contact_display.py`另行适配上述项目提交`cb2e1ec8a8d1ac2b6137aa6a9b781b9a797c6a7f`的`gear_contact/contact_display.py`（SHA256 `bad5623144a828b2230b4b5405ad635e8d117dc38069d4b12e673c3a9d6c8f32`）及`examples/case_contact_plotting.py`的`_project_line_samples`。原许可证AGPL-3.0-only。适配包括导入绑定、蓝红色带、双语标签；本项目新增局部带接口及内外啮合材料坐标转换。`contact_pass_data`数学和数组语义保持源方法；不将本文件冒称为商业软件源码或纳入旧16文件快照身份。

新显示参照在`benchmarks/reference/contact_display_18_77/provenance.json`单独固定cb2e1ec来源；旧MASTA组件基准仍固定c63030d。两者的来源提交与证据范围不能混用。本次只做既有公开结果离线复算，没有运行MASTA，也没有新增机器人总成实测验证。

NumPy、SciPy和可选Matplotlib通过环境依赖安装，不随本项目复制。其许可证由各依赖项目独立提供。
