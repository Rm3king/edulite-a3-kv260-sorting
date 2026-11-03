# 已归档固件

本目录保存分拣位流、设备树、加载描述及对应配置。使用以下命令核对文件完整性：

```bash
cd build/deploy
sha256sum --check sha256sums
```

完整板端部署包由 `build/scripts/package_can.py` 在通过源码回归与构建后生成，包含寄存器驱动、安装脚本和板端软件。部署命令见 [KV260 工作流](../../skill/kv260_pl_workcell_ops/README.md)。
