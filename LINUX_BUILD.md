# RT-PPP Linux 编译与运行

本压缩包包含当前工作区的 RT-PPP 应用源码，不包含预编译 Linux 程序。以下步骤以 Debian/Ubuntu 和 Qt 5 为例。这里的“无窗口运行”指后台运行，不代表不访问网络。

## 1. 安装编译依赖

```sh
sudo apt update
sudo apt install build-essential qtbase5-dev qt5-qmake qtbase5-dev-tools
```

确认 `qmake` 使用 Qt 5：

```sh
qmake -v
```

如果系统默认的 `qmake` 指向 Qt 6，请改用 Qt 5 的路径，常见为 `/usr/lib/qt5/bin/qmake` 或 `qmake-qt5`。

解压后进入源码目录，并给随包提供的 shell 脚本添加执行权限：

```sh
tar -xzf RT-PPP-linux-source-20261028.tar.gz
cd RT-PPP-linux-source-20261028
chmod +x run_headless.sh tests/headless_smoke.sh gpt3/build_worker.sh gfs_python/bootstrap.sh
```

## 2. 编译 RT-PPP

在源码根目录执行：

```sh
mkdir -p build/linux
(
  cd build/linux
  qmake ../../RT-PPP.pro
  make -j"$(nproc)"
)
```

可执行文件生成在 `build/linux/bin/RT-PPP`。中间文件保存在 `build/linux`，不会写入源码根目录。

## 3. 可选运行组件

选择 GPT3 + VMF3 对流层映射模型时，需要额外构建 Fortran worker：

```sh
sudo apt install gfortran
./gpt3/build_worker.sh ./build/linux/bin/gpt3
```

脚本会同时把所需的 `gpt3_5.grd` 模型网格复制到 worker 目录。默认 `1/sin(E)` 映射模式不需要该 worker。

GFS 对流层处理需要 Python worker 和 ecCodes。若使用该功能，安装 Python 环境并运行：

```sh
sudo apt install python3 python3-venv python3-pip
./gfs_python/bootstrap.sh
```

该脚本会安装 `eccodes`，通常需要访问 Python 包源。离线安装时，可先将匹配当前系统和 Python ABI 的 wheel 放入 `gfs_python/wheelhouse/`。如果不选择 GFS 处理，则不需要此 Python 环境。

## 4. 无窗口运行

配置文件至少需要一条已保存的实时流，并设置 `ppp/processingMode=realtime`。推荐通过随包脚本启动：

```sh
./run_headless.sh /etc/rtppp/rtppp_settings.ini
```

脚本默认运行 `build/linux/bin/RT-PPP`，设置 `QT_QPA_PLATFORM=offscreen`，并把配置文件路径传给程序。也可以直接运行：

```sh
QT_QPA_PLATFORM=offscreen ./build/linux/bin/RT-PPP \
  --headless \
  --config /etc/rtppp/rtppp_settings.ini
```

Qt 的 offscreen 平台插件 `libqoffscreen.so` 必须已安装。若 Qt 无法找到该插件，需要检查 Qt 安装目录下的 `plugins/platforms/`。前台运行可按 `Ctrl+C` 停止；服务管理器可发送 `SIGTERM`。

做 10 秒限时启动验证：

```sh
./run_headless.sh /etc/rtppp/rtppp_settings.ini --run-seconds 10
```

包内 smoke 脚本会清除 X11/Wayland 显示变量，并使用本机回环测试端口：

```sh
./tests/headless_smoke.sh ./build/linux/bin/RT-PPP
```

该检查只验证启动和退出，不验证生产 caster 数据接收或 PPP 解算结果。

## 5. systemd 部署

包内 `deploy/rtppp.service` 是模板。安装前要修改服务账号、工作目录、程序和配置路径，并确认文件权限。配置中的 NTRIP 凭据应限制为服务账号可读。RT-PPP 本身不是原生 systemd 服务程序，该 unit 负责监督普通 headless 进程。

## 交付范围

此源码包从当前 Windows 工作区制作，本次没有在 Linux 主机上编译。请在目标 Linux 发行版和 Qt 环境中执行上述编译及 smoke 检查。
