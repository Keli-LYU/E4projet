# E4projet 工作区迁移与环境配置说明书

本文说明如何把当前 `E4projet` 工作区打包复制到另一台电脑，并在新电脑上重新配置运行环境。

当前工作区依赖 ROS 2 Jazzy、CARLA、carla-ros-bridge、Nav2、自定义 ROS 2 包，以及 `AI_Part` 的 Python 深度学习环境。只复制 `E4projet` 文件夹还不够；新电脑还需要安装或复制这些外部环境。

## 1. 当前电脑环境基准

建议新电脑尽量保持以下版本一致：

| 项目 | 当前值 |
| --- | --- |
| Ubuntu | 24.04.4 LTS Noble |
| ROS 2 | Jazzy |
| Python | 3.12.3 |
| CARLA | 0.9.16 |
| CARLA 路径 | `${HOME}/carla` |
| carla-ros-bridge 路径 | `${HOME}/ros-bridge` |
| 项目路径 | 当前 `E4projet` 目录 |
| ROS 工作区 | `E4projet/projection/expleo_nav_stack_yolov7_improve_jazzy` |

当前启动脚本已改为自动使用这些路径：

- 项目目录：根据 `setup/*.sh` 所在位置自动计算
- carla-ros-bridge：默认 `${HOME}/ros-bridge`
- `/opt/ros/jazzy/setup.bash`

如果新电脑目录不同，可以通过环境变量覆盖，例如 `ROS_BRIDGE_DIR=/path/to/ros-bridge bash setup/1.carla-bridge.sh`。

## 2. 打包前检查

在旧电脑上先确认这些关键文件存在：

```bash
cd /path/to/E4projet

ls AI_Part/best_carla_model_13classes_weighted.pth
ls projection/default.rviz
ls projection/expleo_nav_stack_yolov7_improve_jazzy/src/maps/Town10_xodr.yaml
ls projection/expleo_nav_stack_yolov7_improve_jazzy/src/maps/Town10_xodr.pgm
```

注意：`.gitignore` 会忽略地图、图片、模型权重、`build/`、`install/`、`log/` 等文件。因此如果使用 `git clone` 迁移，以下内容不会自动出现，必须单独复制：

- `AI_Part/*.pth` 模型权重
- `projection/expleo_nav_stack_yolov7_improve_jazzy/src/maps/` 地图文件
- 训练数据集、图片、视频、ROS bag 等大文件
- CARLA 安装目录
- `${HOME}/ros-bridge` 工作区

推荐用压缩包复制完整项目源码和必要资源，但不要依赖旧电脑生成的 `build/`、`install/`、`log/`。

示例：

```bash
cd "${HOME}"
tar -czf E4projet-transfer.tar.gz E4projet
tar -czf ros-bridge-transfer.tar.gz ros-bridge
```

如果要离线迁移 CARLA，也可以复制整个 CARLA 目录：

```bash
cd "${HOME}"
tar -czf carla-0.9.16-transfer.tar.gz carla
```

## 3. 新电脑基础环境

新电脑推荐使用 Ubuntu 24.04。先安装基础工具：

```bash
sudo apt update
sudo apt install -y \
  git curl gnupg lsb-release build-essential cmake \
  python3-pip python3-venv python3-colcon-common-extensions \
  python3-rosdep
```

安装 ROS 2 Jazzy Desktop。请以 ROS 2 官方 Jazzy 安装文档为准：

- https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

安装完成后确认：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

初始化 `rosdep`：

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

## 4. 安装 ROS/Nav2 依赖

安装本项目常用的 ROS 2 包：

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro
```

如果后面 `rosdep` 或 `colcon build` 报缺包，根据报错继续安装对应的 `ros-jazzy-*` 包。

## 5. 安装或复制 CARLA 0.9.16

本项目当前使用 CARLA 0.9.16，旧电脑路径是：

```bash
${HOME}/carla
```

新电脑可以选择重新下载 CARLA 0.9.16，也可以复制旧电脑的 `carla` 文件夹。CARLA 官方文档：

- https://carla.readthedocs.io/en/0.9.16/getting_started/

复制后建议放在：

```bash
/home/<新用户名>/carla
```

然后安装匹配 Python 版本的 CARLA Python wheel。Python 3.12 使用 `cp312`：

```bash
python3 -m pip install --user /home/<新用户名>/carla/PythonAPI/carla/dist/carla-0.9.16-cp312-cp312-manylinux_2_31_x86_64.whl
```

验证：

```bash
python3 -c "import carla; print(carla.__file__)"
```

启动 CARLA：

```bash
/home/<新用户名>/carla/CarlaUE4.sh
```

如果 CARLA 在 Windows 运行、ROS 在 WSL2 运行，需要保证 2000 端口可访问。本项目已经提供 WSL 辅助脚本：

```bash
cd /home/<新用户名>/E4projet
python3 projection/wsl-bridge/wsl-bridge.py --print-host
```

必要时手动设置：

```bash
export CARLA_HOST=<CARLA所在主机IP>
export CARLA_PORT=2000
```

## 6. 安装或复制 carla-ros-bridge

当前项目启动脚本依赖：

```bash
${HOME}/ros-bridge/install/local_setup.bash
```

新电脑可以直接复制旧电脑的 `ros-bridge`，但推荐重新编译，避免旧路径残留。

如果重新从源码安装：

```bash
cd /home/<新用户名>
mkdir -p ros-bridge/src
cd ros-bridge/src

git clone https://github.com/carla-simulator/ros-bridge.git
git clone https://github.com/carla-simulator/ros-carla-msgs.git
```

构建：

```bash
cd /home/<新用户名>/ros-bridge
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
```

验证：

```bash
source /opt/ros/jazzy/setup.bash
source /home/<新用户名>/ros-bridge/install/local_setup.bash
ros2 pkg list | grep carla_ros_bridge
```

CARLA ROS bridge 官方文档：

- https://carla.readthedocs.io/projects/ros-bridge/en/latest/run_ros/

## 7. 解压 E4projet 并修正路径

把 `E4projet-transfer.tar.gz` 放到新电脑 `/home/<新用户名>/` 下：

```bash
cd /home/<新用户名>
tar -xzf E4projet-transfer.tar.gz
cd E4projet
```

当前 `setup/*.sh` 已经不依赖固定用户名。可以用下面的命令确认没有旧用户路径残留：

```bash
grep -R "/home/" -n setup projection/wsl-bridge
```

如果 `ros-bridge` 或 CARLA 不在 `${HOME}` 下，不需要改脚本，启动时传环境变量即可：

```bash
ROS_BRIDGE_DIR=/path/to/ros-bridge CARLA_ROOT=/path/to/carla bash projection/wsl-bridge/clean.bash
```

## 8. 配置 AI_Part Python 环境

建议使用虚拟环境：

```bash
cd /home/<新用户名>/E4projet
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r AI_Part/requirements.txt
python3 -m pip install /home/<新用户名>/carla/PythonAPI/carla/dist/carla-0.9.16-cp312-cp312-manylinux_2_31_x86_64.whl
```

如果使用 NVIDIA GPU，请按显卡驱动和 CUDA 环境安装合适版本的 PyTorch。CPU 也能运行，但语义分割推理会明显变慢。

确认模型权重存在：

```bash
ls /home/<新用户名>/E4projet/AI_Part/best_carla_model_13classes_weighted.pth
```

## 9. 清理旧构建并重新编译项目 ROS 工作区

复制过来的 `build/`、`install/`、`log/` 里可能包含旧电脑绝对路径，迁移后不要直接使用。进入项目 ROS 工作区重新编译：

```bash
cd /home/<新用户名>/E4projet/projection/expleo_nav_stack_yolov7_improve_jazzy

rm -rf build install log

source /opt/ros/jazzy/setup.bash
source /home/<新用户名>/ros-bridge/install/local_setup.bash

rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
```

也可以使用项目提供的清理构建脚本：

```bash
cd /home/<新用户名>/E4projet
ROS_BRIDGE_DIR=/home/<新用户名>/ros-bridge \
CARLA_ROOT=/home/<新用户名>/carla \
bash projection/wsl-bridge/clean.bash
```

验证本项目包是否可见：

```bash
source /opt/ros/jazzy/setup.bash
source /home/<新用户名>/ros-bridge/install/local_setup.bash
source /home/<新用户名>/E4projet/projection/expleo_nav_stack_yolov7_improve_jazzy/install/local_setup.bash

ros2 pkg list | grep expleo_nav_stack
ros2 pkg list | grep projection
```

## 10. 启动顺序

建议开多个终端，按顺序启动。

### 终端 1：启动 CARLA

```bash
/home/<新用户名>/carla/CarlaUE4.sh
```

如果 CARLA 在 Windows 里运行，这一步在 Windows 上启动 CARLA。

### 终端 2：启动 CARLA bridge

```bash
cd /home/<新用户名>/E4projet
bash setup/1.carla-bridge.sh
```

### 终端 3：启动 Nav2

```bash
cd /home/<新用户名>/E4projet
bash setup/2.nav2.sh
```

### 终端 4：启动 AI 语义分割节点

如果用了虚拟环境，先激活：

```bash
cd /home/<新用户名>/E4projet
source .venv/bin/activate
bash setup/3.AI_Part.sh
```

### 终端 5：启动 AI 到 ROS 检测转换

```bash
cd /home/<新用户名>/E4projet
source .venv/bin/activate
bash setup/4.ai-ros.sh
```

### 终端 6：启动 RViz2

```bash
cd /home/<新用户名>/E4projet
bash setup/5.RViz2.sh
```

### 可选：跟随 ego vehicle 的 CARLA 视角

```bash
cd /home/<新用户名>/E4projet
source .venv/bin/activate
bash setup/6.carla-follow-ego.sh
```

## 11. 迁移后检查清单

运行前逐项确认：

- `source /opt/ros/jazzy/setup.bash` 不报错
- `/home/<新用户名>/ros-bridge/install/local_setup.bash` 存在
- `E4projet/projection/expleo_nav_stack_yolov7_improve_jazzy/install/local_setup.bash` 存在
- `AI_Part/best_carla_model_13classes_weighted.pth` 存在
- `projection/expleo_nav_stack_yolov7_improve_jazzy/src/maps/Town10_xodr.yaml` 存在
- `python3 -c "import carla"` 成功
- `python3 -c "import torch, cv2"` 成功
- CARLA 已启动，并且 2000 端口可访问
- `ros2 topic list` 能看到 `/carla/...` 话题

## 12. 常见问题

### 12.1 `source .../install/local_setup.bash` 找不到

说明对应工作区还没有成功编译，或路径没有改成新电脑路径。重新执行：

```bash
cd /home/<新用户名>/E4projet/projection/expleo_nav_stack_yolov7_improve_jazzy
source /opt/ros/jazzy/setup.bash
source /home/<新用户名>/ros-bridge/install/local_setup.bash
colcon build --symlink-install
```

### 12.2 `import carla` 失败

通常是 CARLA Python wheel 没安装，或 Python 版本不匹配。Python 3.12 应安装 `cp312` wheel：

```bash
python3 -m pip install --user /home/<新用户名>/carla/PythonAPI/carla/dist/carla-0.9.16-cp312-cp312-manylinux_2_31_x86_64.whl
```

如果 `PYTHONPATH` 中有旧的 `.whl` 路径干扰，可以运行：

```bash
cd /home/<新用户名>/E4projet
python3 projection/wsl-bridge/wsl-bridge.py --check-python
```

### 12.3 ROS 能启动，但没有 `/carla/...` 话题

检查 CARLA 是否已经启动，bridge 是否连到了正确 host：

```bash
cd /home/<新用户名>/E4projet
python3 projection/wsl-bridge/wsl-bridge.py --print-host
```

必要时设置：

```bash
export CARLA_HOST=<CARLA所在主机IP>
export CARLA_PORT=2000
```

然后重新启动 `setup/1.carla-bridge.sh`。

### 12.4 Nav2 报地图文件不存在

检查 `setup/2.nav2.sh` 中的 `map:=...` 路径，并确认地图文件已复制：

```bash
ls /home/<新用户名>/E4projet/projection/expleo_nav_stack_yolov7_improve_jazzy/src/maps/
```

### 12.5 AI 节点报模型权重不存在

检查：

```bash
ls /home/<新用户名>/E4projet/AI_Part/best_carla_model_13classes_weighted.pth
```

如果没有，需要从旧电脑复制。该文件被 `.gitignore` 忽略，不会通过 git 自动迁移。

### 12.6 `colcon build` 报依赖缺失

先运行：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
```

如果仍然缺某个 ROS 包，按报错安装对应 apt 包，例如：

```bash
sudo apt install ros-jazzy-<包名>
```

## 13. 最小恢复流程

如果只想快速恢复到能跑的状态，按这个顺序做：

```bash
# 1. 安装 ROS 2 Jazzy、Nav2、colcon、rosdep

# 2. 安装或复制 CARLA 0.9.16 到 /home/<新用户名>/carla
python3 -m pip install --user /home/<新用户名>/carla/PythonAPI/carla/dist/carla-0.9.16-cp312-cp312-manylinux_2_31_x86_64.whl

# 3. 安装或复制并重编译 ros-bridge
cd /home/<新用户名>/ros-bridge
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install

# 4. 解压 E4projet，确认没有旧用户路径残留
cd /home/<新用户名>/E4projet
grep -R "/home/" -n setup projection/wsl-bridge || true

# 5. 安装 Python 依赖
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r AI_Part/requirements.txt

# 6. 重编译项目 ROS 工作区
cd /home/<新用户名>/E4projet/projection/expleo_nav_stack_yolov7_improve_jazzy
rm -rf build install log
source /opt/ros/jazzy/setup.bash
source /home/<新用户名>/ros-bridge/install/local_setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
```

完成后按第 10 节启动。
