# 复现指南：模拟两节点通信数据包随时间变化的路径

本项目（TinyLEO）提供了 `network_orchestrator` 组件，该组件可以在拥有完整 Linux 网络命名空间（Network Namespaces）权限的服务器中模拟低轨卫星网络，并跟踪两节点通信路径随时间的变化。

由于沙箱（Sandbox）及常规 Docker 容器的权限限制（不支持 `ip netns` 完整创建及底层网络配置），我们将演示如何配置并在您的真实物理机或具备完整权限的虚拟机（Root privileges & Multi-core）上运行。

## 前置环境准备

1. **准备服务器**:
   - 至少一台具备 Root 权限的 Ubuntu 20.04+ 服务器。建议内存 >= 120GB，CPU 核心 >= 32（用于生成数千个卫星网络节点）。
   - 确保安装了 Python 3.10 或更高版本。

2. **安装依赖环境**:
   在具有 Root 权限的终端下执行以下命令安装相关的依赖包（建议使用全局环境或使用 `--break-system-packages`，由于涉及到系统底层网络修改）：

   ```bash
   sudo apt-get update
   sudo apt-get install -y libnetfilter-queue-dev python3-pip openssh-server
   sudo service ssh start

   # 安装 Python 依赖包
   sudo python3 -m pip install argparse numpy requests paramiko tqdm networkx \
       grpcio grpcio-tools NetfilterQueue pyroute2 scapy python-iptables watchdog scipy
   ```

3. **配置 SSH 免密登录（供仿真使用）**:
   **⚠️ 强烈建议：** 此项目会频繁操作底层网络命名空间，必须使用 root 权限。请**务必在独立、隔离的虚拟机或物理测试机中进行**，不要在生产环境中尝试。

   仿真器基于 SSH 与各个容器及命名空间交互，需保证配置文件的机器允许 Root SSH 登录。推荐使用 SSH Key 认证：

   ```bash
   # 1. 如果你还没有 ssh key，请先生成
   ssh-keygen -t rsa -b 4096 -C "tinyleo_test"

   # 2. 将公钥拷贝到目标机器（或本机的 root 账户下）
   # 如果是本机测试：
   sudo mkdir -p /root/.ssh
   sudo cp ~/.ssh/id_rsa.pub /root/.ssh/authorized_keys
   sudo chmod 700 /root/.ssh
   sudo chmod 600 /root/.ssh/authorized_keys
   ```

## 代码配置修改与准备

### 1. 生成 gRPC 代码

在项目目录执行以下命令生成 gRPC 通信代码：

```bash
cd network_orchestrator/southbound/
python3 -m grpc_tools.protoc -I. --python_out=./ --grpc_python_out=./ ./link_failure_grpc/link_failure.proto
```

### 2. 配置服务器连接（tinyleo_config.json）

修改 `network_orchestrator/test/config/tinyleo_config.json` 文件中的 `Machines` 数组，填入本机或目标主机的 IP 和 SSH 信息（若使用 key 登录，password 保持配置中对应的值即可，paramiko 也支持通过秘钥自动连接，具体依据你的环境配置）：

```json
        {
            "IP": "101.6.21.153",  // 替换为你的服务器 IP
            "port": 22,
            "username": "root",
            "password": "xxx"      // 若有密码在此填入，若配置了免密通常可忽略
        }
```

### 3. 配置路径追踪脚本（example_orchestrator.py）

我们在 `network_orchestrator/test/example_orchestrator.py` 中已经进行了适配修改，增加了异常捕获，并确保开启了 `TEST = True`。

关键追踪逻辑在 `for timestamp in range(0, sn.duration):` 循环中：
```python
    if TEST:
        time.sleep(2)
        try:
            # 记录此时的 traceroute（路径数据）
            sn.set_traceroute("GS1", "GS2", f"ts{timestamp}")
        except Exception as e:
            print(f"Skipping traceroute at timestamp {timestamp} due to error: {e}")
```

在这个循环中，仿真器在每个时间戳都会更新一次拓扑（`sn.update_tinyleo_topology(timestamp)`），并向对应的容器内发送指令执行 `traceroute GS1 GS2`。

## 运行仿真与分析

在终端中执行：

```bash
cd network_orchestrator/test/
sudo python3 example_orchestrator.py
```

### 执行过程
1. **拓扑预测与网络创建**：脚本首先加载所有的卫星数据，开始计算每个时间片（Timestamp）的最优拓扑结构。
2. **命名空间创建**：基于 Paramiko SSH 连接，底层会在系统中创建几千个以 `SH1SAT...` 为前缀的网络命名空间（Network Namespace）。
3. **数据打点（Traceroute）**：系统会在指定的时间戳（默认每 20 秒间隔模拟一次时间推移）向两节点发送 traceroute 指令，结果会保存在新生成的文件夹中（例如 `tinyleo-Arbitrary-LeastDelay/result/` 目录下）。

### 查看路径变化结果

运行结束后，你可以通过以下方式查看数据包传输路径随时间的变化：

1. **自动生成的结果文件**：
   在 `network_orchestrator/test/tinyleo-Arbitrary-LeastDelay/result/` 文件夹内，会生成形如 `ts0_traceroute.txt`、`ts1_traceroute.txt` 等日志文件，你可以按时间戳对比路由跳数和经过的节点差异。

2. **进入容器手动验证**：
   你可以将 `example_orchestrator.py` 中的清理代码去掉，然后在仿真运行时运行：
   ```bash
   python3 get_container.py
   ```
   它会输出 `nsenter` 命令（如 `nsenter -m -u -i -n -p -t <PID> bash`），允许你直接进入卫星或者地面站容器，手动执行 `traceroute <目标节点IP>` 来追踪当前路由。
