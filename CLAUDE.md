
## 项目概述

- 本仓库为 ROS2 Navigation2（Nav2）源码仓库，包含导航框架核心包、插件、bringup、RViz 插件与系统测试等。
- 本文件用于为代码智能体（如 Claude Code / Codex CLI）提供**项目级**约束与环境说明，避免在错误的机器/容器/网络环境中执行操作，并确保 Git 工作流一致。

## 其他当前项目（同级目录说明）

> 这些项目不属于本仓库的一部分；如需改动，请用户明确指令后再切换目录操作，并遵循其各自的 `AGENTS.md/CLAUDE.md` 约束。

- `../yahboomcar_ws`：Yahboom ROSMASTER M1 移动机器人 ROS2 Humble 工作空间（含远端容器构建约束）。
- `../eseaskycar_ws`：轮椅小车跟随/避障仿真与算法工程（含 Gazebo 场景与节点重构进展记录）。
- `../wheeltec_ros2`：WheelTec 相关 ROS2 工程（如需介入，请先读取其项目内说明文件）。
- `../uwb-x5-code`：UWB / X5 相关代码目录（如需介入，请先确认目标与运行环境）。
- `../autoware_data`、`../autoware_map`：Autoware 相关数据与地图目录（通常仅数据读取，不建议在此做大规模改动）。

## 远端开发环境（SSH / Docker）

- 远端 SSH 开发主机通过 SSH MCP 连接（connection name: `default`）。
- 宿主机上有 `nav2-humble-dev` Docker 容器：将宿主机 `/home/haizh/work/navigation2` 挂载到容器内 `/root/nav2_ws`。
- **编译与验证在容器内执行**。
- **所有远端命令必须通过 SSH MCP 工具 `mcp__mcp-router__execute-command` 执行**（不要在远端主机上直接手工跑命令，也不要在远端做写入性操作）。
- SSH MCP 执行建议：一次批次不要超过 **5 条命令**，按步骤分批执行，便于定位失败点与避免超长会话。
- ip: 通过 ssh mcp 获取


## Git 约束（强制）

本项目代码在远端宿主机 `/home/haizh/work/navigation2` 通过 Git 管理，但远端 Git 操作有严格限制：

- **远端宿主机仅允许 `git pull`**。
- 严禁在远端执行 `git push`、`git commit` 或任何写入性 Git 操作。
- 本机当前目录下允许执行 `git push`、`git commit` 或任何写入性 Git 操作。
- 所有代码变更/提交/推送必须在本地完成后，再由远端宿主机 `git pull` 同步。
