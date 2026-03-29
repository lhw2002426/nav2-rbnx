# nav2-rbnx

Nav2 与 Robonix 的桥接包。

这个包现在包含：

- 一个 Python 节点 `nav2_rbnx_agent.node`
- MCP 工具：`navigate_to`、`get_nav_status`、`cancel_nav`
- 一个 agent skill：`skills/navigation/SKILL.md`

默认行为：

- 启动时注册到 `robonix-server`
- 暴露 `mcp_tools`
- 自动拉起 `nav2_bringup`
- 优先使用 `navigate_to_pose` action，若不可用则退回 `/goal_pose`
