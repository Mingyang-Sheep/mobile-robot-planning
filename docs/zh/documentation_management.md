<div align="right">

[英文版](../documentation_management.md)

</div>

# 文档管理规范

本文说明本仓库 Markdown 文档的组织、命名和链接规则。当前规则是中英文分文件：默认文件使用英文，中文文件使用并行的 `_zh.md` 文件，或者放在 `docs/zh/` 目录下。

## 文档分层

| 位置 | 用途 | 维护原则 |
|---|---|---|
| `README.md` | GitHub 首页 | 英文项目总览、架构、快速上手、能力表和文档导航 |
| `README_zh.md` | 中文首页 | 根目录 README 的中文对应页 |
| `docs/` | 仓库级英文专题文档 | 仓库默认展示的专题文档 |
| `docs/zh/` | 仓库级中文专题文档 | 与 `docs/` 同名的中文对应页 |
| `src/<package>/README.md` | 包内英文概览 | 简短说明包职责、关键入口、最小命令和 `docs/` 链接 |
| `src/<package>/README_zh.md` | 包内中文概览 | 包内 README 的中文对应页 |
| `src/<package>/DEBUG_NOTE.md` | 包内英文调试笔记 | 只保留快速检查入口 |
| `src/<package>/DEBUG_NOTE_zh.md` | 包内中文调试笔记 | 包内调试笔记的中文对应页 |
| `src/<package>/THIRD_PARTY_NOTICES.md` | 第三方来源和许可证 | 与代码放在一起，方便追溯 |
| `refer/` | 外部参考副本 | 标明参考性质，不作为当前功能承诺 |

不是所有 Markdown 都必须移动到 `docs/`。用户和开发者长文档放在 `docs/`；包内 README、调试笔记和第三方说明留在源码包旁边更清楚。

## 命名规则

新增仓库级英文文档使用小写 snake_case：

```text
quick_start.md
planner_framework.md
coverage_path_planning.md
documentation_management.md
```

`docs/zh/` 下的中文对应页使用同名文件：

```text
docs/quick_start.md
docs/zh/quick_start.md
```

包内中文对应页使用 `_zh`：

```text
src/mr_slam/README.md
src/mr_slam/README_zh.md
src/mr_traditional_planner/DEBUG_NOTE.md
src/mr_traditional_planner/DEBUG_NOTE_zh.md
```

可能已经被外部引用的历史文件名可以保留为兼容入口：

```text
URDF_MIGRATION_GUIDE.md
navigation_launch_reference.md
planner_path_topics.md
```

这类文件只作为入口页，不再承载重复长正文。

## 语言规则

英文文件只使用英文正文。中文文件只使用中文正文。代码命令、包名、文件路径、话题名、ROS 消息名、插件名和上游项目名保持原始写法。

英文文件推荐语言入口：

```markdown
<div align="right">

中文路径：zh/example.md

</div>
```

中文文件推荐语言入口：

```markdown
<div align="right">

英文版：../example.md

</div>
```

普通文档避免使用同页双语段落，中英文放在两个文件中维护。

## 状态图标

能力矩阵中使用可视化状态图标：

| 图标 | 含义 |
|---|---|
| ✅ | 已支持或已存在 |
| ❌ | 未实现 |
| 🟣 | 部分支持、实验性质或间接支持 |

描述支持状态时优先使用这些图标，不再只写“是”或“否”。

## 链接规则

- 仓库内链接使用相对路径，不写本机绝对路径。
- 包内 README 链接 `docs/` 时使用相对路径，例如 `../../docs/planner_framework.md`。
- 图片使用仓库内相对路径，例如 `docs/assets/mobile-robot-planning.png`。
- GIF 演示通过仓库内相对路径引用，例如 `docs/assets/00_project_overview.gif`。README 中不使用 `<video>` 也不嵌入 MP4 文件。
- 不链接不存在的文件；改名时保留兼容入口或同步更新所有引用。

## 内容边界

文档只描述当前仓库真实存在、可在源码中定位或已经明确验证的能力。实验性能力需要明确标注实验性质、基础演示、部分支持等状态。

尤其避免：

- 把调试路径写成机器人实际执行路径；
- 把 BSA / Spiral-STC 博客写成当前已有启动入口；
- 把 `mr_learning` 写成成熟强化学习平台；
- 把相机话题写成完整视觉感知或视觉建图系统；
- 把 WPB Home Mani 的模型接入写成完整机械臂规划与抓取。
