<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>

# 文档管理规范

本文说明本仓库 Markdown 文档的放置位置、命名规则和双语维护方式。目标是让 GitHub 首页、专题文档、ROS package 内说明和第三方来源说明各司其职。

## 文档分层

| 位置 | 用途 | 维护原则 |
|---|---|---|
| `README.md` | GitHub 首页总览 | 展示项目定位、系统架构、Quick Start、核心能力和文档导航，不复制所有专题细节 |
| `docs/` | 仓库级专题文档 | 放面向用户和开发者的教程、参考、迁移指南、排错说明和资料索引 |
| `src/<package>/README.md` | package 就近说明 | 只写包职责、关键入口、最小命令和指向 `docs/` 的链接 |
| `src/<package>/DEBUG_NOTE.md` | 包内短调试入口 | 只保留快速检查信息；长期解释放入 `docs/` |
| `src/<package>/THIRD_PARTY_NOTICES.md` | 第三方来源与许可证 | 保留来源、许可证和合规说明，不随意移动到其他目录 |
| `refer/` | 外部参考副本 | 标明参考性质，不作为当前主工作区功能承诺 |

因此，并不是所有 Markdown 都必须物理移动到 `docs/`。用户级和开发者级长文档应集中在 `docs/`；贴近代码的 README、DEBUG_NOTE 和 THIRD_PARTY_NOTICES 保留在 package 内更清晰。

## 命名规则

新增仓库级文档使用小写 snake_case：

```text
quick_start.md
planner_framework.md
coverage_path_planning.md
documentation_management.md
```

历史文件如果已经被外部链接引用，可以保留为兼容入口，例如：

```text
URDF_MIGRATION_GUIDE.md
navigation_launch_reference.md
planner_path_topics.md
```

这类文件只做 redirect / entry page，不再承载重复正文。

## 双语规则

所有 Markdown 顶部使用统一语言切换：

```markdown
<div align="right">

[中文](#中文) | [English](#english)

</div>
```

正文使用同页锚点：

```markdown
<a id="中文"></a>

# 中文标题

---

<a id="english"></a>

# English Title
```

中文专题文档可以保留中文为主、英文概要为辅；根目录 README 和 package README 应尽量提供结构对称的中英文版本。许可证文本和外部参考原文可以保留原文，但需要增加另一种语言的用途说明。

## 链接规则

- 仓库内链接使用相对路径，不写本机绝对路径。
- package 内 README 链接 `docs/` 时使用相对路径，例如 `../../docs/planner_framework.md`。
- 图片和视频引用使用仓库内相对路径，例如 `docs/mobile-robot-planning.png`。
- 不链接不存在的文件；改名时保留兼容入口或同步更新所有引用。

## 内容边界

文档应只描述当前仓库中真实存在、可在源码中定位或已明确验证的能力。实验性能力需要标注 `Experimental`、`Basic Demo`、`Partial Support` 或类似说明。

尤其需要避免：

- 把 debug path 写成机器人实际执行路径；
- 把 BSA / Spiral-STC 博客写成当前已有 launch 入口；
- 把 `mr_learning` 写成成熟强化学习平台；
- 把 camera topic 写成完整视觉感知或视觉 SLAM 系统；
- 把 WPB Home Mani 的模型接入写成完整机械臂规划与抓取。

## 当前文档状态

当前仓库采用：

- 根目录 `README.md` 作为中英文总览；
- `docs/index.md` 作为专题文档导航；
- `docs/documentation_management.md` 作为文档维护规范；
- package 内 README 作为轻量入口；
- `THIRD_PARTY_NOTICES.md` 作为第三方来源和许可证保留文件；
- `refer/` 下 Markdown 作为外部参考副本。

---

<a id="english"></a>

# Documentation Management

This page defines where Markdown files belong, how they should be named, and how bilingual content is maintained in this repository. The goal is to keep the GitHub front page, topic documentation, package-local notes, and third-party notices clearly separated.

## Documentation Layers

| Location | Purpose | Maintenance rule |
|---|---|---|
| `README.md` | GitHub front page | Present project scope, architecture, Quick Start, core capabilities, and documentation navigation without duplicating every topic page |
| `docs/` | Repository-level topic docs | Store user/developer tutorials, references, migration guides, troubleshooting notes, and source indexes |
| `src/<package>/README.md` | Package-local overview | Keep package responsibility, key entries, minimal commands, and links to `docs/` |
| `src/<package>/DEBUG_NOTE.md` | Short package debug entry | Keep quick checks only; long-term explanations belong in `docs/` |
| `src/<package>/THIRD_PARTY_NOTICES.md` | Third-party sources and licenses | Keep source and license notices close to the package |
| `refer/` | External reference copies | Mark as reference material, not as current feature promises |

Not every Markdown file should be physically moved into `docs/`. Long user/developer documentation belongs in `docs/`; package-local README, DEBUG_NOTE, and THIRD_PARTY_NOTICES files are clearer when kept next to the code they describe.

## Naming

New repository-level documentation should use lowercase snake_case:

```text
quick_start.md
planner_framework.md
coverage_path_planning.md
documentation_management.md
```

Historical files that may already be linked externally can remain as compatibility entries, for example:

```text
URDF_MIGRATION_GUIDE.md
navigation_launch_reference.md
planner_path_topics.md
```

These files should act as redirect / entry pages, not duplicated long-form documentation.

## Bilingual Format

Every Markdown file should start with the same language switch:

```markdown
<div align="right">

[中文](#中文) | [English](#english)

</div>
```

Use same-page anchors:

```markdown
<a id="中文"></a>

# Chinese Title

---

<a id="english"></a>

# English Title
```

Chinese topic pages may keep Chinese as the detailed version and English as a summary. Root README and package README files should be as structurally symmetric as practical. License text and external reference copies may keep their original language, but they should include a clear explanation in the other language.

## Links

- Use repository-relative links, not local absolute paths.
- From package README files to `docs/`, use relative paths such as `../../docs/planner_framework.md`.
- Reference images and videos through repository-relative paths such as `docs/mobile-robot-planning.png`.
- Do not link missing files. When renaming a page, keep a compatibility entry or update all references.

## Scope Discipline

Documentation should describe only capabilities that exist in the repository, can be located in source code, or have been explicitly verified. Experimental capabilities should be marked as `Experimental`, `Basic Demo`, `Partial Support`, or similar wording.

Avoid claiming that:

- debug paths are robot-executed paths;
- BSA / Spiral-STC blog posts are current launch entries;
- `mr_learning` is a mature reinforcement-learning platform;
- camera topics form a complete perception or visual SLAM system;
- WPB Home Mani model integration includes complete arm planning and grasping.

## Current State

This repository currently uses:

- root `README.md` as the bilingual overview;
- `docs/index.md` as the topic documentation hub;
- `docs/documentation_management.md` as the documentation maintenance guide;
- package README files as lightweight local entries;
- `THIRD_PARTY_NOTICES.md` for third-party source and license text;
- Markdown under `refer/` as external reference copies.
