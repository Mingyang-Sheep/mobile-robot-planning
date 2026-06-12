<div align="right">

[Chinese](zh/documentation_management.md)

</div>

# Documentation Management

This page defines how Markdown files are organized, named, and linked in this repository. The current rule is separate-language documentation: default files are English, and Chinese files live in parallel `_zh.md` files or under `docs/zh/`.

## Documentation Layers

| Location | Purpose | Maintenance rule |
|---|---|---|
| `README.md` | GitHub front page | English project overview, architecture, Quick Start, capability tables, and documentation navigation |
| `README_zh.md` | Chinese front page | Chinese counterpart of the root README |
| `docs/` | Repository-level English topic docs | Default documentation shown from the repository |
| `docs/zh/` | Repository-level Chinese topic docs | Chinese counterparts with the same filenames as `docs/` |
| `src/<package>/README.md` | Package-local English overview | Short package responsibility, key entries, minimal commands, and links to `docs/` |
| `src/<package>/README_zh.md` | Package-local Chinese overview | Chinese counterpart of the package README |
| `src/<package>/DEBUG_NOTE.md` | Package-local English debug note | Short quick-check entry only |
| `src/<package>/DEBUG_NOTE_zh.md` | Package-local Chinese debug note | Chinese counterpart of the debug note |
| `src/<package>/THIRD_PARTY_NOTICES.md` | Third-party source and license text | Keep source and license notices close to the code |
| `refer/` | External reference copies | Mark as reference material, not as current feature promises |

Not every Markdown file should be physically moved into `docs/`. Long user and developer documentation belongs in `docs/`; package-local README, DEBUG_NOTE, and THIRD_PARTY_NOTICES files are clearer when kept next to the code they describe.

## Naming

New repository-level English documents use lowercase snake_case:

```text
quick_start.md
planner_framework.md
coverage_path_planning.md
documentation_management.md
```

Chinese counterparts under `docs/zh/` use the same filenames:

```text
docs/quick_start.md
docs/zh/quick_start.md
```

Package-local Chinese counterparts use `_zh`:

```text
src/mr_slam/README.md
src/mr_slam/README_zh.md
src/mr_traditional_planner/DEBUG_NOTE.md
src/mr_traditional_planner/DEBUG_NOTE_zh.md
```

Historical filenames that may already be linked externally can remain as compatibility entries:

```text
URDF_MIGRATION_GUIDE.md
navigation_launch_reference.md
planner_path_topics.md
```

These files should act as entry pages, not duplicated long-form documentation.

## Language Rules

English files should use English prose only. Chinese files should use Chinese prose only. Code commands, package names, file paths, topic names, ROS message names, plugin names, and upstream project names keep their original spelling.

Recommended language switch:

```markdown
<div align="right">

Chinese: zh/example.md

</div>
```

For Chinese files:

```markdown
<div align="right">

English: ../example.md

</div>
```

Avoid same-page bilingual sections in normal documentation. Keep the two languages in separate files.

## Status Icons

Use visual status icons in support matrices:

| Icon | Meaning |
|---|---|
| ✅ | Supported or available |
| ❌ | Not implemented |
| 🟣 | Partial, experimental, or indirect support |

Prefer these icons over plain `yes` / `no` tables when documenting support status.

## Links

- Use repository-relative links, not local absolute paths.
- From package README files to `docs/`, use relative paths such as `../../docs/planner_framework.md`.
- Reference images through repository-relative paths such as `docs/assets/mobile-robot-planning.png`.
- Embed GIF demonstrations through repository-relative paths such as `docs/assets/00_project_overview.gif`. Do not use `<video>` or embed MP4 files in the README.
- Do not link missing files. When renaming a page, keep a compatibility entry or update all references.

## Scope Discipline

Documentation should describe only capabilities that exist in the repository, can be located in source code, or have been explicitly verified. Experimental capabilities should be marked as `Experimental`, `Basic Demo`, `Partial Support`, or similar wording.

Avoid claiming that:

- debug paths are robot-executed paths;
- BSA / Spiral-STC blog posts are current launch entries;
- `mr_learning` is a mature reinforcement-learning platform;
- camera topics form a complete perception or visual SLAM system;
- WPB Home Mani model integration includes complete arm planning and grasping.
