# robotops-trace-integrations

Framework-integration **monorepo** for the RobotOps distributed-tracing SDK +
carrier pivot (ROB-418). One repo, many packages: each framework hook is its own
colcon package that builds + blooms independently into a per-package Debian
(`ros-<distro>-robotops-trace-<pkg>`), but they share one semantic-conventions
package and move in lockstep with the SDK-version contract.

This is the third of three repos in the pivot (see
`robotops-trace-buildout-spec.md` §1):

| Repo | Role |
|---|---|
| `robotops-trace-cpp` | C++ SDK core (apt) |
| `robotops-trace-python` | Python SDK core (PyPI + apt) |
| **`robotops-trace-integrations`** | **framework hooks (this repo)** |

> **Why one repo, many packages?** All integrations share the same
> semantic-conventions header/module and must move with the same SDK-version
> contract; a repo-per-framework split would scatter that across N release
> cadences. colcon handles the multi-package workspace cleanly, and per-package
> debs still give customers à-la-carte install — monorepo source ≠ monolith
> install (spec §1).

## Packages

| Package | Lang / build | Issue | Distributes to | Status |
|---|---|---|---|---|
| [`robotops_trace_semconv`](packages/robotops_trace_semconv) | C++ header-only + Python (ament_cmake) | [ROB-430](https://linear.app/robotops/issue/ROB-430) | apt + PyPI | 🟡 stub |
| [`robotops_trace_rclcpp`](packages/robotops_trace_rclcpp) | C++ (ament_cmake) | [ROB-422](https://linear.app/robotops/issue/ROB-422) | apt | 🟡 stub |
| [`robotops_trace_rclpy`](packages/robotops_trace_rclpy) | Python (ament_python) | [ROB-423](https://linear.app/robotops/issue/ROB-423) | apt + PyPI | 🟡 stub |
| [`robotops_trace_bt_cpp`](packages/robotops_trace_bt_cpp) | C++ (ament_cmake) | [ROB-424](https://linear.app/robotops/issue/ROB-424) | apt | 🟡 stub |
| [`robotops_trace_ros2_control`](packages/robotops_trace_ros2_control) | C++ (ament_cmake) | [ROB-425](https://linear.app/robotops/issue/ROB-425) | apt | 🟡 stub |
| [`robotops_trace_moveit`](packages/robotops_trace_moveit) ★ | C++ (ament_cmake) | [ROB-426](https://linear.app/robotops/issue/ROB-426) | apt | 🟡 stub |

★ `robotops_trace_moveit` carries a not-yet-upstream MoveIt patch (the sole
carried-fork exception, spec §3.4); it is minimized and pushed upstream over time.

**C++ packages:** `robotops_trace_semconv` (header-only), `robotops_trace_rclcpp`,
`robotops_trace_bt_cpp`, `robotops_trace_ros2_control`, `robotops_trace_moveit`.
**Python packages:** `robotops_trace_rclpy` (ament_python), plus the
`robotops_trace_semconv` Python mirror module.

## Status — scaffold only ⚠️

**This repo is a scaffold (this commit is an AI-generated scaffold for developer
review). Every package is a STUB** — no real integration logic. The packages are
filled in by ROB-422, ROB-423, ROB-424, ROB-425, ROB-426, and ROB-430.

**CI is expected to FAIL right now, and that is intentional — it has not been
faked green.** The integration packages depend on the SDK cores:

- `ros-<distro>-robotops-trace-cpp` (apt) — **not yet published**
- `robotops-trace` / `robotops_trace_python` (PyPI + apt) — **not yet published**
- `ros-<distro>-robotops-trace-semconv` (apt) — published from this repo once released

Until those cores are released:

- **`ci.yml`** (per-distro Docker matrix) fails at `rosdep` dependency resolution
  / colcon build. The Dockerfile's `rosdep install` step is deliberately
  non-fatal so the image still builds, but `just ci-inner` (the real colcon
  build) fails.
- **`release.yml` / `release-dev.yml`** fail at `rosdep install` before bloom.
- **`pypi-publish.yml`** additionally needs a one-time PyPI Trusted Publisher
  registration (see the workflow header) before it can publish.

The scaffold is structurally correct: the package manifests declare the right
version-pinned deps, the rosdep mapping is correct, and everything will resolve
once the cores ship.

## CI/CD

| Workflow | Trigger | What it does |
|---|---|---|
| `ci.yml` | push to `feature/**`, `[0-9]*` | per-distro (jazzy/humble × arch) Docker build → colcon build + test |
| `branch-name-validation.yml` | PR | enforce `feature/* \| fix/* \| task/* \| chore/* \| [issue#]-*` → `development`, only `development` → `main` |
| `version-check.yml` | PR → `development`/`main` | per **changed package**: semver + matching CHANGELOG entry |
| `release.yml` | `workflow_dispatch` on `main` | bloom each package → per-package deb → `publish-debian-s3` (prod `apt.robotops.com`); per-package `<pkg>/v<ver>` tags |
| `release-dev.yml` | `workflow_dispatch` on `development` | same → dev `apt.development.robotops.com`; `<pkg>/v<ver>-development-<sha>` tags |
| `pypi-publish.yml` | `workflow_dispatch` | build + publish `robotops_trace_rclpy` / `robotops_trace_semconv` to PyPI/TestPyPI (OIDC Trusted Publishing) |

The `.github/actions/publish-debian-s3` composite action is copied verbatim from
`rmw_robotops` (Aptly stateless rebuild from the S3 pool, GPG-sign, CloudFront
invalidate).

## Versioning contract

Independent semver per package (`package.xml` is the source of truth; the Python
packages mirror it in `setup.py` / `pyproject.toml`). Each package pins the SDK
core with `>=0.1.0`. A core **major** bump fans out a coordinated bump across the
integration packages (spec §2.5). Bump with:

```sh
just bump-version <package> patch|minor|major
```

## Layout

```
packages/
  robotops_trace_semconv/        # ROB-430 — header-only C++ + Python mirror
  robotops_trace_rclcpp/         # ROB-422 — ament_cmake
  robotops_trace_rclpy/          # ROB-423 — ament_python (+ PyPI)
  robotops_trace_bt_cpp/         # ROB-424 — ament_cmake
  robotops_trace_ros2_control/   # ROB-425 — ament_cmake
  robotops_trace_moveit/         # ROB-426 — ament_cmake (★ async patch)
.github/
  workflows/                     # ci, branch-name-validation, version-check, release, release-dev, pypi-publish
  actions/publish-debian-s3/     # shared (verbatim from rmw_robotops)
Dockerfile                       # multi-stage, ARG ROS_DISTRO / APT_REPO_URL
justfile                         # bump-version, build, test, ci-inner
```

## Development

ROS 2 is not installed on the host — all builds run in Docker:

```sh
just build-image            # build the test image (ROS_DISTRO=jazzy by default)
just ROS_DISTRO=humble dev  # interactive shell for humble
```

## License

Apache-2.0 — see [LICENSE](LICENSE). © Robot Ops Inc.
