# robotops-trace-integrations development commands
# Install just: https://github.com/casey/just

# ROS 2 distro selecting the build image (ros:${ROS_DISTRO}-ros-base) and the
# sourced /opt/ros/${ROS_DISTRO} underlay. MUST match the distro the debs are
# built/linked for (jazzy → Ubuntu 24.04 Noble, the default; humble → Ubuntu
# 22.04 Jammy, the arm64/Jetson target). Override with: just ROS_DISTRO=humble <recipe>
export ROS_DISTRO := env_var_or_default('ROS_DISTRO', 'jazzy')
export APT_REPO_URL := env_var_or_default('APT_REPO_URL', 'https://apt.robotops.com')

# Default recipe — show available commands
default:
    @just --list

# ----------------------------------------------------------------------------
# Version management (per package — version-check.yml runs per changed package)
# ----------------------------------------------------------------------------

# Bump a single package's version (usage: just bump-version <pkg> patch|minor|major)
bump-version pkg type:
    #!/usr/bin/env bash
    set -euo pipefail

    PKG="{{pkg}}"
    PKG_XML="packages/${PKG}/package.xml"
    if [[ ! -f "$PKG_XML" ]]; then
        echo "Error: $PKG_XML not found"; exit 1
    fi
    if [[ "{{type}}" != "patch" && "{{type}}" != "minor" && "{{type}}" != "major" ]]; then
        echo "Error: type must be 'patch', 'minor', or 'major'"; exit 1
    fi

    CURRENT=$(grep '<version>' "$PKG_XML" | sed 's/.*<version>\(.*\)<\/version>.*/\1/')
    IFS='.' read -r major minor patch <<< "$CURRENT"
    case "{{type}}" in
        patch) NEW_VERSION="$major.$minor.$((patch + 1))";;
        minor) NEW_VERSION="$major.$((minor + 1)).0";;
        major)
            NEW_VERSION="$((major + 1)).0.0"
            echo "⚠️  MAJOR bump for $PKG: $CURRENT -> $NEW_VERSION"
            echo "⚠️  A robotops-trace-cpp/python core major bump fans out a coordinated bump here (spec §2.5)."
            ;;
    esac

    echo "Bumping $PKG: $CURRENT -> $NEW_VERSION"
    sed -i.bak "s|<version>$CURRENT</version>|<version>$NEW_VERSION</version>|" "$PKG_XML"
    rm "${PKG_XML}.bak"

    # ament_python packages also carry the version in setup.py / pyproject.toml.
    if [[ -f "packages/${PKG}/setup.py" ]]; then
        sed -i.bak "s|version=\"$CURRENT\"|version=\"$NEW_VERSION\"|" "packages/${PKG}/setup.py" && rm "packages/${PKG}/setup.py.bak" || true
    fi
    if [[ -f "packages/${PKG}/pyproject.toml" ]]; then
        sed -i.bak "s|^version = \"$CURRENT\"|version = \"$NEW_VERSION\"|" "packages/${PKG}/pyproject.toml" && rm "packages/${PKG}/pyproject.toml.bak" || true
    fi

    # Add a CHANGELOG entry for this package's version.
    DATE=$(date +%Y-%m-%d)
    {
        echo "$NEW_VERSION ($DATE)"
        echo "-------------------"
        echo ""
        echo "* (${PKG}) "
        echo ""
    } > /tmp/changelog_entry.txt
    awk '/^[0-9]+\.[0-9]+\.[0-9]+ \(/ { if (!inserted) { system("cat /tmp/changelog_entry.txt"); inserted=1 } } { print }' CHANGELOG.rst > /tmp/CHANGELOG.rst.new
    mv /tmp/CHANGELOG.rst.new CHANGELOG.rst
    rm /tmp/changelog_entry.txt

    echo "✅ $PKG bumped to $NEW_VERSION — edit CHANGELOG.rst to add details."

# ----------------------------------------------------------------------------
# Docker
# ----------------------------------------------------------------------------

# Build the test Docker image for the selected distro
build-image:
    DOCKER_BUILDKIT=1 docker build \
        --build-arg ROS_DISTRO={{ROS_DISTRO}} \
        --build-arg APT_REPO_URL={{APT_REPO_URL}} \
        --target test -t robotops-trace-integrations:ci-{{ROS_DISTRO}} .

# Interactive development shell
dev:
    DOCKER_BUILDKIT=1 docker build \
        --build-arg ROS_DISTRO={{ROS_DISTRO}} \
        --target dev -t robotops-trace-integrations:dev-{{ROS_DISTRO}} .
    docker run --rm -it \
        -v "$PWD":/workspace/src/robotops-trace-integrations \
        -w /workspace/src/robotops-trace-integrations \
        robotops-trace-integrations:dev-{{ROS_DISTRO}}

# ----------------------------------------------------------------------------
# Build / test
# ----------------------------------------------------------------------------

# Build the whole colcon workspace (run inside the container)
build:
    #!/usr/bin/env bash
    set -exo pipefail
    source /opt/ros/${ROS_DISTRO}/setup.bash
    cd /workspace
    colcon build --packages-up-to \
        robotops_trace_semconv robotops_trace_rclcpp robotops_trace_rclpy \
        robotops_trace_bt_cpp robotops_trace_bt_cpp_autoattach \
        robotops_trace_rclcpp_autoattach \
        robotops_trace_ros2_control robotops_trace_moveit

# Run colcon tests + lint (inside the container)
test:
    #!/usr/bin/env bash
    set -exo pipefail
    source /opt/ros/${ROS_DISTRO}/setup.bash
    cd /workspace
    colcon test --event-handlers console_direct+
    colcon test-result --verbose

# Full CI suite for GitHub Actions (already inside the container)
ci-inner:
    #!/usr/bin/env bash
    set -exo pipefail
    echo "🚀 Running CI suite (in container)..."
    just build
    just test
    echo "✅ CI suite completed!"

# Clean build artifacts
clean:
    rm -rf build/ install/ log/
