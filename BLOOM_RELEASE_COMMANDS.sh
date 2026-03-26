# ROS2 Studio — Bloom Release Cheatsheet
# Run these commands IN ORDER from your ros2_ws/src/ros2_studio directory

# ─── PHASE 1: Fix files (already done above) ───────────────────────────────
# Copy the new package.xml, setup.py, CHANGELOG.rst into your repo
# Copy .github/workflows/ros2_ci.yml into your repo
# Commit everything:
git add package.xml setup.py CHANGELOG.rst .github/workflows/ros2_ci.yml
git commit -m "chore: prepare package for rosdistro bloom release"
git push origin main

# ─── PHASE 2: Install bloom ────────────────────────────────────────────────
sudo apt-get install python3-bloom python3-catkin-pkg

# ─── PHASE 3: Tag release ──────────────────────────────────────────────────
# Bloom reads the version from package.xml (0.1.0), so tag must match exactly
git tag -a 0.1.0 -m "Release 0.1.0"
git push origin 0.1.0

# ─── PHASE 4: Request ros2-gbp release repository ──────────────────────────
# Open a GitHub issue at:
#   https://github.com/ros2-gbp/ros2-gbp-github-org/issues/new
# Title: "New release repo request: ros2_studio"
# Body:
#   Package name: ros2_studio
#   Source repo: https://github.com/Sourav0607/ROS2-STUDIO
#   Maintainer GitHub: Sourav0607
#   Target distros: humble, jazzy
# They will create: https://github.com/ros2-gbp/ros2_studio-release
# Wait for confirmation (usually 1-2 days)

# ─── PHASE 5: Configure GitHub PAT for bloom ──────────────────────────────
# Go to: GitHub → Settings → Developer settings → Personal access tokens
# Generate token with scopes: public_repo, workflow
# Save to ~/.config/bloom:
cat > ~/.config/bloom << 'EOF'
{
  "github_user": "Sourav0607",
  "oauth_token": "YOUR_PAT_HERE"
}
EOF

# ─── PHASE 6: Run bloom release ────────────────────────────────────────────
# Run from INSIDE your package directory (where package.xml is)
cd ~/ros2_ws/src/ros2_studio

bloom-release \
  --new-track \
  --rosdistro humble \
  --track humble \
  ros2_studio

# bloom will ask you interactively:
#   Release repo URL → https://github.com/ros2-gbp/ros2_studio-release.git
#   Repository name  → ros2_studio
#   VCS type         → git
#   VCS URI          → https://github.com/Sourav0607/ROS2-STUDIO.git
#   VCS branch       → main
# It will then:
#   1. Create release branches in ros2-gbp/ros2_studio-release
#   2. Automatically open a PR against ros/rosdistro

# ─── PHASE 7: After PR is merged ───────────────────────────────────────────
# Timeline:
#   Day 1-2:  rosdistro reviewer merges PR
#   Day 2-3:  Package appears in ros-testing
#             → Test with: sudo apt install ros-humble-ros2-studio (from ros-testing)
#   Week 2-4: Released to main ROS apt repo
#             → Available to everyone with: sudo apt install ros-humble-ros2-studio

# ─── PHASE 8: For Jazzy (after Humble succeeds) ────────────────────────────
bloom-release \
  --new-track \
  --rosdistro jazzy \
  --track jazzy \
  ros2_studio

# ─── SUBSEQUENT RELEASES (e.g. v0.2.0) ────────────────────────────────────
# Update version in package.xml AND setup.py to 0.2.0
# Update CHANGELOG.rst
# git tag 0.2.0 && git push origin 0.2.0
# bloom-release --rosdistro humble --track humble ros2_studio
# (no --new-track needed after first release)