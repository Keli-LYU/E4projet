# Dependency Management in the CI/CD Pipeline

This document explains how dependencies are managed in the CI/CD pipeline for the Expleo Navigation Stack project.

## ROS Package Dependencies

The Expleo Navigation Stack relies on several ROS packages that aren't available through the standard rosdep system. These packages need to be built from source in the CI/CD pipeline.

### Key Dependencies

1. **Navigation2 (nav2)** - Contains `nav2_costmap_2d`, `nav2_behavior_tree`, and other navigation components
   - Repository: https://github.com/ros-planning/navigation2
   - Branch: galactic

2. **Carla Messages** - Contains `carla_msgs` for Carla simulator integration
   - Repository: https://github.com/carla-simulator/ros-carla-msgs
   - Branch: master

## How Dependencies are Handled

In the CI/CD pipeline, these dependencies are handled as follows:

1. **Cloning Required Repositories:**
   ```bash
   git clone https://github.com/ros-planning/navigation2.git -b galactic
   git clone https://github.com/carla-simulator/ros-carla-msgs.git -b master
   ```

2. **Building the Entire Workspace:**
   ```bash
   colcon build --symlink-install
   ```

3. **Handling rosdep Errors:**
   The rosdep command is run with `|| true` to prevent pipeline failures if some dependencies can't be resolved:
   ```bash
   rosdep install --from-paths src --ignore-src --rosdistro=galactic -y || true
   ```

## Local Development Setup

For local development, use the provided `run_tests_locally.sh` script which includes the necessary steps to:
1. Clone the required dependencies
2. Install system dependencies using rosdep
3. Build the workspace
4. Run tests

## Troubleshooting Dependencies

If you encounter dependency issues:

1. **Missing ROS Packages:**
   - Check that all required repositories are cloned correctly
   - Verify you're using the correct branch (galactic for Navigation2)
   - Make sure you've sourced the ROS 2 setup file before building

2. **System Dependency Issues:**
   - Run `rosdep install --from-paths src --ignore-src --rosdistro=galactic -y --rosdistro=galactic -r` to see detailed information about missing dependencies
   - Install any missing system packages manually if needed

3. **Build Failures:**
   - Check the build logs for specific error messages
   - Some packages might need to be built in a specific order - try building problematic packages individually first

## Adding New Dependencies

When adding new dependencies to the project:

1. Add them to your package.xml file using the appropriate tag:
   ```xml
   <depend>package_name</depend>
   ```

2. If the dependency is a standard ROS package available through rosdep, no additional changes are needed

3. If the dependency is a custom package that needs to be built from source:
   - Add the repository clone command to:
     - `.gitlab-ci.yml` file in the build, unit_test, and integration_test jobs
     - `run_tests_locally.sh` script
   - Document the new dependency in this file
