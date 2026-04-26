# GitLab CI/CD Pipeline for Expleo Navigation Stack

This document explains the testing and deployment pipeline setup for the Expleo Navigation Stack.

## Pipeline Stages

The CI/CD pipeline consists of the following stages:

### 1. Linting Stage
- Checks code quality and style conformance
- Uses `cpplint` for C++ code and `flake8` for Python code
- Helps maintain consistent code standards and catch potential issues early
- Results are saved as artifacts for later review

### 2. Build Stage
- Compiles the entire codebase in a ROS 2 Galactic environment
- Uses `colcon build` to build all packages
- Creates a workspace structure identical to what would be used in deployment
- Build artifacts are saved for the testing stage

### 3. Unit Testing Stage
- Runs all unit tests for each package
- Uses `colcon test` to execute tests for individual packages
- Outputs detailed test results that can be reviewed after test execution
- Test logs are saved as artifacts

### 4. Integration Testing Stage
- Runs simulation-based tests using Carla simulator in a headless environment
- Tests the navigation stack's performance in a realistic scenario
- More complex tests that verify the system works as a whole
- Allowed to fail as simulation tests can sometimes be flaky

### 5. Deployment Stage
- Connects to a virtual machine via SSH
- Runs the `script.sh` file to start the navigation stack components
- Only runs on the main branch by default
- Only requires unit tests to pass (integration tests are optional)

## Required Variables

The following variables must be configured in the GitLab CI/CD settings:

- `SSH_USER`: Username to connect to the VM
- `SSH_PASSWORD`: Password for the SSH user (marked as masked)
- `SSH_HOST`: Hostname or IP address of the VM
- `SSH_KNOWN_HOSTS`: The SSH known hosts content for your VM (optional)
- `REMOTE_DIRECTORY`: Path to the directory on the VM where script.sh is located

## Working with the Pipeline

### Dependency Management

The Expleo Navigation Stack depends on several packages that need to be built from source:
- **Navigation2 (nav2)** - For navigation components
- **Carla Messages** - For Carla simulator integration

See the [dependency-management.md](./dependency-management.md) file for details on how these dependencies are handled in the CI/CD pipeline.

### Adding Tests

To add new tests for your packages:
1. Create test files within your package's `test` directory
2. Update your package's `CMakeLists.txt` to include the tests
3. The CI pipeline will automatically run these tests

### Viewing Test Results

Test results can be viewed in the GitLab CI/CD interface:
1. Navigate to the pipeline execution
2. Click on the "test" job
3. View the job output for test execution details
4. Download and examine the test artifacts for detailed logs

### Fixing Build Issues

If the build fails:
1. Check the build job logs for specific error messages
2. Fix any compilation errors in your code
3. Push your changes to trigger a new pipeline run
4. Verify that the build completes successfully

### Troubleshooting Deployment

If deployment fails:
1. Verify that all required CI/CD variables are correctly set
2. Check that the VM is accessible with the provided credentials
3. Confirm that the `script.sh` file exists in the specified directory
4. Examine the deployment job logs for specific error messages
