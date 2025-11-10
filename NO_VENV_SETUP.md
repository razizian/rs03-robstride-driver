# Running Without Virtual Environment

Since you don't want to use virtual environments, you have two options:

## Option 1: Install System Packages (Recommended)

```bash
# Install required system packages
sudo apt update
sudo apt install python3-can python3-dotenv python3-numpy

# Clone and install robstride manually
cd /tmp
git clone https://github.com/sirwart/robstride
cd robstride
sudo python3 setup.py install

# Now you can run tests directly
cd /path/to/cyberfusion_sdk_rs03
python3 examples/hardware_validation.py
```

## Option 2: Use Standalone Test

A minimal standalone test is available that only requires python3-can:

```bash
# Install minimal dependency
sudo apt install python3-can

# Run standalone test
./test_standalone.py
```

This will:
- Check CAN interface
- Send enable/disable commands
- Listen for motor responses
- Verify basic communication

## Option 3: Fix UV Project

Remove the ROS2 dependency from pyproject.toml:

1. Edit `pyproject.toml`
2. Remove the `[project.optional-dependencies]` section
3. Run `uv sync`
4. Then use `uv run python examples/hardware_validation.py`

## Current Issue

The validation scripts are failing because:
1. Python dependencies (python-can, robstride) are not installed system-wide
2. UV is trying to install ROS2 dependencies which aren't available
3. System Python (PEP 668) prevents pip installs without venv

Choose one of the options above to proceed with testing.

