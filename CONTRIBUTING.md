# Contributing to Robotous F/T Sensor Pipeline

Thanks for your interest in contributing!

## Development Setup

```bash
# Clone and set up workspace
git clone https://github.com/ItsLP18/robotous-ft-pipeline.git
cd robotous-ft-pipeline

mkdir -p ~/ft_sensor_ws/src
ln -s $(pwd) ~/ft_sensor_ws/src/ft_sensor_pipeline

cd ~/ft_sensor_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ft_sensor_pipeline
source install/setup.bash
```

## Code Style

- Python: Follow [PEP 8](https://peps.python.org/pep-0008/)
- Type hints on all public functions
- Docstrings on all classes and public methods
- Max line length: 100 characters

## Git Workflow

1. Fork the repository
2. Create a feature branch: `git checkout -b feat/your-feature`
3. Commit with [Conventional Commits](https://www.conventionalcommits.org/):
   - `feat:` new feature
   - `fix:` bug fix
   - `docs:` documentation
   - `refactor:` code restructuring
   - `test:` adding tests
4. Push and open a Pull Request

## Testing

```bash
# Run without hardware
ros2 launch ft_sensor_pipeline ft_pipeline_launch.py use_mock:=true

# Unit tests
python3 -m pytest test/ -v
```

## Reporting Issues

Use the [issue templates](.github/ISSUE_TEMPLATE/) for bug reports and feature requests.
