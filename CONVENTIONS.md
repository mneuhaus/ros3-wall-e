# Project Conventions

## Coding Standards

### Python Code
- Follow PEP 8 style guide
- Maximum line length: 120 characters
- Use type hints for all function/method signatures
- ROS2 nodes follow this structure:
  ```python
  class NodeClassName(Node):
      def __init__(self):
          super().__init__('node_name')
          self.declare_parameters()
          self.setup_hardware()
          self.create_subscriptions()
          self.create_publishers()
  ```

### Firmware Code (Pico)
- Use MicroPython 1.19.1 conventions
- Hardware configuration centralized in class __init__
- Use descriptive names for pins (e.g., `servo2040.SERVO_1`)

## Repository Structure

```
src/
├── package_name/           # ROS2 package
│   ├── resource/           # Launch files and configs
│   ├── scripts/           # Executable Python scripts
│   ├── package_name/      # Python module
│   │   ├── __init__.py
│   │   └── node_file.py   # Node implementation
│   ├── test/              # Unit tests
│   ├── package.xml
│   └── setup.py
backup/                    # System configuration backups
scripts/                   # Development utilities
```

## Naming Conventions

1. Packages:
   - ROS2 packages: `snake_case` (e.g., `motor_controller`)
   - Firmware packages: `device_purpose` (e.g., `servo_2040`)

2. Nodes:
   - Descriptive names ending with `_node` (e.g., `servo_2040_node`)
   - Node classes: `PurposeNode` (e.g., `MotorControllerNode`)

3. Topics/Services:
   - Follow ROS2 naming standards
   - Service names use `camelCase` (e.g., `/setServoPosition`)

## Documentation

- All files start with header comment:
  ```python
  #!/usr/bin/env python3
  """
  Brief description - Max 120 chars
  
  Long description (if needed)
  """
  ```
- Classes and public methods require docstrings
- ROS2 parameters documented in `__init__`
- Each package has a README.md with:
  - Purpose description
  - Launch instructions
  - Parameter documentation

## Git Practices

1. Commit messages:
   - Format: `<type>(<scope>): <subject>`
   - Types: feat, fix, docs, style, refactor, test, chore
   - Example: `feat(motor): Add PID control implementation`

2. Branching:
   - `main`: Stable production code
   - `feature/*`: New functionality
   - `fix/*`: Bug fixes

3. PR Guidelines:
   - Must pass existing CI checks
   - Requires 1 reviewer approval
   - Update documentation with changes

## Testing

1. Unit Tests:
   - Follow Arrange-Act-Assert pattern
   - Test files in `test/` directory
   - Named `test_*.py`

2. Integration Tests:
   - ROS2 launch file testing
   - Hardware interface validation

3. Firmware Tests:
   - Manual validation checklist
   - LED patterns for status reporting

## Hardware-Specific Conventions

1. UART Configuration:
   - Default baudrate: 115200
   - 8 data bits, no parity, 1 stop bit

2. LED Patterns:
   - Index 0: Power/Status indicator
   - Index 1-3: System activity
   - Index 4-9: Application-specific

3. Servo Control:
   - Use degrees (0-180) for positions
   - Disable servos when not in use
   - Smooth transitions between positions