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

## Firmware Update Protocol

1. To safely update firmware:
   - Send `{"command": "enter_bootloader"}` via serial
   - Firmware will reset into bootloader mode within 500ms
   - USB mass storage device will appear for file upload

2. Serial communication must:
   - Use 115200 baud rate
   - Include 100ms pauses between commands
   - Send UTF-8 encoded JSON messages ending with newline

3. Failure recovery:
   - Hardware reset: Hold BOOTSEL while plugging in
   - Automatically resets to normal mode after 5s timeout

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

## Documentation

### Documentation Structure
- All core packages must have appropriate documentation in the `docs/` folder
- Package-specific documentation should be named `<package-name>.md`
- Protocol specifications and other system-wide documentation should have descriptive names
- Use markdown format for all documentation files

### Version Control
- All version numbers follow semantic versioning (MAJOR.MINOR.PATCH)
- Tag releases with version number (v1.0.0)
- Include changelog entries for all changes
- Reference issue numbers in commit messages when applicable

### Code Review Process
1. Create feature branch
2. Submit pull request
3. Pass automated tests
4. Code review by team member
5. Address feedback
6. Merge to main branch

### Error Handling
- Use descriptive error messages
- Log errors with appropriate severity
- Include context in error messages
- Handle all error cases gracefully
- Provide user-friendly error responses

### Configuration Management
- Use ROS2 parameters for configurable values
- Store constants in dedicated config files
- Document all configuration options
- Provide default values
- Include parameter validation

### Documentation Content Requirements
Each package documentation should include:
1. Overview of the package's purpose
2. Hardware requirements and setup (if applicable)
3. Software dependencies
4. Configuration parameters
5. Usage instructions
6. Troubleshooting guide
7. API documentation (if the package provides one)

### Documentation Style
- Use clear, concise language
- Include code examples where appropriate
- Keep documentation up-to-date with code changes
- Use proper markdown formatting
- Include diagrams or images when they help explain concepts
- Document both success and error scenarios
- Include links to relevant external resources

### Protocol Documentation
- Protocol specifications should be comprehensive and include:
  - Command format and syntax
  - All available commands
  - Response formats
  - Error handling
  - Example usage
  - Implementation considerations
