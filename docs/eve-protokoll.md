# **EVE Protocol**

_(Extensible, Versatile, Easy)_

## **1. Command & Response Format**

1. **One Command per Line**
    
    ```css
    COMMAND [ARG=VALUE] [ARG=VALUE] ...
    ```
    
2. **Response**: Echo the **entire command**, followed by a colon `:`, then either
    
    * `OK [optional data]`
    * `ERROR [ERROR_TYPE] [details]`

**Example**

```
SET_GPIO PIN=2 PWM=128
```

**Response**

```
SET_GPIO PIN=2 PWM=128: OK
```

* * *

## **2. Initialization**

### 2.1 `INIT_GPIO`

```php-template
INIT_GPIO PIN=<pin> MODE=<mode> [COUNT=<num_leds>] [FREQ=<hz>]
```

* **PIN**: Numeric pin identifier
* **MODE**: One of `INPUT`, `OUTPUT`, `PWM`, `ADC`, `SERVO`, or `NEOPIXEL`
* **COUNT**: (Optional, for `NEOPIXEL`) Number of LEDs in the strip
* **FREQ**: (Optional) Frequency in Hz, if relevant (e.g., PWM or SERVO)

**Examples**

```
INIT_GPIO PIN=2 MODE=PWM FREQ=5000
INIT_GPIO PIN=3 MODE=SERVO FREQ=50
INIT_GPIO PIN=5 MODE=NEOPIXEL COUNT=8
```

**Responses**

```
INIT_GPIO PIN=2 MODE=PWM FREQ=5000: OK
INIT_GPIO PIN=3 MODE=SERVO FREQ=50: OK
INIT_GPIO PIN=5 MODE=NEOPIXEL COUNT=8: OK
```

* * *

## **3. GPIO Commands**

### 3.1 Digital State

```arduino
SET_GPIO PIN=<pin> STATE=<HIGH|LOW>
```

**Example**

```arduino
SET_GPIO PIN=1 STATE=HIGH
```

**Response**

```arduino
SET_GPIO PIN=1 STATE=HIGH: OK
```

### 3.2 PWM Value

```php-template
SET_GPIO PIN=<pin> PWM=<0–255>
```

**Example**

```
SET_GPIO PIN=2 PWM=128
```

**Response**

```
SET_GPIO PIN=2 PWM=128: OK
```

### 3.3 Bulk GPIO _(Optional)_

```arduino
BULK_GPIO PIN1=<HIGH|LOW|PWM=...> PIN2=...
```

**Example**

```arduino
BULK_GPIO PIN1=HIGH PIN2=PWM=128
```

**Response**

```arduino
BULK_GPIO PIN1=HIGH PIN2=PWM=128: OK
```

* * *

## **4. Servo Commands**

### 4.1 Move Servo

```php-template
MOVE_SERVO PIN=<pin> POS=<angle> [SPEED=<value>]
```

* **POS**: Target angle (e.g., 0–180 degrees)
* **SPEED**: Optional speed parameter

**Example**

```
MOVE_SERVO PIN=3 POS=90 SPEED=50
```

**Response**

```
MOVE_SERVO PIN=3 POS=90 SPEED=50: OK
```

### 4.2 Get Servo Position

```php-template
GET_SERVO_POS PIN=<pin>
```

**Example**

```
GET_SERVO_POS PIN=3
```

**Response**

```
GET_SERVO_POS PIN=3: OK POS=45
```

* * *

## **5. ADC Commands**

### 5.1 Read ADC

```php-template
READ_ADC PIN=<pin>
```

**Example**

```
READ_ADC PIN=4
```

**Response**

```
READ_ADC PIN=4: OK VALUE=768
```

* * *

## **6. Neopixel Commands**

### 6.1 Initialization _(Part of `INIT_GPIO`)_

```php-template
INIT_GPIO PIN=<pin> MODE=NEOPIXEL COUNT=<num_leds>
```

**Example**

```
INIT_GPIO PIN=5 MODE=NEOPIXEL COUNT=8
```

**Response**

```
INIT_GPIO PIN=5 MODE=NEOPIXEL COUNT=8: OK
```

### 6.2 Set Single LED

```php-template
SET_NEOPIXEL PIN=<pin> LED=<index> R=<0-255> G=<0-255> B=<0-255>
```

**Example**

```css
SET_NEOPIXEL PIN=5 LED=0 R=255 G=128 B=64
```

**Response**

```css
SET_NEOPIXEL PIN=5 LED=0 R=255 G=128 B=64: OK
```

### 6.3 Bulk Neopixel Update _(Optional)_

```css
BULK_NEOPIXEL PIN=<pin> L0=<r,g,b> L1=<r,g,b> ...
```

**Example**

```
BULK_NEOPIXEL PIN=5 L0=255,0,0 L1=0,255,0
```

**Response**

```
BULK_NEOPIXEL PIN=5 L0=255,0,0 L1=0,255,0: OK
```

* * *

## **7. System Commands**

### 7.1 Ping

```
PING
```

**Response**

```makefile
PING: OK
```

### 7.2 Reset Firmware

```
RESET_FIRMWARE
```

**Response**

```makefile
RESET_FIRMWARE: OK
```

### 7.3 Reset All Pins

```
RESET_ALL_PINS
```

**Response**

```makefile
RESET_ALL_PINS: OK
```

### 7.4 Reset Individual Pin

```php-template
RESET_PIN PIN=<pin>
```

**Example**

```
RESET_PIN PIN=3
```

**Response**

```
RESET_PIN PIN=3: OK
```

* * *

## **8. Error Handling**

**Format**:

```less
[COMMAND plus arguments]: ERROR [ERROR_TYPE] [additional details]
```

**Examples**:

1. **Invalid Pin**
    
    ```arduino
    SET_GPIO PIN=99 STATE=HIGH
    ```
    
    **Response**
    
    ```arduino
    SET_GPIO PIN=99 STATE=HIGH: ERROR INVALID_PIN PIN=99
    ```
    
2. **Invalid Mode**
    
    ```
    INIT_GPIO PIN=1 MODE=ALIEN
    ```
    
    **Response**
    
    ```vbnet
    INIT_GPIO PIN=1 MODE=ALIEN: ERROR INVALID_MODE MODE=ALIEN
    ```
    
3. **Invalid Frequency**
    
    ```
    INIT_GPIO PIN=2 MODE=PWM FREQ=999999
    ```
    
    **Response**
    
    ```vbnet
    INIT_GPIO PIN=2 MODE=PWM FREQ=999999: ERROR INVALID_FREQ FREQ=999999
    ```
    
4. **Uninitialized Pin**
    
    ```
    READ_ADC PIN=4
    ```
    
    **Response**
    
    ```vbnet
    READ_ADC PIN=4: ERROR UNINITIALIZED_ADC_PIN PIN=4
    ```
    

* * *

## **Appendix: Development Reasoning Behind EVE Protocol**

### **1. Project Requirements and Goals**

The primary objective was to design a communication protocol for controlling a 3D-printed Wall-E robot using ROS-2, with the following constraints and requirements:

* **Simplicity and Readability**: Ensure that the protocol is easy to understand and debug, both for humans and machines.
* **Extensibility**: Allow for future expansions, such as additional hardware components or functionalities.
* **Efficiency**: Minimize communication overhead while maintaining robustness.
* **Separation of Concerns**: Delegate hardware-specific tasks to the MCU, while the Raspberry Pi handles higher-level business logic.

### **2. Initial Design Considerations**

* **Protocol Format**: Started with JSON for its flexibility and readability but shifted to a simpler text-based protocol inspired by G-code to enhance efficiency and ease of parsing on constrained MCUs.
* **Command Structure**: Emphasized one command per line to streamline communication and processing.
* **Response Format**: Decided to echo the entire command in responses to facilitate straightforward parsing and debugging, ensuring clear association between commands and their outcomes.

### **3. Functional Requirements**

* **GPIO Control**: Needed capabilities to set digital states (`HIGH`/`LOW`), PWM values, and handle multiple GPIO pins efficiently.
* **Servo Control**: Required precise movement commands with optional speed parameters and the ability to query servo positions.
* **ADC Integration**: Incorporated analog input capabilities for sensor readings, ensuring proper initialization and data retrieval.
* **Neopixel Support**: Added control over individually addressable LEDs, necessitating commands for setting single LEDs and bulk updates.
* **System Commands**: Included essential commands for system health checks (`PING`) and resets to ensure reliability and recoverability.

### **4. Enhancing Robustness and Flexibility**

* **Initialization Commands**: Unified all pin initializations under `INIT_GPIO` with modes to handle various functionalities (`INPUT`, `OUTPUT`, `PWM`, `ADC`, `SERVO`, `NEOPIXEL`). This consolidation simplifies the protocol and centralizes configuration.
* **Frequency Parameters**: Introduced optional `FREQ` parameters for PWM and servo modes to allow dynamic control over signal timings, catering to advanced use cases and hardware variations.
* **Error Handling**: Developed a comprehensive error response structure that echoes the command and provides specific error types and details, aiding in quick diagnosis and troubleshooting.

### **5. Iterative Refinement**

* **Feedback Incorporation**: Based on iterative discussions, refined command syntax for clarity and consistency, such as switching from `ID` to `PIN` for servo identification and integrating Neopixel initialization within `INIT_GPIO`.
* **Optional Commands**: Added bulk commands (e.g., `BULK_GPIO`, `BULK_NEOPIXEL`) to enhance efficiency when multiple operations are needed simultaneously, without overcomplicating the protocol for simpler use cases.

### **6. Naming and Thematic Consistency**

* **EVE Protocol**: Chose a thematic name inspired by Wall-E’s counterpart, EVE, reflecting the protocol’s design principles—**Extensible, Versatile, and Easy**. This name aligns with the project's robotic and creative nature, fostering an engaging and memorable identity.

### **7. Final Specification Goals**

The final **EVE Protocol** aims to:

* **Balance Simplicity and Functionality**: Provide a straightforward command structure while accommodating a wide range of hardware controls.
* **Ensure Ease of Implementation**: Facilitate easy parsing and handling on both the MCU and Raspberry Pi, enabling efficient development and maintenance.
* **Promote Scalability**: Allow seamless integration of additional features and hardware components in the future without disrupting existing functionalities.
* **Enhance Debugging and Maintenance**: Through clear and consistent response formats, streamline the process of identifying and resolving issues.

* * *

## **EVE Protocol at a Glance**

1. **Command → Response** format:
    * **Echo** the full command text.
    * End with `: OK` or `: ERROR ...`.
2. **Modes**: `INPUT`, `OUTPUT`, `PWM`, `ADC`, `SERVO`, `NEOPIXEL`.
3. **Optional Frequency** for PWM/Servo: `FREQ=<hz>`.
4. **Error Handling**: Provide meaningful error codes and details.
5. **Extensible**: Easy to add new modes or commands.

* * *

This **EVE Protocol** is designed to be **simple**, **extensible**, and suitable for controlling **GPIOs**, **servos**, **Neopixels**, and **ADC** reads—all while allowing advanced parameters like PWM/servo **frequency**. It strikes a good balance between **human readability** and **machine parseability**, making it ideal for complex robotic projects like your 3D-printed Wall-E.
