# OV7670 RGB565 Driver for Arduino Mega 2560

⚠️⚠️⚠️ Work in progress, expect changes

Current project files:

```text
MEGA_code.ino  # Arduino Mega 2560 camera driver
receiver.py    # Python image receiver and saver
```

An Arduino Mega 2560 driver and host-side Python toolkit for capturing images from a **bare OV7670 camera module without FIFO**.

The driver:

- generates the camera XCLK signal;
- configures the OV7670 over its two-wire SCCB interface;
- captures the 8-bit parallel camera bus using direct AVR port access;
- synchronises frames and rows using VSYNC, HREF and PCLK;
- streams raw RGB565 pixels to a computer over UART;
- supports still-image capture and a slow auto-refreshing preview.

> [!IMPORTANT]
> The OV7670 uses 3.3 V power and logic. Do not connect 5 V directly to the camera's power, XCLK or SCCB pins.

## Hardware

- Arduino Mega 2560
- Bare OV7670 camera module without FIFO
- Regulated 3.3 V supply
- 4.7 kΩ pull-up resistor from SDA to 3.3 V
- 4.7 kΩ pull-up resistor from SCL to 3.3 V
- 5 V-to-3.3 V divider or suitable level shifter for XCLK
- Short jumper wires
- Recommended: 100 nF decoupling capacitor close to the camera module

A 3.3 V-compatible buffer such as a 74HCT245 or 74AHCT125 can improve signal reliability on the camera-to-Mega data and timing lines.

## Wiring

| OV7670 | Arduino Mega 2560 |
|---|---|
| D0 | 22 / PA0 |
| D1 | 23 / PA1 |
| D2 | 24 / PA2 |
| D3 | 25 / PA3 |
| D4 | 26 / PA4 |
| D5 | 27 / PA5 |
| D6 | 28 / PA6 |
| D7 | 29 / PA7 |
| VSYNC | 2 / PE4 |
| HREF | 8 / PH5 |
| PCLK | 12 / PB6 |
| XCLK | 10 / OC2A through a 5 V-to-3.3 V divider |
| SIOD | 20 / SDA, with 4.7 kΩ pull-up to 3.3 V |
| SIOC | 21 / SCL, with 4.7 kΩ pull-up to 3.3 V |
| RESET | 3.3 V |
| PWDN | GND |
| VCC | 3.3 V |
| GND | Mega GND |

## Recommended QVGA setup

Upload:

```text
MEGA_code.ino
```

Then use the Python program:

```text
receiver.py
```

The current working RGB565 interpretation is **C byte swapped**:

```python
pixel = (second << 8) | first
```

No red/blue channel swap is applied.

## Python installation

Python 3.10 or newer is recommended.

Install the dependencies:

```bash
python3 -m pip install pyserial Pillow
```

Requires packages are:

```text
pyserial
Pillow
```

Change the serial port near the top of the Python program:

```python
PORT = "/dev/cu.usbmodem112201"
```

Typical alternatives include:

```text
/dev/cu.usbmodem...
/dev/ttyACM0
COM3
```

P.S: The Arduino IDE shows the port that the board is connected to.

Only one application may use the serial port at a time. Close the Arduino Serial Monitor before running the Python script.

## Capture a QVGA still image

Upload to Arduino Board:

```text
MEGA_code.ino
```

Then run:

```bash
python3 receiver.py
```

The matching settings are:

```text
Resolution: 640 × 480
UART: 2,000,000 baud
XCLK: 4 MHz
Frame size: 614,400 bytes
```

VGA is experimental on the Mega 2560. The camera produces data continuously, while the Mega has only 8 KB of SRAM and cannot store a complete frame. The driver therefore captures and transmits each row at the same time. Timing or electrical noise can cause the transfer to end before all 480 rows are received.

For reliable full-resolution capture, an OV7670 module with FIFO or a faster microcontroller is a better platform.

## Serial protocol

The Arduino and Python programs use a small custom UART protocol:

```text
Mega   -> *INIT*
Mega   -> *RDY*
Host   -> G
Mega   -> *IMG*
Mega   -> raw RGB565 frame bytes
```

Frame sizes:

```text
QVGA: 153,600 bytes
VGA:   614,400 bytes
```

The marker strings are used only before the raw frame. No marker is inserted into the pixel data.

## SCCB and image data

SCCB is used only to configure camera registers:

```text
SIOC -> serial control clock
SIOD -> serial control data
```

The actual image is transferred through the parallel camera interface:

```text
D0-D7 -> one camera byte
PCLK  -> byte timing
HREF  -> active row
VSYNC -> frame timing
```

The image is then forwarded from the Mega to Python through UART.

## Exposure

The driver can disable automatic exposure and use fixed values:

```cpp
static constexpr bool USE_MANUAL_EXPOSURE = true;
static constexpr uint16_t MANUAL_EXPOSURE = 12;
static constexpr uint8_t MANUAL_GAIN = 0x00;
```

Typical tuning direction:

```text
lower exposure value -> darker image
higher exposure value -> brighter image
```

Change exposure only after frame timing and byte alignment are stable.

## Troubleshooting

### The built-in LED flashes continuously

The driver entered `fatalError()`. Common causes are:

- XCLK is missing or connected incorrectly;
- SCCB wiring is incorrect;
- SDA or SCL lacks a pull-up to 3.3 V;
- the camera is not powered from 3.3 V;
- the grounds are not connected;
- the OV7670 PID or version register could not be read.

### The image has purple noise or complementary colour sections

This usually indicates a one-byte RGB565 phase error. Confirm that the Arduino is correctly connected to HREF on the camera.


### The image has vertical breaks or random coloured pixels

Likely causes include:

- long jumper wires;
- unstable 3.3 V power;
- missing decoupling;
- unreliable 3.3 V HIGH levels at the 5 V Mega inputs;
- missed PCLK edges.

Shorten the wires and consider a 74HCT/74AHCT input buffer.

### VGA stops before 100%

The Mega has fallen behind the VGA stream. Check that both sides use 2 Mbps and that the VGA sketch uses 4 MHz XCLK. The mode may still be unreliable because the no-FIFO camera cannot pause while the Mega transmits data.

## Acknowledgements

This project was inspired by the timing and no-framebuffer approach used by
[LiveOV7670](https://github.com/indrekluuk/LiveOV7670).

The Arduino Mega implementation, serial protocol, capture logic, and Python
receiver were developed separately for this project.

## Licence

This project is available under the [MIT License](LICENSE).

