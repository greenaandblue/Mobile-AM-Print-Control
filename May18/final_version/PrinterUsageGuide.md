#  KUKA + Prusa Printer Usage Guide(PrintRun)

## 1. Overview

This project integrates:

* **KUKA Robot**
* **Prusa 3D Printers**

  * Prusa i3 MK3S+
  * Prusa i4 / 4S
  * other Prusa printers
* **ROS (Robot Operating System)**
* **Python-based control scripts**

---------------------------------------------------

## 2. Setup

### 2.1 Power On

1. Turn on the **KUKA robot**
2. Turn on your laptop

### 2.2 Network Connection

* Connect your laptop to the **KUKA WiFi network**

---------------------------------------------------

## 3. Initialize ROS Environment

Run the following command in the terminal:

```bash
source devel/setup.bash
```
---------------------------------------------------

## 4. Start ROS Master

In a VS Code terminal, run:

```bash
roscore -p 30001
```
---------------------------------------------------

## 5. Run Control Programs

### 5.1 Open a New Terminal

```bash
source devel/setup.bash
cd <your_workspace_path>
```

### 5.2 Launch Python Nodes

#### Step 1: Start the prinetr_node.py
      cd into folder: fianl_versioin
      use gcode_sender_node.py

```bash
python3 gcode_sender_node.py
```

#### Step 2: Start the test script
      cd into folder: fianl_versioin
      use test_gcode_sender.py

```bash
python3 test_gcode_sender.py
```

### 5.3 Start Motion Control (Optional)
      i remember there's something wrong with (gcode_sender_node + motion),
      so maybe use old version (node + motion).
      i also wrote a new version for motion, pls test the file: motion_based_control.py

If robot motion integration is required:


```bash
python3 motion.py
```
---------------------------------------------------

## 6. Input 3D Printing File

After running `test.py`:

* The terminal will prompt you to enter a file path
* Input the path to your **G-code file**

Example:

```bash
/path/to/your/file.gcode
```
---------------------------------------------------

## 7. 3D Model Preparation (PrusaSlicer)

Before printing, 3D models must be processed using **PrusaSlicer**.

### 7.1 Supported File Formats

* `.stl`
* `.obj`
* `.3mf`
* please refer to the official software

### 7.2 Basic Configuration

#### 7.2.1 Select Material

* Example: **PLA** 

#### 7.2.2 Select Printer Model

Choose the correct printer:

* Example: Prusa i3 MK3S+

#### 7.2.3 Configure Parameters

Adjust as needed:

* Layer height
* Infill density
* Temperature
* Print speed


#### 7.2.4 Generate G-code

```text
Slice → Export G-code
```
---------------------------------------------------

## 8. Troubleshooting

### Problem 1 : Printer Connection Failed

* Before running the system, connect the computer to the printer using a cable
* If the script shows a **connection failure**, you need to check the serial port

#### Check Available Ports

Run the following command in terminal:

```bash
ls /dev/ttyACM*
```

You may see output like:

```bash
/dev/ttyACM0
/dev/ttyACM1
```

#### Update Configuration

* Identify which port is being used (e.g., `/dev/ttyACM0`)
* Update the port in:

  * `config` file
  * `node.py`

Example:

```python
port = "/dev/ttyACM0"
```
#### Notes

* The port may change each time you reconnect the USB
* Always verify the port before running the script if connection fails


