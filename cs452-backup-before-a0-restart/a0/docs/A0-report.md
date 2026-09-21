# CS452/652 Spring 2026 Assignment 0 Report

## 1. Commit Hash

To be filled before submission.

## 2. Program Structure

My program is structured as a polling loop. Each iteration checks the system timer, terminal input, CAN bus messages, sensor updates, and display updates.

Main components:

- Clock module
- Command parser
- Display module
- Train control module
- Sensor module
- Timing measurement code

The source code is organized in the `a0/src/` directory.

## 3. Algorithms and Data Structures

The program uses a polling loop instead of interrupts. This design allows the program to repeatedly check independent real-time activities, including the clock, user input, CAN bus messages, and display updates.

Data structures used:

- Fixed-size arrays for train speeds and switch positions.
- A fixed-size recent-sensor list for displaying the most recently triggered sensors.
- A command parser that identifies the command type and dispatches the corresponding train or switch operation.

## 4. Unimplemented Parts and Known Bugs

To be updated before final submission.

## 5. Clock Correctness

To be filled after testing.

The intended design is to read the Raspberry Pi system timer on each polling-loop iteration and update the displayed time based on elapsed time. The clock should not depend on artificial delays, because delays would slow down command handling and CAN bus polling.

## 6. Train Hardware Response Time

To be filled after testing.

The final submission should report measured response times for train speed commands, reverse commands, switch commands, and sensor events.

## 7. Build Test

The program should build using the following sequence:

```bash
git clone https://git.uwaterloo.ca/y68wu/cs452.git
cd cs452
git checkout <commit hash>
make
```

## 8. Sensor Timing Measurement Code

The timing measurement code is located in:

```text
a0/measurements/sensor_timing.c
```

Timing notes are located in:

```text
a0/measurements/timing_notes.md
```
