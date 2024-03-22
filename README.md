# Lock System with Microcontroller

## Project Overview: 
A lock system using a NUCLEO-L452RE-P development board, featuring PIN code entry through a keypad and visual feedback via a 7-segment display. The system uses an LED as a status indicator for PIN validation.
## Key Features:
PIN Entry Interface: 4x3 matrix keypad for PIN input, with each digit displayed on a 7-segment display for visual confirmation.
PIN Validation: Entered PIN is checked against a predefined code; a green LED lights up to indicate correct entry and system unlock.
### Software Development: 
Utilizes STM32CubeIDE and C programming language and STMicroelectronics' HAL for peripheral initialization and control.
### Hardware Components: 
STMicroelectronics NUCLEO-L452RE-P board with STM32L452RE microcontroller, matrix keypad, 7-segment display, and an onboard LED for status indication.
