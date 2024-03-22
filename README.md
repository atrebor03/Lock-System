Lock System with Microcontroller March 2024

• Developed a secure lock system based on a NUCLEO-L452RE-P board, designed to validate entry through a PIN
code input on a keypad, with visual feedback on a 7-segment display and LED indication for access status.
• Implemented using STM32CubeIDE and the C programming language, the project leverages the Hardware
Abstraction Layer (HAL) provided by STMicroelectronics to manage hardware peripherals efficiently.
• Introduced routines for debounce-handled keypad reading, dynamic 7-segment display updates for each pressed key, and
a PIN verification process that compares the entered code against a predefined correct sequence, triggering the status
LED accordingly.

