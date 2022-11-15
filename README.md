In this program, we implemented _Snake game_ using Arm Keil MDK and STM32Cube MX in STM32 ARM microcontroller. The direction of snake movement can be changed in 4 main directions by using 4 keys from the keypad module or WASD keys through UART. The speed of the snake can be changed using the volume module.

Assumptions:
- LCD is considered as a 20x8 horizontal screen.
- At the beginning of the game, the snake with a length of 3 is located in the upper left corner and starts moving at a speed of 5.0 squares per second.
- The maximum allowed speed of the snake is up to 3 squares per second.
- At any time, one of the empty squares on the screen is randomly selected as an apple.

Instructions:
- By eating apples, the length and speed of the snake increases.
- The direction of the apple relative to the snake's head is displayed by turning on one of the 8 LEDs on the board.
- When the snake hits the specified edges of the screen or itself, the game ends and the score obtained based on the final length of the snake and its average speed is displayed on the LCD.
- At any moment, the speed of the snake is shown on the left two digits and the length of the snake, on the right two digits of 7-segment. They are separated from each other by the _dot_.

The videos of the program's performance and image of the board are included in the folder.
