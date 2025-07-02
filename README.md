# EMBEDDED SOFTWARE FOR THE INTERNET OF THINGS 

| NAME               | MATRICOLA | MAIL                                 |
|--------------------|-----------|--------------------------------------|
| Andrea Costantino  | 123456    | evelin.begher@studenti.unitn.it      |
| Nicolo' Belle'     | 123456    | evelin.begher@studenti.unitn.it      |
| Riccardo Bassan    | 123456    | evelin.begher@studenti.unitn.it      |
| Evelin Begher      | 235188    | evelin.begher@studenti.unitn.it      |

## Indice
- [project overview](#project-overview)
- [documentation](#documentation)
  - [libraries](#libraries)
  - [function](#function)
- [project description](#project-description)
- [system and peripherals initialization](#system-and-peripherals-initialization)
- [dynamic day and night mode](#dynamic-day-and-night-mode)
- [user interface](#user-interface)
  - [pin](#pin)
  - [keypad](#keypad)

## PROJECT OVERVIEW
<br>
This project implements a <b>SECURE KEYLESS DOOR UNLOCKING SYSTEM</b> based on a <b>PIN code</b>, designed for IoT applications in embedded environments. The user can enter a secret code using a <b>virtual numeric keypad</b> displayed on screen, navigable via an analog joystick. Access is granted only if the entered PIN is correct, mimicking the behavior of an electronic lock. After three consecutive incorrect attempts, the system locks for security reasons.  <br><br>
To enhance user experience and environmental adaptability, the device also includes a <b>light sensor</b> that enables automatic switching between day and night modes by changing the graphical interface colors. All interaction is managed through a color display connected via SPI and updated dynamically. <br><br>
This project serves as a complete demonstration of human-machine interaction (HMI), analog input handling, embedded security, and graphical visualization on a microcontroller. It is entirely developed for the MSP432P401R platform.


## DOCUMENTATION 
* ### LIBRARIES
<br>

```c
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/grlib/grlib.h>
#include "HAL_OPT3001.h"
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <string.h>
```

<br>
* ### FUNCTION
## PROJECT DESCRIPTION
* ### SYSTEM AND PERIPHERALS INITIALIZATION
* ### DYNAMIC DAY AND NIGHT MODE
* ### USER INTERFACE
  * #### PIN
  * #### KEYPAD

