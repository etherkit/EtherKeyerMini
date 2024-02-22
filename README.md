# EtherKeyer Mini

A simple keyer that does the basic functions that you need.

![](PCB/EtherKeyerMini.png)

## Functions

### Pushbuttons
#### Short Press
Cancel any message playback by pressing any button or paddle.
- 1 - Play Message Memory 1
- 2 - Play Message Memory 2
- 3 - Play Message Memory 3

#### Long Press
- 1 - Tune Mode (press any key to exit)
- 3 - Enter/Exit UART Mode


### UART Mode
The format for interacting with EtherKeyer Mini via the UART is very simple. In order to place EtherKeyer into UART mode, press and hold button 3 for at least one second. If your serial terminal is open when you do this, you'll get a greeting from EtherKeyer Mini to let you know it is ready for commands.

Each message memory has a maximum size of 40 characters.

#### Serial Terminal Parameters
19200 baud, send New Line only

#### Query
- W? - Keyer speed
- 1? - Message memory 1
- 2? - Message memory 2
- 3? - Message memory 3

#### Parameter Set
- 1:\<message> - Message memory 1
- 2:\<message> - Message memory 2
- 3:\<message> - Message memory 3
- X: - Exit UART Mode
- R: - Reverse Paddle Terminals
- N: - Normal Paddle Terminals