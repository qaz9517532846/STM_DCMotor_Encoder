# STM_DCMotor_Encoder
How to control DC motor and AB Encoder using NUCLEO-H743ZI2 board.

- Hardware:  NUCLEO-H743ZI2 board, [DC motor with AB Encoder](https://www.ruten.com.tw/item/show?21832476641790), Transistor(NPN 2N3904L x 4).

- Software: [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) and [Keil uVision5](https://www2.keil.com/mdk5/uvision/).

------

### STM32 Setting Parameter

| Item                     | Paramter    |
| ---                      | ---         |
| System Clock             | 72 MHz      |
| Prescaler                | 72 - 1      |
| Counter Period           | 100 - 1.    |

------

### DC Motor and AB Encoder

![image](https://github.com/qaz9517532846/STM_DCMotor_Encoder/blob/main/circuit/DCMotor.png)

------

### Circuit

![image](https://github.com/qaz9517532846/STM_DCMotor_Encoder/blob/main/circuit/circuit.png)

------

### PWM Control and AB Encoder process

![image](https://github.com/qaz9517532846/STM_DCMotor_Encoder/blob/main/circuit/PWM_ENCODER.png)

------

### Send data using serial communication.

- Encoder counter

| Address        | Address 0                  | Address 1                  |
| ---            | ---                        | ---                        |
| data type      | UINT8                      | UINT8                      |
| data           | encoder cnt hightbyte      | encoder cnt lowbyte        |

------

### Received data using serial communication.

- Duty

| Address        | Address 0                  | Address 1                  | Address 2|
| ---            | ---                        | ---                        | ---                           |
| data type      | UINT8                      | UINT8                      | UINT8                         |
| data           | direction(0 or 1)          | duty hightbyte (0 - 100)   | duty lowbyte (0 - 99) * 0.01  |

------

### Reference

[1]. PWM (Pulse Width Mod) in STM32. https://controllerstech.com/pwm-in-stm32/

[2]. Incremental ENCODER and STM32. https://controllerstech.com/incremental-encoder-with-stm32/

[3]. STM32H743ZI MCU. https://www.st.com/en/evaluation-tools/nucleo-h743zi.html

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2021 ZM Robotics Software Laboratory.

