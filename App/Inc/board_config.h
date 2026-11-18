#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "main.h"

/* ====================================================================
 * TMC2208 Stepper Driver Pins
 * Active-low Enable: PA0 = HIGH -> Driver OFF, PA0 = LOW -> Driver ON
 * ==================================================================== */
#define BOARD_MOTOR_EN_PORT         GPIOA
#define BOARD_MOTOR_EN_PIN          GPIO_PIN_0
#define BOARD_MOTOR_EN_ACTIVE_LEVEL GPIO_PIN_RESET   /* Low = Enabled */
#define BOARD_MOTOR_EN_DISABLE_LEVEL GPIO_PIN_SET    /* High = Disabled */

/* Motor Trái (Left) */
#define BOARD_MOTOR_L_STEP_PORT     GPIOA
#define BOARD_MOTOR_L_STEP_PIN      GPIO_PIN_1
#define BOARD_MOTOR_L_DIR_PORT      GPIOA
#define BOARD_MOTOR_L_DIR_PIN       GPIO_PIN_2

/* Motor Phải (Right) */
#define BOARD_MOTOR_R_STEP_PORT     GPIOA
#define BOARD_MOTOR_R_STEP_PIN      GPIO_PIN_3
#define BOARD_MOTOR_R_DIR_PORT      GPIOA
#define BOARD_MOTOR_R_DIR_PIN       GPIO_PIN_4

/* ====================================================================
 * MPU6050 6-DoF IMU Pins
 * I2C1: PB6 (SCL), PB7 (SDA)
 * INT: PB3 (EXTI line 3)
 * ==================================================================== */
#define BOARD_MPU_INT_PORT          GPIOB
#define BOARD_MPU_INT_PIN           GPIO_PIN_3
#define BOARD_MPU_INT_IRQN          EXTI3_IRQn

/* ====================================================================
 * NRF24L01+ PA+LNA Wireless Radio Pins
 * SPI2: PB13 (SCK), PB14 (MISO), PB15 (MOSI)
 * CE: PB8, CSN: PB9 (Active Low)
 * IRQ (Optional): PB5
 * ==================================================================== */
#define BOARD_NRF_CE_PORT           GPIOB
#define BOARD_NRF_CE_PIN            GPIO_PIN_8
#define BOARD_NRF_CSN_PORT          GPIOB
#define BOARD_NRF_CSN_PIN           GPIO_PIN_9
#define BOARD_NRF_IRQ_PORT          GPIOB
#define BOARD_NRF_IRQ_PIN           GPIO_PIN_5

/* ====================================================================
 * VL53L0X ToF Distance Sensor Pins
 * I2C2: PB10 (SCL), PB11 (SDA)
 * XSHUT (Optional): PC6
 * ==================================================================== */
#define BOARD_TOF_XSHUT_PORT        GPIOC
#define BOARD_TOF_XSHUT_PIN         GPIO_PIN_6

/* ====================================================================
 * HC-SR05 Ultrasonic Distance Sensors (3 Sensors)
 * Front (Trước): TRIG = PC0, ECHO = PC1
 * Left  (Trái) : TRIG = PC2, ECHO = PC3
 * Right (Phải) : TRIG = PC4, ECHO = PC5
 * NOTE: ECHO must be 3.3V (pass through 5V->3.3V divider)
 * ==================================================================== */
#define BOARD_US_FRONT_TRIG_PORT    GPIOC
#define BOARD_US_FRONT_TRIG_PIN     GPIO_PIN_0
#define BOARD_US_FRONT_ECHO_PORT    GPIOC
#define BOARD_US_FRONT_ECHO_PIN     GPIO_PIN_1

#define BOARD_US_LEFT_TRIG_PORT     GPIOC
#define BOARD_US_LEFT_TRIG_PIN      GPIO_PIN_2
#define BOARD_US_LEFT_ECHO_PORT     GPIOC
#define BOARD_US_LEFT_ECHO_PIN      GPIO_PIN_3

#define BOARD_US_RIGHT_TRIG_PORT    GPIOC
#define BOARD_US_RIGHT_TRIG_PIN     GPIO_PIN_4
#define BOARD_US_RIGHT_ECHO_PORT    GPIOC
#define BOARD_US_RIGHT_ECHO_PIN     GPIO_PIN_5

/* ====================================================================
 * ESP32 Super Mini UART (USART1)
 * PA9: USART1_TX (AF7)
 * PA10: USART1_RX (AF7)
 * ==================================================================== */
#define BOARD_ESP_UART_PORT         GPIOA
#define BOARD_ESP_UART_TX_PIN       GPIO_PIN_9
#define BOARD_ESP_UART_RX_PIN       GPIO_PIN_10
#define BOARD_ESP_UART_BAUDRATE     115200

#endif /* BOARD_CONFIG_H */
