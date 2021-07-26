# STM32-CAN-Test
by JS106351

## Description of Project

This is a simple project in which two ST Microelectronics **STM32F103C8T6** MCUs communicate with each other via the CANBUS protocol.  Each MCU processes each message by way of *polling*, that is without use of interrupts.  Most other projects I have seen online seem to only use the interrupt method, but I want to utilise polling for certain timing reasons.  The CAN baud rate is configured for 100kbps and no filters or masks are utilised (all message frames are accepted).  One board acts a transmitter and sends messages to the other board which acts as a receiver.    

An LED and a VFD display are used as visual indicators that communicaiton is taking place between the boards.  the VFD is just bitbanged SPI through regular GPIO pins.  A timer interrupt is used on both boards to toggle an on board LED to show that the microcontrollers are running.    

## Hardware/Software Info

Project was developed in STM32CubeIDE ver. 1.5.1 using HAL.  The dev boards used are **"STM32 Blue Pill"** which can be found on eBay for cheap.  The CAN transceiver used is a Texas Instruments **TCAN332** IC.    
This code is provided as is.  




