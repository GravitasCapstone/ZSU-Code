/*******************************************************************************
  MPLAB Harmony Project Main Source File

  Company:
    Microchip Technology Inc.
  
  File Name:
    main.c

  Summary:
    This file contains the "main" function for an MPLAB Harmony project.

  Description:
    This file contains the "main" function for an MPLAB Harmony project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state 
    machines of all MPLAB Harmony modules in the system and it calls the 
    "SYS_Tasks" function from within a system-wide "super" loop to maintain 
    their correct operation. These two functions are implemented in 
    configuration-specific files (usually "system_init.c" and "system_tasks.c")
    in a configuration-specific folder under the "src/system_config" folder 
    within this project's top-level folder.  An MPLAB Harmony project may have
    more than one configuration, each contained within it's own folder under
    the "system_config" folder.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

//Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "system/common/sys_module.h"   // SYS function prototypes
#include "driver/usart/drv_usart.h"


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

#define SYS_FREQ 200000000
#define HDC_ADDRESS 0x40            // The address of HDC2080 when the ADDR pin is connected to ground
#define TEMP_LOW 0x00
#define TEMP_HIGH 0x01
#define HUMID_LOW 0x02
#define HUMID_HIGH 0x03 
#define HDC_Meas_Config_Reg 0x0F
#define HDC_Config_Reg 0x0E
void delay_us(unsigned int us);

void delay_ms(unsigned int ms);

void I2C_wait_for_idle(void);
// I2C_start() sends a start condition  

void I2C_start(void);

// I2C_stop() sends a stop condition  
void I2C_stop(void);

// I2C_restart() sends a repeated start/restart condition
void I2C_restart(void);

// I2C_ack() sends an ACK condition
void I2C_ack(void);

// I2C_nack() sends a NACK condition
void I2C_nack(void); // Acknowledge Data bit

// address is I2C slave address, set wait_ack to 1 to wait for ACK bit or anything else to skip ACK checking  
void I2C_write(unsigned char address, char wait_ack);

// value is the value of the data we want to send, set ack_nack to 0 to send an ACK or anything else to send a NACK  
void I2C_read(unsigned char *value, char ack_nack);

// Write byte value to register at reg_address
void HDC2080_write(unsigned char reg_address, unsigned char value);

// Read a byte from register at reg_address and return in *value
void HDC2080_read(unsigned char reg_address, unsigned char *value);

// I2C_init() initialises I2C1 at at frequency of [frequency]Hz
void I2C_init(double frequency);

void HDC_init(void);

void HDC_rate(void);

void HDC_set_meas_mode(void);

void HDC_resolution(void);

void HDC_trigger(void);

int main ( void )
{
    /* Initialize all MPLAB Harmony modules, including application(s). */
    SYS_Initialize ( NULL );

    DRV_HANDLE usart;
    
    // Initialise I2C1 at 100kHz
    I2C_init(100000);
    HDC_reset();
    delay_ms(250);
    HDC_set_meas_mode();
    HDC_rate();
    HDC_resolution();
    HDC_trigger();
    BSP_LEDStateSet(U1_EN, BSP_LED_STATE_OFF);
    BSP_LEDStateSet(USER_LED2, BSP_LED_STATE_ON);
    
    while ( true )
    {
        static uint8_t temp_bytes[2];
        static uint8_t humid_bytes[2];
        static char input;
    
        usart = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING);
        

    
        while (true)
        {
            if (!DRV_USART_ReceiverBufferIsEmpty(usart)) {
                    input = DRV_USART_ReadByte(usart);
                    if (input == 'r')
                    {
                        BSP_LEDStateSet(U1_EN, BSP_LED_STATE_ON);
                        BSP_LEDStateSet(USER_LED1, BSP_LED_STATE_ON);
                        
                        HDC2080_read(TEMP_LOW, &temp_bytes[0]);
                        HDC2080_read(TEMP_HIGH, &temp_bytes[1]);
                        HDC2080_read(HUMID_LOW, &humid_bytes[0]);
                        HDC2080_read(HUMID_HIGH, &humid_bytes[1]);
                        DRV_USART_WriteByte(usart, temp_bytes[0]);
                        
                        delay_ms(250);
                        
                        BSP_LEDStateSet(USER_LED1, BSP_LED_STATE_OFF);
                        
                        delay_ms(250);

                        BSP_LEDStateSet(USER_LED1, BSP_LED_STATE_ON);
                        DRV_USART_WriteByte(usart, temp_bytes[1]);
                        
                        delay_ms(250);
                        
                        BSP_LEDStateSet(USER_LED1, BSP_LED_STATE_OFF);
                        
                        delay_ms(250);

                        BSP_LEDStateSet(USER_LED1, BSP_LED_STATE_ON);
                        DRV_USART_WriteByte(usart, humid_bytes[0]);
                        
                        delay_ms(250);
                        
                        BSP_LEDStateSet(USER_LED1, BSP_LED_STATE_OFF);
                        
                        delay_ms(250);
                        
                        BSP_LEDStateSet(USER_LED1, BSP_LED_STATE_ON);
                        DRV_USART_WriteByte(usart, humid_bytes[1]);
                        
                        delay_ms(250);
                        
                        BSP_LEDStateSet(USER_LED1, BSP_LED_STATE_OFF);
                        
                        delay_ms(250);
                        
                        DRV_USART_WriteByte(usart, 's'); 
                        BSP_LEDStateSet(U1_EN, BSP_LED_STATE_OFF);
                    }else if(input == 'c'){
                        BSP_LEDStateSet(RGB_BLUE, BSP_LED_STATE_ON);
                        BSP_LEDStateSet(RGB_GREEN, BSP_LED_STATE_OFF);
                        BSP_LEDStateSet(RGB_RED, BSP_LED_STATE_OFF);
                    }else if(input == 'h'){
                        BSP_LEDStateSet(RGB_BLUE, BSP_LED_STATE_OFF);
                        BSP_LEDStateSet(RGB_GREEN, BSP_LED_STATE_OFF);
                        BSP_LEDStateSet(RGB_RED, BSP_LED_STATE_ON);
                    }else if(input == 's'){
                        BSP_LEDStateSet(RGB_BLUE, BSP_LED_STATE_OFF);
                        BSP_LEDStateSet(RGB_GREEN, BSP_LED_STATE_ON);
                        BSP_LEDStateSet(RGB_RED, BSP_LED_STATE_OFF);
                    }
                    
            }
            delay_ms(1000);
        }
}

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}

void delay_us(unsigned int us)
{
    
    // Convert microseconds us into how many clock ticks it will take
    us *= SYS_FREQ / 1000000 / 2; // Core Timer updates every 2 ticks

    _CP0_SET_COUNT(0); // Set Core Timer count to 0

    while (us > _CP0_GET_COUNT()); // Wait until Core Timer count reaches the number we calculated earlier
}

void delay_ms(unsigned int ms)
{
    delay_us(ms * 1000);
}

void I2C_wait_for_idle(void)
{
    while(I2C2CON & 0x1F); // Acknowledge sequence not in progress
                                // Receive sequence not in progress
                                // Stop condition not in progress
                                // Repeated Start condition not in progress
                                // Start condition not in progress
    while(I2C2STATbits.TRSTAT); // Bit = 0 ? Master transmit is not in progress
}

// I2C_start() sends a start condition  
void I2C_start()
{
    I2C_wait_for_idle();
    I2C2CONbits.SEN = 1;
    while (I2C2CONbits.SEN == 1);
}

// I2C_stop() sends a stop condition  
void I2C_stop()
{
    I2C_wait_for_idle();
    I2C2CONbits.PEN = 1;
}

// I2C_restart() sends a repeated start/restart condition
void I2C_restart()
{
    I2C_wait_for_idle();
    I2C2CONbits.RSEN = 1;
    while (I2C2CONbits.RSEN == 1);
}

// I2C_ack() sends an ACK condition
void I2C_ack(void)
{
    I2C_wait_for_idle();
    I2C2CONbits.ACKDT = 0; // Set hardware to send ACK bit
    I2C2CONbits.ACKEN = 1; // Send ACK bit, will be automatically cleared by hardware when sent  
    while(I2C2CONbits.ACKEN); // Wait until ACKEN bit is cleared, meaning ACK bit has been sent
}

// I2C_nack() sends a NACK condition
void I2C_nack(void) // Acknowledge Data bit
{
    I2C_wait_for_idle();
    I2C2CONbits.ACKDT = 1; // Set hardware to send NACK bit
    I2C2CONbits.ACKEN = 1; // Send NACK bit, will be automatically cleared by hardware when sent  
    while(I2C2CONbits.ACKEN); // Wait until ACKEN bit is cleared, meaning NACK bit has been sent
}

// address is I2C slave address, set wait_ack to 1 to wait for ACK bit or anything else to skip ACK checking  
void I2C_write(unsigned char address, char wait_ack)
{
    I2C2TRN = address | 0;				// Send slave address with Read/Write bit cleared
//    while (I2C2STATbits.TBF == 1);		// Wait until transmit buffer is empty
    I2C_wait_for_idle();				// Wait until I2C bus is idle
    if (wait_ack) while (I2C2STATbits.ACKSTAT == 1); // Wait until ACK is received  
}

// value is the value of the data we want to send, set ack_nack to 0 to send an ACK or anything else to send a NACK  
void I2C_read(unsigned char *value, char ack_nack)
{
    I2C2CONbits.RCEN = 1;				// Receive enable
    while (I2C2CONbits.RCEN);			// Wait until RCEN is cleared (automatic)  
    while (!I2C2STATbits.RBF);    		// Wait until Receive Buffer is Full (RBF flag)  
    *value = I2C2RCV;    				// Retrieve value from I2C1RCV
    //return value;
    
    if (!ack_nack)						// Do we need to send an ACK or a NACK?  
        I2C_ack();						// Send ACK  
    else
        I2C_nack();						// Send NACK  
}

// Write byte value to register at reg_address
void HDC2080_write(unsigned char reg_address, unsigned char value)
{
    I2C_start();						/* Send start condition */  
    I2C_write(HDC_ADDRESS << 1, 1); /* Send HDC2080's address, read/write bit not set (AD + R) */  
    I2C_write(reg_address, 1);			/* Send the register address (RA) */  
    I2C_write(value, 1);				/* Send the value to set it to */  
    I2C_stop();    						/* Send stop condition */  
}

// Read a byte from register at reg_address and return in *value
void HDC2080_read(unsigned char reg_address, unsigned char *value)
{
    I2C_start();						/* Send start condition */  
    I2C_write(HDC_ADDRESS << 1, 1);	/* Send HDC2080's address, read/write bit not set (AD + R) */  
    I2C_write(reg_address, 1);			/* Send the register address (RA) */  
    I2C_restart();						/* Send repeated start condition */  
    I2C_write(HDC_ADDRESS << 1 | 1, 1);	/* Send HDC2080's address, read/write bit set (AD + W) */  
    I2C_read(value, 1);					/* Read value from the I2C bus */  
    //return value;
    I2C_stop();    						/* Send stop condition */  
}

// I2C_init() initialises I2C1 at at frequency of [frequency]Hz
void I2C_init(double frequency)
{
    double BRG;
    
    I2C2CON = 0;			// Turn off I2C1 module
    I2C2CONbits.DISSLW = 1; // Disable slew rate for 100kHz
    
    BRG = (1 / (2 * frequency)) - 0.000000104;
    BRG *= (SYS_FREQ / 2) - 2;    
    
    I2C2BRG = (int)BRG;		// Set baud rate
    I2C2CONbits.ON = 1;		// Turn on I2C1 module
}
void HDC_reset(void)
{
    static unsigned char config_val;
    static unsigned char reset;
    
    //Reset
    HDC2080_read(HDC_Config_Reg, &config_val);
    reset = config_val | 0x80;
    HDC2080_write(HDC_Config_Reg, reset);
}

void HDC_rate(void)
{
    static unsigned char config_val;
    HDC2080_read(HDC_Config_Reg, &config_val);
    config_val = config_val & 0xDF;
    config_val = config_val | 0x50;
    HDC2080_write(HDC_Config_Reg, config_val);
}

void HDC_set_meas_mode(void)
{
    static unsigned char config_val;
    HDC2080_read(HDC_Meas_Config_Reg, &config_val);
    
//    switch(mode)
//    {
//        case 1: //TEMP and HUMID 
//            config_val = config_val & 0xF9;
//            break;
//            
//        case 2: //TEMP only
//            config_val = config_val & 0xFC;
//            config_val = config_val | 0x02;
//            break;
//            
//        case 3: //HUMID only
//            config_val = config_val & 0xFD;
//            config_val = config_val | 0x04;
//            break;
//        default:
//            config_val = config_val & 0xF9;
//    }
    
    config_val = config_val & 0xF9;
    
    HDC2080_write(HDC_Meas_Config_Reg, config_val);
}

void HDC_resolution(void)
{
    static unsigned char config_val;
    HDC2080_read(HDC_Meas_Config_Reg, &config_val);
    config_val = config_val & 0x3F;
    HDC2080_write(HDC_Meas_Config_Reg, config_val);
}

void HDC_trigger(void)
{
    static unsigned char config_val;
    HDC2080_read(HDC_Meas_Config_Reg, &config_val);
    config_val = config_val | 0x01;
    HDC2080_write(HDC_Meas_Config_Reg, config_val);
}

/*******************************************************************************
 End of File
*/