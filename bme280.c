/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== i2ctmp116.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>
/* Example/Board Header files */
#include "Board.h"


#define TASKSTACKSIZE       640

/*
 *  ======== TMP Registers ========
 */
#define TMP006_REG          0x0001  /* Die Temp Result Register for TMP006 */
#define TMP116_REG          0x0000  /* Die Temp Result Register for TMP116 */
#define BME280_REG          0x0000  /* Die Temp Result Register for TMP006 */

#define TMP006_ADDR         0x41;
#define BME280_ADDR         0x77;
#define TMP116_BP_ADDR      0x48;
#define TMP116_LP_ADDR      0x49;

static Display_Handle display;
//THis value read from Boost Sensor Board:
// Use it if need to verify
uint32_t bme280_compensate_H_int32(int32_t adc_H);
int32_t t_fine;
uint8_t dig_H1; //0
int16_t dig_H2; //379
uint8_t dig_H3; //0
int16_t dig_H4;  //281
int16_t dig_H5; //50
int8_t  dig_H6; //30
/*
 *  ======== mainThread ========
 */
//I2C_Transaction i2cTransaction;
void *mainThread(void *arg0)
{
    uint16_t        sample;
    //uint16_t        temperature;
    int32_t        hum;
    uint16_t       adc_H;
    uint8_t         id;
    //uint8_t         pow;
    uint8_t         txBuffer[2];  //BME280 require ADDR Register>> Command Register
    uint8_t         rxBuffer[8];
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;

    /* Call driver init functions */
    Display_init();
    GPIO_init();
    I2C_init();

    /* Configure the LED and if applicable, the TMP116_EN pin */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
#ifdef Board_GPIO_TMP116_EN
    GPIO_setConfig(Board_GPIO_TMP116_EN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
    /* 1.5 ms reset time for the TMP116 */
    sleep(1);
#endif
//#ifdef Board_GPIO_BMP3001_EN
//    GPIO_setConfig(Board_GPIO_BMP3001_EN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
    /* 1.5 ms reset time for the TMP116 */
//    sleep(1);
//#endif

    /* Open the HOST display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        while (1);
    }

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
    Display_printf(display, 0, 0, "Starting the i2ctmp example.");

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
        Display_printf(display, 0, 0, "Error Initializing I2C\n");
        while (1);
    }
    else {
        Display_printf(display, 0, 0, "I2C Initialized!\n");
    }

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf    = rxBuffer;
    i2cTransaction.readCount  = 1;

    /*
     * Determine which I2C sensor is present.
     * We will prefer sensors in this order: TMP116 (on BoosterPacks),
     * TMP116 (on-board CC32XX LaunchPads), and last TMP006
     * (on older CC32XX LaunchPads).
     */
    /* Try TMP116 values */

    i2cTransaction.slaveAddress = BME280_ADDR;

    //i2cTransaction.slaveAddress = 0x47;
    //READ ID device
    txBuffer[0] = 0xD0;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        //light = (rxBuffer[0] << 8) | (rxBuffer[1]);
        id = rxBuffer[0];
        Display_printf(display, 0, 0, "Device ID: %d (C)",
                        id);

    }

    //READ Config Reg
     txBuffer[0] = 0xF5;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        //light = (rxBuffer[0] << 8) | (rxBuffer[1]);
        id = rxBuffer[0];
        Display_printf(display, 0, 0, "Measure Before Configuration : %d (C)",
        id);
                }
    // Configuration for measuring
    i2cTransaction.writeCount = 2;
    txBuffer[0] = 0xF5;
    //  |b7    |b6     |b5      |b4     |b3     |b2     |b1     |b0     |
    //  | time to sleep         |filter                 |not    |'1':SPI 3,'0':SPI 4|
    txBuffer[1] = 0x2C; //0x2Ct_sb =  62.5ms,filter = 011  see p.30 of data sheet
    //txBuffer[2] = 0x10;
    if (I2C_transfer(i2c, &i2cTransaction))
                    {
                        //light = (rxBuffer[0] << 8) | (rxBuffer[1]);
                        id = rxBuffer[0];
                        Display_printf(display, 0, 0, "Measure After Configuration : %d (C)",
                                        id);
                    }
    // Configuration hum measure:
        i2cTransaction.writeCount = 2;
        txBuffer[0] = 0xF2;
        txBuffer[1] = 0x03; //00: skip, 01: oversamplingx1, 10: oversamplingx2
            //txBuffer[2] = 0x10;
            if (I2C_transfer(i2c, &i2cTransaction))
                            {
                                //light = (rxBuffer[0] << 8) | (rxBuffer[1]);
                id = rxBuffer[0];
                Display_printf(display, 0, 0, "Hum register After Configuration : %d (C)",
                                                id);
                            }
    // Configuration MODE 00: sleep, 01,10: Force, 11: Normal
    i2cTransaction.writeCount = 2;
    txBuffer[0] = 0xF4;
    txBuffer[1] = 0x03; //t_sb =  10ms,filter = 2  see p.30 of datashhet
        //txBuffer[2] = 0x10;
        if (I2C_transfer(i2c, &i2cTransaction))
                        {
                            //light = (rxBuffer[0] << 8) | (rxBuffer[1]);
            id = rxBuffer[0];
            Display_printf(display, 0, 0, "Mode After Configuration : %d (C)",
                                            id);
                        }

    /* Read Calib coefficient of Humidity*/
    // READ dig_H1
    i2cTransaction.writeCount = 1;
    txBuffer[0] = 0xA1;
    if (I2C_transfer(i2c, &i2cTransaction))
    {

        dig_H1 = rxBuffer[0];
        Display_printf(display, 0, 0, "Coefficient dig_H1 : %u (C)",
                                                dig_H1);
     }
    //READ dig_H2 - digH6
    txBuffer[0] = 0xE1;
    i2cTransaction.readCount  = 7;
    if (I2C_transfer(i2c, &i2cTransaction))
       {

           dig_H2 = ((rxBuffer[1]<<8)|rxBuffer[0]);
           Display_printf(display, 0, 0, "Coefficient dig_H2 : %d (C)",
                          dig_H2);
           dig_H3 = rxBuffer[2];
                      Display_printf(display, 0, 0, "Coefficient dig_H3 : %u (C)",
                          dig_H3);
           dig_H4 = ((rxBuffer[3]<<4)|(0x0F&rxBuffer[4]));
           Display_printf(display, 0, 0, "Coefficient dig_H4 : %d (C)",
                          dig_H4);
           dig_H5 = ((rxBuffer[5]<<4)|((0xF0&rxBuffer[4])>>4));
           Display_printf(display, 0, 0, "Coefficient dig_H5 : %d (C)",
                          dig_H5);
           dig_H6 = rxBuffer[6];
           Display_printf(display, 0, 0, "Coefficient dig_H6 : %d (C)",
                          dig_H6);
        }
    /* Take 20 samples and print them out onto the console */
     //i2cTransaction.writeBuf   = txBuffer;
     i2cTransaction.writeCount = 1;
     //i2cTransaction.readBuf    = rxBuffer;
     i2cTransaction.readCount  = 2;

    txBuffer[0] = 0xFD;
    for (sample = 0; sample < 20; sample++) {
        if (I2C_transfer(i2c, &i2cTransaction)) {
            /*
             * Extract degrees C from the received data;
             * see TMP116/006 datasheet
             */
            //temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
            //temperature *= 0.0078125;

            /*
             * If the MSB is set '1', then we have a 2's complement
             * negative value which needs to be sign extended
             */
            //if (rxBuffer[0] & 0x80) {
            //    temperature |= 0xF000;

            //display ambient level light
            /*
            pow = (rxBuffer[0] >> 4) ;
            light = ((rxBuffer[0]&0x0F) << 8) | (rxBuffer[1]);
            while(pow!=0)
            {
                pow -=1;        // Multiply 2^n by left shifting to n times
                light <<=1;
            }
            light *=0.01;
            */
            adc_H = ((rxBuffer[0] << 8) | (rxBuffer[1]));
            hum = bme280_compensate_H_int32((int32_t)adc_H);
            hum >>=10;
            Display_printf(display, 0, 0, "Sample %u: %d (%c)",
                sample, hum,'%'); //temperature
        }
        else {
            Display_printf(display, 0, 0, "I2C Bus fault.");
        }

        /* Sleep for 1 second */
        sleep(1);
    }

    I2C_close(i2c);
    Display_printf(display, 0, 0, "I2C closed!");

    return (NULL);
}
uint32_t bme280_compensate_H_int32(int32_t adc_H)
{
    int32_t v_x1_u32r;

    v_x1_u32r = t_fine - ((int32_t)76800);
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) *
            v_x1_u32r))+ ((int32_t)16384)) >> 15) * (((((((v_x1_u32r*
            ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) +
            ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) +
                    8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
            ((int32_t)dig_H1)) >> 4 ));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r>>12);
}
