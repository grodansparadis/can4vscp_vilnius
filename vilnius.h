// File:  main.h

/* ******************************************************************************
 * VSCP (Very Simple Control Protocol)
 * http://www.vscp.org
 *
 * Copyright (C) 2015 Ake Hedman, Grodans Paradis AB
 *                          <akhe@grodansparadis.com>
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 
 *	This file is part of VSCP - Very Simple Control Protocol 	
 *	http://www.vscp.org
 *
 * ******************************************************************************
 */

#ifndef VILNIUS_H
#define VILNIUS_H

//Defines
#define	TRUE                    1
#define	FALSE                   0

#define MODULE_VOLTAGE          5.0
#define MODULE_AD_MAX_VALUE     1023    // 10-bit

//#define MODULE_VOLTAGE          3.3
//#define MODULE_AD_MAX_VALUE     4095    // 12-bit

// IO0 - RB0 INT0/AN10 (weak pull up)
// IO1 - RB1 INT1/AN8 (weak pull up)
// IO2 - RC3 SCK/SCL (I2C/SPI clock))
// IO3 - RC4 SDI/SDA (I2C data/SPI data in MISO)
// IO4 - RC5 SDO (SPI data out MOSI)
// IO5 - RC6 TX/Sync. clock
// IO6 - RC7 RX/Sync. data
// IO7 - RA2 AD2
// IO8 - RA1 AN1
// IO9 - RA0 AN0

//
// 8 MHz with PLL => 8 MHz
// 1:4 prescaler => 1 MHz (1 uS cycle )
// 1 ms == 1000 uS
// 65535 - 1000 = 64535 = 0xfc17
//
// Timer2 use 250 and prescaler 1:4
//
//#define TIMER0_RELOAD_VALUE		0xfc17

//
// 10 MHz with PLL => 40 MHz
// 1:4 prescaler => 1.25 MHz ( 0.800 uS cycle )
// 1 ms == 1000 uS
// 65535 - 1250 = 64285 = 0xfb1d
//
// Timer2 use 156 and prescaler 1:8
//
#define TIMER0_RELOAD_VALUE		0xfb1d

//
// Timer 2 is used as a 1 ms clock
// 156 is loaded eight time to give ~1250 cycles
// Timer2 use 156 and prescaler 1:4, Postscaler 1:16
// 100 ns * 56 * 4 * 16 ~ 1 ms
//
#define TIMER2_RELOAD_VALUE		156

// ADCCON0 ADC select bits
#define SELECT_ADC0             (0<<2)  // ADC0
#define SELECT_ADC1             (1<<2)  // ADC1
#define SELECT_ADC2             (2<<2)  // ADC2
#define SELECT_ADC3             (3<<2)  // ADC3

#define STATUS_LED              PORTCbits.RC1
#define INIT_BUTTON             PORTCbits.RC0

// Defines for channels 
#define CHANNEL0                PORTAbits.RA3
#define CHANNEL1                PORTAbits.RA3
#define CHANNEL2                PORTAbits.RA1
#define CHANNEL3                PORTAbits.RA0

#define IO0_INPUT               PORTCbits.RC5
#define IO1_INPUT               PORTCbits.RC4
#define IO0_OUTPUT              LATCbits.LATC5
#define IO1_OUTPUT              LATCbits.LATC4
#define IO0_DIR                 TRISCbits.RC5
#define IO1_DIR                 TRISCbits.RC4

// -----------------------------------------------

// Defaults

#define DEFAULT_VILNIUS_CONTROL_MODULE                  0

#define DEFAULT_VILNIUS_CONTROL_CHANNEL0                0x00
#define DEFAULT_VILNIUS_CONTROL_CHANNEL1                0x00
#define DEFAULT_VILNIUS_CONTROL_CHANNEL2                0x00
#define DEFAULT_VILNIUS_CONTROL_CHANNEL3                0x00

#define DEFAULT_AD0_HYSTERESIS                          200
#define DEFAULT_AD1_HYSTERESIS                          200
#define DEFAULT_AD2_HYSTERESIS                          200
#define DEFAULT_AD3_HYSTERESIS                          200

#define DEFAULT_AD0_REPORT_INTERVAL                     0
#define DEFAULT_AD1_REPORT_INTERVAL                     0
#define DEFAULT_AD2_REPORT_INTERVAL                     0
#define DEFAULT_AD3_REPORT_INTERVAL                     0

#define DEFAULT_MEASUREMENT0_REPORT_INTERVAL_MSB        0
#define DEFAULT_MEASUREMENT0_REPORT_INTERVAL_LSB        0
#define DEFAULT_MEASUREMENT1_REPORT_INTERVAL_MSB        0
#define DEFAULT_MEASUREMENT1_REPORT_INTERVAL_LSB        0
#define DEFAULT_MEASUREMENT2_REPORT_INTERVAL_MSB        0
#define DEFAULT_MEASUREMENT2_REPORT_INTERVAL_LSB        0
#define DEFAULT_MEASUREMENT3_REPORT_INTERVAL_MSB        0
#define DEFAULT_MEASUREMENT3_REPORT_INTERVAL_LSB        0

#define DEFAULT_IO_CONTROL                              0

#define DEFAULT_LINEARIZATION_EVENT_SETTING             0
#define DEFAULT_LINEARIZATION_EVENT_CLASS               10
#define DEFAULT_LINEARIZATION_EVENT_TYPE                16

// -----------------------------------------------

// Alarm bits
#define VSCP_ALARM_LOW                                  0x00000001
#define VSCP_ALARM_HIGH                                 0x00000010

// Bits in A/D channel configuration register
#define CONFIG_AD_LOW_ALARM_ENABLE                      0b00000010
#define CONFIG_AD_HIGH_ALARM_ENABLE                     0b00000100
#define CONFIG_AD_USE_A_INPUT                           0b00001000

// Bits in I/O channel configuration register
#define CONFIG_IO0_DIRECTION                            0b00000001  // 0=output, 1=input
#define CONFIG_IO1_DIRECTION                            0b00000010  // 0=output, 1=input
#define CONFIG_IO0_EVENT_ON_CHANGE                      0b00010000
#define CONFIG_IO1_EVENT_ON_CHANGE                      0b00100000
#define CONFIG_IO0_EVENT_SELECT                         0b01000000
#define CONFIG_IO1_EVENT_SELECT                         0b10000000

// Bits for measurement control register
#define CONFIG_MEASUREMENT_UNIT_POS                     0b00001000
#define MEASUREMENT_CTRL_CLASS_BIT_8                    0b01000000

#define MEASUREMENT_CTRL_UNIT_MASK                      0b00011000

// -----------------------------------------------

// * * *  Registers * * *

// * * * * Page 0 * * * *

#define REG0_VILNIUS_ZONE                               0
#define REG0_VILNIUS_SUBZONE                            1

#define REG0_VILNIUS_CH0_ZONE                           2
#define REG0_VILNIUS_CH0_SUBZONE                        3
#define REG0_VILNIUS_CH1_ZONE                           4
#define REG0_VILNIUS_CH1_SUBZONE                        5
#define REG0_VILNIUS_CH2_ZONE                           6
#define REG0_VILNIUS_CH2_SUBZONE                        7
#define REG0_VILNIUS_CH3_ZONE                           8
#define REG0_VILNIUS_CH3_SUBZONE                        9

#define REG0_VILNIUS_IO0_ZONE                           10
#define REG0_VILNIUS_IO0_SUBZONE                        11

#define REG0_VILNIUS_IO1_ZONE                           12
#define REG0_VILNIUS_IO1_SUBZONE                        13

#define REG0_VILNIUS_CONTROL_MODULE                     14

#define REG0_VILNIUS_CONTROL_CHANNEL0                   15
#define REG0_VILNIUS_CONTROL_CHANNEL1                   16
#define REG0_VILNIUS_CONTROL_CHANNEL2                   17
#define REG0_VILNIUS_CONTROL_CHANNEL3                   18

#define REG0_VILNIUS_AD_VALUE_CHANNEL0_MSB              19
#define REG0_VILNIUS_AD_VALUE_CHANNEL0_LSB              20
#define REG0_VILNIUS_AD_VALUE_CHANNEL1_MSB              21
#define REG0_VILNIUS_AD_VALUE_CHANNEL1_LSB              22
#define REG0_VILNIUS_AD_VALUE_CHANNEL2_MSB              23
#define REG0_VILNIUS_AD_VALUE_CHANNEL2_LSB              24
#define REG0_VILNIUS_AD_VALUE_CHANNEL3_MSB              25
#define REG0_VILNIUS_AD_VALUE_CHANNEL3_LSB              26

#define REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL0_MSB    27
#define REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL0_LSB    28
#define REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL1_MSB    29
#define REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL1_LSB    30
#define REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL2_MSB    31
#define REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL2_LSB    32
#define REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL3_MSB    33
#define REG0_VILNIUS_AD_REPORT_INTERVAL_CHANNEL3_LSB    34

#define REG0_VILNIUS_ALARM_LOW_CHANNEL0_MSB             35
#define REG0_VILNIUS_ALARM_LOW_CHANNEL0_LSB             36
#define REG0_VILNIUS_ALARM_LOW_CHANNEL1_MSB             37
#define REG0_VILNIUS_ALARM_LOW_CHANNEL1_LSB             38
#define REG0_VILNIUS_ALARM_LOW_CHANNEL2_MSB             39
#define REG0_VILNIUS_ALARM_LOW_CHANNEL2_LSB             40
#define REG0_VILNIUS_ALARM_LOW_CHANNEL3_MSB             41
#define REG0_VILNIUS_ALARM_LOW_CHANNEL3_LSB             42

#define REG0_VILNIUS_ALARM_HIGH_CHANNEL0_MSB            43
#define REG0_VILNIUS_ALARM_HIGH_CHANNEL0_LSB            44
#define REG0_VILNIUS_ALARM_HIGH_CHANNEL1_MSB            45
#define REG0_VILNIUS_ALARM_HIGH_CHANNEL1_LSB            46
#define REG0_VILNIUS_ALARM_HIGH_CHANNEL2_MSB            47
#define REG0_VILNIUS_ALARM_HIGH_CHANNEL2_LSB            48
#define REG0_VILNIUS_ALARM_HIGH_CHANNEL3_MSB            49
#define REG0_VILNIUS_ALARM_HIGH_CHANNEL3_LSB            50

#define REG0_VILNIUS_HYSTERESIS_CH0_MSB                 51
#define REG0_VILNIUS_HYSTERESIS_CH1_MSB                 52
#define REG0_VILNIUS_HYSTERESIS_CH2_MSB                 53
#define REG0_VILNIUS_HYSTERESIS_CH3_MSB                 54

#define REG0_VILNIUS_MEASUREMENT_REPORT_INTERVAL_CH0_MSB    55
#define REG0_VILNIUS_MEASUREMENT_REPORT_INTERVAL_CH0_LSB    56
#define REG0_VILNIUS_MEASUREMENT_REPORT_INTERVAL_CH1_MSB    57
#define REG0_VILNIUS_MEASUREMENT_REPORT_INTERVAL_CH1_LSB    58
#define REG0_VILNIUS_MEASUREMENT_REPORT_INTERVAL_CH2_MSB    59
#define REG0_VILNIUS_MEASUREMENT_REPORT_INTERVAL_CH2_LSB    60
#define REG0_VILNIUS_MEASUREMENT_REPORT_INTERVAL_CH3_MSB    61
#define REG0_VILNIUS_MEASUREMENT_REPORT_INTERVAL_CH3_LSB    62

#define REG0_VILNIUS_ABSOLUT_LOW_CHANNEL0_MSB           63
#define REG0_VILNIUS_ABSOLUT_LOW_CHANNEL0_LSB           64
#define REG0_VILNIUS_ABSOLUT_LOW_CHANNEL1_MSB           65
#define REG0_VILNIUS_ABSOLUT_LOW_CHANNEL1_LSB           66
#define REG0_VILNIUS_ABSOLUT_LOW_CHANNEL2_MSB           67
#define REG0_VILNIUS_ABSOLUT_LOW_CHANNEL2_LSB           68
#define REG0_VILNIUS_ABSOLUT_LOW_CHANNEL3_MSB           69
#define REG0_VILNIUS_ABSOLUT_LOW_CHANNEL3_LSB           70

#define REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL0_MSB          71
#define REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL0_LSB          72
#define REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL1_MSB          73
#define REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL1_LSB          74
#define REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL2_MSB          75
#define REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL2_LSB          76
#define REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL3_MSB          77
#define REG0_VILNIUS_ABSOLUT_HIGH_CHANNEL3_LSB          78

#define REG0_VILNIUS_CONTROL_IO                         79
#define REG0_VILNIUS_IO_STATE                           80

// * * * Page 1 * * *

#define REG1_VILNIUS_LINEARIZATION_EVENT_SETTING_CH0    0
#define REG1_VILNIUS_LINEARIZATION_EVENT_CLASS_CH0      1
#define REG1_VILNIUS_LINEARIZATION_EVENT_TYPE_CH0       2

#define REG1_VILNIUS_LINEARIZATION_EVENT_SETTING_CH1    3
#define REG1_VILNIUS_LINEARIZATION_EVENT_CLASS_CH1      4
#define REG1_VILNIUS_LINEARIZATION_EVENT_TYPE_CH1       5

#define REG1_VILNIUS_LINEARIZATION_EVENT_SETTING_CH2    6
#define REG1_VILNIUS_LINEARIZATION_EVENT_CLASS_CH2      7
#define REG1_VILNIUS_LINEARIZATION_EVENT_TYPE_CH2       8

#define REG1_VILNIUS_LINEARIZATION_EVENT_SETTING_CH3    9
#define REG1_VILNIUS_LINEARIZATION_EVENT_CLASS_CH3      10
#define REG1_VILNIUS_LINEARIZATION_EVENT_TYPE_CH3       11

#define REG1_VILNIUS_CH0_LINEARIZATION_K_MSB    12
#define REG1_VILNIUS_CH0_LINEARIZATION_K_0      12
#define REG1_VILNIUS_CH0_LINEARIZATION_K_1      13
#define REG1_VILNIUS_CH0_LINEARIZATION_K_2      14
#define REG1_VILNIUS_CH0_LINEARIZATION_K_3      15
#define REG1_VILNIUS_CH0_LINEARIZATION_K_LSB    15

#define REG1_VILNIUS_CH1_LINEARIZATION_K_MSB    16
#define REG1_VILNIUS_CH1_LINEARIZATION_K_0      16
#define REG1_VILNIUS_CH1_LINEARIZATION_K_1      17
#define REG1_VILNIUS_CH1_LINEARIZATION_K_2      18
#define REG1_VILNIUS_CH1_LINEARIZATION_K_3      19
#define REG1_VILNIUS_CH1_LINEARIZATION_K_LSB    19

#define REG1_VILNIUS_CH2_LINEARIZATION_K_MSB    20
#define REG1_VILNIUS_CH2_LINEARIZATION_K_0      20
#define REG1_VILNIUS_CH2_LINEARIZATION_K_1      21
#define REG1_VILNIUS_CH2_LINEARIZATION_K_2      22
#define REG1_VILNIUS_CH2_LINEARIZATION_K_3      23
#define REG1_VILNIUS_CH2_LINEARIZATION_K_LSB    23

#define REG1_VILNIUS_CH3_LINEARIZATION_K_MSB    24
#define REG1_VILNIUS_CH3_LINEARIZATION_K_0      24
#define REG1_VILNIUS_CH3_LINEARIZATION_K_1      25
#define REG1_VILNIUS_CH3_LINEARIZATION_K_2      26
#define REG1_VILNIUS_CH3_LINEARIZATION_K_3      27
#define REG1_VILNIUS_CH3_LINEARIZATION_K_LSB    27

#define REG1_VILNIUS_CH0_LINEARIZATION_M_MSB    28
#define REG1_VILNIUS_CH0_LINEARIZATION_M_0      28
#define REG1_VILNIUS_CH0_LINEARIZATION_M_1      29
#define REG1_VILNIUS_CH0_LINEARIZATION_M_2      30
#define REG1_VILNIUS_CH0_LINEARIZATION_M_3      31
#define REG1_VILNIUS_CH0_LINEARIZATION_M_LSB    31

#define REG1_VILNIUS_CH1_LINEARIZATION_M_MSB    32
#define REG1_VILNIUS_CH1_LINEARIZATION_M_0      32
#define REG1_VILNIUS_CH1_LINEARIZATION_M_1      33
#define REG1_VILNIUS_CH1_LINEARIZATION_M_2      34
#define REG1_VILNIUS_CH1_LINEARIZATION_M_3      35
#define REG1_VILNIUS_CH1_LINEARIZATION_M_LSB    35

#define REG1_VILNIUS_CH2_LINEARIZATION_M_MSB    36
#define REG1_VILNIUS_CH2_LINEARIZATION_M_0      36
#define REG1_VILNIUS_CH2_LINEARIZATION_M_1      37
#define REG1_VILNIUS_CH2_LINEARIZATION_M_2      38
#define REG1_VILNIUS_CH2_LINEARIZATION_M_3      39
#define REG1_VILNIUS_CH2_LINEARIZATION_M_LSB    39

#define REG1_VILNIUS_CH3_LINEARIZATION_M_MSB    40
#define REG1_VILNIUS_CH3_LINEARIZATION_M_0      40
#define REG1_VILNIUS_CH3_LINEARIZATION_M_1      41
#define REG1_VILNIUS_CH3_LINEARIZATION_M_2      42
#define REG1_VILNIUS_CH3_LINEARIZATION_M_3      43
#define REG1_VILNIUS_CH3_LINEARIZATION_M_LSB    43

#define REG1_VILNIUS_CH0_MEASUREMENT_VALUE_MSB  44
#define REG1_VILNIUS_CH0_MEASUREMENT_VALUE_0    44
#define REG1_VILNIUS_CH0_MEASUREMENT_VALUE_1    45
#define REG1_VILNIUS_CH0_MEASUREMENT_VALUE_2    46
#define REG1_VILNIUS_CH0_MEASUREMENT_VALUE_3    47
#define REG1_VILNIUS_CH0_MEASUREMENT_VALUE_LSB  47

#define REG1_VILNIUS_CH1_MEASUREMENT_VALUE_MSB  48
#define REG1_VILNIUS_CH1_MEASUREMENT_VALUE_0    48
#define REG1_VILNIUS_CH1_MEASUREMENT_VALUE_1    49
#define REG1_VILNIUS_CH1_MEASUREMENT_VALUE_2    50
#define REG1_VILNIUS_CH1_MEASUREMENT_VALUE_3    51
#define REG1_VILNIUS_CH1_MEASUREMENT_VALUE_LSB  51

#define REG1_VILNIUS_CH2_MEASUREMENT_VALUE_MSB  52
#define REG1_VILNIUS_CH2_MEASUREMENT_VALUE_0    52
#define REG1_VILNIUS_CH2_MEASUREMENT_VALUE_1    53
#define REG1_VILNIUS_CH2_MEASUREMENT_VALUE_2    54
#define REG1_VILNIUS_CH2_MEASUREMENT_VALUE_3    55
#define REG1_VILNIUS_CH2_MEASUREMENT_VALUE_LSB  55

#define REG1_VILNIUS_CH3_MEASUREMENT_VALUE_MSB  56
#define REG1_VILNIUS_CH3_MEASUREMENT_VALUE_0    56
#define REG1_VILNIUS_CH3_MEASUREMENT_VALUE_1    57
#define REG1_VILNIUS_CH3_MEASUREMENT_VALUE_2    58
#define REG1_VILNIUS_CH3_MEASUREMENT_VALUE_3    59
#define REG1_VILNIUS_CH3_MEASUREMENT_VALUE_LSB  59

//      Page 2

#define DESCION_MATRIX_PAGE                     3
#define DESCION_MATRIX_ROWS                     4
#define REG_DESCION_MATRIX                      0    // Start of matrix

#define NUMBER_OF_REGISTER_PAGES                5

// * * * Persistent storage

#define VSCP_EEPROM_BOOTLOADER_FLAG             0x00	// Reserved for bootloader

#define VSCP_EEPROM_NICKNAME                    0x01	// Persistant nickname id storage
#define VSCP_EEPROM_CONTROL0                    0x02	// Persistant segment crc storage
#define VSCP_EEPROM_CONTROL1                    0x03	// Persistant control byte

//#define EEPROM_ZONE                           0x04	// Zone node belongs to
//#define EEPROM_SUBZONE                        0x05	// Subzone node belongs to

#define VSCP_EEPROM_REG_USERID                  0x06
#define VSCP_EEPROM_REG_USERID1                 0x07
#define VSCP_EEPROM_REG_USERID2                 0x08
#define VSCP_EEPROM_REG_USERID3                 0x09
#define VSCP_EEPROM_REG_USERID4                 0x0A

// The following can be stored in flash or eeprom

#define VSCP_EEPROM_REG_MANUFACTUR_ID0          0x0B
#define VSCP_EEPROM_REG_MANUFACTUR_ID1          0x0C
#define VSCP_EEPROM_REG_MANUFACTUR_ID2          0x0D
#define VSCP_EEPROM_REG_MANUFACTUR_ID3          0x0E

#define VSCP_EEPROM_REG_MANUFACTUR_SUBID0       0x0F
#define VSCP_EEPROM_REG_MANUFACTUR_SUBID1       0x10
#define VSCP_EEPROM_REG_MANUFACTUR_SUBID2       0x11
#define VSCP_EEPROM_REG_MANUFACTUR_SUBID3       0x12

// The following can be stored in program ROM (recommended) or in EEPROM

#define VSCP_EEPROM_REG_GUID                    0x13	// Start of GUID MSB
// 		0x13 - 0x22

#define VSCP_EEPROM_REG_DEVICE_URL              0x23	// Start of Device URL storage
// 		0x23 - 0x42

#define VSCP_EEPROM_END                         0x43	// marks end of VSCP EEPROM usage
                                                        //   (next free position)

// --------------------------------------------------------------------------------

#define VILNIUS_ACTION_NOOP                     0
#define VILNIUS_ACTION_SET                      1
#define VILNIUS_ACTION_CLEAR                    2
#define VILNIUS_ACTION_TOGGLE                   3
#define VILNIUS_ACTION_STATUS                   4
#define VILNIUS_ACTION_STATUSALL                5

// --------------------------------------------------------------------------------

// Function Prototypes

double calculateMeasurement( uint8_t sendoridx );
void doWork(void);
void init(void);
void init_app_ram(void);
void init_app_eeprom(void);
void read_app_register(unsigned char reg);
void write_app_register(unsigned char reg, unsigned char val);
void sendDMatrixInfo(void);
void SendInformationEvent(unsigned char channel, unsigned char eventClass, unsigned char eventTypeId);
void SendAlarmEvent( uint8_t channel  );
void SendDataEvent( uint8_t eventType, uint8_t sensoridx, uint16_t val );
void sendMeasurementEvent( uint8_t channel );
void handleSyncRequest( uint8_t sensoridx, uint8_t zone, uint8_t subzone );

// Handle DM
void doDM(void);

// Actions
void actionSet( uint8_t param );
void actionClear( uint8_t param );
void actionToggle( uint8_t param );
void actionStatus( uint8_t param );
void actionStatusAll( uint8_t param );

// Do application work
void doApplicationOneSecondWork(void);

/*!
        Send Extended ID CAN frame
        @param id CAN extended ID for frame.
        @param size Number of databytes 0-8
        @param pData Pointer to databytes of frame.
        @return TRUE (!=0) on success, FALSE (==0) on failure.
 */
int8_t sendCANFrame(uint32_t id, uint8_t size, uint8_t *pData);

/*!
        Get extended ID CAN frame
        @param pid Pointer to CAN extended ID for frame.
        @param psize Pointer to number of databytes 0-8
        @param pData Pointer to databytes of frame.
        @return TRUE (!=0) on success, FALSE (==0) on failure.
 */
int8_t getCANFrame(uint32_t *pid, uint8_t *psize, uint8_t *pData);

#endif
