// File:  main.h

/* ******************************************************************************
 * VSCP (Very Simple Control Protocol)
 * http://www.vscp.org
 *
 * Copyright (C) 1995-2005 Ake Hedman, eurosource
 * Copyright (C) 1995-2013 Ake Hedman, Grodans Paradis AB
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

#ifndef ACCRA_H
#define ACCRA_H

//Defines
#define	TRUE                    1
#define	FALSE                   0

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

#define STATUS_LED              PORTCbits.RC1
#define INIT_BUTTON             PORTCbits.RC0

// Defines for channels LATBbits.LATB0
#define CHANNEL0                PORTBbits.RB0
#define CHANNEL1                PORTBbits.RB1
#define CHANNEL2                PORTCbits.RC2
#define CHANNEL3                PORTCbits.RC3

#define LATCH_CHANNEL0          LATBbits.LATB0
#define LATCH_CHANNEL1          LATBbits.LATB1
#define LATCH_CHANNEL2          LATCbits.LATC2
#define LATCH_CHANNEL3          LATCbits.LATC3

#define CHANNEL_DIRECTION_UP                0
#define CHANNEL_DIRECTION_DOWN              1


// Bits for counter control register
#define COUNTER_CTRL_DIRECTION              0x01    // Counting direction
#define COUNTER_CTRL_ALARM                  0x02    // Enable alarm level
#define COUNTER_CTRL_RELOAD_ZERO            0x04 	// Reload on zero          
#define COUNTER_CTRL_ENABLED                0x80 	// Relay enabled

// Bits for frequency control registers
#define FREQ_CTRL_LOW_ALARM                 0x01    // Enable frequency low alarm
#define FREQ_CTRL_HIGH_ALARM                0x02    // Enable frequency high alarm  
#define FREQ_CTRL_STREAM                    0x10    // Enable stream events
#define FREQ_CTRL_ENABLE                    0x80    // Enable Input.

// Bits for measurement control registers
#define MEASUREMENT_CTRL_CALCBASE           0x01    // 0 use counter, 1 use frequency
#define MEASUREMENT_CTRL_UNIT0              0x08    // Unit low bit
#define MEASUREMENT_CTRL_UNIT1              0x10    // Unit high bit
#define MEASUREMENT_CTRL_CLASS_BIT_8        0x40    // Bit 8 of the class
#define MEASUREMENT_CTRL_ENABLED            0x80    // Enable measuement calculations

#define MEASUREMENT_CTRL_UNIT_MASK          0b00011000
        
// Alarm bits
#define ALARM_COUNTER0                      0x01    // Counter 0 alarm
#define ALARM_COUNTER1                      0x02    // Counter 1 alarm
#define ALARM_COUNTER2                      0x04    // Counter 2 alarm
#define ALARM_COUNTER3                      0x08    // Counter 3 alarm
#define ALARM_FREQUENCY0                    0x10    // Frequency 0 alarm
#define ALARM_FREQUENCY1                    0x20    // Frequency 1 alarm
#define ALARM_FREQUENCY2                    0x40    // Frequency 2 alarm
#define ALARM_FREQUENCY3                    0x50    // Frequency 3 alarm

// -----------------------------------------------

// Defaults
#define DEFAULT_COUNTER0_HYSTERESIS         200
#define DEFAULT_COUNTER1_HYSTERESIS         200
#define DEFAULT_COUNTER2_HYSTERESIS         200
#define DEFAULT_COUNTER3_HYSTERESIS         200

#define DEFAULT_FREQUENCY0_HYSTERESIS       50
#define DEFAULT_FREQUENCY1_HYSTERESIS       50
#define DEFAULT_FREQUENCY2_HYSTERESIS       50
#define DEFAULT_FREQUENCY3_HYSTERESIS       50

// -----------------------------------------------

// * * *  Registers * * *

// * * * * Page 0 * * * *

#define REG0_ACCRA_ZONE                             0
#define REG0_ACCRA_SUBZONE                          1

#define REG0_ACCRA_CH0_SUBZONE                      2
#define REG0_ACCRA_CH1_SUBZONE                      3
#define REG0_ACCRA_CH2_SUBZONE                      4
#define REG0_ACCRA_CH3_SUBZONE                      5

#define REG0_ACCRA_CONTROL_COUNTER_CH0              6
#define REG0_ACCRA_CONTROL_COUNTER_CH1              7
#define REG0_ACCRA_CONTROL_COUNTER_CH2              8
#define REG0_ACCRA_CONTROL_COUNTER_CH3              9          

#define REG0_ACCRA_CONTROL_FREQ_CH0                 10
#define REG0_ACCRA_CONTROL_FREQ_CH1                 11
#define REG0_ACCRA_CONTROL_FREQ_CH2                 12
#define REG0_ACCRA_CONTROL_FREQ_CH3                 13

#define REG0_ACCRA_RESET_COUNTER_CH0                14  // Not stored in EEPROM
#define REG0_ACCRA_RESET_COUNTER_CH1                15  // Not stored in EEPROM
#define REG0_ACCRA_RESET_COUNTER_CH2                16  // Not stored in EEPROM
#define REG0_ACCRA_RESET_COUNTER_CH3                17  // Not stored in EEPROM

#define REG0_ACCRA_COUNTER_CH0_MSB                  18  // Not stored in EEPROM  *
#define REG0_ACCRA_COUNTER_CH0_0                    18  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH0_1                    19  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH0_2                    20  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH0_3                    21  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH0_LSB                  21  // Not stored in EEPROM

#define REG0_ACCRA_COUNTER_CH1_MSB                  22  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH1_0                    22  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH1_1                    23  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH1_2                    24  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH1_3                    25  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH1_LSB                  25  // Not stored in EEPROM

#define REG0_ACCRA_COUNTER_CH2_MSB                  26  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH2_0                    26  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH2_1                    27  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH2_2                    28  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH2_3                    29  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH2_LSB                  29  // Not stored in EEPROM

#define REG0_ACCRA_COUNTER_CH3_MSB                  30  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH3_0                    30  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH3_1                    31  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH3_2                    32  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH3_3                    33  // Not stored in EEPROM
#define REG0_ACCRA_COUNTER_CH3_LSB                  33  // Not stored in EEPROM

#define REG0_ACCRA_COUNTER_ALARM_CH0_MSB            34  // *
#define REG0_ACCRA_COUNTER_ALARM_CH0_0              34
#define REG0_ACCRA_COUNTER_ALARM_CH0_1              35
#define REG0_ACCRA_COUNTER_ALARM_CH0_2              36
#define REG0_ACCRA_COUNTER_ALARM_CH0_3              37
#define REG0_ACCRA_COUNTER_ALARM_CH0_LSB            37

#define REG0_ACCRA_COUNTER_ALARM_CH1_MSB            38
#define REG0_ACCRA_COUNTER_ALARM_CH1_0              38
#define REG0_ACCRA_COUNTER_ALARM_CH1_1              39
#define REG0_ACCRA_COUNTER_ALARM_CH1_2              40
#define REG0_ACCRA_COUNTER_ALARM_CH1_3              41
#define REG0_ACCRA_COUNTER_ALARM_CH1_LSB            41

#define REG0_ACCRA_COUNTER_ALARM_CH2_MSB            42
#define REG0_ACCRA_COUNTER_ALARM_CH2_0              42
#define REG0_ACCRA_COUNTER_ALARM_CH2_1              43
#define REG0_ACCRA_COUNTER_ALARM_CH2_2              44
#define REG0_ACCRA_COUNTER_ALARM_CH2_3              45
#define REG0_ACCRA_COUNTER_ALARM_CH2_LSB            45

#define REG0_ACCRA_COUNTER_ALARM_CH3_MSB            46
#define REG0_ACCRA_COUNTER_ALARM_CH3_0              46
#define REG0_ACCRA_COUNTER_ALARM_CH3_1              47
#define REG0_ACCRA_COUNTER_ALARM_CH3_2              48
#define REG0_ACCRA_COUNTER_ALARM_CH3_3              49
#define REG0_ACCRA_COUNTER_ALARM_CH3_LSB            49

#define REG0_ACCRA_COUNTER_RELOAD_CH0_MSB           50  // *
#define REG0_ACCRA_COUNTER_RELOAD_CH0_0             50
#define REG0_ACCRA_COUNTER_RELOAD_CH0_1             51
#define REG0_ACCRA_COUNTER_RELOAD_CH0_2             52
#define REG0_ACCRA_COUNTER_RELOAD_CH0_3             53
#define REG0_ACCRA_COUNTER_RELOAD_CH0_LSB           53

#define REG0_ACCRA_COUNTER_RELOAD_CH1_MSB           54
#define REG0_ACCRA_COUNTER_RELOAD_CH1_0             54
#define REG0_ACCRA_COUNTER_RELOAD_CH1_1             55
#define REG0_ACCRA_COUNTER_RELOAD_CH1_2             56
#define REG0_ACCRA_COUNTER_RELOAD_CH1_3             57
#define REG0_ACCRA_COUNTER_RELOAD_CH1_LSB           57

#define REG0_ACCRA_COUNTER_RELOAD_CH2_MSB           58
#define REG0_ACCRA_COUNTER_RELOAD_CH2_0             58
#define REG0_ACCRA_COUNTER_RELOAD_CH2_1             59
#define REG0_ACCRA_COUNTER_RELOAD_CH2_2             60
#define REG0_ACCRA_COUNTER_RELOAD_CH2_3             61
#define REG0_ACCRA_COUNTER_RELOAD_CH2_LSB           61

#define REG0_ACCRA_COUNTER_RELOAD_CH3_MSB           62
#define REG0_ACCRA_COUNTER_RELOAD_CH3_0             62
#define REG0_ACCRA_COUNTER_RELOAD_CH3_1             63
#define REG0_ACCRA_COUNTER_RELOAD_CH3_2             64
#define REG0_ACCRA_COUNTER_RELOAD_CH3_3             65
#define REG0_ACCRA_COUNTER_RELOAD_CH3_LSB           65

#define REG0_ACCRA_COUNTER_HYSTERESIS_CH0_MSB       66
#define REG0_ACCRA_COUNTER_HYSTERESIS_CH0_LSB       67

#define REG0_ACCRA_COUNTER_HYSTERESIS_CH1_MSB       68
#define REG0_ACCRA_COUNTER_HYSTERESIS_CH1_LSB       69

#define REG0_ACCRA_COUNTER_HYSTERESIS_CH2_MSB       70
#define REG0_ACCRA_COUNTER_HYSTERESIS_CH2_LSB       71

#define REG0_ACCRA_COUNTER_HYSTERESIS_CH3_MSB       72
#define REG0_ACCRA_COUNTER_HYSTERESIS_CH3_LSB       73

#define REG0_ACCRA_COUNTER_REPORT_INTERVAL_CH0      74
#define REG0_ACCRA_COUNTER_REPORT_INTERVAL_CH1      75
#define REG0_ACCRA_COUNTER_REPORT_INTERVAL_CH2      76
#define REG0_ACCRA_COUNTER_REPORT_INTERVAL_CH3      77

#define REG0_ACCRA_FREQ_REPORT_INTERVAL_CH0         78
#define REG0_ACCRA_FREQ_REPORT_INTERVAL_CH1         79
#define REG0_ACCRA_FREQ_REPORT_INTERVAL_CH2         80
#define REG0_ACCRA_FREQ_REPORT_INTERVAL_CH3         81

#define REG0_ACCRA_MESURMENT_REPORT_INTERVAL_CH0    82
#define REG0_ACCRA_MESURMENT_REPORT_INTERVAL_CH1    83
#define REG0_ACCRA_MESURMENT_REPORT_INTERVAL_CH2    84
#define REG0_ACCRA_MESURMENT_REPORT_INTERVAL_CH3    85

#define REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH0  86
#define REG0_ACCRA_LINEARIZATION_EVENT_CLASS_CH0    87
#define REG0_ACCRA_LINEARIZATION_EVENT_TYPE_CH0     88

#define REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH1  89
#define REG0_ACCRA_LINEARIZATION_EVENT_CLASS_CH1    90
#define REG0_ACCRA_LINEARIZATION_EVENT_TYPE_CH1     91

#define REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH2  92
#define REG0_ACCRA_LINEARIZATION_EVENT_CLASS_CH2    93
#define REG0_ACCRA_LINEARIZATION_EVENT_TYPE_CH2     94

#define REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH3  95
#define REG0_ACCRA_LINEARIZATION_EVENT_CLASS_CH3    96
#define REG0_ACCRA_LINEARIZATION_EVENT_TYPE_CH3     97

#define REG0_MINUS                                  28              // # registers not written to EEPROM
#define REG0_COUNT                                  (98-REG0_MINUS) // Needed for EEPROM writes

// * * *  Page 1 * * *


#define REG1_ACCRA_CH0_FREQUENCY_MSB        0   // Not stored in EEPROM
#define REG1_ACCRA_CH0_FREQUENCY_0          0   // Not stored in EEPROM
#define REG1_ACCRA_CH0_FREQUENCY_1          1   // Not stored in EEPROM
#define REG1_ACCRA_CH0_FREQUENCY_2          2   // Not stored in EEPROM
#define REG1_ACCRA_CH0_FREQUENCY_3          3   // Not stored in EEPROM
#define REG1_ACCRA_CH0_FREQUENCY_LSB        3   // Not stored in EEPROM

#define REG1_ACCRA_CH1_FREQUENCY_MSB        4   // Not stored in EEPROM
#define REG1_ACCRA_CH1_FREQUENCY_0          4   // Not stored in EEPROM
#define REG1_ACCRA_CH1_FREQUENCY_1          5   // Not stored in EEPROM
#define REG1_ACCRA_CH1_FREQUENCY_2          6   // Not stored in EEPROM
#define REG1_ACCRA_CH1_FREQUENCY_3          7   // Not stored in EEPROM
#define REG1_ACCRA_CH1_FREQUENCY_LSB        7   // Not stored in EEPROM

#define REG1_ACCRA_CH2_FREQUENCY_MSB        8   // Not stored in EEPROM
#define REG1_ACCRA_CH2_FREQUENCY_0          8   // Not stored in EEPROM
#define REG1_ACCRA_CH2_FREQUENCY_1          9   // Not stored in EEPROM
#define REG1_ACCRA_CH2_FREQUENCY_2          10  // Not stored in EEPROM
#define REG1_ACCRA_CH2_FREQUENCY_3          11  // Not stored in EEPROM
#define REG1_ACCRA_CH2_FREQUENCY_LSB        11  // Not stored in EEPROM

#define REG1_ACCRA_CH3_FREQUENCY_MSB        12  // Not stored in EEPROM
#define REG1_ACCRA_CH3_FREQUENCY_0          12  // Not stored in EEPROM
#define REG1_ACCRA_CH3_FREQUENCY_1          13  // Not stored in EEPROM
#define REG1_ACCRA_CH3_FREQUENCY_2          14  // Not stored in EEPROM
#define REG1_ACCRA_CH3_FREQUENCY_3          15  // Not stored in EEPROM
#define REG1_ACCRA_CH3_FREQUENCY_LSB        15  // Not stored in EEPROM

#define REG1_ACCRA_CH0_FREQUENCY_LOW_MSB    16
#define REG1_ACCRA_CH0_FREQUENCY_LOW_0      16
#define REG1_ACCRA_CH0_FREQUENCY_LOW_1      17
#define REG1_ACCRA_CH0_FREQUENCY_LOW_2      18
#define REG1_ACCRA_CH0_FREQUENCY_LOW_3      19
#define REG1_ACCRA_CH0_FREQUENCY_LOW_LSB    19

#define REG1_ACCRA_CH1_FREQUENCY_LOW_MSB    20
#define REG1_ACCRA_CH1_FREQUENCY_LOW_0      20
#define REG1_ACCRA_CH1_FREQUENCY_LOW_1      21
#define REG1_ACCRA_CH1_FREQUENCY_LOW_2      22
#define REG1_ACCRA_CH1_FREQUENCY_LOW_3      23
#define REG1_ACCRA_CH1_FREQUENCY_LOW_LSB    23

#define REG1_ACCRA_CH2_FREQUENCY_LOW_MSB    24
#define REG1_ACCRA_CH2_FREQUENCY_LOW_0      24
#define REG1_ACCRA_CH2_FREQUENCY_LOW_1      25
#define REG1_ACCRA_CH2_FREQUENCY_LOW_2      26
#define REG1_ACCRA_CH2_FREQUENCY_LOW_3      27
#define REG1_ACCRA_CH2_FREQUENCY_LOW_LSB    27

#define REG1_ACCRA_CH3_FREQUENCY_LOW_MSB    28
#define REG1_ACCRA_CH3_FREQUENCY_LOW_0      28
#define REG1_ACCRA_CH3_FREQUENCY_LOW_1      29
#define REG1_ACCRA_CH3_FREQUENCY_LOW_2      30
#define REG1_ACCRA_CH3_FREQUENCY_LOW_3      31
#define REG1_ACCRA_CH3_FREQUENCY_LOW_LSB    31

#define REG1_ACCRA_CH0_FREQUENCY_HIGH_MSB   32
#define REG1_ACCRA_CH0_FREQUENCY_HIGH_0     32
#define REG1_ACCRA_CH0_FREQUENCY_HIGH_1     33
#define REG1_ACCRA_CH0_FREQUENCY_HIGH_2     34
#define REG1_ACCRA_CH0_FREQUENCY_HIGH_3     35
#define REG1_ACCRA_CH0_FREQUENCY_HIGH_LSB   35

#define REG1_ACCRA_CH1_FREQUENCY_HIGH_MSB   36
#define REG1_ACCRA_CH1_FREQUENCY_HIGH_0     36
#define REG1_ACCRA_CH1_FREQUENCY_HIGH_1     37
#define REG1_ACCRA_CH1_FREQUENCY_HIGH_2     38
#define REG1_ACCRA_CH1_FREQUENCY_HIGH_3     39
#define REG1_ACCRA_CH1_FREQUENCY_HIGH_LSB   39

#define REG1_ACCRA_CH2_FREQUENCY_HIGH_MSB   40
#define REG1_ACCRA_CH2_FREQUENCY_HIGH_0     40
#define REG1_ACCRA_CH2_FREQUENCY_HIGH_1     41
#define REG1_ACCRA_CH2_FREQUENCY_HIGH_2     42
#define REG1_ACCRA_CH2_FREQUENCY_HIGH_3     43
#define REG1_ACCRA_CH2_FREQUENCY_HIGH_LSB   43

#define REG1_ACCRA_CH3_FREQUENCY_HIGH_MSB   44
#define REG1_ACCRA_CH3_FREQUENCY_HIGH_0     44
#define REG1_ACCRA_CH3_FREQUENCY_HIGH_1     45
#define REG1_ACCRA_CH3_FREQUENCY_HIGH_2     46
#define REG1_ACCRA_CH3_FREQUENCY_HIGH_3     47
#define REG1_ACCRA_CH3_FREQUENCY_HIGH_LSB   47

#define REG1_ACCRA_CH0_FREQ_HYSTERESIS_MSB  48
#define REG1_ACCRA_CH0_FREQ_HYSTERESIS_LSB  49

#define REG1_ACCRA_CH1_FREQ_HYSTERESIS_MSB  50
#define REG1_ACCRA_CH1_FREQ_HYSTERESIS_LSB  51

#define REG1_ACCRA_CH2_FREQ_HYSTERESIS_MSB  52
#define REG1_ACCRA_CH2_FREQ_HYSTERESIS_LSB  53

#define REG1_ACCRA_CH3_FREQ_HYSTERESIS_MSB  54
#define REG1_ACCRA_CH3_FREQ_HYSTERESIS_LSB  55

#define REG1_MINUS                          16              // # registers not written to EEPROM
#define REG1_COUNT                          (56-REG1_MINUS) // Needed for EEPROM writes


// * * *  Page 2 * * *

#define REG2_ACCRA_CH0_LINEARIZATION_K_MSB      0
#define REG2_ACCRA_CH0_LINEARIZATION_K_0        0
#define REG2_ACCRA_CH0_LINEARIZATION_K_1        1
#define REG2_ACCRA_CH0_LINEARIZATION_K_2        2
#define REG2_ACCRA_CH0_LINEARIZATION_K_3        3
#define REG2_ACCRA_CH0_LINEARIZATION_K_LSB      3

#define REG2_ACCRA_CH1_LINEARIZATION_K_MSB      4
#define REG2_ACCRA_CH1_LINEARIZATION_K_0        4
#define REG2_ACCRA_CH1_LINEARIZATION_K_1        5
#define REG2_ACCRA_CH1_LINEARIZATION_K_2        6
#define REG2_ACCRA_CH1_LINEARIZATION_K_3        7
#define REG2_ACCRA_CH1_LINEARIZATION_K_LSB      7

#define REG2_ACCRA_CH2_LINEARIZATION_K_MSB      8
#define REG2_ACCRA_CH2_LINEARIZATION_K_0        8
#define REG2_ACCRA_CH2_LINEARIZATION_K_1        9
#define REG2_ACCRA_CH2_LINEARIZATION_K_2        10
#define REG2_ACCRA_CH2_LINEARIZATION_K_3        11
#define REG2_ACCRA_CH2_LINEARIZATION_K_LSB      11

#define REG2_ACCRA_CH3_LINEARIZATION_K_MSB      12
#define REG2_ACCRA_CH3_LINEARIZATION_K_0        12
#define REG2_ACCRA_CH3_LINEARIZATION_K_1        13
#define REG2_ACCRA_CH3_LINEARIZATION_K_2        14
#define REG2_ACCRA_CH3_LINEARIZATION_K_3        15
#define REG2_ACCRA_CH3_LINEARIZATION_K_LSB      15

#define REG2_ACCRA_CH0_LINEARIZATION_M_MSB      16
#define REG2_ACCRA_CH0_LINEARIZATION_M_0        16
#define REG2_ACCRA_CH0_LINEARIZATION_M_1        17
#define REG2_ACCRA_CH0_LINEARIZATION_M_2        18
#define REG2_ACCRA_CH0_LINEARIZATION_M_3        19
#define REG2_ACCRA_CH0_LINEARIZATION_M_LSB      19

#define REG2_ACCRA_CH1_LINEARIZATION_M_MSB      20
#define REG2_ACCRA_CH1_LINEARIZATION_M_0        20
#define REG2_ACCRA_CH1_LINEARIZATION_M_1        21
#define REG2_ACCRA_CH1_LINEARIZATION_M_2        22
#define REG2_ACCRA_CH1_LINEARIZATION_M_3        23
#define REG2_ACCRA_CH1_LINEARIZATION_M_LSB      23

#define REG2_ACCRA_CH2_LINEARIZATION_M_MSB      24
#define REG2_ACCRA_CH2_LINEARIZATION_M_0        24
#define REG2_ACCRA_CH2_LINEARIZATION_M_1        25
#define REG2_ACCRA_CH2_LINEARIZATION_M_2        26
#define REG2_ACCRA_CH2_LINEARIZATION_M_3        27
#define REG2_ACCRA_CH2_LINEARIZATION_M_LSB      27

#define REG2_ACCRA_CH3_LINEARIZATION_M_MSB      28
#define REG2_ACCRA_CH3_LINEARIZATION_M_0        28
#define REG2_ACCRA_CH3_LINEARIZATION_M_1        29
#define REG2_ACCRA_CH3_LINEARIZATION_M_2        30
#define REG2_ACCRA_CH3_LINEARIZATION_M_3        31
#define REG2_ACCRA_CH3_LINEARIZATION_M_LSB      31

#define REG2_COUNT                              32  // Needed for EEPROM writes

//      Page 3

#define DESCION_MATRIX_PAGE                     3
#define DESCION_MATRIX_ROWS                     4
#define REG_DESCION_MATRIX                      0    // Start of matrix

#define NUMBER_OF_REGISTER_PAGES                5

// * * * Persistent storage

#define VSCP_EEPROM_BOOTLOADER_FLAG             0x00	// Reserved for bootloader

#define VSCP_EEPROM_NICKNAME                    0x01	// Persistant nickname id storage
#define VSCP_EEPROM_SEGMENT_CRC                 0x02	// Persistant segment crc storage
#define VSCP_EEPROM_CONTROL                     0x03	// Persistant control byte

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

#define ACCRA_ACTION_NOOP                       0
#define ACCRA_ACTION_START                      1
#define ACCRA_ACTION_STOP                       2
#define ACCRA_ACTION_CLEAR                      3
#define ACCRA_ACTION_RELOAD                     4
#define ACCRA_ACTION_COUNT                      5

// --------------------------------------------------------------------------------

// Function Prototypes

void doWork(void);
void init(void);
void init_app_ram(void);
void init_app_eeprom(void);
void read_app_register(unsigned char reg);
void write_app_register(unsigned char reg, unsigned char val);
void sendDMatrixInfo(void);
void SendInformationEvent(unsigned char channel, unsigned char eventClass, unsigned char eventTypeId);
void SendAlarmEvent( uint8_t channel  );

// Handle DM
void doDM(void);

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
