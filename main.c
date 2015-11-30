/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol) 
 * 	http://www.vscp.org
 *
 *  Accra I/O module
 * 	akhe@grodansparadis.com
 *
 *  Copyright (C) 1995-2015 Ake Hedman, Grodans Paradis AB
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

#include "vscp_compiler.h"
#include "vscp_projdefs.h"

#include <p18cxxx.h>
#include <timers.h>
#include <delays.h>
#include <inttypes.h>
#include <string.h>
#include <ecan.h>
#include <vscp_firmware.h>
#include <vscp_class.h>
#include <vscp_type.h>
#include "accra.h"
#include "version.h"

#if defined(RELEASE)

#pragma config WDT = ON, WDTPS = 128
#pragma config OSC = HSPLL
#pragma config BOREN = BOACTIVE
#pragma config STVREN = ON
#pragma config BORV = 3
#pragma config LVP = ON
#pragma config CPB = ON
#pragma config BBSIZ = 2048
#pragma config WRTD  = OFF

#pragma config EBTR0 = OFF
#pragma config EBTR1 = OFF
#pragma config EBTR2 = OFF
#pragma config EBTR3 = OFF

#pragma config EBTRB = OFF

#else

#pragma config WDT = OFF
#pragma config OSC = HSPLL
#pragma config PWRT = ON
#pragma config BOREN = BOACTIVE
#pragma config STVREN = ON
#pragma config BORV = 3
#pragma config LVP = OFF
#pragma config CPB = OFF
#pragma config WRTD  = OFF

#pragma config EBTR0 = OFF
#pragma config EBTR1 = OFF
#pragma config EBTR2 = OFF
#pragma config EBTR3 = OFF

#pragma config EBTRB = OFF

#endif

// Calculate and st required filter and mask
// for the current decision matrix
void calculateSetFilterMask( void );

// The device URL (max 32 characters including null termination)
const uint8_t vscp_deviceURL[] = "www.eurosource.se/accra_1.xml";

volatile unsigned long measurement_clock_sec;  // Clock for one second work

uint16_t sendTimer;         // Timer for CAN send
uint8_t seconds;            // Counter for seconds
uint8_t minutes;            // Counter for minutes
uint8_t hours;              // Counter for hours

uint32_t counter[4];        // Counters
uint32_t lastcounter[4];    // Counter values last second (for frequency)
uint32_t frequency[4];      // Frequency calculations
uint8_t counter2LastState;  // Last state for counter 2
uint8_t counter3LastState;  // Last state for counter 3

// This is a shadow of the EEPROM stored control register for each
// counter
uint8_t counter_control_shadow[ 4 ];

// Flags to reload counters when they reach zero. The IRQ flags this
// and let the main routine do the eeprom read
BOOL bReloadCounter0;
BOOL bReloadCounter1;

// Set to true when an alarm condition is met. Will be reseted
// when alarmcondition is not met.
BOOL bCounterAlarm[ 4 ];
BOOL bFreqLowAlarm[ 4 ];
BOOL bFreqHighAlarm[ 4 ];

// Report interval counters
uint8_t counterReports[ 4 ];
uint8_t frequencyReports[ 4 ];
uint8_t measurementReports[ 4 ];

//__EEPROM_DATA(0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88);

// This table translates registers in page 0 to EEPROM locations
const uint8_t reg2eeprom_pg0[] = {
    /* REG0_ACCRA_ZONE                  */          VSCP_EEPROM_END + 0,
    /* REG0_ACCRA_SUBZONE               */          VSCP_EEPROM_END + 1,
    /* REG0_ACCRA_CH0_SUBZONE           */          VSCP_EEPROM_END + 2,
    /* REG0_ACCRA_CH1_SUBZONE           */          VSCP_EEPROM_END + 3,
    /* REG0_ACCRA_CH2_SUBZONE           */          VSCP_EEPROM_END + 4,
    /* REG0_ACCRA_CH3_SUBZONE           */          VSCP_EEPROM_END + 5,
    /* REG0_ACCRA_CONTROL_COUNTER_CH0   */          VSCP_EEPROM_END + 6,
    /* REG0_ACCRA_CONTROL_COUNTER_CH1   */          VSCP_EEPROM_END + 7,
    /* REG0_ACCRA_CONTROL_COUNTER_CH2   */          VSCP_EEPROM_END + 8,
    /* REG0_ACCRA_CONTROL_COUNTER_CH3   */          VSCP_EEPROM_END + 9,
    /* REG0_ACCRA_CONTROL_FREQ_CH0      */          VSCP_EEPROM_END + 10,
    /* REG0_ACCRA_CONTROL_FREQ_CH1      */          VSCP_EEPROM_END + 11, 
    /* REG0_ACCRA_CONTROL_FREQ_CH2      */          VSCP_EEPROM_END + 12, 
    /* REG0_ACCRA_CONTROL_FREQ_CH3      */          VSCP_EEPROM_END + 13,
    /* REG0_ACCRA_RESET_COUNTER_CH0     */          0xff,
    /* REG0_ACCRA_RESET_COUNTER_CH1     */          0xff,
    /* REG0_ACCRA_RESET_COUNTER_CH2     */          0xff,
    /* REG0_ACCRA_RESET_COUNTER_CH3     */          0xff, 
    /* REG0_ACCRA_COUNTER_CH0_0         */          0xff,
    /* REG0_ACCRA_COUNTER_CH0_1         */          0xff,
    /* REG0_ACCRA_COUNTER_CH0_2         */          0xff,
    /* REG0_ACCRA_COUNTER_CH0_3         */          0xff,
    /* REG0_ACCRA_COUNTER_CH1_0         */          0xff,
    /* REG0_ACCRA_COUNTER_CH1_1         */          0xff,
    /* REG0_ACCRA_COUNTER_CH1_2         */          0xff,
    /* REG0_ACCRA_COUNTER_CH1_3         */          0xff,
    /* REG0_ACCRA_COUNTER_CH2_0         */          0xff,
    /* REG0_ACCRA_COUNTER_CH2_1         */          0xff,
    /* REG0_ACCRA_COUNTER_CH2_2         */          0xff,
    /* REG0_ACCRA_COUNTER_CH2_3         */          0xff,
    /* REG0_ACCRA_COUNTER_CH3_0         */          0xff,
    /* REG0_ACCRA_COUNTER_CH3_1         */          0xff,
    /* REG0_ACCRA_COUNTER_CH3_2         */          0xff,
    /* REG0_ACCRA_COUNTER_CH3_3         */          0xff,                                                    
    /* REG0_ACCRA_COUNTER_ALARM_CH0_0   */          VSCP_EEPROM_END + 14, 
    /* REG0_ACCRA_COUNTER_ALARM_CH0_1   */          VSCP_EEPROM_END + 15,
    /* REG0_ACCRA_COUNTER_ALARM_CH0_2   */          VSCP_EEPROM_END + 16,
    /* REG0_ACCRA_COUNTER_ALARM_CH0_3   */          VSCP_EEPROM_END + 17, 
    /* REG0_ACCRA_COUNTER_ALARM_CH1_0   */          VSCP_EEPROM_END + 18, 
    /* REG0_ACCRA_COUNTER_ALARM_CH1_1   */          VSCP_EEPROM_END + 19,
    /* REG0_ACCRA_COUNTER_ALARM_CH1_2   */          VSCP_EEPROM_END + 20,
    /* REG0_ACCRA_COUNTER_ALARM_CH1_3   */          VSCP_EEPROM_END + 21, 
    /* REG0_ACCRA_COUNTER_ALARM_CH2_0   */          VSCP_EEPROM_END + 22, 
    /* REG0_ACCRA_COUNTER_ALARM_CH2_1   */          VSCP_EEPROM_END + 23,
    /* REG0_ACCRA_COUNTER_ALARM_CH2_2   */          VSCP_EEPROM_END + 24,
    /* REG0_ACCRA_COUNTER_ALARM_CH2_3   */          VSCP_EEPROM_END + 25, 
    /* REG0_ACCRA_COUNTER_ALARM_CH3_0   */          VSCP_EEPROM_END + 26, 
    /* REG0_ACCRA_COUNTER_ALARM_CH3_1   */          VSCP_EEPROM_END + 27,
    /* REG0_ACCRA_COUNTER_ALARM_CH3_2   */          VSCP_EEPROM_END + 28,
    /* REG0_ACCRA_COUNTER_ALARM_CH3_3   */          VSCP_EEPROM_END + 29,   
    /* REG0_ACCRA_COUNTER_RELOAD_CH0_0  */          VSCP_EEPROM_END + 30,
    /* REG0_ACCRA_COUNTER_RELOAD_CH0_1  */          VSCP_EEPROM_END + 31,
    /* REG0_ACCRA_COUNTER_RELOAD_CH0_2  */          VSCP_EEPROM_END + 32,
    /* REG0_ACCRA_COUNTER_RELOAD_CH0_3  */          VSCP_EEPROM_END + 33,
    /* REG0_ACCRA_COUNTER_RELOAD_CH1_0  */          VSCP_EEPROM_END + 34,
    /* REG0_ACCRA_COUNTER_RELOAD_CH1_1  */          VSCP_EEPROM_END + 35,
    /* REG0_ACCRA_COUNTER_RELOAD_CH1_2  */          VSCP_EEPROM_END + 36,
    /* REG0_ACCRA_COUNTER_RELOAD_CH1_3  */          VSCP_EEPROM_END + 37,
    /* REG0_ACCRA_COUNTER_RELOAD_CH2_0  */          VSCP_EEPROM_END + 38,
    /* REG0_ACCRA_COUNTER_RELOAD_CH2_1  */          VSCP_EEPROM_END + 39,
    /* REG0_ACCRA_COUNTER_RELOAD_CH2_2  */          VSCP_EEPROM_END + 40,
    /* REG0_ACCRA_COUNTER_RELOAD_CH2_3  */          VSCP_EEPROM_END + 41,
    /* REG0_ACCRA_COUNTER_RELOAD_CH3_0  */          VSCP_EEPROM_END + 42,
    /* REG0_ACCRA_COUNTER_RELOAD_CH3_1  */          VSCP_EEPROM_END + 43,
    /* REG0_ACCRA_COUNTER_RELOAD_CH3_2  */          VSCP_EEPROM_END + 44,
    /* REG0_ACCRA_COUNTER_RELOAD_CH3_3  */          VSCP_EEPROM_END + 45,   
    /* REG0_ACCRA_COUNTER_HYSTERESIS_CH0_MSB  */    VSCP_EEPROM_END + 46,
    /* REG0_ACCRA_COUNTER_HYSTERESIS_CH0_LSB  */    VSCP_EEPROM_END + 47,
    /* REG0_ACCRA_COUNTER_HYSTERESIS_CH1_MSB  */    VSCP_EEPROM_END + 48,
    /* REG0_ACCRA_COUNTER_HYSTERESIS_CH1_LSB  */    VSCP_EEPROM_END + 49,
    /* REG0_ACCRA_COUNTER_HYSTERESIS_CH2_MSB  */    VSCP_EEPROM_END + 50,
    /* REG0_ACCRA_COUNTER_HYSTERESIS_CH2_LSB  */    VSCP_EEPROM_END + 51,
    /* REG0_ACCRA_COUNTER_HYSTERESIS_CH3_MSB  */    VSCP_EEPROM_END + 52,
    /* REG0_ACCRA_COUNTER_HYSTERESIS_CH3_LSB  */    VSCP_EEPROM_END + 53, 
    /* REG0_ACCRA_COUNTER_REPORT_INTERVAL_CH0 */    VSCP_EEPROM_END + 54,
    /* REG0_ACCRA_COUNTER_REPORT_INTERVAL_CH1 */    VSCP_EEPROM_END + 55,
    /* REG0_ACCRA_COUNTER_REPORT_INTERVAL_CH2 */    VSCP_EEPROM_END + 56,
    /* REG0_ACCRA_COUNTER_REPORT_INTERVAL_CH3 */    VSCP_EEPROM_END + 57,  
    /* REG0_ACCRA_FREQ_REPORT_INTERVAL_CH0    */    VSCP_EEPROM_END + 58, 
    /* REG0_ACCRA_FREQ_REPORT_INTERVAL_CH1    */    VSCP_EEPROM_END + 59,
    /* REG0_ACCRA_FREQ_REPORT_INTERVAL_CH2    */    VSCP_EEPROM_END + 60,
    /* REG0_ACCRA_FREQ_REPORT_INTERVAL_CH3    */    VSCP_EEPROM_END + 61,  
    /* REG0_ACCRA_MESURMENT_REPORT_INTERVAL_CH0 */      VSCP_EEPROM_END + 62, 
    /* REG0_ACCRA_MESURMENT_REPORT_INTERVAL_CH1 */      VSCP_EEPROM_END + 63, 
    /* REG0_ACCRA_MESURMENT_REPORT_INTERVAL_CH2 */      VSCP_EEPROM_END + 64, 
    /* REG0_ACCRA_MESURMENT_REPORT_INTERVAL_CH3 */      VSCP_EEPROM_END + 65,
    /* REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH0 */    VSCP_EEPROM_END + 66,
    /* REG0_ACCRA_LINEARIZATION_EVENT_CLASS_CH0 */      VSCP_EEPROM_END + 67,
    /* REG0_ACCRA_LINEARIZATION_EVENT_TYPE_CH0 */       VSCP_EEPROM_END + 68,
    /* REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH1 */    VSCP_EEPROM_END + 69,
    /* REG0_ACCRA_LINEARIZATION_EVENT_CLASS_CH1 */      VSCP_EEPROM_END + 70,
    /* REG0_ACCRA_LINEARIZATION_EVENT_TYPE_CH1 */       VSCP_EEPROM_END + 71,
    /* REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH2 */    VSCP_EEPROM_END + 72,
    /* REG0_ACCRA_LINEARIZATION_EVENT_CLASS_CH2 */      VSCP_EEPROM_END + 73,
    /* REG0_ACCRA_LINEARIZATION_EVENT_TYPE_CH2 */       VSCP_EEPROM_END + 74,
    /* REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH3 */    VSCP_EEPROM_END + 75,
    /* REG0_ACCRA_LINEARIZATION_EVENT_CLASS_CH3 */      VSCP_EEPROM_END + 76,
    /* REG0_ACCRA_LINEARIZATION_EVENT_TYPE_CH3 */       VSCP_EEPROM_END + 77,                                                    
};

// This table translates registers in page 1 to EEPROM locations
const uint8_t reg2eeprom_pg1[] = {
    /* REG1_ACCRA_CH0_FREQUENCY_0 */                    0xff,
    /* REG1_ACCRA_CH0_FREQUENCY_1 */                    0xff,
    /* REG1_ACCRA_CH0_FREQUENCY_2 */                    0xff,
    /* REG1_ACCRA_CH0_FREQUENCY_3 */                    0xff,  
    /* REG1_ACCRA_CH1_FREQUENCY_0 */                    0xff,
    /* REG1_ACCRA_CH1_FREQUENCY_1 */                    0xff,
    /* REG1_ACCRA_CH1_FREQUENCY_2 */                    0xff,
    /* REG1_ACCRA_CH1_FREQUENCY_3 */                    0xff,
    /* REG1_ACCRA_CH2_FREQUENCY_0 */                    0xff,
    /* REG1_ACCRA_CH2_FREQUENCY_1 */                    0xff,
    /* REG1_ACCRA_CH2_FREQUENCY_2 */                    0xff,
    /* REG1_ACCRA_CH2_FREQUENCY_3 */                    0xff,
    /* REG1_ACCRA_CH3_FREQUENCY_0 */                    0xff,
    /* REG1_ACCRA_CH3_FREQUENCY_1 */                    0xff,
    /* REG1_ACCRA_CH3_FREQUENCY_2 */                    0xff,
    /* REG1_ACCRA_CH3_FREQUENCY_3 */                    0xff,
    /* REG1_ACCRA_CH0_FREQUENCY_LOW_0 */                VSCP_EEPROM_END + 78,
    /* REG1_ACCRA_CH0_FREQUENCY_LOW_1 */                VSCP_EEPROM_END + 79,
    /* REG1_ACCRA_CH0_FREQUENCY_LOW_2 */                VSCP_EEPROM_END + 80,
    /* REG1_ACCRA_CH0_FREQUENCY_LOW_3 */                VSCP_EEPROM_END + 81, 
    /* REG1_ACCRA_CH1_FREQUENCY_LOW_0 */                VSCP_EEPROM_END + 82,
    /* REG1_ACCRA_CH1_FREQUENCY_LOW_1 */                VSCP_EEPROM_END + 83,
    /* REG1_ACCRA_CH1_FREQUENCY_LOW_2 */                VSCP_EEPROM_END + 84,
    /* REG1_ACCRA_CH1_FREQUENCY_LOW_3 */                VSCP_EEPROM_END + 85,
    /* REG1_ACCRA_CH2_FREQUENCY_LOW_0 */                VSCP_EEPROM_END + 86,
    /* REG1_ACCRA_CH2_FREQUENCY_LOW_1 */                VSCP_EEPROM_END + 87,
    /* REG1_ACCRA_CH2_FREQUENCY_LOW_2 */                VSCP_EEPROM_END + 88,
    /* REG1_ACCRA_CH2_FREQUENCY_LOW_3 */                VSCP_EEPROM_END + 89,
    /* REG1_ACCRA_CH3_FREQUENCY_HIGH_0 */               VSCP_EEPROM_END + 90,
    /* REG1_ACCRA_CH3_FREQUENCY_HIGH_1 */               VSCP_EEPROM_END + 91,
    /* REG1_ACCRA_CH3_FREQUENCY_HIGH_2 */               VSCP_EEPROM_END + 92,
    /* REG1_ACCRA_CH3_FREQUENCY_HIGH_3 */               VSCP_EEPROM_END + 93, 
    /* REG1_ACCRA_CH0_FREQUENCY_HIGH_0 */               VSCP_EEPROM_END + 94,
    /* REG1_ACCRA_CH0_FREQUENCY_HIGH_1 */               VSCP_EEPROM_END + 95,
    /* REG1_ACCRA_CH0_FREQUENCY_HIGH_2 */               VSCP_EEPROM_END + 96,
    /* REG1_ACCRA_CH0_FREQUENCY_HIGH_3 */               VSCP_EEPROM_END + 97, 
    /* REG1_ACCRA_CH1_FREQUENCY_HIGH_0 */               VSCP_EEPROM_END + 98,
    /* REG1_ACCRA_CH1_FREQUENCY_HIGH_1 */               VSCP_EEPROM_END + 99,
    /* REG1_ACCRA_CH1_FREQUENCY_HIGH_2 */               VSCP_EEPROM_END + 100,
    /* REG1_ACCRA_CH1_FREQUENCY_HIGH_3 */               VSCP_EEPROM_END + 101,
    /* REG1_ACCRA_CH2_FREQUENCY_HIGH_0 */               VSCP_EEPROM_END + 102,
    /* REG1_ACCRA_CH2_FREQUENCY_HIGH_1 */               VSCP_EEPROM_END + 103,
    /* REG1_ACCRA_CH2_FREQUENCY_HIGH_2 */               VSCP_EEPROM_END + 104,
    /* REG1_ACCRA_CH2_FREQUENCY_HIGH_3 */               VSCP_EEPROM_END + 105,
    /* REG1_ACCRA_CH3_FREQUENCY_HIGH_0 */               VSCP_EEPROM_END + 106,
    /* REG1_ACCRA_CH3_FREQUENCY_HIGH_1 */               VSCP_EEPROM_END + 107,
    /* REG1_ACCRA_CH3_FREQUENCY_HIGH_2 */               VSCP_EEPROM_END + 108,
    /* REG1_ACCRA_CH3_FREQUENCY_HIGH_3 */               VSCP_EEPROM_END + 109,
    /* REG1_ACCRA_CH0_FREQ_HYSTERESIS_MSB */            VSCP_EEPROM_END + 110,
    /* REG1_ACCRA_CH0_FREQ_HYSTERESIS_LSB */            VSCP_EEPROM_END + 111, 
    /* REG1_ACCRA_CH1_FREQ_HYSTERESIS_MSB */            VSCP_EEPROM_END + 112,
    /* REG1_ACCRA_CH1_FREQ_HYSTERESIS_LSB */            VSCP_EEPROM_END + 113,
    /* REG1_ACCRA_CH2_FREQ_HYSTERESIS_MSB */            VSCP_EEPROM_END + 114,
    /* REG1_ACCRA_CH2_FREQ_HYSTERESIS_LSB */            VSCP_EEPROM_END + 115,
    /* REG1_ACCRA_CH3_FREQ_HYSTERESIS_MSB */            VSCP_EEPROM_END + 116,
    /* REG1_ACCRA_CH3_FREQ_HYSTERESIS_LSB */            VSCP_EEPROM_END + 117,                                                        
};

// This table translates registers in page 2 to EEPROM locations
const uint8_t reg2eeprom_pg2[] = {
  /* REG2_ACCRA_CH0_LINEARIZATION_K_0 */                VSCP_EEPROM_END + 118, 
  /* REG2_ACCRA_CH0_LINEARIZATION_K_1 */                VSCP_EEPROM_END + 119,
  /* REG2_ACCRA_CH0_LINEARIZATION_K_2 */                VSCP_EEPROM_END + 120,
  /* REG2_ACCRA_CH0_LINEARIZATION_K_3 */                VSCP_EEPROM_END + 121,
  /* REG2_ACCRA_CH1_LINEARIZATION_K_0 */                VSCP_EEPROM_END + 122, 
  /* REG2_ACCRA_CH1_LINEARIZATION_K_1 */                VSCP_EEPROM_END + 123,
  /* REG2_ACCRA_CH1_LINEARIZATION_K_2 */                VSCP_EEPROM_END + 124,
  /* REG2_ACCRA_CH1_LINEARIZATION_K_3 */                VSCP_EEPROM_END + 125,
  /* REG2_ACCRA_CH2_LINEARIZATION_K_0 */                VSCP_EEPROM_END + 126, 
  /* REG2_ACCRA_CH2_LINEARIZATION_K_1 */                VSCP_EEPROM_END + 127,
  /* REG2_ACCRA_CH2_LINEARIZATION_K_2 */                VSCP_EEPROM_END + 128,
  /* REG2_ACCRA_CH2_LINEARIZATION_K_3 */                VSCP_EEPROM_END + 129,
  /* REG2_ACCRA_CH3_LINEARIZATION_K_0 */                VSCP_EEPROM_END + 130, 
  /* REG2_ACCRA_CH3_LINEARIZATION_K_1 */                VSCP_EEPROM_END + 131,
  /* REG2_ACCRA_CH3_LINEARIZATION_K_2 */                VSCP_EEPROM_END + 132,
  /* REG2_ACCRA_CH3_LINEARIZATION_K_3 */                VSCP_EEPROM_END + 133, 
  /* REG2_ACCRA_CH0_LINEARIZATION_M_0 */                VSCP_EEPROM_END + 134,
  /* REG2_ACCRA_CH0_LINEARIZATION_M_1 */                VSCP_EEPROM_END + 135,
  /* REG2_ACCRA_CH0_LINEARIZATION_M_2 */                VSCP_EEPROM_END + 136,
  /* REG2_ACCRA_CH0_LINEARIZATION_M_3 */                VSCP_EEPROM_END + 137, 
  /* REG2_ACCRA_CH1_LINEARIZATION_M_0 */                VSCP_EEPROM_END + 138,
  /* REG2_ACCRA_CH1_LINEARIZATION_M_1 */                VSCP_EEPROM_END + 139,
  /* REG2_ACCRA_CH1_LINEARIZATION_M_2 */                VSCP_EEPROM_END + 140,
  /* REG2_ACCRA_CH1_LINEARIZATION_M_3 */                VSCP_EEPROM_END + 141, 
  /* REG2_ACCRA_CH2_LINEARIZATION_M_0 */                VSCP_EEPROM_END + 142,
  /* REG2_ACCRA_CH2_LINEARIZATION_M_1 */                VSCP_EEPROM_END + 143,
  /* REG2_ACCRA_CH2_LINEARIZATION_M_2 */                VSCP_EEPROM_END + 144,
  /* REG2_ACCRA_CH2_LINEARIZATION_M_3 */                VSCP_EEPROM_END + 145, 
  /* REG2_ACCRA_CH3_LINEARIZATION_M_0 */                VSCP_EEPROM_END + 146,
  /* REG2_ACCRA_CH3_LINEARIZATION_M_1 */                VSCP_EEPROM_END + 147,
  /* REG2_ACCRA_CH3_LINEARIZATION_M_2 */                VSCP_EEPROM_END + 148,
  /* REG2_ACCRA_CH3_LINEARIZATION_M_3 */                VSCP_EEPROM_END + 149,                                                         
};

#define DECISION_MATRIX_EEPROM_START                    VSCP_EEPROM_END + 150     // always one after above

///////////////////////////////////////////////////////////////////////////////
// Isr() 	- Interrupt Service Routine
//      	- Services Timer0 Overflow
//      	- Services GP3 Pin Change
//////////////////////////////////////////////////////////////////////////////

void interrupt low_priority  interrupt_at_low_vector( void )
{
    // Clock
    if ( INTCONbits.TMR0IF ) { // If a Timer0 Interrupt, Then...

        // Reload value for 1 ms resolution
        WriteTimer0(TIMER0_RELOAD_VALUE);
        
        vscp_timer++;
        vscp_configtimer++;
        measurement_clock_sec++;
        sendTimer++;

        // Check for init. button
        if ( INIT_BUTTON ) {
            vscp_initbtncnt = 0;
        } 
        else {
            // Active
            vscp_initbtncnt++;
        }

        // Status LED
        vscp_statuscnt++;
        if ( ( VSCP_LED_BLINK1 == vscp_initledfunc ) &&
                ( vscp_statuscnt > 100 ) ) {

            if ( STATUS_LED ) {
                STATUS_LED = 0;
            }
            else {
                STATUS_LED = 1;
            }

            vscp_statuscnt = 0;

        }
        else if (VSCP_LED_ON == vscp_initledfunc) {
            STATUS_LED = 1;
            vscp_statuscnt = 0;
        }
        else if (VSCP_LED_OFF == vscp_initledfunc) {
            STATUS_LED = 0;
            vscp_statuscnt = 0;
        }

        INTCONbits.TMR0IF = 0; // Clear Timer0 Interrupt Flag

    }

    return;
}

///////////////////////////////////////////////////////////////////////////////
// Isr() 	- Interrupt Service Routine
//      	- External interrupt 
//      	
//////////////////////////////////////////////////////////////////////////////

void interrupt high_priority  interrupt_at_high_vector( void )
{
    if ( INTCONbits.INT0IF ) {  // External interrupt 0
        if ( counter_control_shadow[ 0 ] & COUNTER_CTRL_ENABLED ) {
            if ( !( counter_control_shadow[ 0 ] & COUNTER_CTRL_DIRECTION ) ) {
                counter[ 0 ]++;
                if ( !counter[ 0 ] ) {
                    bReloadCounter0 = TRUE;
                }
            }
            else {
                counter[ 0 ]--;
                if ( !counter[ 0 ] ) {
                    bReloadCounter0 = TRUE;
                }
            }
        }
        INTCONbits.INT0IF = 0;  // Clear flag
    }
    else if ( INTCON3bits.INT1IF ) { // External interrupt 1
        if ( counter_control_shadow[ 1 ] & COUNTER_CTRL_ENABLED ) {
            if ( !( counter_control_shadow[ 1 ] & COUNTER_CTRL_DIRECTION )  ) {
                counter[ 1 ]++;
                if ( !counter[ 1 ] ) {
                    bReloadCounter1 = TRUE;
                }
            }
            else {
                counter[ 1 ]--;
                if ( !counter[ 1 ] ) {
                    bReloadCounter1 = TRUE;
                }
            }
        }
        INTCON3bits.INT1IF = 0;  // clear flag
    }
    
    return;
}


//***************************************************************************
// Main() - Main Routine
//***************************************************************************


void main()
{
    init();         // Initialize Micro controller
    
    // Check VSCP persistent storage and
    // restore if needed
    if ( !vscp_check_pstorage() ) {

        // Spoiled or not initialized - reinitialize
        init_app_eeprom();

    }
    
    // Initialize data
    init_app_ram();
    
    // Set up counter shadow registers
    counter_control_shadow[ 0 ] = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_CONTROL_COUNTER_CH0 ] );
    counter_control_shadow[ 1 ] = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_CONTROL_COUNTER_CH1 ] );
    counter_control_shadow[ 2 ] = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_CONTROL_COUNTER_CH2 ] );
    counter_control_shadow[ 3 ] = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_CONTROL_COUNTER_CH3 ] );
    
    // Initialize the VSCP functionality
    vscp_init();    
    
    // Set DM filters
    calculateSetFilterMask();
            
    while ( 1 ) {   // Loop Forever

        ClrWdt();   // Feed the dog

        if ( ( vscp_initbtncnt > 250 ) &&
                ( VSCP_STATE_INIT != vscp_node_state ) ) {

            // Init. button pressed
            vscp_nickname = VSCP_ADDRESS_FREE;
            eeprom_write( VSCP_EEPROM_NICKNAME, VSCP_ADDRESS_FREE );
            vscp_init();
            
        }

        // Check for a valid  event
        vscp_imsg.flags = 0;
        vscp_getEvent();

        switch ( vscp_node_state ) {

            case VSCP_STATE_STARTUP: // Cold/warm reset

                // Get nickname from EEPROM
                if (VSCP_ADDRESS_FREE == vscp_nickname) {
                    // new on segment need a nickname
                    vscp_node_state = VSCP_STATE_INIT;
                } 
                else {
                    // been here before - go on
                    vscp_node_state = VSCP_STATE_ACTIVE;
                    vscp_goActiveState();
                }
                break;

            case VSCP_STATE_INIT: // Assigning nickname
                vscp_handleProbeState();
                break;

            case VSCP_STATE_PREACTIVE:  // Waiting for host initialization
                vscp_goActiveState();
                break;

            case VSCP_STATE_ACTIVE:     // The normal state

                // Check for incoming event?
                if (vscp_imsg.flags & VSCP_VALID_MSG) {

                    if ( VSCP_CLASS1_PROTOCOL == vscp_imsg.vscp_class  ) {

                        // Handle protocol event
                        vscp_handleProtocolEvent();

                    }
                    else {    
                        doDM();
					}
                    
                }               
                break;

            case VSCP_STATE_ERROR: // Everything is *very* *very* bad.
                vscp_error();
                break;

            default: // Should not be here...
                vscp_node_state = VSCP_STATE_STARTUP;
                break;

        }

        // do a measurement if needed
        if ( measurement_clock_sec > 1000 ) {

            measurement_clock_sec = 0;
 
            // Do VSCP one second jobs
            vscp_doOneSecondWork();

            if ( VSCP_STATE_ACTIVE == vscp_node_state ) {
                // Do VSCP one second jobs
                doApplicationOneSecondWork();
            }

        }

        // Timekeeping
        if ( seconds > 59 ) {

            seconds = 0;
            minutes++;

            if ( minutes > 59 ) {
                minutes = 0;
                hours++;
            }

            if ( hours > 23 ) hours = 0;

        }

        // Do loop work
        doWork();

    } // while
}

///////////////////////////////////////////////////////////////////////////////
// Init. - Initialization Routine
//  

void init()
{   
    //uint8_t msgdata[ 8 ];

    // Initialize the uP
    
    // Deselect comparators and ADC's so PORTA/PORTB
    // can be used as inputs.
    ADCON1 = 0x0f;
    //CMCON = 0x07;     // Not implemented on PIC18F2580

    // PORTA
    // RA0/AN0  - output
    // RA1/AN1  - output
    // RA2/AN2  - output
    // RA3/AN3  - output
    // RA4      - output
    // RA5/AN4  - output
    TRISA = 0b00000000;
    PORTA = 0x00;  // Default off

    // PortB

    // RB0/AN10     - Channel 0
    // RB1/AN8      - Channel 1
    // RB2 CAN TX   - output
    // RB3 CAN RX   - input
    // RB4/AN9      - Not used = output.
    // RB5/LVPGM    - Not used = output.
    // RB&/PGC      - Not used = output.
    // RB7/PGD      - Not used = output.
    TRISB = 0b00001011;
    PORTB = 0x00;  // Default off

    // RC0 - Input  - Init. button
    // RC1 - Output - Status LED - Default off
    // RC2 - Input  - Channel 2.
    // RC3 - Input  - Channel 3.
    // RC4 - Output - Not used = output.
    // RC5 - Output - Not used = output.
    // RC6 - Output - Not used = output.
    // RC7 - Output - Not used = output.
    TRISC = 0b00001101;
    PORTC = 0x00;   // Default off

/*
    // Sensor 0 timer
    OpenTimer0( TIMER_INT_OFF &
                    T0_16BIT &
                    T0_PS_1_2 &
                    T0_SOURCE_INT);

    // Sensor 1 timer
    OpenTimer1( TIMER_INT_OFF &
                    T1_16BIT_RW &
                    T1_SOURCE_INT &
                    T1_PS_1_1 &
                    T1_OSC1EN_OFF &
                    T1_SYNC_EXT_OFF);

    // Timer 2 is used as a 1 ms clock
    // 156 is loaded eight time to give ~1250 cycles
    PR2 = TIMER2_RELOAD_VALUE; // Set reload value
    OpenTimer2( TIMER_INT_ON &
                    T2_PS_1_4 &
                    T2_POST_1_16 );

 */

    OpenTimer0( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_8 );
    WriteTimer0( TIMER0_RELOAD_VALUE );

    // Initialize CAN
    ECANInitialize();

    // Must be in config. mode to change many of settings.
    //ECANSetOperationMode(ECAN_OP_MODE_CONFIG);

    // Return to Normal mode to communicate.
    //ECANSetOperationMode(ECAN_OP_MODE_NORMAL);

    /*
        msgdata[ 0 ] = 1;
            msgdata[ 1 ] = 2;
            msgdata[ 2 ] = 3;
            msgdata[ 3 ] = 4;

            if ( vscp18f_sendMsg( 0x123,  msgdata, 4, CAN_TX_PRIORITY_0 & CAN_TX_XTD_FRAME & CAN_TX_NO_RTR_FRAME ) ) {
                    ;
            }

     */

    INTCON2bits.TMR0IP = 0;         // Timer 0 low priority
    
    RCONbits.IPEN = 1;          // Enable interrupt priority
    
    
    // External interrupt 0 (counter 0)
    INTCON2bits.INTEDG0 = 1;    // Interrupt on rising edge
    INTCONbits.INT0IE = 1;      // Enable external interrupt 0
    
    // External interrupt 1 (counter 1)
    INTCON2bits.INTEDG1 = 1;    // Interrupt on rising edge
    INTCON3bits.INT1IE = 1;     // Enable external interrupt 0
    
    // Enable peripheral interrupt
    INTCONbits.PEIE = 1;
    
    // Enable low priority interrupt
    //INTCONbits.GIE = 1;
    
    INTCONbits.GIE_GIEH = 1;    // high priority interrupts are enabled
    INTCONbits.PEIE_GIEL = 1;   // low priority interrupts are enabled

    return;
}

///////////////////////////////////////////////////////////////////////////////
// init_app_ram
//

void init_app_ram( void )
{
    uint8_t i;
    
    measurement_clock_sec = 0;      // start a new measurement cycle

    seconds = 0;
    minutes = 0;
    hours = 0;
    
    counter2LastState = 0;          // Expect signal to be low
    counter3LastState = 0;          // Expect signal to be low
    
    for ( i=0; i<4; i++ ) {
        counter[ i ] = 0;
        frequency[ i ] = 0;
    }
   
    // No counter reloads yet
    bReloadCounter0 = FALSE;
    bReloadCounter1 = FALSE;
    
    // Reset alarm flags
    memset( bCounterAlarm, 0, sizeof( bCounterAlarm ) );
    memset( bFreqLowAlarm, 0,  sizeof( bFreqLowAlarm ) );
    memset( bFreqHighAlarm, 0,  sizeof( bFreqHighAlarm ) );
    
    // Reset report counters
    memset( counterReports, 0, sizeof( counterReports ) );
    memset( frequencyReports, 0,  sizeof( frequencyReports ) );
    memset( measurementReports, 0,  sizeof( measurementReports ) );
    
}


///////////////////////////////////////////////////////////////////////////////
// init_app_eeprom
//

void init_app_eeprom(void)
{
    unsigned char i, j;

    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_ZONE ], 0 );
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_SUBZONE ], 0 );
        
    for ( i=0; i<4; i++ ) {        
        eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_CH0_SUBZONE + i ], i );
        eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_CONTROL_COUNTER_CH0 + i ], COUNTER_CTRL_ENABLED );        
        eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_CONTROL_FREQ_CH0 + i ], FREQ_CTRL_ENABLE );        
        eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_ALARM_CH0_MSB + i], 0 );        
        eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_ALARM_CH1_MSB + i ], 0 );        
        eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_ALARM_CH2_MSB + i ], 0 );        
        eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_ALARM_CH3_MSB + i ], 0 );        
        eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH0_MSB + i ], 0 );        
        eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH1_MSB + i ], 0 );
        eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH2_MSB + i ], 0 );
        eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH3_MSB + i ], 0 );
        eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_REPORT_INTERVAL_CH0 + i ], 0 );
        eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_FREQ_REPORT_INTERVAL_CH0 + i ], 0 );
        eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_MESURMENT_REPORT_INTERVAL_CH0 + i ], 0 );
    }
    
    // Counter hysteresis
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_HYSTERESIS_CH0_MSB ],
                        ( DEFAULT_COUNTER0_HYSTERESIS >> 8 ) & 0xff );
        
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_HYSTERESIS_CH0_LSB ],
                        DEFAULT_COUNTER0_HYSTERESIS & 0xff );
    
    eeprom_write( reg2eeprom_pg0[REG0_ACCRA_COUNTER_HYSTERESIS_CH1_MSB ],
                        ( DEFAULT_COUNTER1_HYSTERESIS >> 8 ) & 0xff );
        
    eeprom_write( reg2eeprom_pg0[REG0_ACCRA_COUNTER_HYSTERESIS_CH1_LSB ],
                        DEFAULT_COUNTER1_HYSTERESIS & 0xff );
    
    eeprom_write( reg2eeprom_pg0[REG0_ACCRA_COUNTER_HYSTERESIS_CH2_MSB ],
                        ( DEFAULT_COUNTER2_HYSTERESIS >> 8 ) & 0xff );
        
    eeprom_write( reg2eeprom_pg0[REG0_ACCRA_COUNTER_HYSTERESIS_CH2_LSB ],
                        DEFAULT_COUNTER2_HYSTERESIS & 0xff );
    
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_HYSTERESIS_CH3_MSB ],
                        ( DEFAULT_COUNTER3_HYSTERESIS >> 8 ) & 0xff );
        
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_HYSTERESIS_CH3_LSB ],
                        DEFAULT_COUNTER0_HYSTERESIS & 0xff );
    
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH0 ], 0 );
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_CLASS_CH0 ], 0 );
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_TYPE_CH0 ], 0 );
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH1 ], 0 );
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_CLASS_CH1 ], 0 );
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_TYPE_CH1 ], 0 );
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH2 ], 0 );
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_CLASS_CH2 ], 0 );
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_TYPE_CH2 ], 0 );
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH3 ], 0 );
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_CLASS_CH3 ], 0 );
    eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_TYPE_CH3 ], 0 );

    // Page 1
    
    // Frequency limits
    for ( int i=0; i<(4*8); i++ ) {
        eeprom_write( reg2eeprom_pg1[ REG1_ACCRA_CH0_FREQUENCY_LOW_MSB ] + i, 0 );
    }
    
    // Hysteresis channel 0
    eeprom_write( reg2eeprom_pg1[ REG1_ACCRA_CH0_FREQ_HYSTERESIS_MSB ],
                            ( DEFAULT_FREQUENCY0_HYSTERESIS >> 8 ) & 0xff );
    
    eeprom_write( reg2eeprom_pg1[ REG1_ACCRA_CH0_FREQ_HYSTERESIS_LSB ],
                            DEFAULT_FREQUENCY0_HYSTERESIS & 0xff );
    
    // Hysteresis channel 1
    eeprom_write( reg2eeprom_pg1[ REG1_ACCRA_CH1_FREQ_HYSTERESIS_MSB ],
                            ( DEFAULT_FREQUENCY1_HYSTERESIS >> 8 ) & 0xff  );
    
    eeprom_write( reg2eeprom_pg1[ REG1_ACCRA_CH1_FREQ_HYSTERESIS_LSB ],
                            DEFAULT_FREQUENCY1_HYSTERESIS & 0xff );
    
    // Hysteresis channel 2
    eeprom_write( reg2eeprom_pg1[ REG1_ACCRA_CH2_FREQ_HYSTERESIS_MSB ],
                            ( DEFAULT_FREQUENCY2_HYSTERESIS >> 8 ) & 0xff  );
    
    eeprom_write( reg2eeprom_pg1[ REG1_ACCRA_CH2_FREQ_HYSTERESIS_LSB ],
                            DEFAULT_FREQUENCY2_HYSTERESIS & 0xff );
    
    // Hysteresis channel 3
    eeprom_write( reg2eeprom_pg1[ REG1_ACCRA_CH3_FREQ_HYSTERESIS_MSB ],
                            ( DEFAULT_FREQUENCY3_HYSTERESIS >> 8 ) & 0xff  );
    
    eeprom_write( reg2eeprom_pg1[ REG1_ACCRA_CH3_FREQ_HYSTERESIS_LSB ],
                            DEFAULT_FREQUENCY3_HYSTERESIS & 0xff );
    
    
    // * * *  Page 2 * * *
    
    for ( i=0; i<(4*8); i++ ) {
        eeprom_write( reg2eeprom_pg2[ REG2_ACCRA_CH0_LINEARIZATION_K_MSB + i ], 0 );
    }
    
    // * * * Decision Matrix * * *
    
    // All elements disabled.
    for ( i = 0; i < DESCION_MATRIX_ROWS; i++ ) {
        for ( j = 0; j < 8; j++ ) {
            eeprom_write( DECISION_MATRIX_EEPROM_START + i * 8 + j, 0 );
        }
    }
    
    // Set DM filters
    calculateSetFilterMask();

}

///////////////////////////////////////////////////////////////////////////////
// doWork
//
// The actual work is done here.
//

void doWork(void)
{
    // Check for zero and if a reload is needed
    if ( bReloadCounter0 && 
            ( counter_control_shadow[ 0 ] & COUNTER_CTRL_RELOAD_ZERO ) ) {
        counter[ 0 ] = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH0_0 ] ) << 24 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH0_1 ] ) << 16 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH0_2 ] ) << 8 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH0_3 ] );
        bReloadCounter0 = 0;
    }
    
    // Check for zero and if a reload is needed
    if ( bReloadCounter1 && 
            ( counter_control_shadow[ 1 ] & COUNTER_CTRL_RELOAD_ZERO ) ) {
        counter[ 1 ] = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH1_0 ] ) << 24 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH1_1 ] ) << 16 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH1_2 ] ) << 8 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH1_3 ] );
        bReloadCounter1 = 0;
    }
    // Check if counter2 channel changed state
    if ( PORTCbits.RC2 != counter2LastState ) {
        
        // Save new value
        counter2LastState = PORTCbits.RC2;
        
        // If high we count
        if ( PORTCbits.RC2 && ( counter_control_shadow[ 2 ] & COUNTER_CTRL_ENABLED ) ) {
            
            if ( !( counter_control_shadow[ 2 ] & COUNTER_CTRL_DIRECTION )  ) {
                counter[ 2 ]++;
            }
            else {
                counter[ 2 ]--;
            }
            
            // Check for zero and if a reload is needed
            if ( !counter[ 2 ] && 
                        ( counter_control_shadow[ 2 ] & COUNTER_CTRL_RELOAD_ZERO ) ) {
                    counter[ 2 ] = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH2_0 ] ) << 24 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH2_1 ] ) << 16 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH2_2 ] ) << 8 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH2_3 ] );
                }
            
        }
    }
    
    // Check if counter3 channel changed state
    if ( PORTCbits.RC3 != counter3LastState ) {
        
        // Save new value
        counter3LastState = PORTCbits.RC3;
        
        // If high we count
        if ( PORTCbits.RC3 && ( counter_control_shadow[ 3 ] & COUNTER_CTRL_ENABLED ) ) {
            if ( !( counter_control_shadow[ 3 ] & COUNTER_CTRL_DIRECTION )  ) {
                counter[ 3 ]++;                
            }
            else {
                counter[ 3 ]--;
            }
            
            // Check for zero and if a reload is needed
            if ( !counter[ 3 ] && 
                        ( counter_control_shadow[ 3 ] & COUNTER_CTRL_RELOAD_ZERO ) ) {
                    counter[ 3 ] = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH3_0 ] ) << 24 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH3_1 ] ) << 16 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH3_2 ] ) << 8 +
                                        eeprom_read( reg2eeprom_pg0[REG0_ACCRA_COUNTER_RELOAD_CH3_3 ] );
            }
            
        }
    }
    
    
    if ( VSCP_STATE_ACTIVE == vscp_node_state ) {
		;
    }
    
}

///////////////////////////////////////////////////////////////////////////////
// doApplicationOneSecondWork
//

void doApplicationOneSecondWork(void)
{
    uint8_t data[8];
    uint32_t alarmvalue;
    uint16_t hysteresis;
    double val;
    int i;
     
    // calculate frequency
    for ( i=0; i<4; i++ ) {
        frequency[ i ] = counter[ i ] - lastcounter[ i ];
        lastcounter[ i ] = counter[ i ];
    }
    
    // Check for counter alarm
    for ( i=0; i<4; i++ ) {
        
        // Must be enabled
        if ( !( counter_control_shadow[ i ] & COUNTER_CTRL_ENABLED ) ) continue;
        
        if ( counter_control_shadow[ i ] & COUNTER_CTRL_ALARM ) {    
        
            alarmvalue = ( (uint32_t)eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_ALARM_CH0_0 + (4*i) ] ) << 24 ) +
                            ( (uint32_t)eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_ALARM_CH0_1 + (4*i) ] ) << 16 ) +
                            ( (uint32_t)eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_ALARM_CH0_2 + (4*i) ] ) << 8 ) +
                            (uint32_t)eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_ALARM_CH0_3 + (4*i) ] );
            hysteresis = ( eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_HYSTERESIS_CH0_MSB + (4*i) ] ) << 8 ) +
                            eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_HYSTERESIS_CH0_LSB + (4*i) ] );
        
            // if value is over limit and no alarm yet
            if ( ( counter[ i ] > alarmvalue ) && !bCounterAlarm[ i ] ) {
            
                vscp_alarmstatus |= ALARM_COUNTER0 + i; // Flag that alarm occurred
                bCounterAlarm[ i ] = TRUE;
            
                // Send alarm event
                SendAlarmEvent( i );
            
            }
        
        }
        
        // Check if alarm condition is over
        if ( bCounterAlarm[ i ] && ( counter[ i ] < ( alarmvalue - hysteresis ) ) ) {
            bCounterAlarm[ i ] = FALSE;
        }
        
    }
    
    // Check for low frequency alarm 
    for ( i=0; i<4; i++ ) {
    
        // Must be enabled
        if ( !( eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_CONTROL_FREQ_CH0 + i ] ) & FREQ_CTRL_ENABLE ) ) continue;
        
        if ( !bFreqLowAlarm[ i ] && eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_CONTROL_FREQ_CH0 + i ] ) & FREQ_CTRL_LOW_ALARM ) {
        
            alarmvalue = ( (uint32_t)eeprom_read( reg2eeprom_pg1[ REG1_ACCRA_CH0_FREQUENCY_LOW_0 + (4*i) ] ) << 24 ) +
                            ( (uint32_t)eeprom_read( reg2eeprom_pg1[ REG1_ACCRA_CH0_FREQUENCY_LOW_1 + (4*i) ] ) << 16 ) +
                            ( (uint32_t)eeprom_read( reg2eeprom_pg1[ REG1_ACCRA_CH0_FREQUENCY_LOW_2 + (4*i) ] ) << 8 ) +
                            (uint32_t)eeprom_read( reg2eeprom_pg1[ REG1_ACCRA_CH0_FREQUENCY_LOW_3 + (4*i) ] );
            hysteresis = ( eeprom_read( reg2eeprom_pg1[ REG1_ACCRA_CH0_FREQ_HYSTERESIS_MSB + (4*i) ] ) << 8 ) +
                            eeprom_read( reg2eeprom_pg1[ REG1_ACCRA_CH0_FREQ_HYSTERESIS_LSB + (4*i) ] );
        
            // if value is over limit and no alarm yet
            if ( frequency[ i ] < alarmvalue ) {
            
                vscp_alarmstatus |= ALARM_COUNTER0 + i; // Flag that alarm occurred
                bFreqLowAlarm[ i ] = TRUE;
            
                // Send alarm event
                SendAlarmEvent( i + 3 );
            
            }
        
        }
        
        // Check if alarm condition is over
        if ( bFreqLowAlarm[ i ] && ( frequency[ i ] > ( alarmvalue + hysteresis ) ) ) {
            bFreqLowAlarm[ i ] = FALSE;
        }
        
    }
    
    // Check for high frequency alarm 
    for ( i=0; i<4; i++ ) {
    
        // Must be enabled
        if ( !( eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_CONTROL_FREQ_CH0 + i ] ) & FREQ_CTRL_ENABLE ) ) continue;
        
        if ( !bFreqHighAlarm[ i ] && eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_CONTROL_FREQ_CH0 + i ] ) & FREQ_CTRL_HIGH_ALARM ) {
        
            alarmvalue = ( (uint32_t)eeprom_read( reg2eeprom_pg1[ REG1_ACCRA_CH0_FREQUENCY_HIGH_0 + (4*i) ] ) << 24 ) +
                            ( (uint32_t)eeprom_read( reg2eeprom_pg1[ REG1_ACCRA_CH0_FREQUENCY_HIGH_1 + (4*i) ] ) << 16 ) +
                            ( (uint32_t)eeprom_read( reg2eeprom_pg1[ REG1_ACCRA_CH0_FREQUENCY_HIGH_2 + (4*i) ] ) << 8 ) +
                            (uint32_t)eeprom_read( reg2eeprom_pg1[ REG1_ACCRA_CH0_FREQUENCY_HIGH_3 + (4*i) ] );
            hysteresis = ( eeprom_read( reg2eeprom_pg1[ REG1_ACCRA_CH0_FREQ_HYSTERESIS_MSB + (4*i) ] ) << 8 ) +
                            eeprom_read( reg2eeprom_pg1[ REG1_ACCRA_CH0_FREQ_HYSTERESIS_LSB + (4*i) ] );
        
            // if value is over limit and no alarm yet
            if ( ( frequency[ i ] > alarmvalue )  ) {
            
                vscp_alarmstatus |= ALARM_FREQUENCY0 + i; // Flag that alarm occurred
                bFreqHighAlarm[ i ] = TRUE;
            
                // Send alarm event
                SendAlarmEvent( i + 6 );
            
            }
        
        }
        
        // Check if alarm condition is over
        if ( bFreqHighAlarm[ i ] && ( frequency[ i ] < ( alarmvalue - hysteresis ) ) ) {
            bFreqHighAlarm[ i ] = FALSE;
        }
        
    }
    
    
    // Check if counter stream event should be sent 
    for ( i=0; i<4; i++ ) {
        
        // Must be enabled
        if ( !( counter_control_shadow[ i ] & COUNTER_CTRL_ENABLED ) ) continue;
                
        // Interval must be non zero
        if ( !eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_REPORT_INTERVAL_CH0 + i ] ) ) continue;

        counterReports[ i ]++;
        
        if ( counterReports[ i ] > eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_REPORT_INTERVAL_CH0 + i ] ) ) {
            
            counterReports[ i ] = 0;
            
            // Send counter report
            
            data[ 0 ] = 0b01100000 + i; // Data coding: Integer, no unit, channel as index
            
            // Send counter
            data[ 4 ] = counter[ i ] & 0xff;
            data[ 3 ] = ( counter[ i ] >> 8 ) & 0xff;
            data[ 2 ] = ( counter[ i ] >> 16 ) & 0xff;
            data[ 1 ] = ( counter[ i ] >> 24 ) & 0xff;    
            
            sendVSCPFrame( VSCP_CLASS1_DATA,
                                VSCP_TYPE_DATA_COUNT,
                                vscp_nickname,
                                VSCP_PRIORITY_NORMAL,
                                5,
                                data );
            
        }
        
    }
    
    
    // Check if frequency stream event should be sent 
    for ( i=0; i<4; i++ ) {
        
        // Must be enabled
        if ( !( eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_CONTROL_FREQ_CH0 + i ] ) & COUNTER_CTRL_ENABLED ) ) continue;
        
        // Interval must be non zero
        if ( !eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_FREQ_REPORT_INTERVAL_CH0 + i ] ) ) continue;
        
        frequencyReports[ i ]++;
        
        if ( frequencyReports[ i ] > eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_FREQ_REPORT_INTERVAL_CH0 + i ] ) ) {
            
            frequencyReports[ i ] = 0;
            
            // Send frequency report
            
            data[ 0 ] = 0b01100000 + i; // Data coding: Integer, unit = Hz, channel as index
            
            // Send counter
            data[ 4 ] = frequency[ i ] & 0xff;
            data[ 3 ] = ( frequency[ i ] >> 8 ) & 0xff;
            data[ 2 ] = ( frequency[ i ] >> 16 ) & 0xff;
            data[ 1 ] = ( frequency[ i ] >> 24 ) & 0xff;    
            
            sendVSCPFrame( VSCP_CLASS1_MEASUREMENT,
                                VSCP_TYPE_MEASUREMENT_FREQUENCY,
                                vscp_nickname,
                                VSCP_PRIORITY_NORMAL,
                                5,
                                data );
            
        }
        
    }
    
    
    // Check if measurement stream event should be sent 
    for ( i=0; i<4; i++ ) {
        
        // Must be enabled
        if ( !( eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH0 + 3*i ] ) & 
                                                MEASUREMENT_CTRL_ENABLED ) ) continue;
        
        // Interval must be non zero
        if ( !eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_MESURMENT_REPORT_INTERVAL_CH0 + i ] ) ) continue;
        
        measurementReports[ i ]++;
        
        if ( measurementReports[ i ] > eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_MESURMENT_REPORT_INTERVAL_CH0 + i ] ) ) {
            
            measurementReports[ i ] = 0;
            
            uint8_t floatBuf[4];
            
            floatBuf[ 0 ] = eeprom_read( reg2eeprom_pg2[ REG2_ACCRA_CH0_LINEARIZATION_K_3 + 4*i ] );
            floatBuf[ 1 ] = eeprom_read( reg2eeprom_pg2[ REG2_ACCRA_CH0_LINEARIZATION_K_2 + 4*i ] );
            floatBuf[ 2 ] = eeprom_read( reg2eeprom_pg2[ REG2_ACCRA_CH0_LINEARIZATION_K_1 + 4*i ] );
            floatBuf[ 3 ] = eeprom_read( reg2eeprom_pg2[ REG2_ACCRA_CH0_LINEARIZATION_K_0 + 4*i ] );
            double k = *((double *)floatBuf);
            
            floatBuf[ 0 ] = eeprom_read( reg2eeprom_pg2[ REG2_ACCRA_CH0_LINEARIZATION_M_3 + 4*i ] );
            floatBuf[ 1 ] = eeprom_read( reg2eeprom_pg2[ REG2_ACCRA_CH0_LINEARIZATION_M_2 + 4*i ] );
            floatBuf[ 2 ] = eeprom_read( reg2eeprom_pg2[ REG2_ACCRA_CH0_LINEARIZATION_M_1 + 4*i ] );
            floatBuf[ 3 ] = eeprom_read( reg2eeprom_pg2[ REG2_ACCRA_CH0_LINEARIZATION_M_0 + 4*i ] );
            double m = *((double *)floatBuf);
            
            /*
            double k = (double)eeprom_read( reg2eeprom_pg2[REG2_ACCRA_CH0_LINEARIZATION_K_0 + 4*i ] ) +
                            (double)( (uint32_t)eeprom_read( reg2eeprom_pg2[ REG2_ACCRA_CH0_LINEARIZATION_K_1 + 4*i ] ) << 8 ) +
                            (double)( (uint32_t)eeprom_read( reg2eeprom_pg2[ REG2_ACCRA_CH0_LINEARIZATION_K_2 + 4*i ] ) << 16 ) +
                            (double)( (uint32_t)eeprom_read( reg2eeprom_pg2[ REG2_ACCRA_CH0_LINEARIZATION_K_3 + 4*i ] ) << 24 );
            double m = (double)eeprom_read( reg2eeprom_pg2[ REG2_ACCRA_CH0_LINEARIZATION_M_MSB + 4*i ] ) +
                            (double)( (uint32_t)eeprom_read( reg2eeprom_pg2[ REG2_ACCRA_CH0_LINEARIZATION_M_MSB + 4*i ] ) << 8 ) +
                            (double)( (uint32_t)eeprom_read( reg2eeprom_pg2[ REG2_ACCRA_CH0_LINEARIZATION_M_MSB + 4*i ] ) << 16 ) +
                            (double)( (uint32_t)eeprom_read( reg2eeprom_pg2[ REG2_ACCRA_CH0_LINEARIZATION_M_MSB + 4*i ] ) << 24 );
            */
            
            // Send measurement report
            if ( eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH0 + 3*i ] ) & 
                                                    MEASUREMENT_CTRL_CALCBASE ) {
            
                // use frequency for the calculation
                
                // Floating point value 0v10100000
                // unit from control settings (bits 3,4)
                // Channel as index
                data[ 0 ] = 
                    ( 0b10100000 | ( eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH0 + 3*i ] ) & 
                                                        MEASUREMENT_CTRL_UNIT_MASK ) ) + i; 
                
                // Do calculation
                
                val =  k*frequency[ i ] + m;
                
                   
            
            }
            else {
                
                // use the counter for the calculation
                
                data[ 0 ] = 
                    ( 0b10100000 | ( eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH0 + 3*i ] ) & 
                                                        MEASUREMENT_CTRL_UNIT_MASK ) ) + i; 
                
                // Do calculation
                
                val =  k*counter[ i ] + m;
                
            }
            
            // Set data
            uint8_t *p = (uint8_t *)&val;
            data[ 4 ] = p[ 3 ];
            data[ 3 ] = p[ 2 ];
            data[ 2 ] = p[ 1 ];
            data[ 1 ] = p[ 0 ];
            
  
            uint16_t vscpclass = (( eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_SETTING_CH0 + 3*i ] ) & MEASUREMENT_CTRL_CLASS_BIT_8 ) ? 512 : 0 ) +
                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_CLASS_CH0 + 3*i ] );
            uint8_t vscptype = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_LINEARIZATION_EVENT_TYPE_CH0 + 3*i ] );
                    
            sendVSCPFrame( vscpclass,
                                vscptype,
                                vscp_nickname,
                                VSCP_PRIORITY_NORMAL,
                                5,
                                data );
            
        }
        
    }
    
}

///////////////////////////////////////////////////////////////////////////////
// Get Major version number for this hardware module
//

unsigned char getMajorVersion()
{
    return FIRMWARE_MAJOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get Minor version number for this hardware module
//

unsigned char getMinorVersion()
{
    return FIRMWARE_MINOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get Subminor version number for this hardware module
//

unsigned char getSubMinorVersion()
{
    return FIRMWARE_SUB_MINOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get GUID from EEPROM
//

#ifdef ENABLE_WRITE_2PROTECTED_LOCATIONS

void vscp_setGUID(uint8_t idx, uint8_t data ) {
    if ( idx>15 ) return;
    eeprom_write(VSCP_EEPROM_REG_GUID + idx, data);
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Get Manufacturer id and subid from EEPROM
//

#ifdef ENABLE_WRITE_2PROTECTED_LOCATIONS

void vscp_setManufacturerId( uint8_t idx, uint8_t data ) {
    if ( idx>7 ) return;
    eeprom_write(VSCP_EEPROM_REG_MANUFACTUR_ID0 + idx, data);
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Get the bootloader algorithm code
//

unsigned char getBootLoaderAlgorithm(void)
{
    return VSCP_BOOTLOADER_PIC1;
}

///////////////////////////////////////////////////////////////////////////////
// Get the buffer size
//

unsigned char getBufferSize(void)
{
    return 8; // Standard CAN frame
}

///////////////////////////////////////////////////////////////////////////////
//  vscp_readNicknamePermanent
//

uint8_t vscp_readNicknamePermanent(void)
{
    return eeprom_read( VSCP_EEPROM_NICKNAME );
}

///////////////////////////////////////////////////////////////////////////////
//  vscp_writeNicknamePermanent
//

void vscp_writeNicknamePermanent(uint8_t nickname)
{
    eeprom_write( VSCP_EEPROM_NICKNAME, nickname );
}

///////////////////////////////////////////////////////////////////////////////
// vscp_getZone
//

uint8_t vscp_getZone(void)
{
    return eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_ZONE ] );
}

///////////////////////////////////////////////////////////////////////////////
// vscp_getSubzone
//

uint8_t vscp_getSubzone(void)
{
    return eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_SUBZONE ] );
}

///////////////////////////////////////////////////////////////////////////////
// vscp_readAppReg
//

uint8_t vscp_readAppReg(uint8_t reg)
{    
    uint8_t rv;

    rv = 0x00; // default read
    
    // * * *  Page 0  * * *
    if ( 0 == vscp_page_select ) {

        // Zone
        if ( reg == REG0_ACCRA_ZONE ) {
            rv = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_ZONE ] );
        }            
        // SubZone
        else if ( reg == REG0_ACCRA_SUBZONE ) {
            rv = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_SUBZONE ] );
        }
        // Channel subzones & control registers
        else if ( ( reg >= REG0_ACCRA_CH0_SUBZONE ) && 
                    ( reg <= REG0_ACCRA_CONTROL_FREQ_CH3 ) ) {
            rv = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_CH0_SUBZONE + 
                                ( reg - REG0_ACCRA_CH0_SUBZONE ) ] );
        }
        // Reset counter locations
        else if ( ( reg >= REG0_ACCRA_RESET_COUNTER_CH0 ) && 
                    ( reg <= REG0_ACCRA_RESET_COUNTER_CH3 ) ) {
            rv = 0x00;
        }
        // Counter 0
        else if ( ( reg >= REG0_ACCRA_COUNTER_CH0_MSB ) && 
                    ( reg <= REG0_ACCRA_COUNTER_CH0_LSB ) ) {
            uint8_t *p = (uint8_t *)&counter[ 0 ];
            rv = p[ 3 - ( reg - REG0_ACCRA_COUNTER_CH0_MSB ) ];    // Byte order in PIC is little endian
        }
        // Counter 1
        else if ( ( reg >= REG0_ACCRA_COUNTER_CH1_MSB ) && 
                    ( reg <= REG0_ACCRA_COUNTER_CH1_LSB ) ) {
            uint8_t *p = (uint8_t *)&counter[ 1 ];
            rv = p[ 3 - ( reg - REG0_ACCRA_COUNTER_CH1_MSB ) ];    // Byte order in PIC is little endian
        }
        // Counter 2
        else if ( ( reg >= REG0_ACCRA_COUNTER_CH2_MSB ) && 
                    ( reg <= REG0_ACCRA_COUNTER_CH2_LSB ) ) {
            uint8_t *p = (uint8_t *)&counter[ 2 ];
            rv = p[ 3 - ( reg - REG0_ACCRA_COUNTER_CH2_MSB ) ];    // Byte order in PIC is little endian
        }
        // Counter 3
        else if ( ( reg >= REG0_ACCRA_COUNTER_CH3_MSB ) && 
                    ( reg <= REG0_ACCRA_COUNTER_CH3_LSB ) ) {
            uint8_t *p = (uint8_t *)&counter[ 3 ];
            rv = p[ 3 - ( reg - REG0_ACCRA_COUNTER_CH3_MSB ) ];    // Byte order in PIC is little endian
        }
        // The rest
        else if ( ( reg >= REG0_ACCRA_COUNTER_ALARM_CH0_MSB ) && 
                    ( reg <= REG0_ACCRA_LINEARIZATION_EVENT_TYPE_CH3 ) ) {
            rv = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_ALARM_CH0_MSB + 
                                ( reg - REG0_ACCRA_COUNTER_ALARM_CH0_MSB ) ] );
        }
        
    } // page 0
    
    // * * *  Page 1  * * *
    else if ( 1 == vscp_page_select ) {
        
        // Frequency for counter 0
        if ( ( reg >= REG1_ACCRA_CH0_FREQUENCY_MSB ) && 
                    ( reg <= REG1_ACCRA_CH0_FREQUENCY_LSB ) ) {
            uint8_t *p = (uint8_t *)&frequency[ 0 ];
            rv = p[ 3 - ( reg - REG1_ACCRA_CH0_FREQUENCY_MSB ) ];    // Byte order in PIC is little endian
        }
        // Frequency for counter 1
        else if ( ( reg >= REG1_ACCRA_CH1_FREQUENCY_MSB ) && 
                    ( reg <= REG1_ACCRA_CH1_FREQUENCY_LSB ) ) {
            
            uint8_t *p = (uint8_t *)&frequency[ 1 ];
            rv = p[ 3 - ( reg - REG1_ACCRA_CH1_FREQUENCY_MSB ) ];    // Byte order in PIC is little endian
            
        }
        // Frequency for counter 2
        else if ( ( reg >= REG1_ACCRA_CH2_FREQUENCY_MSB ) && 
                    ( reg <= REG1_ACCRA_CH2_FREQUENCY_LSB ) ) {
            
            uint8_t *p = (uint8_t *)&frequency[ 2 ];
            rv = p[ 3 - ( reg - REG1_ACCRA_CH2_FREQUENCY_MSB ) ];    // Byte order in PIC is little endian
            
        }
        // Frequency for counter 3
        else if ( ( reg >= REG1_ACCRA_CH3_FREQUENCY_MSB ) && 
                    ( reg <= REG1_ACCRA_CH3_FREQUENCY_LSB ) ) {
            
            uint8_t *p = (uint8_t *)&frequency[ 3 ];
            rv = p[ 3 - ( reg - REG1_ACCRA_CH3_FREQUENCY_MSB ) ];    // Byte order in PIC is little endian
            
        }    
        else if ( ( reg >= REG1_ACCRA_CH0_FREQUENCY_LOW_MSB ) && 
                    ( reg <= REG1_ACCRA_CH3_FREQ_HYSTERESIS_LSB ) ) {
            
            rv = eeprom_read( reg2eeprom_pg1[ REG1_ACCRA_CH0_FREQUENCY_LOW_MSB +
                                ( reg - REG1_ACCRA_CH0_FREQUENCY_LOW_MSB ) ] );
            
        }
        
    } // page 1
    
    // * * *  Page 2  * * *
    else if ( 2 == vscp_page_select ) {
        
        if ( ( reg >= REG2_ACCRA_CH0_LINEARIZATION_K_MSB ) && 
                    ( reg <= REG2_ACCRA_CH3_LINEARIZATION_M_LSB ) ) {
            
            rv = eeprom_read( reg2eeprom_pg2[ reg ] );
            
        }
        
    } // page 2
    
 
    // * * *  Page 3  * * *
    else if ( 3 == vscp_page_select ) {
        
        if ( reg < ( REG_DESCION_MATRIX + 8*DESCION_MATRIX_ROWS ) ) {
            rv = eeprom_read( DECISION_MATRIX_EEPROM_START + reg );
        }
        
    } // page 3
    
    return rv;

}

///////////////////////////////////////////////////////////////////////////////
// vscp_writeAppReg
//

uint8_t vscp_writeAppReg( uint8_t reg, uint8_t val )
{
    uint8_t rv;
    BOOL bInfoEvent = FALSE;
    BOOL bOn = FALSE;

    rv = ~val; // error return
    
    // * * *  Page 0  * * *
    if ( 0 == vscp_page_select ) {

        // Zone
        if ( reg == REG0_ACCRA_ZONE ) {
            eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_ZONE ], val);
            rv = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_ZONE ]);
        }
        else if ( reg == REG0_ACCRA_SUBZONE ) {
            // SubZone
            eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_SUBZONE ], val);
            rv = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_SUBZONE ] );
        }
        
        // Channel sub zones
        else if ( ( reg >= REG0_ACCRA_CH0_SUBZONE ) && 
                       ( reg <= REG0_ACCRA_CH3_SUBZONE ) ) {
            eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_CH0_SUBZONE + 
                                ( reg - REG0_ACCRA_CH0_SUBZONE ) ], val );
            rv = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_CH0_SUBZONE + 
                                ( reg - REG0_ACCRA_CH0_SUBZONE ) ] );
        }
        
        // channel control registers
        else if ( ( reg >= REG0_ACCRA_CONTROL_COUNTER_CH0 ) && 
                       ( reg <= REG0_ACCRA_CONTROL_COUNTER_CH3 ) ) {
            eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_CONTROL_COUNTER_CH0 + 
                                ( reg - REG0_ACCRA_CONTROL_COUNTER_CH0 ) ], val );
            rv = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_CONTROL_COUNTER_CH0 + 
                                ( reg - REG0_ACCRA_CONTROL_COUNTER_CH0 ) ] );
            // Also update counter shadow control register
            counter_control_shadow[ reg - REG0_ACCRA_CONTROL_COUNTER_CH0 ] = val;
        }
        
        // Frequency control register
        else if ( ( reg >= REG0_ACCRA_CONTROL_FREQ_CH0 ) && 
                       ( reg <= REG0_ACCRA_CONTROL_FREQ_CH3 ) ) {
            eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_CONTROL_FREQ_CH0 + 
                                ( reg - REG0_ACCRA_CONTROL_FREQ_CH0 ) ], val );
            rv = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_CONTROL_FREQ_CH0 + 
                                ( reg - REG0_ACCRA_CONTROL_FREQ_CH0 ) ] );
        }
        
        // Reset counter 0
        else if ( ( reg == REG0_ACCRA_RESET_COUNTER_CH0 ) ) {
            
            if ( 0x55 == val ) {
                // If reload on zero is activated load value else set to zero
                if ( counter_control_shadow[ 0 ] & COUNTER_CTRL_RELOAD_ZERO ) {
                    counter[ 0 ] = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH0_0 ] ) << 24 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH0_1 ] ) << 16 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH0_2 ] ) << 8 +
                                        eeprom_read( reg2eeprom_pg0[REG0_ACCRA_COUNTER_RELOAD_CH0_3 ] );
                }
                else {
                    counter[ 0 ] = 0;
                }
                rv = 0x55;
            }
            else {
                rv = 0;
            }
       
        }
        
        // Reset counter 1
        else if ( ( reg == REG0_ACCRA_RESET_COUNTER_CH1 ) ) {
            
            if ( 0x55 == val ) {
                // If reload on zero is activated load value else set to zero
                if ( counter_control_shadow[ 1 ] & COUNTER_CTRL_RELOAD_ZERO ) {
                    counter[ 1 ] = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH1_0 ] ) << 24 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH1_1 ] ) << 16 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH1_2 ] ) << 8 +
                                        eeprom_read( reg2eeprom_pg0[REG0_ACCRA_COUNTER_RELOAD_CH1_3 ] );
                }
                else {
                    counter[ 1 ] = 0;
                }
                rv = 0x55;
            }
            else {
                rv = 0;
            }
       
        }
        
        // Reset counter 2
        else if ( ( reg == REG0_ACCRA_RESET_COUNTER_CH2 ) ) {
            
            if ( 0x55 == val ) {
                // If reload on zero is activated load value else set to zero
                if ( counter_control_shadow[ 2 ] & COUNTER_CTRL_RELOAD_ZERO ) {
                    counter[ 2 ] = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH2_0 ] ) << 24 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH2_1 ] ) << 16 +
                                        eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH2_2 ] ) << 8 +
                                        eeprom_read( reg2eeprom_pg0[REG0_ACCRA_COUNTER_RELOAD_CH2_3 ] );
                }
                else {
                    counter[ 2 ] = 0;
                }
                rv = 0x55;
            }
            else {
                rv = 0;
            }
       
        }
        
        // Reset counter 3
        else if ( ( reg == REG0_ACCRA_RESET_COUNTER_CH3 ) ) {
            
            if ( 0x55 == val ) {
                // If reload on zero is activated load value else set to zero
                if ( counter_control_shadow[ 3 ] & COUNTER_CTRL_RELOAD_ZERO ) {
                    counter[ 3 ] = (uint32_t)eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH3_0 ] ) << 24 +
                                        (uint32_t)eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH3_1 ] ) << 16 +
                                        (uint32_t)eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_RELOAD_CH3_2 ] ) << 8 +
                                        (uint32_t)eeprom_read( reg2eeprom_pg0[REG0_ACCRA_COUNTER_RELOAD_CH3_3 ] );
                }
                else {
                    counter[ 3 ] = 0;
                }
                rv = 0x55;
            }
            else {
                rv = 0;
            }
       
        }
        
        // The rest...
        else if ( ( reg >= REG0_ACCRA_COUNTER_ALARM_CH0_MSB ) && 
                       ( reg <= REG0_ACCRA_LINEARIZATION_EVENT_TYPE_CH3 ) ) {
            eeprom_write( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_ALARM_CH0_MSB +
                                ( reg - REG0_ACCRA_COUNTER_ALARM_CH0_MSB ) ], val );
            rv = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_COUNTER_ALARM_CH0_MSB +
                                ( reg - REG0_ACCRA_COUNTER_ALARM_CH0_MSB ) ] );
        }
        
    } // page 0
    
    // * * *  Page 1  * * *
    else if ( 1 == vscp_page_select ) {
    
        if ( ( reg >= REG1_ACCRA_CH0_FREQUENCY_LOW_MSB ) && 
                       ( reg <= REG1_ACCRA_CH3_FREQ_HYSTERESIS_LSB ) ) {
            eeprom_write( reg2eeprom_pg1[ reg ], val );
            rv = eeprom_read( reg2eeprom_pg1[ reg ] );
        }
        
    } // page 1
    
    // * * *  Page 2  * * *
    else if ( 2 == vscp_page_select ) {
    
        if ( ( reg >= REG2_ACCRA_CH0_LINEARIZATION_K_MSB ) && 
                       ( reg <= REG2_ACCRA_CH3_LINEARIZATION_M_LSB ) ) {
            eeprom_write( reg2eeprom_pg2[ reg ], val );
            rv = eeprom_read( reg2eeprom_pg2[ reg ] );
        }
        
    } // page 2   
    
    // * * *  Page 3  * * *
    else if ( 3 == vscp_page_select ) {
    
        if ( reg < (REG_DESCION_MATRIX + DESCION_MATRIX_ROWS * 8) ) {
            eeprom_write( DECISION_MATRIX_EEPROM_START + reg, val );
            calculateSetFilterMask();  // Calculate new hardware filter
            rv = eeprom_read( DECISION_MATRIX_EEPROM_START + reg );
        }
        
    } // page 3
        
    // --------------------------------------------------------------------------

    return rv;

}

///////////////////////////////////////////////////////////////////////////////
// Send Decision Matrix Information
//

void sendDMatrixInfo(void)
{
    vscp_omsg.priority = VSCP_PRIORITY_MEDIUM;
    vscp_omsg.flags = VSCP_VALID_MSG + 2;
    vscp_omsg.vscp_class = VSCP_CLASS1_PROTOCOL;
    vscp_omsg.vscp_type = VSCP_TYPE_PROTOCOL_GET_MATRIX_INFO_RESPONSE;

    vscp_omsg.data[ 0 ] = DESCION_MATRIX_ROWS;
    vscp_omsg.data[ 1 ] = REG_DESCION_MATRIX;

    vscp_sendEvent(); // Send data
}


///////////////////////////////////////////////////////////////////////////////
// SendInformationEvent
//

void SendInformationEvent( unsigned char channel,
                            unsigned char eventClass,
                            unsigned char eventTypeId )
{
    uint8_t data[3];

    data[ 0 ] = channel; // Register
    data[ 1 ] = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_ZONE ] );
    data[ 2 ] = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_CH0_SUBZONE + ( channel & 0x03 ) ] );
    sendVSCPFrame( eventClass,
                    eventTypeId,
                    vscp_nickname,
                    VSCP_PRIORITY_MEDIUM,
                    3,
                    data );
}

///////////////////////////////////////////////////////////////////////////////
// SendAlarmEvent
//

void SendAlarmEvent( uint8_t channel  )
{
    uint8_t data[3];

    data[ 0 ] = channel;    // Channel 0-3 for counter, channel 4-6 for frequency low, channel 7-9 for 
                            // frequency high
    data[ 1 ] = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_ZONE ] );
    data[ 2 ] = eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_CH0_SUBZONE + ( channel & 0x03 ) ] );
    sendVSCPFrame( VSCP_CLASS1_ALARM,
                    VSCP_TYPE_ALARM_ALARM,
                    vscp_nickname,
                    VSCP_PRIORITY_HIGH,
                    3,
                    data );
}



///////////////////////////////////////////////////////////////////////////////
// Do decision Matrix handling
// 
// The routine expects vscp_imsg to contain a valid incoming event
//

void doDM(void)
{
    unsigned char i;
    unsigned char dmflags;
    unsigned short class_filter;
    unsigned short class_mask;
    unsigned char type_filter;
    unsigned char type_mask;

    // Don't deal with the protocol functionality
    if ( VSCP_CLASS1_PROTOCOL == vscp_imsg.vscp_class ) return;

    for (i = 0; i<DESCION_MATRIX_ROWS; i++) {

        // Get DM flags for this row
        dmflags = eeprom_read( DECISION_MATRIX_EEPROM_START + (8 * i) );

        // Is the DM row enabled?
        if ( dmflags & VSCP_DM_FLAG_ENABLED ) {

            // Should the originating id be checked and if so is it the same?
            if ( ( dmflags & VSCP_DM_FLAG_CHECK_OADDR ) &&
                    ( vscp_imsg.oaddr != eeprom_read( DECISION_MATRIX_EEPROM_START + (8 * i) ) ) ) {
                continue;
            }

            // Check if zone should match and if so if it match
            if ( dmflags & VSCP_DM_FLAG_CHECK_ZONE ) {
                if ( 255 != vscp_imsg.data[ 1 ] ) {
                    if ( vscp_imsg.data[ 1 ] != eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_ZONE ] ) ) {
                        continue;
                    }
                }
            }

            // Check if sub zone should match and if so if it match
            if ( dmflags & VSCP_DM_FLAG_CHECK_SUBZONE ) {
                if ( 255 != vscp_imsg.data[ 2 ] ) {
                    if ( vscp_imsg.data[ 2 ] != eeprom_read( reg2eeprom_pg0[ REG0_ACCRA_SUBZONE ] ) ) {
                        continue;
                    }
                }
            }
            
            class_filter = ( ( dmflags & VSCP_DM_FLAG_CLASS_FILTER) << 8 ) +
                    eeprom_read( DECISION_MATRIX_EEPROM_START +
                                    (8 * i) +
                                    VSCP_DM_POS_CLASSFILTER );
            
            class_mask = ( ( dmflags & VSCP_DM_FLAG_CLASS_MASK ) << 7 ) +
                    eeprom_read( DECISION_MATRIX_EEPROM_START +
                                    (8 * i) +
                                    VSCP_DM_POS_CLASSMASK);
            
            type_filter = eeprom_read( DECISION_MATRIX_EEPROM_START +
                                        (8 * i) +
                                        VSCP_DM_POS_TYPEFILTER);
            
            type_mask = eeprom_read( DECISION_MATRIX_EEPROM_START +
                                        (8 * i) +
                                        VSCP_DM_POS_TYPEMASK);
                           
            if ( !( ( class_filter ^ vscp_imsg.vscp_class ) & class_mask ) &&
                    !( ( type_filter ^ vscp_imsg.vscp_type ) & type_mask ) ) {

                // OK Trigger this action
                switch ( eeprom_read( DECISION_MATRIX_EEPROM_START + 
                                        (8 * i) + 
                                        VSCP_DM_POS_ACTION ) ) {

                    case ACCRA_ACTION_NOOP:
                        break;
                        
                } // case
                
            } // Filter/mask
            
        } // Row enabled
        
    } // for each row
    
}


///////////////////////////////////////////////////////////////////////////////
//                        VSCP Required Methods
//////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////
// Get Major version number for this hardware module
//

unsigned char vscp_getMajorVersion()
{
    return FIRMWARE_MAJOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get Minor version number for this hardware module
//

unsigned char vscp_getMinorVersion()
{
    return FIRMWARE_MINOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get Subminor version number for this hardware module
//

unsigned char vscp_getSubMinorVersion()
{
    return FIRMWARE_SUB_MINOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// getVSCP_GUID
//
// Get GUID from EEPROM
//

uint8_t vscp_getGUID(uint8_t idx)
{
    return eeprom_read( VSCP_EEPROM_REG_GUID + idx );
}


///////////////////////////////////////////////////////////////////////////////
// getDeviceURL
//
// Get device URL from EEPROM
//

uint8_t vscp_getMDF_URL(uint8_t idx)
{
    return vscp_deviceURL[ idx ];
}

///////////////////////////////////////////////////////////////////////////////
// Get Manufacturer id and subid from EEPROM
//

uint8_t vscp_getUserID(uint8_t idx)
{
    return eeprom_read( VSCP_EEPROM_REG_USERID + idx );
}

///////////////////////////////////////////////////////////////////////////////
//  setVSCPUserID
//

void vscp_setUserID(uint8_t idx, uint8_t data)
{
    eeprom_write( idx + VSCP_EEPROM_REG_USERID, data );
}

///////////////////////////////////////////////////////////////////////////////
// getVSCPManufacturerId
// 
// Get Manufacturer id and subid from EEPROM
//

uint8_t vscp_getManufacturerId(uint8_t idx)
{
    return eeprom_read( VSCP_EEPROM_REG_MANUFACTUR_ID0 + idx );
}

///////////////////////////////////////////////////////////////////////////////
// Get the bootloader algorithm code
//

uint8_t vscp_getBootLoaderAlgorithm(void)
{
    return VSCP_BOOTLOADER_PIC1;
}

///////////////////////////////////////////////////////////////////////////////
// Get the buffer size
//

uint8_t vscp_getBufferSize(void)
{
    return 8; // Standard CAN frame
}


///////////////////////////////////////////////////////////////////////////////
//  getNickname
//

uint8_t vscp_getNickname(void)
{
    return eeprom_read(VSCP_EEPROM_NICKNAME);
}

///////////////////////////////////////////////////////////////////////////////
//  setNickname
//

void vscp_setNickname(uint8_t nickname)
{
    eeprom_write(VSCP_EEPROM_NICKNAME, nickname);
}

///////////////////////////////////////////////////////////////////////////////
//  getSegmentCRC
//

uint8_t vscp_getSegmentCRC(void)
{
    return eeprom_read( VSCP_EEPROM_SEGMENT_CRC );
}

///////////////////////////////////////////////////////////////////////////////
//  setSegmentCRC
//

void vscp_setSegmentCRC(uint8_t crc)
{
    eeprom_write( VSCP_EEPROM_SEGMENT_CRC, crc );
}

///////////////////////////////////////////////////////////////////////////////
//  setVSCPControlByte
//

void vscp_setControlByte(uint8_t ctrl)
{
    eeprom_write(VSCP_EEPROM_CONTROL, ctrl);
}


///////////////////////////////////////////////////////////////////////////////
//  getVSCPControlByte
//

uint8_t vscp_getControlByte(void)
{
    return eeprom_read(VSCP_EEPROM_CONTROL);
}

///////////////////////////////////////////////////////////////////////////////
//  vscp_getEmbeddedMdfInfo
//

void vscp_getEmbeddedMdfInfo(void)
{
    // No embedded DM so we respond with info about that

    vscp_omsg.priority = VSCP_PRIORITY_NORMAL;
    vscp_omsg.flags = VSCP_VALID_MSG + 3;
    vscp_omsg.vscp_class = VSCP_CLASS1_PROTOCOL;
    vscp_omsg.vscp_type = VSCP_TYPE_PROTOCOL_RW_RESPONSE;

    vscp_omsg.data[ 0 ] = 0;
    vscp_omsg.data[ 1 ] = 0;
    vscp_omsg.data[ 2 ] = 0;

    // send the event
    vscp_sendEvent();
}

///////////////////////////////////////////////////////////////////////////////
// vscp_goBootloaderMode
//

void vscp_goBootloaderMode( uint8_t algorithm )
{
    if ( VSCP_BOOTLOADER_PIC1 != algorithm  ) return;

    // OK, We should enter boot loader mode
    // 	First, activate bootloader mode
    eeprom_write(VSCP_EEPROM_BOOTLOADER_FLAG, VSCP_BOOT_FLAG);

    // Reset processor
    Reset();
}

///////////////////////////////////////////////////////////////////////////////
//  vscp_getMatrixInfo
//

void vscp_getMatrixInfo(char *pData)
{
    uint8_t i;

    vscp_omsg.data[ 0 ] = DESCION_MATRIX_ROWS;  // Matrix is seven rows
    vscp_omsg.data[ 1 ] = REG_DESCION_MATRIX;   // Matrix start offset
    vscp_omsg.data[ 2 ] = 0;                    // Matrix start page
    vscp_omsg.data[ 3 ] = DESCION_MATRIX_PAGE;
    vscp_omsg.data[ 4 ] = 0;                    // Matrix end page
    vscp_omsg.data[ 5 ] = DESCION_MATRIX_PAGE;
    vscp_omsg.data[ 6 ] = 0;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_getFamilyCode
//

uint32_t vscp_getFamilyCode() {
    return 0L;
}


///////////////////////////////////////////////////////////////////////////////
// vscp_getFamilyType
//

uint32_t vscp_getFamilyType() {
    return 0;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_restoreDefaults
//

void vscp_restoreDefaults() {
    init_app_eeprom();
    init_app_ram();
}


///////////////////////////////////////////////////////////////////////////////
// vscp_getRegisterPagesUsed
//

uint8_t vscp_getRegisterPagesUsed( void )
{
    return NUMBER_OF_REGISTER_PAGES; // One pages used
}

///////////////////////////////////////////////////////////////////////////////
// sendVSCPFrame
//

int8_t sendVSCPFrame( uint16_t vscpclass,
                        uint8_t vscptype,
                        uint8_t nodeid,
                        uint8_t priority,
                        uint8_t size,
                        uint8_t *pData )
{
    uint32_t id = ( (uint32_t)priority << 26 ) |
                    ( (uint32_t)vscpclass << 16 ) |
                    ( (uint32_t)vscptype << 8 ) |
                    nodeid; // node address (our address)

    if ( !sendCANFrame( id, size, pData ) ) {
        return FALSE;
    }

    return TRUE;
}


///////////////////////////////////////////////////////////////////////////////
// getVSCPFrame
//

int8_t getVSCPFrame(uint16_t *pvscpclass,
                        uint8_t *pvscptype,
                        uint8_t *pNodeId,
                        uint8_t *pPriority,
                        uint8_t *pSize,
                        uint8_t *pData)
{
    uint32_t id;

    if ( !getCANFrame(&id, pSize, pData) ) {
        return FALSE;
    }

    *pNodeId = id & 0x0ff;
    *pvscptype = (id >> 8) & 0xff;
    *pvscpclass = (id >> 16) & 0x1ff;
    *pPriority = (uint16_t) (0x07 & (id >> 26));

    return TRUE;
}


///////////////////////////////////////////////////////////////////////////////
// sendCANFrame
//

int8_t sendCANFrame(uint32_t id, uint8_t dlc, uint8_t *pdata)
{
    uint8_t rv = FALSE;
    sendTimer = 0;

    while ( sendTimer < 1000 ) {
        if ( ECANSendMessage( id, pdata, dlc, ECAN_TX_XTD_FRAME ) ) {
            rv = TRUE;
            break;
        }
    }

    vscp_omsg.flags = 0;

    return rv;
}

///////////////////////////////////////////////////////////////////////////////
// getCANFrame
//

int8_t getCANFrame(uint32_t *pid, uint8_t *pdlc, uint8_t *pdata)
{
    ECAN_RX_MSG_FLAGS flags;

    // Don't read in new event if there already is a event
    // in the input buffer
    if (vscp_imsg.flags & VSCP_VALID_MSG) return FALSE;

    if ( ECANReceiveMessage( pid, pdata, pdlc, &flags) ) {

        // RTR not interesting
        if (flags & ECAN_RX_RTR_FRAME) return FALSE;

        // Must be extended frame
        if (!(flags & ECAN_RX_XTD_FRAME)) return FALSE;

        return TRUE;

    }

    return FALSE;
}

///////////////////////////////////////////////////////////////////////////////
// calculateSetFilterMask
//
// Calculate and set required filter and mask
// for the current decision matrix
//

void calculateSetFilterMask( void )
{
    uint8_t i,j;
    uint8_t lastOID;
    uint32_t rowmask;
    uint32_t rowfilter;

    // Reset filter masks
    uint32_t mask = 0xffffffff; // Just id 0x00000000 will come true
    uint32_t filter = 0x00000000;

    // Go through all DM rows
    for ( i=0; i < DESCION_MATRIX_ROWS; i++ ) {

        // No need to check not active DM rows
        if ( eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_FLAGS ) & VSCP_DM_FLAG_ENABLED ) {

            // build the mask
            // ==============
            // We receive
            //  - all priorities
            //  - hardcoded and not hardcoded
            //  - from all nodes

            rowmask =
                    // Bit 9 of class mask
                    ( (uint32_t)( eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_FLAGS ) & VSCP_DM_FLAG_CLASS_MASK ) << 23 ) |
                    // Rest of class mask
                    ( (uint32_t)eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_CLASSMASK ) << 16 ) |
                    // Type mask
                    ( (uint32_t)eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_TYPEMASK ) << 8 ) |
                    // Hardcoded bit
                    //( ( eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_FLAGS ) & VSCP_DM_FLAG_HARDCODED ) << 20 ) |   
					// OID  - handle later
					0xff;
                    

            // build the filter
            // ================

            rowfilter =
                    // Bit 9 of class filter
                    ( (uint32_t)( eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_FLAGS ) & VSCP_DM_FLAG_CLASS_FILTER ) << 24 ) |
                    // Rest of class filter
                    ( (uint32_t)eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_CLASSFILTER ) << 16 ) |
                    // Type filter
                    ( (uint32_t)eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_TYPEFILTER ) << 8 ) |
                    // OID Mask cleared if not same OID for all or one or more
                    // rows don't have OID check flag set.
                    eeprom_read( VSCP_EEPROM_END + 8*i );

            if ( 0 == i ) filter = rowfilter;   // Hack for first iteration loop

            // Form the mask - if one mask have a don't care (0)
            // the final mask should also have a don't care on that position
            mask &= rowmask;

            // Check the calculated filter and the current
            // filter to see if the bits are the same. If they
            // are not then clear the mask at that position
            for ( j=0; j<32; j++ ) {
                // If two bits are different we clear the mask bit
                if ( ( ( filter >> j ) & 1 ) != ( ( rowfilter >> j ) & 1 ) ) {
                    mask &= ~(1<<j);
                }
            }

            // Form the filter
            // if two bits are not the same they will be zero
            // All zeros will be zero
            // All ones will be one
            filter &= rowfilter;

            // Not check OID?
            if ( !eeprom_read( VSCP_EEPROM_END + 8*i + VSCP_DM_POS_FLAGS ) & VSCP_DM_FLAG_CHECK_OADDR ) {
                // No should not be checked for this position
                // This mean that we can't filter on a specific OID
                // so mask must be a don't care
                mask &= ~0xff;
            }

            if ( i ) {
                // If the current OID is different than the previous
                // we accept all
                for (j = 0; j < 8; j++) {
                    if ((lastOID >> i & 1)
                            != (eeprom_read(VSCP_EEPROM_END + 8*i ) >> i & 1)) {
                        mask &= (1 << i);
                    }
                }

                lastOID = eeprom_read(VSCP_EEPROM_END + 8*i );

            } 
            else {
                // First round we just store the OID
                lastOID = eeprom_read(VSCP_EEPROM_END + 8*i );
            }

        }
    }
    
    // Must be in Config mode to change settings.
    ECANSetOperationMode( ECAN_OP_MODE_CONFIG );

    //Set mask 1
    ECANSetRXM1Value( mask, ECAN_MSG_XTD );

    // Set filter 1
    ECANSetRXF1Value( filter, ECAN_MSG_XTD );

    // Return to Normal mode to communicate.
    ECANSetOperationMode( ECAN_OP_MODE_NORMAL );
  
}

