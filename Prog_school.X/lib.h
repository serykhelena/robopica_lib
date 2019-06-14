// This is a guard condition so that contents of this file are not included
// more than once.  
#include "pragma.h"
#include "define.h"

#include <xc.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>



void lcd_clk(void); /*генерация импульса на вход EN*/
void lcd_command(unsigned char outbyte); /*отправить команду (4-битный режим работы) */
void lcd_putc(char outbyte); /* отправить данные (4-битная операция) */
void lcd_puts(unsigned char line,const char *p); // *вывод строки на экран*
void inttolcd(unsigned char posi, long value); //вывод на экран значений пере-менных
void lcd_init(); // инициализация LCD-дисплея
void Adc_init();    // инициализация модуля АЦП
int read_Adc(int channel);  // чтение значений с указанного канала АЦП
//void make_noise();   // звуковой сигнал с частотой 1/freq

/*
 * @brief   Initialization of H-interrupt 
 *          from external source (button RB0/INT0)
 */
void high_interrupt_init( void );

void low_interrupt_init( );

void motor_init();   // инициализация моторов
void motor_a_change_Speed (signed char speed);  // управление скоростью вращенения 
void motor_b_change_Speed (signed char speed);  // управление скоростью вращенения 
//void timer_init();
//void interrupt low_priority  LIisr (void);
//void interrupt HIisr (void);
//
void LED_on(int turn_on);
int read_sensor_1();
int read_sensor_2();
int read_sensor_3();


/*
 * @brief   Initialization of all lld-units in robot
 */
void init_all_units( void ); 


/*****************************************/
/***************** TESTS *****************/
/*****************************************/

/*
 * @brief   Blinking LED (RB3)
 *          frequency 2 Hz (500 ms)
 *          write on lcd "LED is OK"
 * @note    Include delay 500 ms !!!! 
 */
void testLEDRoutine( void );

/*
 * @brief   If button is pressed, LED is on 
 *          Not require simultaneous pressing both buttons 
 */
void testButtonRoutine( void ); 

/*
 * @brief   WARNING! Noise is unstoppable  
 *          Turn off the power to stop it
 *          frequency is 15 kHz 
 * @note    Include addition delay ( 500 ms ) !!!!
 */
void testNoiseRoutine( void ); 

/*
 * @brief   Press the button to change sensor
 *          On lcd you will see:
 *              1st line : NUMBER of Sensor and NAME
 *              2nd line : VALUE of Sensor  
 * @note    VERY IMPORTANT!!! 
 *          Call OUTSIDE the while( 1 ) loop  !!!!    
 *          Include addition delay ( 100 ms ) !!!!
 */
void testAllSensorsRoutine( void ); 

/*
 * @brief   Check ref-sensors (bottom sensors)
 *          On lcd you will see:
 *              1st line : REF1: VALUE1 | REF2:
 *              2nd line : REF2: VALUE3 | VALUE2 
 * @note    VERY IMPORTANT!!! 
 *          Call OUTSIDE the while( 1 ) loop  !!!!    
 *          Include addition delay ( 200 ms ) !!!!
 */
void testRefSensors( void );

/*
 * @brief   Check distance sensor
 *          On lcd you will see:
 *              1st line : IR-sensor
 *              2nd line : ADC VALUE 
 * @note    VERY IMPORTANT!!! 
 *          Call OUTSIDE the while( 1 ) loop  !!!!    
 *          Include addition delay ( 200 ms ) !!!!
 */
void testDistanceSensorRoutine( void );

/*
 * @brief   Check light sensor
 *          On lcd you will see:
 *              1st line : Light sensor
 *              2nd line : ADC VALUE 
 * @note    VERY IMPORTANT!!! 
 *          Call OUTSIDE the while( 1 ) loop  !!!!    
 *          Include addition delay ( 200 ms ) !!!!
 */
void testLightSensorRoutine( void );

/*
 * @brief   Check noise sensor
 *          On lcd you will see:
 *              1st line : Noise sensor
 *              2nd line : ADC VALUE 
 * @note    VERY IMPORTANT!!! 
 *          Call OUTSIDE the while( 1 ) loop  !!!!    
 *          Include addition delay ( 200 ms ) !!!!
 */
void testNoiseSensorRoutine( void );

/*
 * @brief   Check temperature sensor
 *          On lcd you will see:
 *              1st line : Temperature
 *              2nd line : ADC VALUE 
 * @note    VERY IMPORTANT!!! 
 *          Call OUTSIDE the while( 1 ) loop  !!!!    
 *          Include addition delay ( 100 ms ) !!!!
 */
void testTemperatureSensorRoutine( void );

/*
 * @brief   Check potentiometr
 *          On lcd you will see:
 *              1st line : Potentiometr
 *              2nd line : ADC VALUE 
 * @note    VERY IMPORTANT!!! 
 *          Call OUTSIDE the while( 1 ) loop  !!!!    
 *          Include addition delay ( 100 ms ) !!!!
 */
void testPotentiometrRoutine( void );