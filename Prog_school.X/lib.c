#include "lib.h"

int make_noise = 0;
int sensor_1 = 0, sensor_2 = 0, sensor_3 = 0;

void lcd_clk(void) /*генерация импульса на вход EN*/
{
  LCD_E = 1;
  __delay_us(E_pulse_with);
  LCD_E = 0;
  __delay_us(E_pulse_with);
}

void lcd_command(unsigned char outbyte) /*отправить команду (4-битный ре-жим работы) */
{
  LCD_RS=0; //режим передачи команды
  LCD_Data4=(LCD_Data4&0x0f)|(outbyte&0xf0); // отправка старших четырех бит
  lcd_clk();
  LCD_Data4=(LCD_Data4&0x0f)|((outbyte<<4)&0xf0); // отправка младших че-тырех бит
  lcd_clk();
  __delay_ms(1);
}

void lcd_putc(char outbyte) /* отправить данные (4-битная операция) */
{
  LCD_RS=1; //режим передачи данных
  LCD_Data4=(LCD_Data4&0x0f)|(outbyte&0xf0);
  lcd_clk();
  LCD_Data4=(LCD_Data4&0x0f)|((outbyte<<4)&0xf0);
  lcd_clk();
  
}

void lcd_puts(unsigned char line,const char *p) // *вывод строки на экран*
{
//    lcd_clear();
	lcd_origin();         // переход к нулевому адресу LCD
	lcd_command(line);			// установить адрес LCD 00H
	while(*p)                  // проверить, равен ли указатель 0
	{
	 lcd_putc(*p);             // отправить данные на LCD
	 p++;                      // увеличить адрес на 1
	}
//    __delay_ms(5);
}

void inttolcd(unsigned char posi, long value) //вывод на экран значений пере-менных
{
	char buff[16];
	itoa(buff,value,10);
	lcd_puts(posi,buff);
}

void lcd_init() // инициализация LCD-дисплея
{
  TRISD &= 0x03;// перевод выводов RD4-RD7в режим выходов
  LCD_Data4 &= 0b00001111;//установка нулей на линии передачи данных
  LCD_E=0;
  LCD_RS=0;
  __delay_ms(1);
/*инициализация дисплея*/
  LCD_Data4=(LCD_Data4&0x0f)|0x30;
  lcd_clk();
 __delay_ms(1);
  LCD_Data4=(LCD_Data4&0x0f)|0x30;
 lcd_clk();
__delay_ms(1);
  LCD_Data4=(LCD_Data4&0x0f)|0x30;
  lcd_clk();
  __delay_ms(1);
/*---------------------------------*/
  LCD_Data4=(LCD_Data4&0x0f)|0x20;	// переключить на 4-битный режим пе-ре-дачи
  lcd_clk();
  __delay_ms(1);
  lcd_command(0x28);//установить N=1, F=0 (две строки, размер символа 5*8 точек
  lcd_command(0x01);	// очистить всё
  lcd_command(0x06);	// автоматически передвинуть курсор после байта
  lcd_command(0x0C);	// дисплей включён, курсора нет, не моргает
  lcd_command(0x02);	// начальная позиция
  lcd_command(0x01);	// очистить всё снова
}

void Adc_init()
{
    TRISA|=0b00101111; //перевод выводов RA0, RA1, RA2, RA3, RA5 в режим входов
    TRISE|=0b00000111; //перевод выводов RE0, RE1, RE2 в режим входов
    ADCON1bits.PCFG=0b0111; // конфигурация аналого-цифровых портов
    ADCON1bits.VCFG=0b00; // опорное напряжение Vss Vdd
    ADCON2bits.ACQT=0b111;// время преобразования 20 Tad
    ADCON2bits.ADCS=0b110;// частота преобразования Fosc/64
    ADCON2bits.ADFM=0;//левое смещение
    ADCON0bits.ADON=1;   // модуль АЦП включен
}

int read_Adc(int channel)
{
    ADCON0bits.CHS=channel; // выбор аналогового канала
    ADCON0bits.GO_DONE=1; // запуск преобразования
    while(ADCON0bits.GO_DONE==1);      
    return (ADRESH<<2)+(ADRESL>>6);//запись результата преобразования
}

//void make_noise()
//{
//    TRISC0 = 0;
//    LATCbits.LATC0 = 1;
//    __delay_ms(0.02);
//    LATCbits.LATC0 = 0;
//    __delay_ms(0.02);
//}

void high_interrupt_init( void )
{
    RCONbits.IPEN=1; // two-level interrupt is enabled
    INTCON2bits.INTEDG0=0; // interrupt on the falling edge of the input signal INT0
    INTCONbits.INT0IF=0; // reset the interrupt flag from an external source
    INTCONbits.GIEH=1; // the high-level interrupt is enabled 
    INTCONbits.INT0IE=1;// interruption from an external source is enabled    
}

void low_interrupt_init( )
{
    T0CONbits.T08BIT=0;// setting timer №0 to 16-bit mode 
    T0CONbits.PSA=0;// pre-divider is used 
    T0CONbits.T0PS=0b111;//pre-divider = 256 
    T0CONbits.T0CS=0;// selection of the internal source clock 
    TMR0H=217;// the write high byte of initial value 
    TMR0L=217;// the write low byte of the initial value 
    T0CONbits.TMR0ON=1; 
    RCONbits.IPEN=1; // two-level interrupt is enabled
    INTCON2bits.TMR0IP=0;// assign an interrupt a low priority 
    INTCONbits.TMR0IF=0;// reset the interrupt flag on overflow of timer 0 
    INTCONbits.GIEH=1; // the high-level interrupt is enabled
    INTCONbits.GIEL=1; // low-level interrupts is enabled
    INTCONbits.TMR0IE=1;// overflow interrupt for timer 0 
}

void motor_init()
{
    TRISDbits.RD0=0;
    TRISDbits.RD1=0;
    TRISBbits.RB1=0;
    TRISBbits.RB2=0;
    TRISCbits.TRISC1=0;
    TRISCbits.TRISC2=0;
    CCP1CONbits.CCP1M=0b1100; //задаём режим работы модуля CCP1 (ШИМ)
    CCP1CONbits.P1M=0b00; //задействован только один вывод P1A
    CCP2CONbits.CCP2M=0b1111; // задаём режим работы модуля CCP2(ШИМ)
    CCPR1L=0; // задаём нулевую скважность
    CCPR2L=0; // задаём нулевую скважность
    PR2=124;// задаём период ШИМ
    T2CONbits.T2CKPS=0b00; //задаём предделитель модуля Timer2 равным 1
    T2CONbits.TMR2ON=1;// включение модуля Timer2
}

void motor_a_change_Speed (signed char speed)
{
    if (speed>0) // движение вперёд
    {
        CCPR1L=speed;
        PORTDbits.RD0=0;
        PORTDbits.RD1=1;
    }
    else if (speed<0) // движение назад
    {
        CCPR1L =-speed;
        PORTDbits.RD0=1;
        PORTDbits.RD1=0;
    }
    else //остановка
    {
        CCPR1L=0;
        PORTDbits.RD0=0;
        PORTDbits.RD1=0;
    }
}

void motor_b_change_Speed (signed char speed)
{
    if (speed>0) // движение вперёд
    {
        CCPR2L=speed;
        PORTBbits.RB1=0;
        PORTBbits.RB2=1;
    }
    else if (speed<0) // движение назад
    {
        CCPR2L =-speed;
        PORTBbits.RB1=1;
        PORTBbits.RB2=0;
    }
    else //остановка
    {
        CCPR2L = 0;
        PORTBbits.RB1=0;
        PORTBbits.RB2=0;
    }
}


void timer_init()
{
//    TRISBbits.RB3=0;//настройка RB3 на выход
    TRISCbits.RC0 = 0;
    T0CONbits.T08BIT=1;//настройка таймера №0 на 16-битный режим работы
    T0CONbits.PSA=1;//разрешение использовать предделитель
    T0CONbits.T0PS=0b111;//предделитель равен 256
    T0CONbits.T0CS=0;//выбор внутреннего источника тактовых импульсов
    TMR0H=0;//запись старшего байта начального значения
    TMR0L=200;//запись младшего байта начального значения
    T0CONbits.TMR0ON=1;
    RCONbits.IPEN=1; // разрешение двухуровневых прерываний
    INTCON2bits.TMR0IP=0;//присвоение прерыванию низкого приоритета
    INTCONbits.TMR0IF=0;//обнуление флага прерывания по переполнению таймера 0
    INTCONbits.GIEH=1; //разрешение высокоуровневых прерываний
    INTCONbits.GIEL=1; //разрешение низкоуровневых прерываний
    INTCONbits.TMR0IE=1;//разрешение прерывания по переполнению таймера 0

}

//void interrupt low_priority  LIisr (void)
//{
//
//    if(make_noise == 1)
//    {
//        LATCbits.LATC0 = !LATCbits.LATC0;
//    }
//    INTCONbits.TMR0IF=0;
//}

void A4_is_on()
{ 
TRISAbits.TRISA4 = 1;
}

void B0_is_on()
{ 
TRISBbits.TRISB0 = 1;
}

//void main(void) 
//{
//    TRISC0 = 0;
//    lcd_init();
//    Adc_init();
//    motor_init();
//    timer_init();
//    
//            
//    int dist_sensor = 0; 
//       
//    while(1)
//    { 
//        lcd_clear();       
//        dist_sensor = read_Adc(3);
//        
//        if(dist_sensor < 300)
//        {
//            flag = 1;
//            motor_a_change_Speed(120);
//            motor_b_change_Speed(120);
//        }   
//        else
//        {
//            flag = 0;
//        }
//        
//        
//        inttolcd(0x85, dist_sensor);
//        lcd_puts(0xC0, "Ok");
//        __delay_ms(50);
//    }
//    
//    
//}

void LED_on(int turn_on)
{
    TRISBbits.RB3 = 0;
    if(turn_on == 1)
    {
        LATBbits.LATB3 = 1;
    }
    else
    {
        LATBbits.LATB3 = 0;
    }
}

int read_sensor_1()
{
    sensor_1 = read_Adc(0);
    return sensor_1;
}

int read_sensor_2()
{
    sensor_2 = read_Adc(1);
    return sensor_2;
}

int read_sensor_3()
{
    sensor_3 = read_Adc(2);
    return sensor_3;
}


void init_all_units( void )
{
    lcd_init();
    Adc_init();
    motor_init();
    timer_init();
}


void testLEDRoutine( void )
{
    TRISBbits.RB3 = 0;  // LED
    LATBbits.LATB3 =! PORTBbits.RB3;
    lcd_puts(0x80, "LED is OK");
    __delay_ms( 500 );
}

void testButtonRoutine( void )
{
    TRISBbits.RB3 = 0;          // LED
    TRISBbits.RB0 = 1;          // Button RB0
    TRISAbits.RA4 = 1;          // Button RA4 
    
    if( !PORTBbits.RB0 )        // if button RB0 is pressed 
    {
        __delay_ms( 40 );       // to remove contacs bouncing 
        if( !PORTBbits.RB0 )    // if button RB0 is pressed 
        {
            LATBbits.LATB3 = 1;
            lcd_puts( 0x80, "Button B0 is OK");
        }
    }
    else if( !PORTAbits.RA4 )   // if button RA4 is pressed 
    {
        __delay_ms( 40 );       // to remove contacs bouncing 
        if( !PORTAbits.RA4 )    // if button RB0 is pressed 
        {
            LATBbits.LATB3 = 1;
            lcd_puts( 0xC0, "Button A4 is OK");
        }
    }
    else
    {
        LATBbits.LATB3 = 0;
        lcd_clear( );
    }
}

void testNoiseRoutine( void )
{
    TRISCbits.RC0 = 0;
    
    for( int i = 0; i <= 7500; i++)
    {
        LATCbits.LATC0 =! PORTCbits.RC0;
        __delay_us( 67 );    // 15 kHz 
    }
    lcd_puts( 0xC0, "Noise is OK");

}

void testAllSensorsRoutine( void )
{
    TRISBbits.RB0 = 1;          // Button RB0
    uint8_t press_count = 0; 
    uint16_t sensor_val = 0; 
    
    
    while( 1 )
    {
        if( !PORTBbits.RB0 )        // if button RB0 is pressed 
        {
            __delay_ms( 50 );       // to remove contacs bouncing 
            if( !PORTBbits.RB0 )    // if button RB0 is pressed 
            {
                press_count += 1; 
                if(press_count > 7) press_count = 0;
            }
        }
        
        sensor_val = read_Adc( press_count ); 
        
        inttolcd( 0x80, press_count );
        inttolcd( 0xC6, sensor_val );
        switch( press_count )
        {
            case 0:
                lcd_puts( 0x82, "REF sensor" );
                lcd_puts( 0xC0, "Val: " );
                break;
                
            case 1:
                lcd_puts( 0x82, "REF sensor" );
                lcd_puts( 0xC0, "Val: " );
                break;
                
            case 2:
                lcd_puts( 0x82, "REF sensor" );
                lcd_puts( 0xC0, "Val: " );
                break;
                
            case 3:
                lcd_puts( 0x82, "IR sensor" );
                lcd_puts( 0xC0, "Val: " );
                break;
                
            case 4:
                lcd_puts( 0x82, "Light sensor" );
                lcd_puts( 0xC0, "Val: " );
                break;
            
            case 5:
                lcd_puts( 0x82, "Noise sensor" );
                lcd_puts( 0xC0, "Val: " );
                break;
                
            case 6:
                lcd_puts( 0x82, "Temp sensor" );
                lcd_puts( 0xC0, "Val: " );
                break;
            
            case 7:
                lcd_puts( 0x82, "Potentiometr" );
                lcd_puts( 0xC0, "Val: " );
                break;
             
            default:
                lcd_puts( 0x82, "RED CODE" );
                break; 
        }
        
        __delay_ms( 100 );
        lcd_clear( );
        
    }
}

void testRefSensors( void )
{
    uint16_t ref_1_val = 0; 
    uint16_t ref_2_val = 0;
    uint16_t ref_3_val = 0;
    
    while( 1 )
    {
        ref_1_val = read_Adc( 0 );
        ref_2_val = read_Adc( 1 );
        ref_3_val = read_Adc( 2 );
        
        lcd_puts( 0x80, "REF1: " );
        inttolcd( 0x86, ref_1_val );
        lcd_puts( 0x89, "|" );
        lcd_puts( 0x8a, "REF2: " );
        inttolcd( 0xCb, ref_1_val );
        lcd_puts( 0xC9, "|" );        
        lcd_puts( 0xC0, "REF3: " );
        inttolcd( 0xC6, ref_1_val );
        
        __delay_ms( 200 );
        lcd_clear( );
    }
}

void testDistanceSensorRoutine( void )
{
    uint16_t sensor_val = 0; 
    
    while( 1 )
    {
        sensor_val = read_Adc( 3 );
        
        lcd_puts( 0x80, "IR-sensor");
        inttolcd(0xC0, sensor_val);
        
        __delay_ms( 200 );
        lcd_clear( );
    }
}

void testLightSensorRoutine( void )
{
    uint16_t sensor_val = 0;
    
    while( 1 )
    {
        sensor_val = read_Adc( 4 );
        
        lcd_puts( 0x80, "Light sensor");
        inttolcd(0xC0, sensor_val);
        
        __delay_ms( 200 );
        lcd_clear( );
    }
}

void testNoiseSensorRoutine( void )
{
    uint16_t sensor_val = 0;
    
    while( 1 )
    {
        sensor_val = read_Adc( 5 );
        
        lcd_puts( 0x80, "Noise sensor");
        inttolcd(0xC0, sensor_val);
        
        __delay_ms( 200 );
        lcd_clear( );
    }
}

void testTemperatureSensorRoutine( void )
{
    uint16_t sensor_val = 0;
    
    while( 1 )
    {
        sensor_val = read_Adc( 6 );
        
        lcd_puts( 0x80, "Temperature");
        inttolcd(0xC0, sensor_val);
        
        __delay_ms( 100 );
        lcd_clear( );
    }
}

void testPotentiometrRoutine( void )
{
    uint16_t sensor_val = 0;
    
    while( 1 )
    {
        sensor_val = read_Adc( 7 );
        
        lcd_puts( 0x80, "Potentiometr");
        inttolcd(0xC0, sensor_val);
        
        __delay_ms( 100 );
        lcd_clear( );
    }
}