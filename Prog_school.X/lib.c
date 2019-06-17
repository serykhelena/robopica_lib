#include "lib.h"

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

void lcd_puts(uint8_t line,const uint8_t *p) 
{
	lcd_origin();         // переход к нулевому адресу LCD
	lcd_command(line);			// установить адрес LCD 00H
	while(*p)                  // проверить, равен ли указатель 0
	{
	 lcd_putc(*p);             // отправить данные на LCD
	 p++;                      // увеличить адрес на 1
	}
}

void inttolcd(uint8_t posi, int32_t value) 
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

int16_t read_Adc(int16_t channel)
{
    ADCON0bits.CHS=channel; // выбор аналогового канала
    ADCON0bits.GO_DONE=1; // запуск преобразования
    while(ADCON0bits.GO_DONE==1);      
    return (ADRESH<<2)+(ADRESL>>6);//запись результата преобразования
}

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

void motor_a_change_Speed (int8_t speed)
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

void motor_b_change_Speed (int8_t speed)
{
    if (speed>0) // move forward
    {
        CCPR2L=speed;
        PORTBbits.RB1=0;
        PORTBbits.RB2=1;
    }
    else if (speed<0) // move backward
    {
        CCPR2L =-speed;
        PORTBbits.RB1=1;
        PORTBbits.RB2=0;
    }
    else //stop 
    {
        CCPR2L = 0;
        PORTBbits.RB1=0;
        PORTBbits.RB2=0;
    }
}

int8_t isInitialized = 0;

void init_all_units( void )
{
    if( isInitialized )
        return;
    
    lcd_init();
    Adc_init();
    motor_init();
    
    isInitialized = 1; 
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

void testMotors( void )
{
    TRISBbits.RB0 = 1; 
    uint8_t but_count = 0;
    
    while( 1 )
    {
        if( !PORTBbits.RB0 )
        {
            __delay_ms( 70 );
            if( !PORTBbits.RB0 )
            {
                but_count += 1;
            }
        }
    
        switch( but_count )
        {
            case 1:
                motor_a_change_Speed( 120 );
                motor_b_change_Speed( 120 ); 
                break;

            case 2:
                motor_a_change_Speed( -120 );
                motor_b_change_Speed( -120 ); 
                break;

            case 3:
                motor_a_change_Speed( 0 );
                motor_b_change_Speed( 0 ); 
                break;
                
            default: ;
        }
        but_count = (but_count >= 4) ? 0 : but_count; 
        
        lcd_puts( 0x80, "Button pressed");
        inttolcd(0xC0, but_count);
        
    }
}