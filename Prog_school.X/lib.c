#include "lib.h"

void lcd_clk(void) /*��������� �������� �� ���� EN*/
{
  LCD_E = 1;
  __delay_us(E_pulse_with);
  LCD_E = 0;
  __delay_us(E_pulse_with);
}

void lcd_command(unsigned char outbyte) /*��������� ������� (4-������ ��-��� ������) */
{
  LCD_RS=0; //����� �������� �������
  LCD_Data4=(LCD_Data4&0x0f)|(outbyte&0xf0); // �������� ������� ������� ���
  lcd_clk();
  LCD_Data4=(LCD_Data4&0x0f)|((outbyte<<4)&0xf0); // �������� ������� ��-����� ���
  lcd_clk();
  __delay_ms(1);
}

void lcd_putc(char outbyte) /* ��������� ������ (4-������ ��������) */
{
  LCD_RS=1; //����� �������� ������
  LCD_Data4=(LCD_Data4&0x0f)|(outbyte&0xf0);
  lcd_clk();
  LCD_Data4=(LCD_Data4&0x0f)|((outbyte<<4)&0xf0);
  lcd_clk();
  
}

void lcd_puts(uint8_t line,const uint8_t *p) 
{
	lcd_origin();         // ������� � �������� ������ LCD
	lcd_command(line);			// ���������� ����� LCD 00H
	while(*p)                  // ���������, ����� �� ��������� 0
	{
	 lcd_putc(*p);             // ��������� ������ �� LCD
	 p++;                      // ��������� ����� �� 1
	}
}

void inttolcd(uint8_t posi, int32_t value) 
{
	char buff[16];
	itoa(buff,value,10);
	lcd_puts(posi,buff);
}

void lcd_init() // ������������� LCD-�������
{
  TRISD &= 0x03;// ������� ������� RD4-RD7� ����� �������
  LCD_Data4 &= 0b00001111;//��������� ����� �� ����� �������� ������
  LCD_E=0;
  LCD_RS=0;
  __delay_ms(1);
/*������������� �������*/
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
  LCD_Data4=(LCD_Data4&0x0f)|0x20;	// ����������� �� 4-������ ����� ��-��-����
  lcd_clk();
  __delay_ms(1);
  lcd_command(0x28);//���������� N=1, F=0 (��� ������, ������ ������� 5*8 �����
  lcd_command(0x01);	// �������� ��
  lcd_command(0x06);	// ������������� ����������� ������ ����� �����
  lcd_command(0x0C);	// ������� �������, ������� ���, �� �������
  lcd_command(0x02);	// ��������� �������
  lcd_command(0x01);	// �������� �� �����
}

void Adc_init()
{
    TRISA|=0b00101111; //������� ������� RA0, RA1, RA2, RA3, RA5 � ����� ������
    TRISE|=0b00000111; //������� ������� RE0, RE1, RE2 � ����� ������
    ADCON1bits.PCFG=0b0111; // ������������ �������-�������� ������
    ADCON1bits.VCFG=0b00; // ������� ���������� Vss Vdd
    ADCON2bits.ACQT=0b111;// ����� �������������� 20 Tad
    ADCON2bits.ADCS=0b110;// ������� �������������� Fosc/64
    ADCON2bits.ADFM=0;//����� ��������
    ADCON0bits.ADON=1;   // ������ ��� �������
}

int16_t read_Adc(int16_t channel)
{
    ADCON0bits.CHS=channel; // ����� ����������� ������
    ADCON0bits.GO_DONE=1; // ������ ��������������
    while(ADCON0bits.GO_DONE==1);      
    return (ADRESH<<2)+(ADRESL>>6);//������ ���������� ��������������
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
    T0CONbits.T08BIT=0;// setting timer �0 to 16-bit mode 
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
    CCP1CONbits.CCP1M=0b1100; //����� ����� ������ ������ CCP1 (���)
    CCP1CONbits.P1M=0b00; //������������ ������ ���� ����� P1A
    CCP2CONbits.CCP2M=0b1111; // ����� ����� ������ ������ CCP2(���)
    CCPR1L=0; // ����� ������� ����������
    CCPR2L=0; // ����� ������� ����������
    PR2=124;// ����� ������ ���
    T2CONbits.T2CKPS=0b00; //����� ������������ ������ Timer2 ������ 1
    T2CONbits.TMR2ON=1;// ��������� ������ Timer2
}

void motor_a_change_Speed (int8_t speed)
{
    if (speed>0) // �������� �����
    {
        CCPR1L=speed;
        PORTDbits.RD0=0;
        PORTDbits.RD1=1;
    }
    else if (speed<0) // �������� �����
    {
        CCPR1L =-speed;
        PORTDbits.RD0=1;
        PORTDbits.RD1=0;
    }
    else //���������
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

void task_1_5(void) 
{
    uint8_t a=0, count=0;
    init_all_units( );
    
    TRISB3=0; /* Setting RB3 as a digital output */
    TRISA4=1; /* Setting RA4 as a digital input */
    TRISC0=0;

    while(1)
    {
        if ( !PORTAbits.RA4 )
        {
            __delay_ms(110);
            if( !PORTAbits.RA4 ) count += 1;
        }
        
        if ( count > 3 ) count = 0;
        
        switch (count)
        {
            case 0:     // turn off  
                LATCbits.LATC0 = 0;
                LATBbits.LATB3 = 0;
                break;
                
            case 1:     // LED blinking
                LATBbits.LATB3 =! PORTBbits.RB3;
                __delay_ms(100);
                break;

            case 2:     // LED && Noise
                LATBbits.LATB3 =! PORTBbits.RB3; 
                for(int i=0;i<12000;i++)
                {
                    if( !PORTAbits.RA4 ) break; 
                    LATCbits.LATC0 =! PORTCbits.RC0;
                    __delay_us(40);
                }
                break;
        
            default: ;
        }        
        a = !a;
    } 
}

void task_2_3(void)
{
    init_all_units(); 
    uint16_t a = 0, adc_value = 0, potentiometer = 0;
    float b = 0, d = 0;

    while(1)  
    {
        potentiometer = read_Adc(7);
        d = potentiometer / 147;
        b = round( d );
        if( abs( potentiometer - b * 147 ) > 6 ) a = floor(d);  //dead zone
        inttolcd( 0x86, a );
        adc_value = read_Adc( a );
        inttolcd( 0xC0, adc_value );
        __delay_ms(200);
        lcd_puts( 0x86, " " );
        lcd_puts( 0xC0, "                ");
    }
}