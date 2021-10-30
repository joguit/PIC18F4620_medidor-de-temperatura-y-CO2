/*
 * Interfacing PIC16F887 microcontroller with DS18B20 temperature sensor.
 * C Code for MPLAB XC8 compiler.
 * Internal oscillator used @ 8MHz.
 * This is a free software with NO WARRANTY.
 * https://simple-circuit.com/
*/ 
/*
 *Se añade sensor de C02, se utiliza un array con lecturas tomadas experimentalmente
 * para los intervalos, y luego se interpola linealmente en un intervalo.
 *Se utiliza la memoria eeprom para guardar variable max co2
*/
/*Listado Errores:
 *Error 10 .Lectura muy baja
 *Error 11. Lectura muy alta
 */

// set configuration words
#define DS18B20_PIN      PORTDbits.RD5
#define DS18B20_PIN_Dir  TRISDbits.RD5
#define  led_verde PORTDbits.RD1//PORTDbits.RD1
#define _XTAL_FREQ 8000000

#pragma config OSC=INTIO67,MCLRE=ON,WDT=OFF,LVP=OFF //INTIO67->0xF8	Internal oscillator block, port function on RA6 and RA7.

#include <xc.h>
#include <stdint.h>        // include stdint header
#include <stdio.h> 
#include <usart.h>
#include <delays.h>
#include "I2C_LCD.h
#include <stdlib.h>


//definicion de variables
//unsigned int CO2[17]={500,506,533,547,560,572,700,738,1012,1217,1276,1506,1905,2240,3325,4700,5700};
//float volt[17]={0.90,0.91,0.96,0.98,0.99,1.01,1.04,1.05,1.09,1.1,1.115,1.15,1.2,1.22,1.29,1.33,1.37};

//nuevos valores interpolados con grafica excel:
//:co2:415,440,470,
//:volt 0.60,0.70,0.8,

//entradas
#define  interruptor_1 PORTDbits.RD3//PORTDbits.RD1
#define  interruptor_2  PORTDbits.RD2//PORTDbits.RD0
#define  pulsador_2 PORTDbits.RD4//PORTDbits.RD4  inputs=1 trisd 00011100
#define  zumbador PORTDbits.RD7
#define puente_1 PORTCbits.RC2


unsigned int CO2[21]={395,415,440,470,500,506,533,547,560,572,700,738,1012,1217,1276,1506,1905,2240,3325,4700,5700};
float volt[21]={0.5,0.60,0.70,0.8,0.90,0.91,0.96,0.98,0.99,1.01,1.04,1.05,1.09,1.1,1.115,1.15,1.2,1.22,1.29,1.33,1.37};

unsigned int sensor_CO2;
uint16_t raw_temp;
char  temp[] = "000.0000 C";
char buf[10];
float volts;
float interpol=0;
unsigned short numero=45000;  //límites unsigned short: 0 a 65535 -límite c02
//unsigned short numero=0xFFFF;  //límites: 0 a 65535
unsigned char address_2a = 0x00;//;direccion memoria eeprom
unsigned char address_2b = 0x01;//;
unsigned char address_1a = 0x02;//;
//alarmas
unsigned short MAX_CO2=1500;
char estado_zumbador='n';
//no usado
struct datos_eprom {
    int MAX_CO2;
    char alarma;
};

//funciones
unsigned int read_analog_channel(unsigned int); 
unsigned char getch();
uint8_t UART_GetC();


void setup(void);

__bit ds18b20_start()
{  
  DS18B20_PIN = 0;      // send reset pulse to the DS18B20 sensor
  DS18B20_PIN_Dir = 0;  // configure DS18B20_PIN pin as output
  __delay_us(500);      // wait 500 us
 
  DS18B20_PIN_Dir = 1;  // configure DS18B20_PIN pin as input
  __delay_us(100);  //100// wait 100 us to read the DS18B20 sensor response
 
  if (!DS18B20_PIN )
  { 
    __delay_us(400);    // wait 400 us
    return 1;           // DS18B20 sensor is present
  }
 
  return 0;   // connection error
}

void ds18b20_write_bit(uint8_t value)
{
  DS18B20_PIN = 0;
  DS18B20_PIN_Dir = 0;  // configure DS18B20_PIN pin as output
  __delay_us(2);        // wait 2 us

  DS18B20_PIN = (__bit)value;
  __delay_us(80);       // wait 80 us

  DS18B20_PIN_Dir = 1;  // configure DS18B20_PIN pin as input
  __delay_us(2);        // wait 2 us
}

void ds18b20_write_byte(uint8_t value)
{
  for(uint8_t i = 0; i < 8; i++)
    ds18b20_write_bit(value >> i);
}

__bit ds18b20_read_bit(void)
{
  static __bit value;

  DS18B20_PIN = 0;
  DS18B20_PIN_Dir = 0;  // configure DS18B20_PIN pin as output
  __delay_us(2);

  DS18B20_PIN_Dir = 1;  // configure DS18B20_PIN pin as input
  __delay_us(5);        // wait 5 us

  value = DS18B20_PIN;  // read and store DS18B20 state
  __delay_us(100);      // wait 100 us

  return value;
}

uint8_t ds18b20_read_byte(void)
{
  uint8_t value = 0;

  for(uint8_t i = 0; i < 8; i++)
    value |= ds18b20_read_bit() << i;

  return value;
}

__bit ds18b20_read(uint16_t *raw_temp_value)
{ 
  if (!ds18b20_start())   // send start pulse
    return 0;             // return 0 if error

  ds18b20_write_byte(0xCC);   // send skip ROM command
  ds18b20_write_byte(0x44);   // send start conversion command

  while(ds18b20_read_byte() == 0);  // wait for conversion complete

  if (!ds18b20_start())  // send start pulse
    return 0;            // return 0 if error

  ds18b20_write_byte(0xCC);  // send skip ROM command
  ds18b20_write_byte(0xBE);  // send read command

  // read temperature LSB byte and store it on raw_temp_value LSB byte
  *raw_temp_value  = ds18b20_read_byte();
  // read temperature MSB byte and store it on raw_temp_value MSB byte
  *raw_temp_value |= (uint16_t)(ds18b20_read_byte() << 8);
  
  return 1;   // OK --> return 1
}


char FuncionMenuMaxCo(void);
char FuncionLeeAlarmamOnOff(void);
unsigned short FuncionSeleccion(char letra);
unsigned short LeeMaxCo2Eeprom(void);

/*************************** main function *********************/
void main(void)
{

    OSCCON = 0x70;   // set internal oscillator to 8MHz
    setup(); // Configure the PIC
  
   


  I2C_Master_Init();
  LCD_Init(0x4E); // Initialize LCD module with I2C address = 0x4E
  //LCD_Init(0x27); 
  LCD_Set_Cursor(1, 1);
  //char num[12]="456";
  LCD_Write_String("Sensor C02 y temp.");
  LCD_Set_Cursor(2, 1);
  LCD_Write_String("con PIC18F4620");








   printf("\r\r\r\r\r\rInicio programa....");
    struct datos_eprom var1_eprom;
	
	
	
	
	

	//leer valor de eeprom y asignarlo a variable
    var1_eprom.MAX_CO2=1503;
    var1_eprom.alarma='n';
	printf("\r------------------");
    printf("\rPruebas struct max co2 eprom=%d",var1_eprom.MAX_CO2);
    printf("\rsizeof char :%d",sizeof(char));
	printf("\rsizeof int :%d",sizeof(int));
	printf("\rsizeof unsigned short:%d",sizeof(unsigned short));
  
  //guardamos valores en EEPROM:  
  
  
  /*
    volatile unsigned char value1 = 0x09;
    unsigned char address_1a = 0x00;//0xE5;
	eeprom_write(address_1a, value1);     // Writing value 0x9 to EEPROM address 0xE5        
    value1 = eeprom_read (address_1a); 
	
	
	volatile unsigned char value2 = 3;
    unsigned char address_1b = 0x01;//;
	eeprom_write(address_1b, value2);     // Writing value 0x9 to EEPROM address 0xE5        
    value2 = eeprom_read (address_1b);    // Reading the value from address 0xE5
  */
  	
  
  
 //si se ha activado interruptor de configuracion entrar en menú configuracion 
  if ( interruptor_2 == 1)
  { 
    

     printf("\r--------------------------------------------");
     printf("\r");
     printf("\r MENU CONFIG. MAX_CO2 Y ALARMA");
	 printf("\r");
     printf("\r--------------------------------------------");
	 
	 unsigned short s1=LeeMaxCo2Eeprom();
    // printf("\rMax_co:%d",max_co1,s1);
		
	char max_co1 =FuncionMenuMaxCo();
	unsigned short dato_max_co2=FuncionSeleccion(max_co1);
    printf("\rmax_co:%c sel:%d",max_co1,dato_max_co2);
	if (dato_max_co2)
    	{
		   printf("\rModificamos eeprom");
		  
     	   unsigned short numer=dato_max_co2;
		   
 		   volatile unsigned char nu21=numer>>8;
		   eeprom_write(address_2a, nu21);
		   volatile unsigned char nu22=numer & 0xFF;
	       
	       eeprom_write(address_2b, nu22); 
		   printf("\rnu1 y nu2 :%x ,%x",nu21,nu22);
	       //convertimos 2 char en numero decimal sin signo
	       unsigned short resultado10=(nu21<<8 )+ nu22;
	       printf("\r Resultado modifica eeprom:%u",resultado10);//%u para decimal sin signo
	       MAX_CO2=resultado10;
		   
    	}
	    else
	    {
		   printf("\rNo modificamos eeprom");
    	}
	
	
	
    char habilita_alarma=FuncionLeeAlarmamOnOff();
	eeprom_write(address_1a, habilita_alarma);
    printf("\r habilita alarma=%c",habilita_alarma );
     
		 
	    int segundos=3; //segundos de espera
	    for(int j=0;j<segundos;j++)
	    {
	       for(int i=0; i<20; i++)
		   {
             __delay_ms(50); // Pause for a second
           }
	    }
   }
   
  //Leer unsigned short en eeprom
	unsigned char p1=eeprom_read (address_2a); 
	unsigned char p2=eeprom_read (address_2b); 
	char lect_zumbador=eeprom_read (address_1a); 
	
	printf("\rnu1 y nu2 :%x ,%x",p1,p2);
	//convertimos 2 char en numero decimal sin signo
	unsigned short resultado9=(p1<<8 )+ p2;
	printf("\rLectura inicial eeprom co2:%u",resultado9);//%u para decimal sin signo
	MAX_CO2=resultado9;
	
	//char habilita_alarma=FuncionLeeAlarmamOnOff();
    printf("\r 2.habilita alarma=%c",lect_zumbador);
  
  
 for(int i=0; i<20; i++){
            __delay_ms(50); // Pause for a second
        }
 /*for(int j=32; j<256; j++){
	 printf("\rCaracter[%d]=%c",j,j);
            __delay_ms(50); // Pause for a second
        }
	*/	
  printf("\r----------------------------*****************");
  printf("\r");
  printf("\r Termometro y medidor de C02");
  printf("\r  by Jordi Guitard");
  printf("\r  version 1.0");
  printf("\r----------------------------*****************");
//  printf("\r valor1 del eeprom:%d",value1);
  
  temp[8] =186; //223;   // put degree symbol ( ° )

  while(1)
  {  


 //destinado a: RESERVA
      if (!puente_1)
       { 
        //printf("\r\rPuente RC2 cerrado");
		}
      else
	   {
		printf("\r\rPuente RC2 abierto. MODO DEBUG");
		printf("\r SET alarma MAX_CO2=%d",MAX_CO2);
		}  

       

     //leemos valor del sensor analogico;
     sensor_CO2 = read_analog_channel(0);
     volts=(sensor_CO2*1.01)/203;
	 	 
	/*busqueda y comparacion*/
	 int j=0;
     while ( volts > volt[j] )
     {j=j+1;}
     //gestion fuera de rango
     if (j==0)
	 { printf("\rError 10:lectura muy baja CO2 inf. a 395ppm");
 
         LCD_Set_Cursor(2, 1);
		 LCD_Write_String("Lectura baja CO2");
 
 
	 }
     else 
     if (j==21)
     printf("\rError 11:lectura muy alta CO2 sup.a 5700ppm");
     else
     {   //INTERPOLACION:
         printf("\rindice =%d; volt=%1.2fv ; ppm C02  %d- %d",j,volt[j],CO2[j-1],CO2[j]);
         interpol=((volts-volt[j-1])*(CO2[j]-CO2[j-1])/(volt[j]-volt[j-1]))+CO2[j-1];
         printf("\rCO2 (interpolado)=%4.0f ppm",interpol);
		 
		 //ftoa(num,1234);
		
         itoa(buf, interpol, 10);
		
		 
		// LCD_Clear();
		 
		 
		 
		 
		 
     }
	 

	 
	
       
	//formateado de la temperatura:
    if(ds18b20_read(&raw_temp) )
    {
      if(raw_temp & 0x8000)  // if the temperature is negative
      {
        temp[0] = '-';             // put minus sign (-)
        raw_temp = (~raw_temp) + 1;  // change temperature value to positive form
      }

      else
      {
        if((raw_temp >> 4) >= 100)  // if the temperature >= 100 °C
          temp[0] = '1';            // put 1 of hundreds
        else                        // otherwise
          temp[0] = ' '; //'0'           // put space ' '
      }

     // put the first two digits ( for tens and ones)
     // temp[1] = ( (raw_temp >> 4) / 10 ) % 10 + '0';  // put tens digit
     // temp[2] =   (raw_temp >> 4)        % 10 + '0';  // put ones digit
	  temp[1] = ( (raw_temp >> 1) / 10 ) % 10 + '0';  // put tens digit
      temp[2] =   (raw_temp >> 1)        % 10 + '0';  // put ones digit
	  // temp[1] = ( (raw_temp ) / 10 ) % 10 + '0';  // put tens digit
      //temp[2] =   (raw_temp )        % 10 + '0';  // put ones digit
	  
	  // put the 4 fraction digits (digits after the point)
      // why 625?  because we're working with 12-bit resolution (default resolution)
      temp[4] = ( (raw_temp & 0x0F) * 625) / 1000 + '0';          // put thousands digit
      temp[5] = (((raw_temp & 0x0F) * 625) / 100 ) % 10 + '0';    // put hundreds digit
      temp[6] = (((raw_temp & 0x0F) * 625) / 10 )  % 10 + '0';    // put tens digit
      temp[7] = ( (raw_temp & 0x0F) * 625) % 10 + '0';            // put ones digit

      printf("\rTemperatura:");
      printf(temp);    // print temp
	  
	     LCD_Clear();
		 LCD_Set_Cursor(1, 1);
         LCD_Write_String("Temp=");
		 
	  
		 LCD_Set_Cursor(1,5 );
         LCD_Write_String(temp);
		 
		 
		 LCD_Set_Cursor(2, 1);
		 LCD_Write_String("CO2=");
		 LCD_Set_Cursor(2, 6);
         LCD_Write_String(buf);
		 
		 LCD_Set_Cursor(2, 11);
         LCD_Write_String("ppm");
	  
	  
	  
    }

    else
    {
              // move cursor to column 4 row 2
      printf("\rError! Lectura Temperatura. ");
    }


 //led simple para indicar funcionamiento parpadeando
     led_verde=1;
	 
	 if (puente_1)
	 {
	 printf("\r388.MAX_CO2:%d lect zumba:%c",MAX_CO2,lect_zumbador);
	 }
	 
	 
	 if (interpol>MAX_CO2 && (lect_zumbador=='n'))
	 {   printf("\r389.alarma habilitada");
		 zumbador=1;
	 }
     for(int i=0; i<20; i++)
	   {
            __delay_ms(50); // Pause for a second
        }
		led_verde=0;
		zumbador=0;

    printf("\r ");
    for(int i=0; i<10; i++){
            __delay_ms(50); // Pause for a second
        }
   
  }

}

/*************************** end main function ********************************/
void setup(void)
{
   //contador=0;
    //enable_count=0;
	
    // Set clock frequency (section 2 of the PIC18F4620 Data Sheet)
    // Set Fosc = 8MHz, which gives Tcy = 0.5us
    //OSCCON = 0b01110000;//original
    //  OSCCON = 0b01100000; //0110=4mhz
    // Set up ADC (section 19 of the PIC18F4620 Data Sheet)
    // Enable AN0-7 as analog inputs
    ADCON1 = 0b00000111;
    // Left justified result, manual acquisition time,
    // AD clock source = 8 x Tosc
    ADCON2 = 0b00000001;
  
    // Set up Port B (section 9.2 of the PIC18F4620 Data Sheet)
    // RB0-5 outputs, RB6-7 inputs
    LATB = 0b00000000;
    TRISB = 0b11000000;
    TRISCbits.RC2=1; //input puente_1
    // Set up Port D (section 9.4 of the PIC18F4620 Data Sheet)
    //------------CAMBIAR --> RD0-3 digital outputs, RD4-7 digital inputs
    LATD = 0b00000000;
    //TRISD = 0b11110000;
    TRISD = 0b01111100;// 0b11111100
    //led_verde=0;//inicilmente apagar led
    
    
    // Set up PWM (section 15 of the PIC18F4620 Data Sheet)
    // Set PWM frequency=36kHz and duty cycle=50% on both channels
    CCP1CON = 0b00001100;   // PWM on CCP1
    CCP2CON = 0b00001100;   // PWM on CCP2
    TRISC = 0b11111001;     // CCP1, CCP2 as outputs
    T2CON = 0b00000100;     // Enable TMR2 with prescale = 1:1
    PR2 = 55;               // period = (PR2+1) * Tcy * prescale
    CCPR1L = 27;            // Ch 1 duty cycle = CCPR1L / PR2
    CCPR2L = 27;            // Ch 2 duty cycle = CCPR1L / PR2
     
     
    // Set up USART (section 18 of the PIC18F4620 Data Sheet)
    // baud_rate = Fosc/(16*(spbrg+1))
    //           = 8000000/(16*(207+1)) = 2403.846 bits/s
    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_OFF
        & USART_ASYNCH_MODE & USART_EIGHT_BIT
        & USART_CONT_RX & USART_BRGH_HIGH, 51);//valor 207 para 2400 baud, 51 para 9600, 25 para 9600 y 4000000
}

//funciones para puerto serie
void putch(char data) {    
     while (!TXIF)    
         continue;    
     TXREG = data;    
 }    
uint8_t UART_GetC()
   {     
         while  (RCIF == 0) ; //ESPERA RECIBIR DATO
        if (OERR)	//SI HAY ERROR
        {  //LIMPIA OVERRRUN BIT DE ERROR
			CREN=0;
			CREN=1;
		}	
       		
		return RCREG;
   }

 unsigned char getch(void)
 {/*retrieve one byte*/
    while (!RCIF)  /*SET WHEN REGISTER IS NOT EMPTY*/
	//_delay(100);
	continue;
    return RCREG;
 }
 
 // Read voltage from the specified channel.
// This function takes approximately 35us.
unsigned int read_analog_channel(unsigned int n)
{
    unsigned int voltage;
  
    ADCON0 = n << 2;
    ADCON0bits.ADON = 1;
    Delay10TCYx(3); // 15us charging time
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO); // Await result (11us approx)
  
    // Return the result (a number between 0 and 1023)
    voltage = ADRESH;
    voltage = (voltage << 2) + (ADRESL >> 6);
    return voltage;
}



char FuncionLeeAlarmamOnOff(void)
{
  char input;
   printf("\r");
    printf("\rMenu 2. Configuracion alarma ON\/OFF.");
    printf("\rIntroduce valor alarma o[n] of[f]:");
    //while('\n'!= UART_GetC());
    input= UART_GetC();
    //printf("\nfunc lee alarma:Has introducido %c",input);
    return input;

}

char FuncionMenuMaxCo(void)
{ char input;
    printf("\r\r");
	printf("\rMenu 1. Configuracion limite alarma CO2.");
    printf("\rIntroduce valor max_CO2:");
	printf("\r   [a].Alarma max. 1112ppm.");
	printf("\r   [b].Alarma max. 1606ppm.");
	printf("\r   [c].Alarma max. 2307ppm.");
	printf("\r   [d].Alarma max. 3509ppm.");
	printf("\r   [e].Salir sin cambios");
	printf("\r");
    input= UART_GetC();
    //printf("\nfunc lee max co:Has introducido %c",input);
    return input;
}

unsigned short FuncionSeleccion(char letra)
{ char let=letra;
  unsigned short result=0;
  switch (let)
     { case 'a':
          printf("\rselec-has introducido a");
          result=1112;
          break;
       case 'b':
          printf("\rselec-has introducido b");
          result=1606;
          break;
	   case 'c':
          printf("\rselec-has introducido c");
          result=2307;
          break;
	   case 'd':
          printf("\rselec-has introducido d");
          result=3509;
          break;
	   default:
          printf("\rNo se han modificado datos");
      
     }
     return result;

}

unsigned short LeeMaxCo2Eeprom(void)
{
	//printf("\r_________maxco2 %d",MAX_CO2);
	unsigned char p1=eeprom_read (address_2a); 
	unsigned char p2=eeprom_read (address_2b); 
	//convertimos 2 char en numero decimal sin signo
	unsigned short resultado9=(p1<<8 )+ p2;
	printf("\rLectura inicial eeprom co2:%u",resultado9);//%u para decimal sin signo
	//MAX_CO2=resultado9;
	return resultado9;
	
}
