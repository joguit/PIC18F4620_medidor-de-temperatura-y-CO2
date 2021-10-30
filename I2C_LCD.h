/*LIBRERIAS I2C_LCD*/


#define _XTAL_FREQ 8000000 // Fosc

#define LCD_BACKLIGHT          0x08
#define LCD_NOBACKLIGHT        0x00
#define LCD_FIRST_ROW          0x80
#define LCD_SECOND_ROW         0xC0
#define LCD_THIRD_ROW          0x94
#define LCD_FOURTH_ROW         0xD4
#define LCD_CLEAR              0x01
#define LCD_RETURN_HOME        0x02
#define LCD_ENTRY_MODE_SET     0x04
#define LCD_CURSOR_OFF         0x0C
#define LCD_UNDERLINE_ON       0x0E
#define LCD_BLINK_CURSOR_ON    0x0F
#define LCD_MOVE_CURSOR_LEFT   0x10
#define LCD_MOVE_CURSOR_RIGHT  0x14
#define LCD_TURN_ON            0x0C
#define LCD_TURN_OFF           0x08
#define LCD_SHIFT_LEFT         0x18
#define LCD_SHIFT_RIGHT        0x1E
#define LCD_TYPE               2       // 0 -> 5x7 | 1 -> 5x10 | 2 -> 2 lines


#define I2C_BaudRate           100000
#define SCL_D                  TRISC3
#define SDA_D                  TRISC4





#define I2C_BaudRate 100000 // I2C Baud Rate = 100 Kbps


unsigned char RS, i2c_add, BackLight_State = LCD_BACKLIGHT;

//-----------------[ Functions' Prototypes ]----------------
extern void I2C_Master_Init(void);
extern void I2C_Wait(void);
extern void I2C_Start(void);
extern void I2C_Stop(void);
extern void I2C_Restart(void);
extern void I2C_ACK(void);
extern void I2C_NACK(void);
extern unsigned char I2C_Write(unsigned char Data);
//----------------------------------------------------------

//---[ LCD Routines ]---

extern void LCD_Init(unsigned char I2C_Add);
extern void IO_Expander_Write(unsigned char Data);
extern void LCD_Write_4Bit(unsigned char Nibble);
extern void LCD_CMD(unsigned char CMD);
extern void LCD_Set_Cursor(unsigned char ROW, unsigned char COL);
extern void LCD_Write_Char(char);
extern void LCD_Write_String(char*);
extern void Backlight();
extern void noBacklight();
extern void LCD_SR();
extern void LCD_SL();
extern void LCD_Clear();

//-----------------------------------------
void I2C_Master_Init()
{
  SSPCON1 = 0x28;
  SSPCON2 = 0x00;
  SSPSTAT = 0x00;
  SSPADD = ((_XTAL_FREQ/4)/I2C_BaudRate) - 1;
  TRISC3 = 1;
  TRISC4 = 1; 
}
void I2C_Wait()
{
  while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
}
void I2C_Start()
{
  //---[ Initiate I2C Start Condition Sequence ]---
  I2C_Wait();
  SEN = 1;
}
void I2C_Stop()
{
  //---[ Initiate I2C Stop Condition Sequence ]---
  I2C_Wait();
  PEN = 1;
}
void I2C_Restart()
{
  //---[ Initiate I2C Restart Condition Sequence ]---
  I2C_Wait();
  RSEN = 1;
}
void I2C_ACK(void)
{
  //---[ Send ACK - For Master Receiver Mode ]---
  I2C_Wait();
  ACKDT = 0; // 0 -> ACK, 1 -> NACK
  ACKEN = 1; // Send ACK Signal!
}
void I2C_NACK(void)
{
  //---[ Send NACK - For Master Receiver Mode ]---
  I2C_Wait();
  ACKDT = 1; // 1 -> NACK, 0 -> ACK
  ACKEN = 1; // Send NACK Signal!
}
unsigned char I2C_Write(unsigned char Data)
{
  //---[ Send Byte, Return The ACK/NACK ]---
  I2C_Wait();
  SSPBUF = Data;
  I2C_Wait();
  return ACKSTAT;
}

//---------------[ LCD Routines ]----------------
//------------------------------------------------------

void LCD_Init(unsigned char I2C_Add) 
{
  i2c_add = I2C_Add;
  IO_Expander_Write(0x00);
  __delay_ms(30);
  LCD_CMD(0x03);
  __delay_ms(5);
  LCD_CMD(0x03);
  __delay_ms(5);
  LCD_CMD(0x03);
  __delay_ms(5);
  LCD_CMD(LCD_RETURN_HOME);
  __delay_ms(5);
  LCD_CMD(0x20 | (LCD_TYPE << 2));
  __delay_ms(50);
  LCD_CMD(LCD_TURN_ON);
  __delay_ms(50);
  LCD_CMD(LCD_CLEAR);
  __delay_ms(50);
  LCD_CMD(LCD_ENTRY_MODE_SET | LCD_RETURN_HOME);
  __delay_ms(50);
}

void IO_Expander_Write(unsigned char Data) 
{
  I2C_Start();
  I2C_Write(i2c_add);
  I2C_Write(Data | BackLight_State);
  I2C_Stop();
}

void LCD_Write_4Bit(unsigned char Nibble) 
{
  // Get The RS Value To LSB OF Data  
  Nibble |= RS;
  IO_Expander_Write(Nibble | 0x04);
  IO_Expander_Write(Nibble & 0xFB);
  __delay_us(50);
}

void LCD_CMD(unsigned char CMD) 
{
  RS = 0; // Command Register Select
  LCD_Write_4Bit(CMD & 0xF0);
  LCD_Write_4Bit((CMD << 4) & 0xF0);
}

void LCD_Write_Char(char Data)
{
  RS = 1;  // Data Register Select
  LCD_Write_4Bit(Data & 0xF0);
  LCD_Write_4Bit((Data << 4) & 0xF0);
}

void LCD_Write_String(char* Str)
{
    for(int i=0; Str[i]!='\0'; i++)
       LCD_Write_Char(Str[i]); 
}

void LCD_Set_Cursor(unsigned char ROW, unsigned char COL) 
{    
  switch(ROW) 
  {
    case 2:
      LCD_CMD(0xC0 + COL-1);
      break;
    case 3:
      LCD_CMD(0x94 + COL-1);
      break;
    case 4:
      LCD_CMD(0xD4 + COL-1);
      break;
    // Case 1  
    default:
      LCD_CMD(0x80 + COL-1);
  }
}

void Backlight() 
{
  BackLight_State = LCD_BACKLIGHT;
  IO_Expander_Write(0);
}

void noBacklight() 
{
  BackLight_State = LCD_NOBACKLIGHT;
  IO_Expander_Write(0);
}

void LCD_SL()
{
  LCD_CMD(0x18);
  __delay_us(40);
}

void LCD_SR()
{
  LCD_CMD(0x1C);
  __delay_us(40);
}

void LCD_Clear()
{
  LCD_CMD(0x01); 
  __delay_us(40);
}