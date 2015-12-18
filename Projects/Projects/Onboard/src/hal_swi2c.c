#include "hal_board.h"
#include "hal_swi2c.h"

/*
#define Delay5us()  for(int i = 0; i < 9; i++)asm("nop");
#define Set_Pin(pin, set) if(pin == SDA){P0_6 = set;}else{P0_7 = set;}
#define Get_Pin(pin)  (pin == SDA)?P0_6:P0_7
*/

#define SDA 1
#define SCL 0

#define Delay5us() asm("nop")
#define Set_Pin(pin, set) if( pin == SDA ){ P0_6 = set;}else{P0_7 = set;}
#define Get_Pin(pin)  (pin == SDA)?P0_6:P0_7

#define Set_SDA_Input()   do {  P0DIR &= ~0x40; } while ( 0 ) 
#define Set_SDA_Output()  do {  P0DIR |= 0x40; } while ( 0 )


static void I2C_Start( void )
{
    Set_Pin(SDA, 1);                    //拉高数据线
    Set_Pin(SCL, 1);                    //拉高时钟线
    Delay5us();                         //延时
    Set_Pin(SDA, 0);                    //产生下降沿
    Delay5us();                         //延时
    Set_Pin(SCL, 0);                    //拉低时钟线
}

/**************************************
停止信号
**************************************/
static void I2C_Stop(void)
{
    Set_Pin(SDA, 0);                    //拉低数据线
    Set_Pin(SCL, 1);                    //拉高时钟线
    Delay5us();                         //延时
    Set_Pin(SDA, 1);                    //产生上升沿
    Delay5us();                         //延时
}

/**************************************
发送应答信号
入口参数:ack (0:ACK 1:NAK)
**************************************/
static void I2C_SendACK( bool nack )
{
    if ( nack ) 
    {
      Set_Pin( SDA,1 );                 //写应答信号
    }
    else
    {
      Set_Pin( SDA,0 );
    }
    Set_Pin(SCL, 1);                    //拉高时钟线
    Delay5us();                         //延时
    Set_Pin(SCL, 0);                    //拉低时钟线
    Delay5us();                         //延时
}

/**************************************
接收应答信号
**************************************/
static bool I2C_RecvACK(void)
{
    bool cy = 0;
    Set_Pin(SCL, 1);                     //拉高时钟线
    Set_SDA_Input() ; //P0DIR &= 0xbf;
    Delay5us();                          //延时
    cy = Get_Pin(SDA);                   //读应答信号
    Set_SDA_Output() ; //P0DIR |= 0xff;
    Set_Pin(SCL, 0);                     //拉低时钟线
    Delay5us();                          //延时

    return cy;
}

/**************************************
向IIC总线发送一个字节数据
**************************************/
static void I2C_SendByte( uint8 dat )
{
    uint8 i;
    uint8 mask = 0x80;
    for ( i = 0; i< 8; i++ )               //8位计数器
    {
        if( dat & mask )                  //移出数据的最高位
        {
            Set_Pin(SDA, 1);
        }
        else
        {
            Set_Pin(SDA, 0);
        }			        //送数据口
        mask >>= 1;
        Set_Pin(SCL, 1);                //拉高时钟线
        Delay5us();                     //延时
        Set_Pin(SCL, 0);                //拉低时钟线
        Delay5us();                     //延时
    }
    I2C_RecvACK();
}

/**************************************
从IIC总线接收一个字节数据
**************************************/
static uint8 I2C_RecvByte( void )
{
    uint8 i;
    uint8 dat = 0;

    Set_Pin(SDA, 1);                    //使能内部上拉,准备读取数据,
    for ( i=0; i<8; i++ )         //8位计数器
    {
        dat <<= 1;
        Set_Pin( SCL, 1 );                //拉高时钟线
        Set_SDA_Input() ;   // P0DIR &= 0xbf;
        Delay5us();             //延时
        dat |= Get_Pin(SDA);             //读数据               
        Set_SDA_Output() ; //P0DIR |= 0xff;
        Set_Pin( SCL, 0 );                //拉低时钟线
        Delay5us();             //延时
    }
    return dat;
}

//***************************************************

void  halI2CHWInit( void )
{
  P0DIR |= 0xc0 ;
  
  Set_Pin(SDA, 1);                    //拉高数据线
  Set_Pin(SCL, 1);                    //拉高时钟线
}

void  HalI2CWrite( uint8 slaveAddr, uint8 regAddr,uint8 regData)
{
    I2C_Start();                    //起始信号
    I2C_SendByte( slaveAddr<<1 );   //发送设备地址+写信号
    I2C_SendByte( regAddr );        //内部寄存器地址，请参考中文pdf 
    I2C_SendByte( regData );        //内部寄存器数据，请参考中文pdf
    I2C_Stop();                     //发送停止信号
}

//********单字节读取内部寄存器*************************
uint8 HalI2CRead( uint8 slaveAddr,uint8 regAddr )
{  
    uint8 data;
    I2C_Start();                           //起始信号
    I2C_SendByte( slaveAddr <<1 );         //发送设备地址+写信号
    I2C_SendByte( regAddr );               //发送存储单元地址
    I2C_Start();                           //起始信号
    I2C_SendByte( (slaveAddr<<1)|0x01 );   //发送设备地址+读信号
    data = I2C_RecvByte();                 //读出寄存器数据
    I2C_SendACK(1);   
    I2C_Stop();                            //停止信号
    return data; 
}
//******************************************************
//
//******************************************************

void  HalI2CMulWrite( uint8 slaveAddr,uint8 regAddr,uint8 * buf,uint8 n )
{   
    volatile uint8 i = 0 ;
    I2C_Start();                          //起始信号
    I2C_SendByte( slaveAddr<<1 );         //发送设备地址+写信号
    I2C_SendByte( regAddr );              //发送存储单元地址
    for ( i = 0; i < n; i ++ )            //连续读取n个地址数据，存储中BUF
    {
       I2C_SendByte( buf[i] );		
    }
    I2C_Stop();                           //停止信号
    Delay5us();
}

void  HalI2CMulRead( uint8 slaveAddr,uint8 regAddr, uint8 * buf,uint8 n )
{
    volatile uint8 i;
    uint8 last = n - 1 ; 
    I2C_Start();                            //起始信号
    I2C_SendByte( slaveAddr<<1 );           //发送设备地址+写信号
    I2C_SendByte( regAddr );                //发送存储单元地址
    I2C_Start();                            //起始信号
    I2C_SendByte( (slaveAddr<<1)|0x01 );    //发送设备地址+读信号
    for ( i = 0; i< n ; i ++ )              //连续读取n个地址数据，存储中BUF
    {
      buf[i] = I2C_RecvByte();
      if ( i == last )
      {
         I2C_SendACK( 1 );                  //最后一个数据需要回NOACK
      }
      else
      {
        I2C_SendACK(0);                     //回应ACK
      }
    }
   I2C_Stop();                              //停止信号
   Delay5us();	
}

