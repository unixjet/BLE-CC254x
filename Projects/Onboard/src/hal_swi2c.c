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
    Set_Pin(SDA, 1);                    //����������
    Set_Pin(SCL, 1);                    //����ʱ����
    Delay5us();                         //��ʱ
    Set_Pin(SDA, 0);                    //�����½���
    Delay5us();                         //��ʱ
    Set_Pin(SCL, 0);                    //����ʱ����
}

/**************************************
ֹͣ�ź�
**************************************/
static void I2C_Stop(void)
{
    Set_Pin(SDA, 0);                    //����������
    Set_Pin(SCL, 1);                    //����ʱ����
    Delay5us();                         //��ʱ
    Set_Pin(SDA, 1);                    //����������
    Delay5us();                         //��ʱ
}

/**************************************
����Ӧ���ź�
��ڲ���:ack (0:ACK 1:NAK)
**************************************/
static void I2C_SendACK( bool nack )
{
    if ( nack ) 
    {
      Set_Pin( SDA,1 );                 //дӦ���ź�
    }
    else
    {
      Set_Pin( SDA,0 );
    }
    Set_Pin(SCL, 1);                    //����ʱ����
    Delay5us();                         //��ʱ
    Set_Pin(SCL, 0);                    //����ʱ����
    Delay5us();                         //��ʱ
}

/**************************************
����Ӧ���ź�
**************************************/
static bool I2C_RecvACK(void)
{
    bool cy = 0;
    Set_Pin(SCL, 1);                     //����ʱ����
    Set_SDA_Input() ; //P0DIR &= 0xbf;
    Delay5us();                          //��ʱ
    cy = Get_Pin(SDA);                   //��Ӧ���ź�
    Set_SDA_Output() ; //P0DIR |= 0xff;
    Set_Pin(SCL, 0);                     //����ʱ����
    Delay5us();                          //��ʱ

    return cy;
}

/**************************************
��IIC���߷���һ���ֽ�����
**************************************/
static void I2C_SendByte( uint8 dat )
{
    uint8 i;
    uint8 mask = 0x80;
    for ( i = 0; i< 8; i++ )               //8λ������
    {
        if( dat & mask )                  //�Ƴ����ݵ����λ
        {
            Set_Pin(SDA, 1);
        }
        else
        {
            Set_Pin(SDA, 0);
        }			        //�����ݿ�
        mask >>= 1;
        Set_Pin(SCL, 1);                //����ʱ����
        Delay5us();                     //��ʱ
        Set_Pin(SCL, 0);                //����ʱ����
        Delay5us();                     //��ʱ
    }
    I2C_RecvACK();
}

/**************************************
��IIC���߽���һ���ֽ�����
**************************************/
static uint8 I2C_RecvByte( void )
{
    uint8 i;
    uint8 dat = 0;

    Set_Pin(SDA, 1);                    //ʹ���ڲ�����,׼����ȡ����,
    for ( i=0; i<8; i++ )         //8λ������
    {
        dat <<= 1;
        Set_Pin( SCL, 1 );                //����ʱ����
        Set_SDA_Input() ;   // P0DIR &= 0xbf;
        Delay5us();             //��ʱ
        dat |= Get_Pin(SDA);             //������               
        Set_SDA_Output() ; //P0DIR |= 0xff;
        Set_Pin( SCL, 0 );                //����ʱ����
        Delay5us();             //��ʱ
    }
    return dat;
}

//***************************************************

void  halI2CHWInit( void )
{
  P0DIR |= 0xc0 ;
  
  Set_Pin(SDA, 1);                    //����������
  Set_Pin(SCL, 1);                    //����ʱ����
}

void  HalI2CWrite( uint8 slaveAddr, uint8 regAddr,uint8 regData)
{
    I2C_Start();                    //��ʼ�ź�
    I2C_SendByte( slaveAddr<<1 );   //�����豸��ַ+д�ź�
    I2C_SendByte( regAddr );        //�ڲ��Ĵ�����ַ����ο�����pdf 
    I2C_SendByte( regData );        //�ڲ��Ĵ������ݣ���ο�����pdf
    I2C_Stop();                     //����ֹͣ�ź�
}

//********���ֽڶ�ȡ�ڲ��Ĵ���*************************
uint8 HalI2CRead( uint8 slaveAddr,uint8 regAddr )
{  
    uint8 data;
    I2C_Start();                           //��ʼ�ź�
    I2C_SendByte( slaveAddr <<1 );         //�����豸��ַ+д�ź�
    I2C_SendByte( regAddr );               //���ʹ洢��Ԫ��ַ
    I2C_Start();                           //��ʼ�ź�
    I2C_SendByte( (slaveAddr<<1)|0x01 );   //�����豸��ַ+���ź�
    data = I2C_RecvByte();                 //�����Ĵ�������
    I2C_SendACK(1);   
    I2C_Stop();                            //ֹͣ�ź�
    return data; 
}
//******************************************************
//
//******************************************************

void  HalI2CMulWrite( uint8 slaveAddr,uint8 regAddr,uint8 * buf,uint8 n )
{   
    volatile uint8 i = 0 ;
    I2C_Start();                          //��ʼ�ź�
    I2C_SendByte( slaveAddr<<1 );         //�����豸��ַ+д�ź�
    I2C_SendByte( regAddr );              //���ʹ洢��Ԫ��ַ
    for ( i = 0; i < n; i ++ )            //������ȡn����ַ���ݣ��洢��BUF
    {
       I2C_SendByte( buf[i] );		
    }
    I2C_Stop();                           //ֹͣ�ź�
    Delay5us();
}

void  HalI2CMulRead( uint8 slaveAddr,uint8 regAddr, uint8 * buf,uint8 n )
{
    volatile uint8 i;
    uint8 last = n - 1 ; 
    I2C_Start();                            //��ʼ�ź�
    I2C_SendByte( slaveAddr<<1 );           //�����豸��ַ+д�ź�
    I2C_SendByte( regAddr );                //���ʹ洢��Ԫ��ַ
    I2C_Start();                            //��ʼ�ź�
    I2C_SendByte( (slaveAddr<<1)|0x01 );    //�����豸��ַ+���ź�
    for ( i = 0; i< n ; i ++ )              //������ȡn����ַ���ݣ��洢��BUF
    {
      buf[i] = I2C_RecvByte();
      if ( i == last )
      {
         I2C_SendACK( 1 );                  //���һ��������Ҫ��NOACK
      }
      else
      {
        I2C_SendACK(0);                     //��ӦACK
      }
    }
   I2C_Stop();                              //ֹͣ�ź�
   Delay5us();	
}

