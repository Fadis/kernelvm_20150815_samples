#include "LPC11xx.h"
#include "core_cm0.h"
#include "globals.hpp"

enum i2c_state {
  I2C_IDLE,
  I2C_WR_STARTED,
  I2C_RD_STARTED,
  DATA_NACK
};

#define I2CONSET_AA (1<<2)
#define I2CONSET_SI (1<<3)
#define I2CONSET_STO (1<<4)
#define I2CONSET_STA (1<<5)
#define I2CONSET_I2EN (1<<6)

#define I2CONCLR_AAC (1<<2)
#define I2CONCLR_SIC (1<<3)
#define I2CONCLR_STAC (1<<5)
#define I2CONCLR_I2ENC (1<<6)

volatile i2c_state I2CSlaveState = I2C_IDLE;
volatile uint8_t system_state = 0x35;

extern "C"
__attribute__((interrupt("IRQ")))
void I2C_IRQHandler(void) 
{
  //ステータス値を受け取り
  uint8_t StatValue = LPC_I2C->STAT;
  switch ( StatValue ) {
	case 0x60:					/* An own SLA_W has been received. */
	case 0x68:
	LPC_I2C->CONSET = I2CONSET_AA;	/* assert ACK after SLV_W is received */
	LPC_I2C->CONCLR = I2CONCLR_SIC;
	I2CSlaveState = I2C_WR_STARTED;
	break;

	case 0x80:					/*  data receive */
	case 0x90:
	if ( I2CSlaveState == I2C_WR_STARTED ) {
	  rbuf.push( LPC_I2C->DAT );
	  LPC_I2C->CONSET = I2CONSET_AA;	/* assert ACK after data is received */
	}
	else {
	  LPC_I2C->CONCLR = I2CONCLR_AAC;	/* assert NACK */
	}
	LPC_I2C->CONCLR = I2CONCLR_SIC;
	break;

	case 0xA8:	// An own SLA_R has been received.
	case 0xB0:	//arbitration lost
	LPC_I2C->CONSET = I2CONSET_AA;		/* assert ACK  */
	LPC_I2C->DAT = system_state;//write the data こいつをここに入れればＯＫ
	LPC_I2C->CONCLR = I2CONCLR_SIC;
	I2CSlaveState = I2C_RD_STARTED;
	break;

	case 0xB8:	// Data byte has been transmitted
	case 0xC8:	// Last data byte

	if ( I2CSlaveState == I2C_RD_STARTED )
	{
	  LPC_I2C->DAT = system_state; //write the data こいつをここに入れればＯＫ
	  LPC_I2C->CONSET = I2CONSET_AA;		/* assert ACK  */
	}
	else
	{
	  LPC_I2C->CONCLR = I2CONCLR_AAC;		/* assert NACK  */
	}
	LPC_I2C->CONCLR = I2CONCLR_SIC;
	break;

	case 0xC0:					/* Data byte has been transmitted, NACK */
	//LPC_I2C->CONCLR = I2CONCLR_AAC;			/* assert NACK  */
	LPC_I2C->CONCLR = I2CONCLR_SIC;
	//I2CSlaveState = DATA_NACK;
	break;

	case 0xA0:					/* Stop condition or repeated start has */
	LPC_I2C->CONSET = I2CONSET_AA;	/* been received, assert ACK.  */
	LPC_I2C->CONCLR = I2CONCLR_SIC;
	I2CSlaveState = I2C_IDLE;
	break;

	default:
	LPC_I2C->CONCLR = I2CONCLR_SIC;
	LPC_I2C->CONSET = I2CONSET_I2EN | I2CONSET_SI;
	break;
  }

  return;
}


void I2CSlaveInit( void ) 
{

  LPC_SYSCON->PRESETCTRL |= (0x1<<1);

  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<5);
  LPC_IOCON->PIO0_4 &= ~0x3F;	/*  I2C I/O config */
  LPC_IOCON->PIO0_4 |= 0x01;		/* I2C SCL */
  LPC_IOCON->PIO0_5 &= ~0x3F;	
  LPC_IOCON->PIO0_5 |= 0x01;		/* I2C SDA */

  /*--- Clear flags ---*/
  LPC_I2C->CONCLR = I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC | I2CONCLR_I2ENC;

  /*--- Reset registers ---*/
  LPC_I2C->SCLL   = 240;//  48MHzでBitRate=100kHzになるように設定した。
  LPC_I2C->SCLH   = 240;//

  LPC_I2C->ADR0 = 0x40;//スレーブアドレスの設定はココ
  I2CSlaveState = I2C_IDLE;
  
  /* Enable the I2C Interrupt */
  NVIC_EnableIRQ(I2C_IRQn);

  LPC_I2C->CONSET = I2CONSET_I2EN | I2CONSET_SI;
  return;
}
