#ifndef LPC1114_I2C_HPP
#define LPC1114_I2C_HPP

extern volatile uint8_t system_state;

extern "C"
__attribute__((interrupt("IRQ")))
void I2C_IRQHandler(void);
void I2CSlaveInit( void );

#endif

