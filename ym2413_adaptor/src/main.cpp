/********************************************************************************
 * Copyright (c) 2015 Naomasa Matsubayashi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ******************************************************************************/
#include <cstdio>
#include "LPC11xx.h"
#include "core_cm0.h"
#include "i2c.hpp"
#include "globals.hpp"

int main() {
  LPC_IOCON->PIO0_2 = (1<<4)|(1<<6)|(1<<7);
  LPC_IOCON->PIO0_3 = (1<<4)|(1<<6)|(1<<7);
  LPC_IOCON->PIO0_7 = (1<<4)|(1<<6)|(1<<7);
  LPC_IOCON->R_PIO1_0 = (1<<0)|(1<<4)|(1<<6)|(1<<7);
  LPC_IOCON->R_PIO1_1 = (1<<0)|(1<<4)|(1<<6)|(1<<7);
  LPC_IOCON->R_PIO1_2 = (1<<0)|(1<<4)|(1<<6)|(1<<7);
  LPC_IOCON->SWDIO_PIO1_3 = (1<<0)|(1<<4)|(1<<6)|(1<<7);
  LPC_IOCON->PIO1_4 = (1<<4)|(1<<6)|(1<<7);
  LPC_IOCON->PIO1_5 = (1<<4)|(1<<6)|(1<<7);
  LPC_IOCON->PIO1_6 = (1<<4)|(1<<6)|(1<<7);
  LPC_IOCON->PIO1_7 = (1<<4)|(1<<6)|(1<<7);
  LPC_IOCON->PIO1_8 = (1<<4)|(1<<6)|(1<<7);
  LPC_IOCON->PIO1_9 = (1<<4)|(1<<6)|(1<<7);
  LPC_IOCON->PIO1_10 = (1<<4)|(1<<6)|(1<<7);
  LPC_GPIO1->DIR =0x000007FF;
  LPC_GPIO0->DIR =(1<<2)|(1<<3)|(1<<7);
//  LPC_GPIO1->DATA = 0x000007FF;
  SysTick_Config( SystemCoreClock/1000*5 );
  I2CSlaveInit();
  while( 1 ) {
    if( !rbuf.empty() ) {
      synth.push( rbuf.pop() );
    }
  }
}

