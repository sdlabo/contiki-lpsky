/*
 * Copyright (c) 2015, Software Defined Laboratory.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Low power tools for sky mote (telosb)
 * \author
 *         Shunsuke Saruwatari <saru@inf.shizuoka.ac.jp>
 */

#include "contiki.h"
#include "contiki-conf.h"
#include "cc2420.h"
#include "cc2420_const.h"

void lpsky_lpm4()
{
  _BIS_SR(GIE | SCG0 | SCG1 | CPUOFF | OSCOFF); /* LPM4 */
}

void lpsky_lpm3()
{
  _BIS_SR(GIE | SCG0 | SCG1 | CPUOFF); /* LPM3 */
}

void lpsky_cc2420_on()
{
  cc2420_init();
}

void lpsky_cc2420_off()
{
  SET_VREG_INACTIVE();
}

#define SPI_FLASH_INS_WREN        0x06
#define SPI_FLASH_INS_WRDI        0x04
#define SPI_FLASH_INS_RDSR        0x05
#define SPI_FLASH_INS_WRSR        0x01
#define SPI_FLASH_INS_READ        0x03
#define SPI_FLASH_INS_FAST_READ   0x0b
#define SPI_FLASH_INS_PP          0x02
#define SPI_FLASH_INS_SE          0xd8
#define SPI_FLASH_INS_BE          0xc7
#define SPI_FLASH_INS_DP          0xb9
#define SPI_FLASH_INS_RES         0xab
#define M25P80_SIGNATURE          0x13

int lpsky_xmem_on()
{
  unsigned char signature;
  spl_t s;

  s = splhigh();
  SPI_FLASH_ENABLE();

  SPI_WRITE(SPI_FLASH_INS_RES);
  SPI_WRITE(0);                 /* Three dummy bytes */
  SPI_WRITE(0);                 /* Three dummy bytes */
  SPI_WRITE(0);                 /* Three dummy bytes */

  SPI_FLUSH();
  SPI_READ(signature);

  SPI_FLASH_DISABLE();
  splx(s);

  if(signature == M25P80_SIGNATURE){
    return 0;                   /* Success */
  }else{
    return -1;                  /* Fail */
  }
}

void lpsky_xmem_off()
{
  spl_t s;

  s = splhigh();
  SPI_FLASH_ENABLE();

  SPI_WRITE(SPI_FLASH_INS_DP);

  SPI_FLASH_DISABLE();
  splx(s);
}

void lspky_adc_on()
{
  ADC12CTL0 |= REFON;
  ADC12CTL0 |= ADC12ON;
}

void lpsky_adc_off()
{
  ADC12CTL0 &= ~ADC12ON;
  ADC12CTL0 &= ~REFON;
}

void lpsky_spi_on()
{
  SPI_FLASH_ENABLE();
  CC2420_SPI_ENABLE();
}

void lpsky_spi_off()
{
  SPI_FLASH_DISABLE();
  CC2420_SPI_DISABLE();
}
