/*
 * Copyright (c) 2006, Swedish Institute of Computer Science
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
 */

#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include "cc2420.h"
#include "dev/ds2411/ds2411.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "dev/uart1.h"
#include "dev/watchdog.h"
#include "dev/xmem.h"
#include "lib/random.h"
#include "net/netstack.h"
#include "net/mac/frame802154.h"
#include "net/rime/rime.h"

#include "sys/node-id.h"
#include "cfs-coffee-arch.h"
#include "cfs/cfs-coffee.h"
#include "sys/autostart.h"

extern int msp430_dco_required;

#define UIP_OVER_MESH_CHANNEL 8

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else /* DEBUG */
#define PRINTF(...)
#endif /* DEBUG */

void init_platform(void);

// node_idが設定されていなかったらds2411から読み込むみたい
// でもds2411は使いたくないのでここはいらない
// ここでnodeidをrimeに設定している
/*---------------------------------------------------------------------------*/
static void set_rime_addr(void)
{
  linkaddr_t addr;
  int i;

  memset(&addr, 0, sizeof(linkaddr_t));

  if(node_id == 0){
    for(i = 0; i < sizeof(linkaddr_t); ++i) {
      addr.u8[i] = ds2411_id[7 - i];
    }
  }else{
    addr.u8[0] = node_id & 0xff;
    addr.u8[1] = node_id >> 8;
  }

  linkaddr_set_node_addr(&addr);
  PRINTF("Rime started with address ");
  for(i = 0; i < sizeof(addr.u8) - 1; i++) {
    PRINTF("%d.", addr.u8[i]);
  }

  PRINTF("%d\n", addr.u8[i]);
}
/*---------------------------------------------------------------------------*/
#if DEBUG
static void print_processes(struct process * const processes[])
{
  /*  const struct process * const * p = processes;*/
  printf("Starting");
  while(*processes != NULL) {
    printf(" '%s'", (*processes)->name);
    processes++;
  }
  putchar('\n');
}
#endif /* DEBUG */
/*--------------------------------------------------------------------------*/
static void start_autostart_processes()
{
#if DEBUG
  print_processes(autostart_processes);
#endif /* DEBUG */
  autostart_start(autostart_processes);
}
/*---------------------------------------------------------------------------*/
static void start_network_layer()
{
  start_autostart_processes();
  /* To support link layer security in combination with
     NETSTACK_CONF_WITH_IPV4 and
   * TIMESYNCH_CONF_ENABLED further things may need to be moved here */
}
/*---------------------------------------------------------------------------*/

void hardware_init()
{
  /*
   * Initalize hardware.
   */
  msp430_cpu_init();
  clock_init();
  leds_init();
  leds_on(LEDS_RED);

#if DEBUG
  uart1_init(BAUD2UBR(115200)); /* Must come before first printf */
#endif /* DEBUG */

  leds_on(LEDS_GREEN);
  ds2411_init(); // 0.034 mA -> 0.016 mA


  leds_on(LEDS_BLUE);
  xmem_init(); //0.022 mA -> 0.016 mA


  leds_off(LEDS_RED);
  rtimer_init(); //0.016 mA

  /*
   * Hardware initialization done!
   */
}

int address_init()
{
  /* XXX hack: Fix it so that the 802.15.4 MAC address is compatible
     with an Ethernet MAC address - byte 0 (byte 2 in the DS ID)
     cannot be odd. */
  ds2411_id[2] &= 0xfe;

  node_id = 1;
  node_id_restore(); //using xmem
}

int main(int argc, char **argv)
{
  hardware_init();
  address_init();
  random_init(ds2411_id[0] + node_id);

  leds_off(LEDS_BLUE);
  /*
   * Initialize Contiki and our processes.
   */
  process_init(); //0.016 mA

  process_start(&etimer_process, NULL);

  ctimer_init(); //0.040 mA -> 0.017

  init_platform(); //0.017 mA

  set_rime_addr(); //0.016 mA

  cc2420_init(); //0.4375 mA -> 0.016 mA


  {
    uint8_t longaddr[8];
    uint16_t shortaddr;

    shortaddr = (linkaddr_node_addr.u8[0] << 8) +
      linkaddr_node_addr.u8[1];
    memset(longaddr, 0, sizeof(longaddr));
    linkaddr_copy((linkaddr_t *)&longaddr, &linkaddr_node_addr);
    PRINTF("MAC %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x ",
           longaddr[0], longaddr[1], longaddr[2], longaddr[3],
           longaddr[4], longaddr[5], longaddr[6], longaddr[7]);

    cc2420_set_pan_addr(IEEE802154_PANID, shortaddr, longaddr); //0.016 mA
  }

  PRINTF(CONTIKI_VERSION_STRING " started. ");
  if(node_id > 0) {
    PRINTF("Node id is set to %u.\n", node_id);
  } else {
    PRINTF("Node id is not set.\n");
  }

  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init(); //0.016 mA

  PRINTF("%s %s %s, channel check rate %lu Hz, radio channel %u\n",
         NETSTACK_LLSEC.name, NETSTACK_MAC.name, NETSTACK_RDC.name,
         CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0? 1:
                         NETSTACK_RDC.channel_check_interval()),
         CC2420_CONF_CHANNEL);

  uart1_set_input(serial_line_input_byte);
  serial_line_init(); //0.016 mA

  leds_off(LEDS_GREEN);

  watchdog_start(); //0.016 mA

  NETSTACK_LLSEC.bootstrap(start_network_layer);

  /*
   * This is the scheduler loop.
   */

  /*  watchdog_stop();*/
  while(1) {
    int r;
    do{
      /* Reset watchdog. */
      watchdog_periodic();
      r = process_run();
    }while(r > 0);

    /*
     * Idle processing.
     */
    int s = splhigh();        /* Disable interrupts. */
    /* uart1_active is for avoiding LPM3 when still sending or receiving */
    if(process_nevents() != 0 || uart1_active()){
      splx(s);            /* Re-enable interrupts. */
    }else{
      watchdog_stop();
      /* check if the DCO needs to be on - if so - only LPM 1 */
      if(msp430_dco_required){
        // ここでエラーメッセージを吐くようにするという手もある
        // uartがONになっている時だけここに来るはず
        _BIS_SR(GIE | CPUOFF); /* LPM1 sleep for DMA to work!. */
      } else {
        /* LPM3 sleep. This statement will block until the CPU is
           woken up by an interrupt that sets the wake up flag. */
        _BIS_SR(GIE | SCG0 | SCG1 | CPUOFF); 
        //0.03750 mA
      }
      /* We get the current processing time for interrupts that was
         done during the LPM and store it for next time around.  */
      dint();
      eint();
      watchdog_start();
    }
  }

  return 0;
}

/*---------------------------------------------------------------------------*/
