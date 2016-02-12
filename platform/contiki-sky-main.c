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
#include "dev/slip.h"
#include "dev/uart1.h"
#include "dev/watchdog.h"
#include "dev/xmem.h"
#include "lib/random.h"
#include "net/netstack.h"
#include "net/mac/frame802154.h"

#if NETSTACK_CONF_WITH_IPV6
#include "net/ipv6/uip-ds6.h"
#endif /* NETSTACK_CONF_WITH_IPV6 */

#include "net/rime/rime.h"

#include "sys/node-id.h"
#include "cfs-coffee-arch.h"
#include "cfs/cfs-coffee.h"
#include "sys/autostart.h"

#if UIP_CONF_ROUTER

#ifndef UIP_ROUTER_MODULE
#ifdef UIP_CONF_ROUTER_MODULE
#define UIP_ROUTER_MODULE UIP_CONF_ROUTER_MODULE
#else /* UIP_CONF_ROUTER_MODULE */
#define UIP_ROUTER_MODULE rimeroute
#endif /* UIP_CONF_ROUTER_MODULE */
#endif /* UIP_ROUTER_MODULE */

extern const struct uip_router UIP_ROUTER_MODULE;
#endif /* UIP_CONF_ROUTER */

#if DCOSYNCH_CONF_ENABLED
static struct timer mgt_timer;
#endif
extern int msp430_dco_required;

#ifndef NETSTACK_CONF_WITH_IPV4
#define NETSTACK_CONF_WITH_IPV4 0
#endif

#if NETSTACK_CONF_WITH_IPV4
#include "net/ip/uip.h"
#include "net/ipv4/uip-fw.h"
#include "net/ipv4/uip-fw-drv.h"
#include "net/ipv4/uip-over-mesh.h"
static struct uip_fw_netif slipif =
  {UIP_FW_NETIF(192,168,1,2, 255,255,255,255, slip_send)};
static struct uip_fw_netif meshif =
  {UIP_FW_NETIF(172,16,0,0, 255,255,0,0, uip_over_mesh_send)};

#endif /* NETSTACK_CONF_WITH_IPV4 */

#define UIP_OVER_MESH_CHANNEL 8
#if NETSTACK_CONF_WITH_IPV4
static uint8_t is_gateway;
#endif /* NETSTACK_CONF_WITH_IPV4 */

#ifdef EXPERIMENT_SETUP
#include "experiment-setup.h"
#endif

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else /* DEBUG */
#define PRINTF(...)
#endif /* DEBUG */

void init_platform(void);

/*---------------------------------------------------------------------------*/
#if 0
int
force_float_inclusion()
{
  extern int __fixsfsi;
  extern int __floatsisf;
  extern int __mulsf3;
  extern int __subsf3;

  return __fixsfsi + __floatsisf + __mulsf3 + __subsf3;
}
#endif
/*---------------------------------------------------------------------------*/
void uip_log(char *msg) { puts(msg); }

/*---------------------------------------------------------------------------*/
#if 0
void
force_inclusion(int d1, int d2)
{
  snprintf(NULL, 0, "%d", d1 % d2);
}
#endif
/*---------------------------------------------------------------------------*/
static void
set_rime_addr(void)
{
  linkaddr_t addr;
  int i;

  memset(&addr, 0, sizeof(linkaddr_t));
#if NETSTACK_CONF_WITH_IPV6
  memcpy(addr.u8, ds2411_id, sizeof(addr.u8));
#else
  if(node_id == 0) {
    for(i = 0; i < sizeof(linkaddr_t); ++i) {
      addr.u8[i] = ds2411_id[7 - i];
    }
  } else {
    addr.u8[0] = node_id & 0xff;
    addr.u8[1] = node_id >> 8;
  }
#endif
  linkaddr_set_node_addr(&addr);
  PRINTF("Rime started with address ");
  for(i = 0; i < sizeof(addr.u8) - 1; i++) {
    PRINTF("%d.", addr.u8[i]);
  }
  PRINTF("%d\n", addr.u8[i]);
}
/*---------------------------------------------------------------------------*/
#if !PROCESS_CONF_NO_PROCESS_NAMES
static void
print_processes(struct process * const processes[])
{
  /*  const struct process * const * p = processes;*/
  printf("Starting");
  while(*processes != NULL) {
    printf(" '%s'", (*processes)->name);
    processes++;
  }
  putchar('\n');
}
#endif /* !PROCESS_CONF_NO_PROCESS_NAMES */
/*--------------------------------------------------------------------------*/
#if NETSTACK_CONF_WITH_IPV4
static void
set_gateway(void)
{
  if(!is_gateway) {
    leds_on(LEDS_RED);
    PRINTF("%d.%d: making myself the IP network gateway.\n\n",
	   linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
    PRINTF("IPv4 address of the gateway: %d.%d.%d.%d\n\n",
	   uip_ipaddr_to_quad(&uip_hostaddr));
    uip_over_mesh_set_gateway(&linkaddr_node_addr);
    uip_over_mesh_make_announced_gateway();
    is_gateway = 1;
  }
}
#endif /* NETSTACK_CONF_WITH_IPV4 */
/*---------------------------------------------------------------------------*/
static void
start_autostart_processes()
{
#if !PROCESS_CONF_NO_PROCESS_NAMES
  print_processes(autostart_processes);
#endif /* !PROCESS_CONF_NO_PROCESS_NAMES */
  autostart_start(autostart_processes);
}
/*---------------------------------------------------------------------------*/
#if NETSTACK_CONF_WITH_IPV6
static void
start_uip6()
{
  NETSTACK_NETWORK.init();
  
  process_start(&tcpip_process, NULL);

#if DEBUG
  PRINTF("Tentative link-local IPv6 address ");
  {
    uip_ds6_addr_t *lladdr;
    int i;
    lladdr = uip_ds6_get_link_local(-1);
    for(i = 0; i < 7; ++i) {
      PRINTF("%02x%02x:", lladdr->ipaddr.u8[i * 2],
             lladdr->ipaddr.u8[i * 2 + 1]);
    }
    PRINTF("%02x%02x\n", lladdr->ipaddr.u8[14], lladdr->ipaddr.u8[15]);
  }
#endif /* DEBUG */

  if(!UIP_CONF_IPV6_RPL) {
    uip_ipaddr_t ipaddr;
    int i;
    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
    uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
    uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
    PRINTF("Tentative global IPv6 address ");
    for(i = 0; i < 7; ++i) {
      PRINTF("%02x%02x:",
             ipaddr.u8[i * 2], ipaddr.u8[i * 2 + 1]);
    }
    PRINTF("%02x%02x\n",
           ipaddr.u8[7 * 2], ipaddr.u8[7 * 2 + 1]);
  }
}
#endif /* NETSTACK_CONF_WITH_IPV6 */
/*---------------------------------------------------------------------------*/
static void
start_network_layer()
{
#if NETSTACK_CONF_WITH_IPV6
  start_uip6();
#endif /* NETSTACK_CONF_WITH_IPV6 */
  start_autostart_processes();
  /* To support link layer security in combination with NETSTACK_CONF_WITH_IPV4 and
   * TIMESYNCH_CONF_ENABLED further things may need to be moved here */
}
/*---------------------------------------------------------------------------*/
#if WITH_TINYOS_AUTO_IDS
uint16_t TOS_NODE_ID = 0x1234; /* non-zero */
uint16_t TOS_LOCAL_ADDRESS = 0x1234; /* non-zero */
#endif /* WITH_TINYOS_AUTO_IDS */
int
main(int argc, char **argv)
{
  /*
   * Initalize hardware.
   */
  msp430_cpu_init();
  clock_init();
  leds_init();
  leds_on(LEDS_RED);


  uart1_init(BAUD2UBR(115200)); /* Must come before first printf */

  leds_on(LEDS_GREEN);
  ds2411_init(); // 0.034 mA -> 0.016 mA



  /* XXX hack: Fix it so that the 802.15.4 MAC address is compatible
     with an Ethernet MAC address - byte 0 (byte 2 in the DS ID)
     cannot be odd. */
  ds2411_id[2] &= 0xfe;

  leds_on(LEDS_BLUE);
  xmem_init(); //0.022 mA -> 0.016 mA


  leds_off(LEDS_RED);
  rtimer_init(); //0.016 mA

  /*
   * Hardware initialization done!
   */

  /* Initialize energest first (but after rtimer)
   */
  energest_init(); //0.016 mA
  ENERGEST_ON(ENERGEST_TYPE_CPU);


  node_id = 1;

  node_id_restore(); //using xmem



  /* for setting "hardcoded" IEEE 802.15.4 MAC addresses */
#ifdef IEEE_802154_MAC_ADDRESS
  {
    uint8_t ieee[] = IEEE_802154_MAC_ADDRESS;
    memcpy(ds2411_id, ieee, sizeof(uip_lladdr.addr));
    ds2411_id[7] = node_id & 0xff;
  }
#endif

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
    do {
      /* Reset watchdog. */
      watchdog_periodic();
      r = process_run();
    } while(r > 0);

    /*
     * Idle processing.
     */
    int s = splhigh();		/* Disable interrupts. */
    /* uart1_active is for avoiding LPM3 when still sending or receiving */
    if(process_nevents() != 0 || uart1_active()) {
      splx(s);			/* Re-enable interrupts. */
    } else {
      static unsigned long irq_energest = 0;

      /* Re-enable interrupts and go to sleep atomically. */
      ENERGEST_OFF(ENERGEST_TYPE_CPU);
      ENERGEST_ON(ENERGEST_TYPE_LPM);
      /* We only want to measure the processing done in IRQs when we
	 are asleep, so we discard the processing time done when we
	 were awake. */
      energest_type_set(ENERGEST_TYPE_IRQ, irq_energest);
      watchdog_stop();
      /* check if the DCO needs to be on - if so - only LPM 1 */
      if (msp430_dco_required) {
//      if(0){
	_BIS_SR(GIE | CPUOFF); /* LPM1 sleep for DMA to work!. */
      } else {
	_BIS_SR(GIE | SCG0 | SCG1 | CPUOFF); /* LPM3 sleep. This
						statement will block
						until the CPU is
						woken up by an
						interrupt that sets
						the wake up flag. */
        //0.03750 mA
      }
      /* We get the current processing time for interrupts that was
	 done during the LPM and store it for next time around.  */
      dint();
      irq_energest = energest_type_time(ENERGEST_TYPE_IRQ);
      eint();
      watchdog_start();
      ENERGEST_OFF(ENERGEST_TYPE_LPM);
      ENERGEST_ON(ENERGEST_TYPE_CPU);
    }
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
#if LOG_CONF_ENABLED
void
log_message(char *m1, char *m2)
{
  printf("%s%s\n", m1, m2);
}
#endif /* LOG_CONF_ENABLED */
