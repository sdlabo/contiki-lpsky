#include "contiki.h"
#include "dev/leds.h"
#include "net/netstack.h"
#include "contiki-lpsky.h"
#include <stdio.h>


PROCESS(lpsky_sample_process, "lpsky sample process");
AUTOSTART_PROCESSES(&lpsky_sample_process);


PROCESS_THREAD(lpsky_sample_process, ev, data)
{

  static struct etimer et;
  PROCESS_BEGIN();

  while(1){
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    leds_off(LEDS_BLUE);

    NETSTACK_MAC.off(0);
    lpsky_cc2420_off();
    lpsky_xmem_off();
    lpsky_lpm4();

    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    leds_on(LEDS_BLUE);
  }

  PROCESS_END();
}
