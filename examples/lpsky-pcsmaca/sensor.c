#include "contiki.h"
#include <stdio.h>
#include "dev/leds.h"
#include "rime.h"

PROCESS(test03_process, "test03 process");
AUTOSTART_PROCESSES(&test03_process);

static void unicast_recv(struct unicast_conn *c)
{

}

static struct unicast_conn uc;
static const struct unicast_callbacks unicast_call = {unicast_recv};

PROCESS_THREAD(test03_process, ev, data)
{
  static struct etimer et;
  linkaddr_t addr;

  PROCESS_BEGIN();

  unicast_open(&uc, 128, &unicast_call);

  while(1){
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    leds_toggle(LEDS_BLUE);
    packetbuf_copyfrom("hello", 6);
    addr.u8[0] = 2;
    addr.u8[1] = 0;
    unicast_send(&uc, &addr);
  }

  PROCESS_END();
}
