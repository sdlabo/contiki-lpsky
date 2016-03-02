#include "contiki.h"
#include <stdio.h>
#include "dev/leds.h"
#include "rime.h"

PROCESS(send_process, "send process");
AUTOSTART_PROCESSES(&send_process);

static void unicast_recv(struct unicast_conn *c)
{

}

static struct unicast_conn uc;
static const struct unicast_callbacks unicast_call = {unicast_recv};

PROCESS_THREAD(send_process, ev, data)
{
  static struct etimer et;
  linkaddr_t addr;

  PROCESS_BEGIN();

  unicast_open(&uc, 128, &unicast_call);

  while(1){
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    leds_toggle(LEDS_BLUE);
    packetbuf_copyfrom("hellohellohellohellohello", 30);
    addr.u8[0] = 1;
    addr.u8[1] = 0;
    printf("unicast sending\n");
    unicast_send(&uc, &addr);
  }

  PROCESS_END();
}
