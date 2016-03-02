#include "contiki.h"
#include <stdio.h>
#include "dev/leds.h"
#include "rime.h"

PROCESS(recv_process, "recv process");
AUTOSTART_PROCESSES(&recv_process);

static void unicast_recv(struct unicast_conn *c, const linkaddr_t *from)
{
  printf("unicast message received from %d.%d\n",
         from->u8[0], from->u8[1]);
  printf("unicast message received '%s'\n", (char *)packetbuf_dataptr());
}

static struct unicast_conn uc;
static const struct unicast_callbacks unicast_call = {unicast_recv};

PROCESS_THREAD(recv_process, ev, data)
{
  static struct etimer et;

  PROCESS_BEGIN();

  unicast_open(&uc, 128, &unicast_call);

  while(1){
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    leds_toggle(LEDS_BLUE);
  }

  PROCESS_END();
}
