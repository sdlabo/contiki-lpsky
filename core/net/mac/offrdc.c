/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
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
 *         A off RDC implementation that uses framer for headers.
 * \author
 *         Shunsuke Saruwatari <saru@inf.shizuoka.ac.jp>
 */

#include "net/mac/mac-sequence.h"
#include "net/mac/offrdc.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/netstack.h"
#include "net/rime/rimestats.h"
#include "sys/node-id.h"
#include <string.h>

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifdef NULLRDC_CONF_ADDRESS_FILTER
#define NULLRDC_ADDRESS_FILTER NULLRDC_CONF_ADDRESS_FILTER
#else
#define NULLRDC_ADDRESS_FILTER 1
#endif /* NULLRDC_CONF_ADDRESS_FILTER */

#include "sys/rtimer.h"
#include "dev/watchdog.h"

#define ACK_WAIT_TIME                      RTIMER_SECOND / 2500
#define AFTER_ACK_DETECTED_WAIT_TIME       RTIMER_SECOND / 1500

#ifndef OFFRDC_ACK
#error Needed to set OFFRDC_ACK
#endif

#define ACK_LEN 3

/*---------------------------------------------------------------------------*/
static int send_one_packet(mac_callback_t sent, void *ptr)
{
  int ret;
  int last_sent_ok = 0;

  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, OFFRDC_ACK);

  if(NETSTACK_FRAMER.create_and_secure() < 0){
    /* Failed to allocate space for headers */
    PRINTF("nullrdc: send failed, too large header\n");
    ret = MAC_TX_ERR_FATAL;
  }else{
    int is_broadcast;
    uint8_t dsn;
    dsn = ((uint8_t *)packetbuf_hdrptr())[2] & 0xff;

    NETSTACK_RADIO.prepare(packetbuf_hdrptr(), packetbuf_totlen());

    is_broadcast = packetbuf_holds_broadcast();

    if(NETSTACK_RADIO.receiving_packet() ||
       (!is_broadcast && NETSTACK_RADIO.pending_packet())){

      /* Currently receiving a packet over air or the radio has
         already received a packet that needs to be read before
         sending with auto ack. */
      ret = MAC_TX_COLLISION;
    }else{
      if(!is_broadcast){
        RIMESTATS_ADD(reliabletx);
      }

#if OFFRDC_ACK
      NETSTACK_RADIO.on();
#endif
      switch(NETSTACK_RADIO.transmit(packetbuf_totlen())){
      case RADIO_TX_OK:
        if(is_broadcast) {
          ret = MAC_TX_OK;
        }else{
          rtimer_clock_t wt;

          PRINTF("offrdc: saru: waiting for ACK\n");
          /* Check for ack */
          wt = RTIMER_NOW();
          watchdog_periodic();
          while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + ACK_WAIT_TIME));

          ret = MAC_TX_NOACK;
          if(NETSTACK_RADIO.receiving_packet() ||
             NETSTACK_RADIO.pending_packet() ||
             NETSTACK_RADIO.channel_clear() == 0){
            int len;
            uint8_t ackbuf[ACK_LEN];

            if(AFTER_ACK_DETECTED_WAIT_TIME > 0){
              wt = RTIMER_NOW();
              watchdog_periodic();
              while(RTIMER_CLOCK_LT(RTIMER_NOW(),
                                    wt + AFTER_ACK_DETECTED_WAIT_TIME));
            }

            if(NETSTACK_RADIO.pending_packet()){
              len = NETSTACK_RADIO.read(ackbuf, ACK_LEN);
              if(len == ACK_LEN && ackbuf[2] == dsn){
                /* Ack received */
                RIMESTATS_ADD(ackrx);
                ret = MAC_TX_OK;
              }else{
                /* Not an ack or ack not for us: collision */
                ret = MAC_TX_COLLISION;
              }
            }
          }else{
            PRINTF("nullrdc tx noack\n");
          }
        }
        break;
      case RADIO_TX_COLLISION:
        ret = MAC_TX_COLLISION;
        break;
      default:
        ret = MAC_TX_ERR;
        break;
      }
    }
  }
  if(ret == MAC_TX_OK) {
    last_sent_ok = 1;
  }

  if(node_id != 1){
    NETSTACK_RADIO.off();
  }

  mac_call_sent_callback(sent, ptr, ret, 1);

  return last_sent_ok;
}
/*---------------------------------------------------------------------------*/
static void send_packet(mac_callback_t sent, void *ptr)
{
  send_one_packet(sent, ptr);
}
/*---------------------------------------------------------------------------*/
static void send_list(mac_callback_t sent,
                      void *ptr,
                      struct rdc_buf_list *buf_list)
{
  while(buf_list != NULL){
    /* We backup the next pointer, as it may be nullified by
     * mac_call_sent_callback() */
    struct rdc_buf_list *next = buf_list->next;
    int last_sent_ok;

    queuebuf_to_packetbuf(buf_list->buf);
    last_sent_ok = send_one_packet(sent, ptr);

    /* If packet transmission was not successful, we should back off and let
     * upper layers retransmit, rather than potentially sending out-of-order
     * packet fragments. */
    if(!last_sent_ok){
      return;
    }

    buf_list = next;
  }
}
/*---------------------------------------------------------------------------*/
static void packet_input(void)
{
  if(packetbuf_datalen() == ACK_LEN){
    /* Ignore ack packets */
    PRINTF("nullrdc: ignored ack\n"); 
  }else if(NETSTACK_FRAMER.parse() < 0){
    PRINTF("nullrdc: failed to parse %u\n", packetbuf_datalen());
#if NULLRDC_ADDRESS_FILTER
  }else if(!linkaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                          &linkaddr_node_addr) &&
           !packetbuf_holds_broadcast()){
    PRINTF("nullrdc: not for us\n");
#endif /* NULLRDC_ADDRESS_FILTER */
  }else{
    int duplicate = 0;

    /* Check for duplicate packet. */
    duplicate = mac_sequence_is_duplicate();
    if(duplicate){
      /* Drop the packet. */
      PRINTF("nullrdc: drop duplicate link layer packet %u\n",
             packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO));
    }else{
      mac_sequence_register_seqno();
    }

    if(!duplicate) {
      NETSTACK_MAC.input();
    }
  }
}
/*---------------------------------------------------------------------------*/
static int on(void)
{
  return NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
static int off(int keep_radio_on)
{
  if(keep_radio_on) {
    return NETSTACK_RADIO.on();
  }else{
    return NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static unsigned short channel_check_interval(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static void init(void)
{
  if(node_id != 1){
    NETSTACK_RADIO.off();
  }else{
    on();
  }
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver offrdc = {
  "offrdc",
  init,
  send_packet,
  send_list,
  packet_input,
  on,
  off,
  channel_check_interval,
};
/*---------------------------------------------------------------------------*/