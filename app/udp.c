/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Demonstrating the sending and receiving of UDP data
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#if MODULE_GNRC_UDP
#include <stdio.h>
#include <inttypes.h>

#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/pktdump.h"
#include "timex.h"
#include "xtimer.h"


void send_udp(char *addr_str, uint16_t port, uint8_t *data, uint16_t datalen)
{
    ipv6_addr_t addr;

    /* parse destination address */
    if (ipv6_addr_from_str(&addr, addr_str) == NULL) {
        puts("Error: unable to parse destination address");
        return;
    }

      gnrc_pktsnip_t *payload, *udp, *ip;
      /* allocate payload */
      payload = gnrc_pktbuf_add(NULL, data, datalen, GNRC_NETTYPE_UNDEF);
      if (payload == NULL) {
          puts("Error: unable to copy data to packet buffer");
          return;
      }
      /* allocate UDP header, set source port := destination port */
      udp = gnrc_udp_hdr_build(payload, port, port);
      if (udp == NULL) {
          puts("Error: unable to allocate UDP header");
          gnrc_pktbuf_release(payload);
          return;
      }
      /* allocate IPv6 header */
      ip = gnrc_ipv6_hdr_build(udp, NULL, &addr);
      if (ip == NULL) {
          puts("Error: unable to allocate IPv6 header");
          gnrc_pktbuf_release(udp);
          return;
      }
      /* send packet */
      if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
          puts("Error: unable to locate UDP thread");
          gnrc_pktbuf_release(ip);
          return;
      }
}
#endif
