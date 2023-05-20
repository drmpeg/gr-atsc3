/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_ALPBBHEADER_BB_IMPL_H
#define INCLUDED_ATSC3_ALPBBHEADER_BB_IMPL_H

#include <atsc3/alpbbheader_bb.h>
#include <string.h>
#include <time.h>
#include <zlib.h>
#include <arpa/inet.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include "atsc3_defines.h"


namespace gr {
  namespace atsc3 {

    class alpbbheader_bb_impl : public alpbbheader_bb
    {
     private:
      int kbch;
      int count;
      int bbcount;
      int packets;
      int total;
      int segments;
      int dnp;
      int trigger;
      int lls_mode;
      int lls_send;
      int lls_count;
      int lls_index;
      int remainder;
      struct timespec tai_last;
      unsigned char llsgz[1024];
      unsigned char llstemp[1024];
      int lls_length;
      int llsgz_length;
      unsigned short identification;
      unsigned char src_addr[sizeof(in_addr)];
      unsigned char dst_addr[sizeof(in_addr)];
      void sendbits(unsigned char b, unsigned char *out);
      int checksum(unsigned short *, int, int);
      void sendlls(unsigned char *out);

      const static char xml[];
      const static char SLT[];
      const static char Service[];
      const static char BroadcastSvcSignaling[];
      const static char Service_end[];
      const static char SLT_end[];

     public:
      alpbbheader_bb_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_lls_insertion_mode_t llsmode);
      ~alpbbheader_bb_impl();

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_ALPBBHEADER_BB_IMPL_H */
