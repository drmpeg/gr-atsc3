/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_ALPBBHEADER_BB_IMPL_H
#define INCLUDED_ATSC3_ALPBBHEADER_BB_IMPL_H

#include <atsc3/alpbbheader_bb.h>
#include "atsc3_defines.h"

namespace gr {
  namespace atsc3 {

    class alpbbheader_bb_impl : public alpbbheader_bb
    {
     private:
      int kbch;
      int count;
      void sendbits(unsigned char b, unsigned char *out);

     public:
      alpbbheader_bb_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate);
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
