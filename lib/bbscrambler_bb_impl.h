/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_BBSCRAMBLER_BB_IMPL_H
#define INCLUDED_ATSC3_BBSCRAMBLER_BB_IMPL_H

#include <atsc3/bbscrambler_bb.h>
#include "atsc3_defines.h"

namespace gr {
  namespace atsc3 {

    class bbscrambler_bb_impl : public bbscrambler_bb
    {
     private:
      int kbch;
      unsigned char bb_randomize[FRAME_SIZE_NORMAL];
      void init_bb_randomizer(void);

     public:
      bbscrambler_bb_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate);
      ~bbscrambler_bb_impl();

      int work(
              int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_BBSCRAMBLER_BB_IMPL_H */
