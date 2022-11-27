/* -*- c++ -*- */
/*
 * Copyright 2022 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_CYCLICPREFIXER_CC_IMPL_H
#define INCLUDED_ATSC3_CYCLICPREFIXER_CC_IMPL_H

#include <atsc3/cyclicprefixer_cc.h>
#include "atsc3_defines.h"

namespace gr {
  namespace atsc3 {

    class cyclicprefixer_cc_impl : public cyclicprefixer_cc
    {
     private:
      int symbol;
      int fl_mode;
      int fftsamples;
      int gisamples;
      int preamble_syms;
      int payload_syms;
      int Nextra;
      int Nfinal;

     public:
      cyclicprefixer_cc_impl(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_frame_length_mode_t flmode, int flen, unsigned int vlength);
      ~cyclicprefixer_cc_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_CYCLICPREFIXER_CC_IMPL_H */
