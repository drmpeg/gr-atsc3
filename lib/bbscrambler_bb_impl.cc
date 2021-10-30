/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "bbscrambler_bb_impl.h"

namespace gr {
  namespace atsc3 {

    using input_type = unsigned char;
    using output_type = unsigned char;
    bbscrambler_bb::sptr
    bbscrambler_bb::make(atsc3_framesize_t framesize, atsc3_code_rate_t rate)
    {
      return gnuradio::make_block_sptr<bbscrambler_bb_impl>(
        framesize, rate);
    }


    /*
     * The private constructor
     */
    bbscrambler_bb_impl::bbscrambler_bb_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate)
      : gr::sync_block("bbscrambler_bb",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      if (framesize == FECFRAME_NORMAL) {
        switch (rate) {
          case C2_15:
            kbch = 8448;
            break;
          case C3_15:
            kbch = 12768;
            break;
          case C4_15:
            kbch = 17088;
            break;
          case C5_15:
            kbch = 21408;
            break;
          case C6_15:
            kbch = 25728;
            break;
          case C7_15:
            kbch = 30048;
            break;
          case C8_15:
            kbch = 34368;
            break;
          case C9_15:
            kbch = 38688;
            break;
          case C10_15:
            kbch = 43008;
            break;
          case C11_15:
            kbch = 47328;
            break;
          case C12_15:
            kbch = 51648;
            break;
          case C13_15:
            kbch = 55968;
            break;
          default:
            kbch = 0;
            break;
        }
      }
      else if (framesize == FECFRAME_SHORT) {
        switch (rate) {
          case C2_15:
            kbch = 1992;
            break;
          case C3_15:
            kbch = 3072;
            break;
          case C4_15:
            kbch = 4152;
            break;
          case C5_15:
            kbch = 5232;
            break;
          case C6_15:
            kbch = 6312;
            break;
          case C7_15:
            kbch = 7392;
            break;
          case C8_15:
            kbch = 8472;
            break;
          case C9_15:
            kbch = 9552;
            break;
          case C10_15:
            kbch = 10632;
            break;
          case C11_15:
            kbch = 11712;
            break;
          case C12_15:
            kbch = 12792;
            break;
          case C13_15:
            kbch = 13872;
            break;
          default:
            kbch = 0;
            break;
        }
      }
      init_bb_randomizer();
      set_output_multiple(kbch / 8);
    }

    /*
     * Our virtual destructor.
     */
    bbscrambler_bb_impl::~bbscrambler_bb_impl()
    {
    }

    void
    bbscrambler_bb_impl::init_bb_randomizer(void)
    {
      int sr = 0x18f;
      int b;

      for (int i = 0; i < FRAME_SIZE_NORMAL; i++) {
        bb_randomize[i] = ((sr & 0x4) << 5) | ((sr & 0x8 ) << 3) | ((sr & 0x10) << 1) | \
                          ((sr & 0x20) >> 1) | ((sr & 0x200) >> 6) | ((sr & 0x1000) >> 10) | \
                          ((sr & 0x2000) >> 12) | ((sr & 0x8000) >> 15);
        b = sr & 1;
        sr >>= 1;
        if (b) {
          sr ^= POLYNOMIAL;
        }
      }
    }

    int
    bbscrambler_bb_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);

      for (int i = 0; i < noutput_items; i += kbch / 8) {
        for (int j = 0; j < kbch / 8; ++j) {
          out[i + j] = in[i + j] ^ bb_randomize[j];
        }
      }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace atsc3 */
} /* namespace gr */
