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
    bbscrambler_bb::make(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_plp_fec_mode_t fecmode)
    {
      return gnuradio::make_block_sptr<bbscrambler_bb_impl>(
        framesize, rate, fecmode);
    }


    /*
     * The private constructor
     */
    bbscrambler_bb_impl::bbscrambler_bb_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_plp_fec_mode_t fecmode)
      : gr::sync_block("bbscrambler_bb",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      int nbch = 0;
      int num_fec_bits = 0;

      switch (fecmode) {
        case PLP_FEC_NONE:
          num_fec_bits = 0;
          break;
        case PLP_FEC_CRC32:
          num_fec_bits = 32;
          break;
        case PLP_FEC_BCH:
          if (framesize == FECFRAME_NORMAL) {
            num_fec_bits = 192;
          }
          else if (framesize == FECFRAME_SHORT) {
            num_fec_bits = 168;
          }
          break;
        default:
          num_fec_bits = 0;
          break;
      }
      if (framesize == FECFRAME_NORMAL) {
        switch (rate) {
          case C2_15:
            nbch = 8640;
            break;
          case C3_15:
            nbch = 12960;
            break;
          case C4_15:
            nbch = 17280;
            break;
          case C5_15:
            nbch = 21600;
            break;
          case C6_15:
            nbch = 25920;
            break;
          case C7_15:
            nbch = 30240;
            break;
          case C8_15:
            nbch = 34560;
            break;
          case C9_15:
            nbch = 38880;
            break;
          case C10_15:
            nbch = 43200;
            break;
          case C11_15:
            nbch = 47520;
            break;
          case C12_15:
            nbch = 51840;
            break;
          case C13_15:
            nbch = 56160;
            break;
          default:
            nbch = 0;
            break;
        }
      }
      else if (framesize == FECFRAME_SHORT) {
        switch (rate) {
          case C2_15:
            nbch = 2160;
            break;
          case C3_15:
            nbch = 3240;
            break;
          case C4_15:
            nbch = 4320;
            break;
          case C5_15:
            nbch = 5400;
            break;
          case C6_15:
            nbch = 6480;
            break;
          case C7_15:
            nbch = 7560;
            break;
          case C8_15:
            nbch = 8640;
            break;
          case C9_15:
            nbch = 9720;
            break;
          case C10_15:
            nbch = 10800;
            break;
          case C11_15:
            nbch = 11880;
            break;
          case C12_15:
            nbch = 12960;
            break;
          case C13_15:
            nbch = 14040;
            break;
          default:
            nbch = 0;
            break;
        }
      }
      kbch = nbch - num_fec_bits;
      init_bb_randomizer();
      set_output_multiple(kbch);
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
      int b, packed;

      for (int i = 0; i < FRAME_SIZE_NORMAL;) {
        packed = ((sr & 0x4) << 5) | ((sr & 0x8) << 3) | ((sr & 0x10) << 1) | \
                 ((sr & 0x20) >> 1) | ((sr & 0x200) >> 6) | ((sr & 0x1000) >> 10) | \
                 ((sr & 0x2000) >> 12) | ((sr & 0x8000) >> 15);
        for (int n = 7; n >= 0; n--) {
          bb_randomize[i++] = packed & (1 << n) ? 1 : 0;
        }
        b = sr & 1;
        sr >>= 1;
        if (b) {
          sr ^= POLYNOMIAL;
        }
      }
      bb_randomize64 = (uint64_t*)&bb_randomize[0];
    }

    int
    bbscrambler_bb_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      auto in = static_cast<const uint64_t*>(input_items[0]);
      auto out = static_cast<uint64_t*>(output_items[0]);

      for (int i = 0; i < noutput_items; i += kbch) {
        for (int j = 0; j < kbch / 8; j++) {
          *out++ = *in++ ^ bb_randomize64[j];
        }
      }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace atsc3 */
} /* namespace gr */
