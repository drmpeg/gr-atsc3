/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "alpbbheader_bb_impl.h"

namespace gr {
  namespace atsc3 {

    using input_type = unsigned char;
    using output_type = unsigned char;
    alpbbheader_bb::sptr
    alpbbheader_bb::make(atsc3_framesize_t framesize, atsc3_code_rate_t rate)
    {
      return gnuradio::make_block_sptr<alpbbheader_bb_impl>(
        framesize, rate);
    }


    /*
     * The private constructor
     */
    alpbbheader_bb_impl::alpbbheader_bb_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate)
      : gr::block("alpbbheader_bb",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      count = 0;
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
      set_output_multiple(kbch);
    }

    /*
     * Our virtual destructor.
     */
    alpbbheader_bb_impl::~alpbbheader_bb_impl()
    {
    }

    void
    alpbbheader_bb_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items / 8;
    }

    void
    alpbbheader_bb_impl::sendbits(unsigned char b, unsigned char *out)
    {
      for (int n = 7; n >= 0; n--) {
        *out++ = b & (1 << n) ? 1 : 0;
      }
    }

    int
    alpbbheader_bb_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      int consumed = 0;
      int pointer;
      unsigned char bits;

      for (int i = 0; i < noutput_items; i += kbch) {
        for (int j = 0; j < (int)((kbch - 16) / 8); j++) {
          if (j == 0) {
            pointer = count;
            if (pointer == 0) {
             pointer = count;
            }
            else {
             pointer = 188 - count;
            }
            bits = (pointer & 0x7f) | 0x80;
            sendbits(bits, out);
            out += 8;
            bits = (pointer >> 5) & 0xfc;
            sendbits(bits, out);
            out += 8;
          }
          if (count == 0) {
            if (*in != 0x47) {
              GR_LOG_WARN(d_logger, "Transport Stream sync error!");
            }
            bits = 0xe2; /* one TS packet per ALP packet */
            in++;
          }
          else {
            bits = *in++;
          }
          count = (count + 1) % 188;
          consumed++;
          sendbits(bits, out);
          out += 8;
        }
      }

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (consumed);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace atsc3 */
} /* namespace gr */
