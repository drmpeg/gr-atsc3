/* -*- c++ -*- */
/*
 * Copyright 2022 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "cyclicprefixer_cc_impl.h"

namespace gr {
  namespace atsc3 {

    using input_type = gr_complex;
    using output_type = gr_complex;
    cyclicprefixer_cc::sptr
    cyclicprefixer_cc::make(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_frame_length_mode_t flm, int fl, unsigned int vlength)
    {
      return gnuradio::make_block_sptr<cyclicprefixer_cc_impl>(
        fftsize, numpayloadsyms, numpreamblesyms, guardinterval, flm, fl, vlength
        );
    }


    /*
     * The private constructor
     */
    cyclicprefixer_cc_impl::cyclicprefixer_cc_impl(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_frame_length_mode_t flm, int fl, unsigned int vlength)
      : gr::block("cyclicprefixer_cc",
              gr::io_signature::make(1, 1, sizeof(input_type) * vlength),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      fftsamples = vlength;
      preamble_syms = numpreamblesyms;
      payload_syms = numpayloadsyms;
      switch (guardinterval) {
        case GI_1_192:
          gisamples = 192;
          break;
        case GI_2_384:
          gisamples = 384;
          break;
        case GI_3_512:
          gisamples = 512;
          break;
        case GI_4_768:
          gisamples = 768;
          break;
        case GI_5_1024:
          gisamples = 1024;
          break;
        case GI_6_1536:
          gisamples = 1536;
          break;
        case GI_7_2048:
          gisamples = 2048;
          break;
        case GI_8_2432:
          gisamples = 2432;
          break;
        case GI_9_3072:
          gisamples = 3072;
          break;
        case GI_10_3648:
          gisamples = 3648;
          break;
        case GI_11_4096:
          gisamples = 4096;
          break;
        case GI_12_4864:
          gisamples = 4864;
          break;
        default:
          gisamples = 192;
          break;
      }
      Nextra = ((fl * 6912) - BOOTSTRAP_SAMPLES) - numpreamblesyms * (fftsamples + gisamples) - numpayloadsyms * (fftsamples + gisamples);
      Nfinal = Nextra % numpayloadsyms;
      Nextra = Nextra / numpayloadsyms;
      symbol = 0;
      flmode = flm;
      if (flm == FLM_SYMBOL_ALIGNED) {
        set_relative_rate(fftsamples + gisamples);
        set_output_multiple(fftsamples + gisamples);
      }
      else {
        set_relative_rate(fftsamples + gisamples + Nextra + Nfinal);
        set_output_multiple(fftsamples + gisamples + Nextra + Nfinal);
      }
    }

    /*
     * Our virtual destructor.
     */
    cyclicprefixer_cc_impl::~cyclicprefixer_cc_impl()
    {
    }

    void
    cyclicprefixer_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      if (flmode == FLM_TIME_ALIGNED) {
        ninput_items_required[0] = noutput_items / (fftsamples + gisamples + Nextra + Nfinal);
      }
      else {
        ninput_items_required[0] = noutput_items / (fftsamples + gisamples);
      }
    }

    int
    cyclicprefixer_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      int symbols;
      int produced = 0;

      if (flmode == FLM_TIME_ALIGNED) {
        symbols = noutput_items / (fftsamples + gisamples + Nextra + Nfinal);
        for (int i = 0; i < symbols; i++) {
          if (symbol >= preamble_syms) {
            if (symbol == (preamble_syms + payload_syms) - 1) {
              memcpy(out + gisamples + Nextra, in, fftsamples * sizeof(gr_complex));
              memcpy(out, (in + fftsamples) - (gisamples + Nextra), (gisamples + Nextra) * sizeof(gr_complex));
              memcpy(out + gisamples + Nextra + fftsamples, in, Nfinal * sizeof(gr_complex));
              in += fftsamples;
              out += (fftsamples + gisamples + Nextra + Nfinal);
              produced += (fftsamples + gisamples + Nextra + Nfinal);
            }
            else {
              memcpy(out + gisamples + Nextra, in, fftsamples * sizeof(gr_complex));
              memcpy(out, (in + fftsamples) - (gisamples + Nextra), (gisamples + Nextra) * sizeof(gr_complex));
              in += fftsamples;
              out += (fftsamples + gisamples + Nextra);
              produced += (fftsamples + gisamples + Nextra);
            }
          }
          else {
            memcpy(out + gisamples, in, fftsamples * sizeof(gr_complex));
            memcpy(out, in + fftsamples - gisamples, gisamples * sizeof(gr_complex));
            in += fftsamples;
            out += (fftsamples + gisamples);
            produced += (fftsamples + gisamples);
          }
          symbol++;
          if (symbol == preamble_syms + payload_syms) {
            symbol = 0;
          }
        }
      }
      else {
        symbols = noutput_items / (fftsamples + gisamples);
        for (int i = 0; i < symbols; i++) {
          memcpy(out + gisamples, in, fftsamples * sizeof(gr_complex));
          memcpy(out, in + fftsamples - gisamples, gisamples * sizeof(gr_complex));
          in += fftsamples;
          out += (fftsamples + gisamples);
          produced += (fftsamples + gisamples);
        }
      }

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (symbols);

      // Tell runtime system how many output items we produced.
      return produced;
    }

  } /* namespace atsc3 */
} /* namespace gr */
