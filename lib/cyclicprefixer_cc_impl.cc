/* -*- c++ -*- */
/*
 * Copyright 2022,2023 Ron Economos.
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
    cyclicprefixer_cc::make(atsc3_cyclicprefixer_mode_t submode, atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_fftsize_t altfftsize, int altnumpayloadsyms, int altnumpreamblesyms, atsc3_guardinterval_t altguardinterval, atsc3_frame_length_mode_t flmode, int flen, unsigned int vlength)
    {
      return gnuradio::make_block_sptr<cyclicprefixer_cc_impl>(
        submode, fftsize, numpayloadsyms, numpreamblesyms, guardinterval, altfftsize, altnumpayloadsyms, altnumpreamblesyms, altguardinterval, flmode, flen, vlength
        );
    }


    /*
     * The private constructor
     */
    cyclicprefixer_cc_impl::cyclicprefixer_cc_impl(atsc3_cyclicprefixer_mode_t submode, atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_fftsize_t altfftsize, int altnumpayloadsyms, int altnumpreamblesyms, atsc3_guardinterval_t altguardinterval, atsc3_frame_length_mode_t flmode, int flen, unsigned int vlength)
      : gr::block("cyclicprefixer_cc",
              gr::io_signature::make(1, 1, sizeof(input_type) * vlength),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      int altfftsamples, altgisamples;
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
      switch (altfftsize) {
        case FFTSIZE_8K:
          altfftsamples = 8192;
          break;
        case FFTSIZE_16K:
          altfftsamples = 16384;
          break;
        case FFTSIZE_32K:
          altfftsamples = 32768;
          break;
        default:
          altfftsamples = 8192;
          break;
      }
      switch (altguardinterval) {
        case GI_1_192:
          altgisamples = 192;
          break;
        case GI_2_384:
          altgisamples = 384;
          break;
        case GI_3_512:
          altgisamples = 512;
          break;
        case GI_4_768:
          altgisamples = 768;
          break;
        case GI_5_1024:
          altgisamples = 1024;
          break;
        case GI_6_1536:
          altgisamples = 1536;
          break;
        case GI_7_2048:
          altgisamples = 2048;
          break;
        case GI_8_2432:
          altgisamples = 2432;
          break;
        case GI_9_3072:
          altgisamples = 3072;
          break;
        case GI_10_3648:
          altgisamples = 3648;
          break;
        case GI_11_4096:
          altgisamples = 4096;
          break;
        case GI_12_4864:
          altgisamples = 4864;
          break;
        default:
          altgisamples = 192;
          break;
      }
      if (submode == CYCLICPREFIXER_SINGLE_SUBFRAME) {
        Nextra = ((flen * 6912) - BOOTSTRAP_SAMPLES) - numpreamblesyms * (fftsamples + gisamples) - numpayloadsyms * (fftsamples + gisamples);
        Nfinal = Nextra % numpayloadsyms;
        Nextra = Nextra / numpayloadsyms;
      }
      else if (submode == CYCLICPREFIXER_DUAL_SUBFRAME_SUB0) {
        Nextra = ((flen * 6912) - BOOTSTRAP_SAMPLES) - numpreamblesyms * (fftsamples + gisamples) - ((numpayloadsyms * (fftsamples + gisamples)) + (altnumpayloadsyms * (altfftsamples + altgisamples)));
        Nfinal = 0;
        Nextra = Nextra / (numpayloadsyms + altnumpayloadsyms);
      }
      else {
        Nextra = ((flen * 6912) - BOOTSTRAP_SAMPLES) - altnumpreamblesyms * (altfftsamples + altgisamples) - ((numpayloadsyms * (fftsamples + gisamples)) + (altnumpayloadsyms * (altfftsamples + altgisamples)));
        Nfinal = Nextra % (numpayloadsyms + altnumpayloadsyms);
        Nextra = Nextra / (numpayloadsyms + altnumpayloadsyms);
      }
      symbol = 0;
      fl_mode = flmode;
      if (flmode == FLM_SYMBOL_ALIGNED) {
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
      if (fl_mode == FLM_TIME_ALIGNED) {
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

      if (fl_mode == FLM_TIME_ALIGNED) {
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
