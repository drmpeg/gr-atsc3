/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "bootstrap_cc_impl.h"
#include <gnuradio/math.h>

namespace gr {
  namespace atsc3 {

    using input_type = gr_complex;
    using output_type = gr_complex;
    bootstrap_cc::sptr
    bootstrap_cc::make(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_showlevels_t showlevels)
    {
      return gnuradio::make_block_sptr<bootstrap_cc_impl>(
        fftsize, numpayloadsyms, numpreamblesyms, guardinterval, showlevels);
    }


    /*
     * The private constructor
     */
    bootstrap_cc_impl::bootstrap_cc_impl(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_showlevels_t showlevels)
      : gr::block("bootstrap_cc",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type))),
        show_levels(showlevels),
        real_positive(0.0),
        real_negative(0.0),
        imag_positive(0.0),
        imag_negative(0.0),
        real_positive_threshold(1.0),
        real_negative_threshold(-1.0),
        imag_positive_threshold(1.0),
        imag_negative_threshold(-1.0),
        real_positive_threshold_count(0),
        real_negative_threshold_count(0),
        imag_positive_threshold_count(0),
        imag_negative_threshold_count(0),
        bootstrap_fft(BOOTSTRAP_FFT_SIZE, 1)
    {
      int symbols;
      int symbol_size;
      int guard_interval;

      symbols = numpreamblesyms + numpayloadsyms;
      switch (fftsize) {
        case FFTSIZE_8K:
          symbol_size = 8192;
          break;
        case FFTSIZE_16K:
          symbol_size = 16384;
          break;
        case FFTSIZE_32K:
          symbol_size = 32768;
          break;
        default:
          symbol_size = 8192;
          break;
      }
      switch (guardinterval) {
        case GI_1_192:
          guard_interval = 192;
          break;
        case GI_2_384:
          guard_interval = 384;
          break;
        case GI_3_512:
          guard_interval = 512;
          break;
        case GI_4_768:
          guard_interval = 768;
          break;
        case GI_5_1024:
          guard_interval = 1024;
          break;
        case GI_6_1536:
          guard_interval = 1536;
          break;
        case GI_7_2048:
          guard_interval = 2048;
          break;
        case GI_8_2432:
          guard_interval = 2432;
          break;
        case GI_9_3072:
          guard_interval = 3072;
          break;
        case GI_10_3648:
          guard_interval = 3648;
          break;
        case GI_11_4096:
          guard_interval = 4096;
          break;
        case GI_12_4864:
          guard_interval = 4864;
          break;
        default:
          guard_interval = 192;
          break;
      }
      init_pseudo_noise_sequence();
      init_zadoff_chu_sequence();
      frame_items = (symbols * symbol_size) + (symbols * guard_interval);
      insertion_items = frame_items + ((BOOTSTRAP_FFT_SIZE + guard_interval) * 4);
      set_output_multiple(insertion_items);
    }

    /*
     * Our virtual destructor.
     */
    bootstrap_cc_impl::~bootstrap_cc_impl()
    {
    }

    void
    bootstrap_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = frame_items * (noutput_items / insertion_items);
    }

    void
    bootstrap_cc_impl::init_pseudo_noise_sequence(void)
    {
      int sr = 0x19d;

      for (int i = 0; i < ZADOFF_CHU_LENGTH; i++) {
        int b = ((sr) ^ (sr >> 1) ^ (sr >> 2) ^ (sr >> 15) ^ (sr >> 16)) & 1;
        pnseq[i] = sr & 1;
        sr >>= 1;
        if (b) {
          sr |= 0x10000;
        }
      }
    }

    void
    bootstrap_cc_impl::init_zadoff_chu_sequence(void)
    {
      int q = 137;
      double normalization = 2048.0 / std::sqrt(1498.0);

      for (int n = 0; n < ZADOFF_CHU_LENGTH; n++) {
        zcseq[n] = std::exp(gr_complexd(0.0, 1.0) * gr_complexd(GR_M_PI * q * double(-1 * n * (n + 1)) / 1499.0, 0.0));
        if (pnseq[n]) {
          zcseq[n] = -zcseq[n];
        }
        if (n < 32) {
          printf("%f, %f\n", zcseq[n].real() * normalization, zcseq[n].imag() * normalization);
        }
      }
    }

    const gr_complex zero = gr_complex(0.0, 0.0);

    int
    bootstrap_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      gr_complex* level;

      for (int i = 0; i < noutput_items; i += insertion_items) {
        level = out;
        for (int j = 0; j < NUM_BOOTSTRAP_SYMBOLS; j++) {
          if (j == 0) {
            for (int n = 0; n < C_SIZE; n++) {
              *out++ = zero;
            }
            for (int n = 0; n < BOOTSTRAP_FFT_SIZE; n++) {
              *out++ = zero;
            }
            for (int n = 0; n < B_SIZE; n++) {
              *out++ = zero;
            }
          }
          else {
            for (int n = 0; n < B_SIZE; n++) {
              *out++ = zero;
            }
            for (int n = 0; n < C_SIZE; n++) {
              *out++ = zero;
            }
            for (int n = 0; n < BOOTSTRAP_FFT_SIZE; n++) {
              *out++ = zero;
            }
          }
        }
        memcpy(out, in, sizeof(gr_complex) * frame_items);
        if (show_levels == SHOWLEVELS_ON) {
          for (int j = 0; j < insertion_items; j++) {
            if (level[j].real() > real_positive) {
              real_positive = level[j].real();
            }
            if (level[j].real() < real_negative) {
              real_negative = level[j].real();
            }
            if (level[j].imag() > imag_positive) {
              imag_positive = level[j].imag();
            }
            if (level[j].imag() < imag_negative) {
              imag_negative = level[j].imag();
            }
            if (level[j].real() > real_positive_threshold) {
              real_positive_threshold_count++;
            }
            if (level[j].real() < real_negative_threshold) {
              real_negative_threshold_count++;
            }
            if (level[j].imag() > imag_positive_threshold) {
              imag_positive_threshold_count++;
            }
            if (level[j].imag() < imag_negative_threshold) {
              imag_negative_threshold_count++;
            }
          }
          printf("peak real = %+e, %+e, %d, %d\n",
                 real_positive,
                 real_negative,
                 real_positive_threshold_count,
                 real_negative_threshold_count);
          printf("peak imag = %+e, %+e, %d, %d\n",
                 imag_positive,
                 imag_negative,
                 imag_positive_threshold_count,
                 imag_negative_threshold_count);
        }
      }
      out += frame_items;
      in += frame_items;

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (frame_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace atsc3 */
} /* namespace gr */
