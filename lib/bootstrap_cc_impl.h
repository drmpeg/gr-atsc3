/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_BOOTSTRAP_CC_IMPL_H
#define INCLUDED_ATSC3_BOOTSTRAP_CC_IMPL_H

#include <atsc3/bootstrap_cc.h>
#include "atsc3_defines.h"
#include <gnuradio/fft/fft.h>

#define BOOTSTRAP_FFT_SIZE 2048
#define B_SIZE 504
#define C_SIZE 520
#define NUM_BOOTSTRAP_SYMBOLS 4
#define ZADOFF_CHU_LENGTH 1499
#define VALID_SIGNALLING_BITS 8
#define SYSTEM_BANDWIDTH_6MHZ 0
#define BSR_COEFFICIENT 2

namespace gr {
  namespace atsc3 {

    class bootstrap_cc_impl : public bootstrap_cc
    {
     private:
      int frame_items;
      int insertion_items;
      int pnseq[ZADOFF_CHU_LENGTH * (NUM_BOOTSTRAP_SYMBOLS / 2)];
      gr_complex zcseq[ZADOFF_CHU_LENGTH];
      gr_complex bootstrap_freq[BOOTSTRAP_FFT_SIZE];
      gr_complex bootstrap_time[4][BOOTSTRAP_FFT_SIZE];
      gr_complex bootstrap_partb[4][BOOTSTRAP_FFT_SIZE];
      void init_pseudo_noise_sequence(void);
      void init_zadoff_chu_sequence(void);
      int gray_code_cyclic_shift(int signal_bits);
      unsigned char reversebits(unsigned char b);

      int show_levels;
      float real_positive;
      float real_negative;
      float imag_positive;
      float imag_negative;
      float real_positive_threshold;
      float real_negative_threshold;
      float imag_positive_threshold;
      float imag_negative_threshold;
      int real_positive_threshold_count;
      int real_negative_threshold_count;
      int imag_positive_threshold_count;
      int imag_negative_threshold_count;

      fft::fft_complex_rev bootstrap_fft;

     public:
      bootstrap_cc_impl(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_min_time_to_next_t frameinterval, atsc3_l1_fec_mode_t l1bmode, atsc3_showlevels_t showlevels);
      ~bootstrap_cc_impl();

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_BOOTSTRAP_CC_IMPL_H */
