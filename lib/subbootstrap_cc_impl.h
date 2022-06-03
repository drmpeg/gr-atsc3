/* -*- c++ -*- */
/*
 * Copyright 2022 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_SUBBOOTSTRAP_CC_IMPL_H
#define INCLUDED_ATSC3_SUBBOOTSTRAP_CC_IMPL_H

#include <atsc3/subbootstrap_cc.h>
#include "atsc3_defines.h"
#include <gnuradio/fft/fft.h>
#include <gnuradio/filter/fir_filter.h>

#define BOOTSTRAP_FFT_SIZE 2048
#define B_SIZE 504
#define C_SIZE 520
#define NUM_BOOTSTRAP_SYMBOLS 4
#define ZADOFF_CHU_LENGTH 1499
#define VALID_SIGNALLING_BITS 8
#define SYSTEM_BANDWIDTH_6MHZ 0
#define BSR_COEFFICIENT 2
#define PADDING_SAMPLES 16 // TODO: calculate this from the length of the input filter in set_taps

namespace gr {
  namespace atsc3 {

    class subbootstrap_cc_impl : public subbootstrap_cc
    {
     private:
      int symbol_size[2];
      int guard_interval[2];
      int frame_items[2];
      int insertion_items;
      int pnseq[ZADOFF_CHU_LENGTH * (NUM_BOOTSTRAP_SYMBOLS / 2)];
      gr_complex zcseq[ZADOFF_CHU_LENGTH];
      gr_complex bootstrap_freq[BOOTSTRAP_FFT_SIZE];
      gr_complex bootstrap_time[4][BOOTSTRAP_FFT_SIZE];
      gr_complex bootstrap_partb[4][BOOTSTRAP_FFT_SIZE];
      gr_complex bootstrap_symbol[(BOOTSTRAP_FFT_SIZE + B_SIZE + C_SIZE) * NUM_BOOTSTRAP_SYMBOLS + (PADDING_SAMPLES * 2)];
      gr_complex bootstrap_resample[((BOOTSTRAP_FFT_SIZE + B_SIZE + C_SIZE) * NUM_BOOTSTRAP_SYMBOLS * 9) / 8];
      void init_pseudo_noise_sequence(void);
      void init_zadoff_chu_sequence(void);
      int gray_code_cyclic_shift(int signal_bits);
      unsigned char reversebits(unsigned char b);
      std::vector<float> design_resampler_filter_float(const unsigned interpolation, const unsigned decimation, const float fractional_bw);
      std::vector<gr_complex> design_resampler_filter(const unsigned interpolation, const unsigned decimation, const float fractional_bw);
      void set_taps(const std::vector<gr_complex>& taps);
      void install_taps(const std::vector<gr_complex>& taps);
      unsigned interpolation() const {return d_firs.size();}
      unsigned decimation() const {return d_decimation;}

      unsigned d_decimation;
      std::vector<gr_complex> d_new_taps;
      std::vector<gr::filter::kernel::fir_filter<gr_complex, gr_complex, gr_complex>> d_firs;

      int output_mode;
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
      subbootstrap_cc_impl(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_fftsize_t fftsize2nd, int numpayloadsyms2nd, atsc3_guardinterval_t guardinterval2nd, atsc3_pilotpattern_t pilotpattern2nd, atsc3_min_time_to_next_t frameinterval, atsc3_l1_fec_mode_t l1bmode, atsc3_bootstrap_mode_t outputmode, atsc3_showlevels_t showlevels, float vclip);
      ~subbootstrap_cc_impl();

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_SUBBOOTSTRAP_CC_IMPL_H */
