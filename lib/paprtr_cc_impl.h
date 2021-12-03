/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_PAPRTR_CC_IMPL_H
#define INCLUDED_ATSC3_PAPRTR_CC_IMPL_H

#include <atsc3/paprtr_cc.h>
#include "atsc3_defines.h"
#include <gnuradio/fft/fft.h>

#define MAX_CARRIERS 27649
#define MAX_FFTSIZE 32768
#define MAX_PAPRTONES 288

enum atsc3_carrier_type_t {
    DATA_CARRIER = 1,
    PREAMBLE_CARRIER,
    SCATTERED_CARRIER,
    CONTINUAL_CARRIER,
    TRPAPR_CARRIER
};

enum atsc3_symbol_type_t {
    PREAMBLE_SYMBOL = 1,
    SBS_SYMBOL,
    DATA_SYMBOL
};

namespace gr {
  namespace atsc3 {

    class paprtr_cc_impl : public paprtr_cc
    {
     private:
      fft::fft_complex_rev papr_fft;
      int papr_fft_size;
      int symbols;
      int fft_size;
      int left_nulls;
      int right_nulls;
      int papr_mode;
      int carriers;
      int max_carriers;
      int preamble_carriers;
      int preamble_symbols;
      double v_clip;
      int num_iterations;
      const int* tr_papr_map;
      int tr_carrier_map[MAX_CARRIERS];
      volk::vector<gr_complex> ones_freq;
      volk::vector<gr_complex> ones_time;
      volk::vector<gr_complex> c;
      volk::vector<gr_complex> ctemp;
      volk::vector<float> magnitude;
      volk::vector<gr_complex> r;
      volk::vector<gr_complex> rNew;
      volk::vector<gr_complex> v;
      float alphaLimit[MAX_PAPRTONES];
      float alphaLimitMax[MAX_PAPRTONES];
      int N_TR;
      int dx;
      int dy;
      int shift;
      int frame_symbols[4352];
      void init_pilots(int);

      const static int carriers_table[3][5];
      const static int trpapr_table_8K[72];
      const static int trpapr_table_16K[144];
      const static int trpapr_table_32K[288];
      const static int trpapr_alt_table_8K[72];
      const static int trpapr_alt_table_16K[144];
      const static int trpapr_alt_table_32K[288];


     public:
      paprtr_cc_impl(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_pilotpattern_t pilotpattern, atsc3_first_sbs_t firstsbs, atsc3_papr_t paprmode, atsc3_reduced_carriers_t cred, float vclip, int iterations, unsigned int vlength);
      ~paprtr_cc_impl();

      int work(
              int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_PAPRTR_CC_IMPL_H */
