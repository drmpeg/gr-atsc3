/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_PILOTGENERATOR_CC_IMPL_H
#define INCLUDED_ATSC3_PILOTGENERATOR_CC_IMPL_H

#include <atsc3/pilotgenerator_cc.h>
#include "atsc3_defines.h"
#include <gnuradio/fft/fft.h>
#include <vector>

#define MAX_CARRIERS 27649

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

    class pilotgenerator_cc_impl : public pilotgenerator_cc
    {
     private:
      int fft_size;
      int pilot_pattern;
      int papr_mode;
      int cred_coeff;
      int symbols;
      int carriers;
      int max_carriers;
      int preamble_carriers;
      int preamble_dx;
      int preamble_symbols;
      int dx;
      int dy;
      int input_cells;
      float first_preamble_normalization;
      float preamble_normalization;
      float data_normalization;
      gr_complex pr_bpsk[2];
      gr_complex sp_bpsk[2];
      gr_complex cp_bpsk[2];
      int prbs[MAX_CARRIERS];
      int frame_symbols[4352];
      std::vector<std::vector<int>> data_carrier_map;
      void init_prbs(void);
      void init_pilots(void);

      fft::fft_complex_rev ofdm_fft;
      int ofdm_fft_size;

      const static int carriers_table[3][5];
      const static int preamble_dx_table[32];
      const static double preamble_power_table[32];
      const static double scattered_power_table[16][5];
      const static double preamble_ifft_power_table[32][5];
      const static double data_ifft_power_table_8K[16][5][5];
      const static double data_ifft_power_table_16K[16][5][5];
      const static double data_ifft_power_table_32K[16][5][5];
      const static int continual_pilot_table_8K[48];
      const static int continual_pilot_table_16K[96];
      const static int continual_pilot_table_32K[192];
      const static int trpapr_table_8K[72];
      const static int trpapr_table_16K[144];
      const static int trpapr_table_32K[288];
      const static int trpapr_alt_table_8K[72];
      const static int trpapr_alt_table_16K[144];
      const static int trpapr_alt_table_32K[288];
      const static int preamble_cells_table[32][5];
      const static int data_cells_table_8K[16][5];
      const static int data_cells_table_16K[16][5];
      const static int data_cells_table_32K[16][5];
      const static int sbs_cells_table_8K[16][5];
      const static int sbs_cells_table_16K[16][5];
      const static int sbs_cells_table_32K[16][5];

     public:
      pilotgenerator_cc_impl(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_scattered_pilot_boost_t pilotboost, atsc3_first_sbs_t firstsbs, atsc3_reduced_carriers_t cred, atsc3_papr_t paprmode, unsigned int vlength);
      ~pilotgenerator_cc_impl();

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_PILOTGENERATOR_CC_IMPL_H */
