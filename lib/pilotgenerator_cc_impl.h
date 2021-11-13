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
#include <vector>

#define MAX_CARRIERS 27649

enum atsc3_carrier_type_t {
    DATA_CARRIER = 1,
    PREAMBLE_CARRIER,
    SCATTERED_CARRIER,
    CONTINUAL_CARRIER
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
      int symbols;
      int carriers;
      int max_carriers;
      int preamble_carriers;
      int preamble_dx;
      int preamble_symbols;
      int dx;
      int dy;
      int input_cells;
      gr_complex pr_bpsk[2];
      gr_complex sp_bpsk[2];
      gr_complex cp_bpsk[2];
      int prbs[MAX_CARRIERS];
      int frame_symbols[4352];
      std::vector<std::vector<int>> data_carrier_map;
      void init_prbs(void);
      void init_pilots(void);

      const static int carriers_table[3][5];
      const static int preamble_dx_table[32];
      const static double preamble_power_table[32];
      const static double scattered_power_table[16][5];
      const static int continual_pilot_table_8K[48];
      const static int preamble_cells_table[32][5];
      const static int data_cells_table_8K[16][5];
      const static int data_cells_table_16K[16][5];
      const static int data_cells_table_32K[16][5];
      const static int sbs_cells_table_8K[16][5];
      const static int sbs_cells_table_16K[16][5];
      const static int sbs_cells_table_32K[16][5];

     public:
      pilotgenerator_cc_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation, atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, int plpsize, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_scattered_pilot_boost_t pilotboost, atsc3_first_sbs_t firstsbs, atsc3_l1_fec_mode_t l1bmode, atsc3_l1_fec_mode_t l1dmode);
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
