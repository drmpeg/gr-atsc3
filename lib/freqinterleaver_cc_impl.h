/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_FREQINTERLEAVER_CC_IMPL_H
#define INCLUDED_ATSC3_FREQINTERLEAVER_CC_IMPL_H

#include <atsc3/freqinterleaver_cc.h>
#include "atsc3_defines.h"

enum atsc3_symbol_type_t {
    PREAMBLE_SYMBOL = 1,
    SBS_SYMBOL,
    DATA_SYMBOL
};

namespace gr {
  namespace atsc3 {

    class freqinterleaver_cc_impl : public freqinterleaver_cc
    {
     private:
      int fft_size;
      int symbols;
      int output_cells;
      int interleaver_mode;
      std::vector<std::vector<int>> HevenFP;
      std::vector<std::vector<int>> HoddFP;
      std::vector<std::vector<int>> HevenP;
      std::vector<std::vector<int>> HoddP;
      std::vector<std::vector<int>> HevenSBS;
      std::vector<std::vector<int>> HoddSBS;
      std::vector<std::vector<int>> Heven;
      std::vector<std::vector<int>> Hodd;
      int frame_cells[4532];
      int frame_symbols[4352];
      void init_address(int firstpreamblecells, int preamblecells, int sbscells, int datacells);

      const static int bitperm8keven[12];
      const static int bitperm8kodd[12];
      const static int bitperm16keven[13];
      const static int bitperm16kodd[13];
      const static int bitperm32k[14];
      const static int preamble_cells_table[32][5];
      const static int data_cells_table_8K[16][5];
      const static int data_cells_table_16K[16][5];
      const static int data_cells_table_32K[16][5];
      const static int sbs_cells_table_8K[16][5];
      const static int sbs_cells_table_16K[16][5];
      const static int sbs_cells_table_32K[16][5];

     public:
      freqinterleaver_cc_impl(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_first_sbs_t firstsbs, atsc3_frequency_interleaver_t mode, atsc3_reduced_carriers_t cred, atsc3_papr_t paprmode);
      ~freqinterleaver_cc_impl();

      int work(
              int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_FREQINTERLEAVER_CC_IMPL_H */
