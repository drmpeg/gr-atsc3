/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_BCH_BB_IMPL_H
#define INCLUDED_ATSC3_BCH_BB_IMPL_H

#include <atsc3/bch_bb.h>
#include "atsc3_defines.h"
#include <bitset>

namespace gr {
  namespace atsc3 {

    class bch_bb_impl : public bch_bb
    {
     private:
      int kbch;
      int nbch;
      int frame_size;
      int plp_fec_mode;
      int num_fec_bits;
      unsigned int crc32_table[256];

      std::bitset<MAX_BCH_PARITY_BITS> crc_table[256];
      unsigned int num_parity_bits;
      std::bitset<MAX_BCH_PARITY_BITS> polynome;

      void calculate_crc_table();
      int poly_mult(const int*, int, const int*, int, int*);
      void bch_poly_build_tables(void);
      void crc32_init(void);

     public:
      bch_bb_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_plp_fec_mode_t fecmode);
      ~bch_bb_impl();

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_BCH_BB_IMPL_H */
