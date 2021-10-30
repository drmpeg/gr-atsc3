/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_LDPC_BB_IMPL_H
#define INCLUDED_ATSC3_LDPC_BB_IMPL_H

#include <atsc3/ldpc_bb.h>
#include "atsc3_defines.h"

namespace gr {
  namespace atsc3 {

    class ldpc_bb_impl : public ldpc_bb
    {
     private:
      int frame_size;
      int frame_size_type;
      int nbch;
      int code_rate;
      int ldpc_type;
      int q_val;
      int q1_val;
      int q2_val;
      int m1_val;
      int m2_val;
      int ldpc_lut_index[FRAME_SIZE_NORMAL];
      void ldpc_lookup_generate(void);

      std::vector<uint16_t*> ldpc_lut; // Pointers into ldpc_lut_data.
      std::vector<uint16_t> ldpc_lut_data;

      template <typename entry_t, size_t rows, size_t cols>
      void ldpc_bf(entry_t (&table)[rows][cols])
      {
        size_t max_lut_arraysize = 0;
        const unsigned int pbits = (frame_size) - nbch;
        const unsigned int q = q_val;
        for (auto& row : table) { /* count the entries in the table */
          max_lut_arraysize += row[0];
        }

        max_lut_arraysize *= 360;   /* 360 bits per table entry */
        max_lut_arraysize /= pbits; /* spread over all parity bits */

        for (auto& ldpc_lut_index_entry : ldpc_lut_index) {
          ldpc_lut_index_entry = 1;
        }

        uint16_t max_index = 0;
        for (unsigned int row = 0; row < rows; row++) {
          for (unsigned int n = 0; n < 360; n++) {
            for (unsigned int col = 1; col <= table[row][0]; col++) {
              unsigned int current_pbit = (table[row][col] + (n * q)) % pbits;
              ldpc_lut_index[current_pbit]++;
              if (ldpc_lut_index[current_pbit] > max_index) {
                max_index = ldpc_lut_index[current_pbit];
              }
            }
          }
        }
        max_lut_arraysize += 1 + (max_index - max_lut_arraysize); /* 1 for the size at the start of the array */

        /* Allocate a 2D Array with pbits * max_lut_arraysize
         * while preserving two-subscript access
         * see
         * https://stackoverflow.com/questions/29375797/copy-2d-array-using-memcpy/29375830#29375830
         */
        ldpc_lut.resize(pbits);
        ldpc_lut_data.resize(pbits * max_lut_arraysize);
        ldpc_lut_data[0] = 1;
        ldpc_lut[0] = ldpc_lut_data.data();
        for (unsigned int i = 1; i < pbits; i++) {
          ldpc_lut[i] = ldpc_lut[i - 1] + max_lut_arraysize;
          ldpc_lut[i][0] = 1;
        }
        uint16_t im = 0;
        for (unsigned int row = 0; row < rows; row++) {
          for (unsigned int n = 0; n < 360; n++) {
            for (unsigned int col = 1; col <= table[row][0]; col++) {
              unsigned int current_pbit = (table[row][col] + (n * q)) % pbits;
              ldpc_lut[current_pbit][ldpc_lut[current_pbit][0]] = im;
              ldpc_lut[current_pbit][0]++;
            }
            im++;
          }
        }
      }

      const static uint16_t ldpc_tab_2_15N[29][21];
      const static uint16_t ldpc_tab_3_15N[41][16];
      const static uint16_t ldpc_tab_4_15N[53][14];
      const static uint16_t ldpc_tab_5_15N[64][12];
      const static uint16_t ldpc_tab_6_15N[72][28];
      const static uint16_t ldpc_tab_7_15N[87][10];
      const static uint16_t ldpc_tab_8_15N[96][20];
      const static uint16_t ldpc_tab_9_15N[108][20];
      const static uint16_t ldpc_tab_10_15N[120][15];
      const static uint16_t ldpc_tab_11_15N[132][16];
      const static uint16_t ldpc_tab_12_15N[144][15];
      const static uint16_t ldpc_tab_13_15N[156][14];

      const static uint16_t ldpc_tab_2_15S[15][8];
      const static uint16_t ldpc_tab_3_15S[12][12];
      const static uint16_t ldpc_tab_4_15S[15][11];
      const static uint16_t ldpc_tab_5_15S[17][11];
      const static uint16_t ldpc_tab_6_15S[18][31];
      const static uint16_t ldpc_tab_7_15S[21][25];
      const static uint16_t ldpc_tab_8_15S[24][33];
      const static uint16_t ldpc_tab_9_15S[27][17];
      const static uint16_t ldpc_tab_10_15S[30][26];
      const static uint16_t ldpc_tab_11_15S[33][13];
      const static uint16_t ldpc_tab_12_15S[36][15];
      const static uint16_t ldpc_tab_13_15S[39][14];

     public:
      ldpc_bb_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate);
      ~ldpc_bb_impl();

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_LDPC_BB_IMPL_H */
