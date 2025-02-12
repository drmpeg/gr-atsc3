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
      unsigned char buffer[FRAME_SIZE_NORMAL] __attribute__ ((aligned(64)));
      void ldpc_lookup_generate(void);

      std::vector<uint16_t*> ldpc_lut; // Pointers into ldpc_lut_data.
      std::vector<uint16_t> ldpc_lut_data;
      std::vector<uint16_t*> ldpc_lut_aux; // Pointers into ldpc_lut_data.
      std::vector<uint16_t> ldpc_lut_aux_data;

      template <typename entry_t, size_t rows, size_t cols>
      void ldpc_bf_type_b(entry_t (&table)[rows][cols])
      {
        uint16_t max_lut_arraysize = 0;
        const unsigned int pbits = (frame_size) - nbch;
        const unsigned int q = q_val;

        for (auto& ldpc_lut_index_entry : ldpc_lut_index) {
          ldpc_lut_index_entry = 1; /* 1 for the size at the start of the array */
        }

        for (unsigned int row = 0; row < rows; row++) {
          for (unsigned int n = 0; n < 360; n++) {
            for (unsigned int col = 1; col <= table[row][0]; col++) {
              unsigned int current_pbit = (table[row][col] + (n * q)) % pbits;
              ldpc_lut_index[current_pbit]++;
              if (ldpc_lut_index[current_pbit] > max_lut_arraysize) {
                max_lut_arraysize = ldpc_lut_index[current_pbit];
              }
            }
          }
        }
        if (max_lut_arraysize & 0x1) { /* Optimize for RISC-V */
          max_lut_arraysize++;
        }

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

      template <typename entry_t, size_t rows, size_t cols>
      void ldpc_bf_type_a(entry_t (&table)[rows][cols])
      {
        int im = 0;
        int row;
        uint16_t max_lut_arraysize = 0;
        const unsigned int pbits = (frame_size) - nbch;

        for (auto& ldpc_lut_index_entry : ldpc_lut_index) {
          ldpc_lut_index_entry = 1; /* 1 for the size at the start of the array */
        }

        for (unsigned int row = 0; row < rows; row++) {
          if (im == nbch) {
            break;
          }
          for (unsigned int n = 0; n < 360; n++) {
            for (unsigned int col = 1; col <= table[row][0]; col++) {
              if ((im % 360) == 0) {
                unsigned int current_pbit = table[row][col];
                ldpc_lut_index[current_pbit]++;
                if (ldpc_lut_index[current_pbit] > max_lut_arraysize) {
                  max_lut_arraysize = ldpc_lut_index[current_pbit];
                }
              }
              else {
                if (table[row][col] < m1_val) {
                  unsigned int current_pbit = (table[row][col] + (n * q1_val)) % m1_val;
                  ldpc_lut_index[current_pbit]++;
                  if (ldpc_lut_index[current_pbit] > max_lut_arraysize) {
                    max_lut_arraysize = ldpc_lut_index[current_pbit];
                  }
                }
                else {
                  unsigned int current_pbit = m1_val + (table[row][col] - m1_val + (n * q2_val)) % m2_val;
                  ldpc_lut_index[current_pbit]++;
                  if (ldpc_lut_index[current_pbit] > max_lut_arraysize) {
                    max_lut_arraysize = ldpc_lut_index[current_pbit];
                  }
                }
              }
            }
            im++;
          }
        }
        im = 0;
        if (max_lut_arraysize & 0x1) { /* Optimize for RISC-V */
          max_lut_arraysize++;
        }

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
        ldpc_lut_aux.resize(pbits);
        ldpc_lut_aux_data.resize(pbits * max_lut_arraysize);
        ldpc_lut_aux_data[0] = 1;
        ldpc_lut_aux[0] = ldpc_lut_aux_data.data();
        for (unsigned int i = 1; i < pbits; i++) {
          ldpc_lut_aux[i] = ldpc_lut_aux[i - 1] + max_lut_arraysize;
          ldpc_lut_aux[i][0] = 1;
        }
        for (row = 0; row < (int)rows; row++) {
          if (im == nbch) {
            break;
          }
          for (int n = 0; n < 360; n++) {
            for (int col = 1; col <= table[row][0]; col++) {
              if ((im % 360) == 0) {
                unsigned int current_pbit = table[row][col];
                ldpc_lut[current_pbit][ldpc_lut[current_pbit][0]] = im;
                ldpc_lut[current_pbit][0]++;
              }
              else {
                if (table[row][col] < m1_val) {
                  unsigned int current_pbit = (table[row][col] + (n * q1_val)) % m1_val;
                  ldpc_lut[current_pbit][ldpc_lut[current_pbit][0]] = im;
                  ldpc_lut[current_pbit][0]++;
                }
                else {
                  unsigned int current_pbit = m1_val + (table[row][col] - m1_val + (n * q2_val)) % m2_val;
                  ldpc_lut[current_pbit][ldpc_lut[current_pbit][0]] = im;
                  ldpc_lut[current_pbit][0]++;
                }
              }
            }
            im++;
          }
        }
        for (;row < (int)rows; row++) {
          for (int n = 0; n < 360; n++) {
            for (int col = 1; col <= table[row][0]; col++) {
              if ((im % 360) == 0) {
                unsigned int current_pbit = table[row][col];
                ldpc_lut_aux[current_pbit][ldpc_lut_aux[current_pbit][0]] = im;
                ldpc_lut_aux[current_pbit][0]++;
              }
              else {
                if (table[row][col] < m1_val) {
                  unsigned int current_pbit = (table[row][col] + (n * q1_val)) % m1_val;
                  ldpc_lut_aux[current_pbit][ldpc_lut_aux[current_pbit][0]] = im;
                  ldpc_lut_aux[current_pbit][0]++;
                }
                else {
                  unsigned int current_pbit = m1_val + (table[row][col] - m1_val + (n * q2_val)) % m2_val;
                  ldpc_lut_aux[current_pbit][ldpc_lut_aux[current_pbit][0]] = im;
                  ldpc_lut_aux[current_pbit][0]++;
                }
              }
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

      // Disallow copy/move because of the raw pointers.
      ldpc_bb_impl(const ldpc_bb_impl&) = delete;
      ldpc_bb_impl(ldpc_bb_impl&&) = delete;
      ldpc_bb_impl& operator=(const ldpc_bb_impl&) = delete;
      ldpc_bb_impl& operator=(ldpc_bb_impl&&) = delete;

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_LDPC_BB_IMPL_H */
