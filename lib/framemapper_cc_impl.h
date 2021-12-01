/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_FRAMEMAPPER_CC_IMPL_H
#define INCLUDED_ATSC3_FRAMEMAPPER_CC_IMPL_H

#include <atsc3/framemapper_cc.h>
#include "atsc3_defines.h"
#include <bitset>
#include <deque>
#include <vector>

#define NBCH_3_15 3240
#define NBCH_6_15 6480
#define MAX_INTERLEAVER_DEPTH 1448

typedef struct {
  int version;
  int mimo_scattered_pilot_encoding;
  int lls_flag;
  int time_info_flag;
  int return_channel_flag;
  int papr_reduction;
  int frame_length_mode;
  int frame_length;
  int excess_samples_per_symbol;
  int time_offset;
  int additional_samples;
  int num_subframes;
  int preamble_num_symbols;
  int preamble_reduced_carriers;
  int L1_Detail_content_tag;
  int L1_Detail_size_bytes;
  int L1_Detail_fec_type;
  int L1_Detail_additional_parity_mode;
  int L1_Detail_total_cells;
  int first_sub_mimo;
  int first_sub_miso;
  int first_sub_fft_size;
  int first_sub_reduced_carriers;
  int first_sub_guard_interval;
  int first_sub_num_ofdm_symbols;
  int first_sub_scattered_pilot_pattern;
  int first_sub_scattered_pilot_boost;
  int first_sub_sbs_first;
  int first_sub_sbs_last;
  long long reserved;
  int crc;
} L1_Basic;

typedef struct {
  int version;
  int num_rf;
  int bonded_bsid;
  int reserved_0;
  int time_sec;
  int time_msec;
  int time_usec;
  int time_nsec;
  int mimo;
  int miso;
  int fft_size;
  int reduced_carriers;
  int guard_interval;
  int num_ofdm_symbols;
  int scattered_pilot_pattern;
  int scattered_pilot_boost;
  int sbs_first;
  int sbs_last;
  int subframe_multiplex;
  int frequency_interleaver;
  int sbs_null_cells;
  int num_plp;
  int plp_id;
  int plp_lls_flag;
  int plp_layer;
  int plp_start;
  int plp_size;
  int plp_scrambler_type;
  int plp_fec_type;
  int plp_mod;
  int plp_cod;
  int plp_TI_mode;
  int plp_fec_block_start;
  int plp_CTI_fec_block_start;
  int plp_num_channel_bonded;
  int plp_channel_bonding_format;
  int plp_bonded_rf_id;
  int plp_mimo_stream_combining;
  int plp_mimo_IQ_interleaving;
  int plp_mimo_PH;
  int plp_type;
  int plp_num_subslices;
  int plp_subslice_interval;
  int plp_TI_extended_interleaving;
  int plp_CTI_depth;
  int plp_CTI_start_row;
  int plp_HTI_inter_subframe;
  int plp_HTI_num_ti_blocks;
  int plp_HTI_num_fec_blocks_max;
  int plp_HTI_num_fec_blocks;
  int plp_HTI_cell_interleaver;
  int plp_ldm_injection_level;
  int bsid;
  long long reserved;
  int crc;
} L1_Detail;

typedef struct {
  L1_Basic l1basic_data;
  L1_Detail l1detail_data;
} L1Signalling;

typedef struct{
    int table_length;
    int d[LDPC_ENCODE_TABLE_LENGTH];
    int p[LDPC_ENCODE_TABLE_LENGTH];
}ldpc_encode_table;

namespace gr {
  namespace atsc3 {

    class framemapper_cc_impl : public framemapper_cc
    {
     private:
      int l1b_mode;
      int l1d_mode;
      int plp_size;
      int first_sbs;
      int symbols;
      int total_cells;
      int sbsnullcells;
      int preamble_syms;
      L1Signalling L1_Signalling[1];
      int add_l1basic(gr_complex*, int);
      int add_l1detail(gr_complex*, int, int);
      int add_crc32_bits(unsigned char*, int);
      void init_fm_randomizer(void);
      void init_ti_randomizer(void);
      void calculate_crc_table();
      int poly_mult(const int*, int, const int*, int, int*);
      void bch_poly_build_tables(void);
      void block_interleaver(unsigned char *l1, const unsigned char *l1t, gr_complex *out, int mode, int rows, int l1select);
      unsigned char l1_temp[FRAME_SIZE_SHORT];
      unsigned char l1_basic[FRAME_SIZE_SHORT];
      unsigned char l1_detail[FRAME_SIZE_SHORT];
      unsigned char fm_randomize[MAX_L1DETAIL_MSG_SIZE];
      std::bitset<MAX_BCH_PARITY_BITS> crc_table[256];
      int num_parity_bits;
      std::bitset<MAX_BCH_PARITY_BITS> polynome;
      int q_val;
      int q1_val;
      int q2_val;
      int m1_val;
      int m2_val;
      int ldpc_lut_index[FRAME_SIZE_SHORT];
      unsigned char buffer[FRAME_SIZE_SHORT];
      void ldpc_lookup_generate(void);
      ldpc_encode_table ldpc_encode_1st;
      ldpc_encode_table ldpc_encode_2nd;
      gr_complex m_qpsk[4];
      gr_complex m_16qam[16];
      gr_complex m_64qam[64];
      gr_complex m_l1b_256qam[256];
      gr_complex m_l1d_256qam[256];
      int frame_symbols[4352];
      long long samples;
      int frame_samples;
      long long cells;
      int fec_cells;

      gr_complex l1_dummy[FRAME_SIZE_SHORT];

      int ti_mode;
      int ti_depth;
      int ti_randomize[(MAX_INTERLEAVER_DEPTH * MAX_INTERLEAVER_DEPTH) * 4];
      int commutator;
      gr_complex ti_qpsk[4];
      gr_complex ti_16qam[16];
      gr_complex ti_64qam[64];
      gr_complex ti_256qam[256];
      gr_complex *time_interleaver;
      std::vector<std::deque<gr_complex>> delay_line;

      std::vector<uint16_t*> ldpc_lut; // Pointers into ldpc_lut_data.
      std::vector<uint16_t> ldpc_lut_data;

      template <typename entry_t, size_t rows, size_t cols>
      void ldpc_bf_type_b(entry_t (&table)[rows][cols])
      {
        size_t max_lut_arraysize = 0;
        const unsigned int pbits = FRAME_SIZE_SHORT - NBCH_6_15;
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

      template <typename entry_t, size_t rows, size_t cols>
      void ldpc_bf_type_a(entry_t (&table)[rows][cols])
      {
        int im = 0;
        int index = 0;
        int row;
        for (row = 0; row < (int)rows; row++) {
          if (im == NBCH_3_15) {
            break;
          }
          for (int n = 0; n < 360; n++) {
            for (int col = 1; col <= table[row][0]; col++) {
              if ((im % 360) == 0) {
                ldpc_encode_1st.p[index] = table[row][col];
                ldpc_encode_1st.d[index] = im;
                index++;
              }
              else {
                if (table[row][col] < m1_val) {
                  ldpc_encode_1st.p[index] = (table[row][col] + (n * q1_val)) % m1_val;
                  ldpc_encode_1st.d[index] = im;
                  index++;
                }
                else {
                  ldpc_encode_1st.p[index] = m1_val + (table[row][col] - m1_val + (n * q2_val)) % m2_val;
                  ldpc_encode_1st.d[index] = im;
                  index++;
                }
              }
            }
            im++;
          }
        }
        ldpc_encode_1st.table_length = index;
        index = 0;
        for (;row < (int)rows; row++) {
          for (int n = 0; n < 360; n++) {
            for (int col = 1; col <= table[row][0]; col++) {
              if ((im % 360) == 0) {
                ldpc_encode_2nd.p[index] = table[row][col];
                ldpc_encode_2nd.d[index] = im;
                index++;
              }
              else {
                if (table[row][col] < m1_val) {
                  ldpc_encode_2nd.p[index] = (table[row][col] + (n * q1_val)) % m1_val;
                  ldpc_encode_2nd.d[index] = im;
                  index++;
                }
                else {
                  ldpc_encode_2nd.p[index] = m1_val + (table[row][col] - m1_val + (n * q2_val)) % m2_val;
                  ldpc_encode_2nd.d[index] = im;
                  index++;
                }
              }
            }
            im++;
          }
        }
        ldpc_encode_2nd.table_length = index;
      }

      const static int shortening_table[8][18];
      const static uint16_t ldpc_tab_3_15S[12][12];
      const static uint16_t ldpc_tab_6_15S[18][31];
      const static int group_table[8][36];
      const static gr_complex mod_table_16QAM[12][4];
      const static gr_complex mod_table_64QAM[12][16];
      const static gr_complex mod_table_256QAM[12][64];
      const static int preamble_cells_table[32][5];
      const static int data_cells_table_8K[16][5];
      const static int data_cells_table_16K[16][5];
      const static int data_cells_table_32K[16][5];
      const static int sbs_cells_table_8K[16][5];
      const static int sbs_cells_table_16K[16][5];
      const static int sbs_cells_table_32K[16][5];
      const static int sbs_data_cells_table_8K[16][5][5];
      const static int sbs_data_cells_table_16K[16][5][5];
      const static int sbs_data_cells_table_32K[16][5][5];

     public:
      framemapper_cc_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation, atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_scattered_pilot_boost_t pilotboost, atsc3_first_sbs_t firstsbs, atsc3_frequency_interleaver_t fimode, atsc3_time_interleaver_mode_t timode, atsc3_time_interleaver_depth_t tidepth, atsc3_plp_fec_mode_t fecmode, atsc3_papr_t paprmode, atsc3_reduced_carriers_t cred, atsc3_l1_fec_mode_t l1bmode, atsc3_l1_fec_mode_t l1dmode);
      ~framemapper_cc_impl();

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_FRAMEMAPPER_CC_IMPL_H */
