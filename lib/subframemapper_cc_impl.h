/* -*- c++ -*- */
/*
 * Copyright 2022,2023 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_SUBFRAMEMAPPER_CC_IMPL_H
#define INCLUDED_ATSC3_SUBFRAMEMAPPER_CC_IMPL_H

#include <atsc3/subframemapper_cc.h>
#include "atsc3_defines.h"
#include <bitset>
#include <deque>
#include <vector>

#define NBCH_3_15 3240
#define NBCH_6_15 6480
#define MAX_INTERLEAVER_DEPTH 1448
#define NUM_SUBFRAMES 2
#define NUM_PLPS 1

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
  L1_Detail l1detail_data[NUM_SUBFRAMES][NUM_PLPS];
} L1Signalling;

namespace gr {
  namespace atsc3 {

    class subframemapper_cc_impl : public subframemapper_cc
    {
     private:
      int l1b_mode;
      int l1d_mode;
      int plp_size[NUM_SUBFRAMES];
      int first_sbs[NUM_SUBFRAMES];
      int symbols[NUM_SUBFRAMES];
      int total_cells[NUM_SUBFRAMES];
      int sbsnullcells[NUM_SUBFRAMES];
      int preamble_syms[NUM_SUBFRAMES];
      int max_output_cells;
      L1Signalling L1_Signalling[1];
      int add_l1basic(gr_complex*, int, int);
      struct l1_detail_params_t add_l1detail(gr_complex*, int, int, int, int);
      int add_crc32_bits(unsigned char*, int);
      void init_fm_randomizer(void);
      void init_ti_randomizer(void);
      void calculate_crc_table();
      int poly_mult(const int*, int, const int*, int, int*);
      void bch_poly_build_tables(void);
      void block_interleaver(unsigned char *l1, const unsigned char *l1t, gr_complex *out, int mode, int rows, int l1select);
      void init_address(int);
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
      gr_complex m_qpsk[4];
      gr_complex m_16qam[16];
      gr_complex m_64qam[64];
      gr_complex m_l1b_256qam[256];
      gr_complex m_l1d_256qam[256];
      int frame_symbols[NUM_SUBFRAMES][4352];
      long long samples;
      int frame_samples[NUM_SUBFRAMES];
      long long cells[NUM_SUBFRAMES];
      int fec_cells[NUM_SUBFRAMES];
      int plp_lls_flag[NUM_SUBFRAMES];

      gr_complex l1_dummy[FRAME_SIZE_SHORT];

      int ti_mode[NUM_SUBFRAMES];
      int ti_depth[NUM_SUBFRAMES];
      int ti_blocks[NUM_SUBFRAMES];
      int ti_fecblocks[NUM_SUBFRAMES];
      int ti_fecblocks_max[NUM_SUBFRAMES];
      int Nfec_ti_max[NUM_SUBFRAMES];
      int ti_randomize[(MAX_INTERLEAVER_DEPTH * MAX_INTERLEAVER_DEPTH) * 4];
      int commutator[NUM_SUBFRAMES];
      gr_complex ti_qpsk[4];
      gr_complex ti_16qam[16];
      gr_complex ti_64qam[64];
      gr_complex ti_256qam[256];
      std::vector<gr_complex> time_interleaver[NUM_SUBFRAMES];
      std::vector<gr_complex> hybrid_time_interleaver[NUM_SUBFRAMES];
      std::vector<std::vector<std::vector<int>>> HtimeLr[NUM_SUBFRAMES];
      std::vector<std::vector<int>> HtimePr[NUM_SUBFRAMES];
      std::vector<std::vector<int>> HtimeTBI[NUM_SUBFRAMES];
      std::vector<int> HtimeNfec[NUM_SUBFRAMES];
      std::vector<std::deque<gr_complex>> delay_line[NUM_SUBFRAMES];

      std::vector<uint16_t*> ldpc_lut_a; // Pointers into ldpc_lut_data.
      std::vector<uint16_t> ldpc_lut_a_data;
      std::vector<uint16_t*> ldpc_lut_a_aux; // Pointers into ldpc_lut_data.
      std::vector<uint16_t> ldpc_lut_a_aux_data;
      std::vector<uint16_t*> ldpc_lut_b; // Pointers into ldpc_lut_data.
      std::vector<uint16_t> ldpc_lut_b_data;

#include "ldpc_lut.h"

      const static int shortening_table[8][18];
      const static uint16_t ldpc_tab_3_15S[12][12];
      const static uint16_t ldpc_tab_6_15S[18][31];
      const static int group_table[8][36];
      const static gr_complex mod_table_16QAM[12][4];
      const static gr_complex mod_table_64QAM[12][16];
      const static gr_complex mod_table_256QAM[12][64];

     public:
      subframemapper_cc_impl(atsc3_framesize_t framesizesub0, atsc3_code_rate_t ratesub0, atsc3_plp_fec_mode_t fecmodesub0, atsc3_constellation_t constellationsub0, atsc3_fftsize_t fftsizesub0, int numpayloadsymssub0, int numpreamblesyms, atsc3_guardinterval_t guardintervalsub0, atsc3_pilotpattern_t pilotpatternsub0, atsc3_scattered_pilot_boost_t pilotboostsub0, atsc3_first_sbs_t firstsbs, atsc3_frequency_interleaver_t fimodesub0, atsc3_time_interleaver_mode_t timodesub0, atsc3_time_interleaver_depth_t tidepthsub0, int tiblockssub0, int tifecblocksmaxsub0, int tifecblockssub0, atsc3_lls_insertion_mode_t llsmodesub0, atsc3_reduced_carriers_t credsub0, atsc3_miso_t misomodesub0, atsc3_framesize_t framesizesub1, atsc3_code_rate_t ratesub1, atsc3_plp_fec_mode_t fecmodesub1, atsc3_constellation_t constellationsub1, atsc3_fftsize_t fftsizesub1, int numpayloadsymssub1, atsc3_guardinterval_t guardintervalsub1, atsc3_pilotpattern_t pilotpatternsub1, atsc3_scattered_pilot_boost_t pilotboostsub1, atsc3_frequency_interleaver_t fimodesub1, atsc3_time_interleaver_mode_t timodesub1, atsc3_time_interleaver_depth_t tidepthsub1, int tiblockssub1, int tifecblocksmaxsub1, int tifecblockssub1, atsc3_lls_insertion_mode_t llsmodesub1, atsc3_reduced_carriers_t credsub1, atsc3_miso_t misomodesub1, atsc3_frame_length_mode_t flmode, int flen, atsc3_papr_t paprmode, atsc3_l1_fec_mode_t l1bmode, atsc3_l1_fec_mode_t l1dmode);
      ~subframemapper_cc_impl();

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_SUBFRAMEMAPPER_CC_IMPL_H */
