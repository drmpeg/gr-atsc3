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
  int reserved;
  int crc;
} L1_Detail;

typedef struct {
  L1_Basic l1basic_data;
  L1_Detail l1detail_data;
} L1Signalling;

namespace gr {
  namespace atsc3 {

    class framemapper_cc_impl : public framemapper_cc
    {
     private:
      L1Signalling L1_Signalling[1];
      void add_l1basic(gr_complex*);
      void add_l1detail(gr_complex*);
      int add_crc32_bits(unsigned char*, int);
      void init_fm_randomizer(void);
      void calculate_crc_table();
      int poly_mult(const int*, int, const int*, int, int*);
      void bch_poly_build_tables(void);
      unsigned char l1_interleave[FRAME_SIZE_SHORT];
      unsigned char fm_randomize[FRAME_SIZE_SHORT];
      std::bitset<MAX_BCH_PARITY_BITS> crc_table[256];
      int num_parity_bits;
      std::bitset<MAX_BCH_PARITY_BITS> polynome;

      gr_complex l1basic_cache[1840];

     public:
      framemapper_cc_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation, atsc3_guardinterval_t guardinterval);
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
