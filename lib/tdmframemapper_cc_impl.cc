/* -*- c++ -*- */
/*
 * Copyright 2022,2023 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "tdmframemapper_cc_impl.h"
#include "params.h"
#include <algorithm>

namespace gr {
  namespace atsc3 {

    using input_type = gr_complex;
    using output_type = gr_complex;
    tdmframemapper_cc::sptr
    tdmframemapper_cc::make(atsc3_framesize_t framesizeplp0, atsc3_code_rate_t rateplp0, atsc3_plp_fec_mode_t fecmodeplp0, atsc3_constellation_t constellationplp0, atsc3_time_interleaver_mode_t timodeplp0, atsc3_time_interleaver_depth_t tidepthplp0, int tiblocksplp0, int tifecblocksmaxplp0, int tifecblocksplp0, int plpsizeplp0, atsc3_lls_insertion_mode_t llsmodeplp0, atsc3_framesize_t framesizeplp1, atsc3_code_rate_t rateplp1, atsc3_plp_fec_mode_t fecmodeplp1, atsc3_constellation_t constellationplp1, atsc3_time_interleaver_mode_t timodeplp1, atsc3_time_interleaver_depth_t tidepthplp1, int tiblocksplp1, int tifecblocksmaxplp1, int tifecblocksplp1, int plpsizeplp1, atsc3_lls_insertion_mode_t llsmodeplp1, atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_scattered_pilot_boost_t pilotboost, atsc3_first_sbs_t firstsbs, atsc3_frequency_interleaver_t fimode, atsc3_reduced_carriers_t cred, atsc3_frame_length_mode_t flmode, int flen, atsc3_miso_t misomode, atsc3_papr_t paprmode, atsc3_l1_fec_mode_t l1bmode, atsc3_l1_fec_mode_t l1dmode)
    {
      return gnuradio::make_block_sptr<tdmframemapper_cc_impl>(
        framesizeplp0, rateplp0, fecmodeplp0, constellationplp0, timodeplp0, tidepthplp0, tiblocksplp0, tifecblocksmaxplp0, tifecblocksplp0, plpsizeplp0, llsmodeplp0, framesizeplp1, rateplp1, fecmodeplp1, constellationplp1, timodeplp1, tidepthplp1, tiblocksplp1, tifecblocksmaxplp1, tifecblocksplp1, plpsizeplp1, llsmodeplp1, fftsize, numpayloadsyms, numpreamblesyms, guardinterval, pilotpattern, pilotboost, firstsbs, fimode, cred, flmode, flen, misomode, paprmode, l1bmode, l1dmode);
    }


    /*
     * The private constructor
     */
    tdmframemapper_cc_impl::tdmframemapper_cc_impl(atsc3_framesize_t framesizeplp0, atsc3_code_rate_t rateplp0, atsc3_plp_fec_mode_t fecmodeplp0, atsc3_constellation_t constellationplp0, atsc3_time_interleaver_mode_t timodeplp0, atsc3_time_interleaver_depth_t tidepthplp0, int tiblocksplp0, int tifecblocksmaxplp0, int tifecblocksplp0, int plpsizeplp0, atsc3_lls_insertion_mode_t llsmodeplp0, atsc3_framesize_t framesizeplp1, atsc3_code_rate_t rateplp1, atsc3_plp_fec_mode_t fecmodeplp1, atsc3_constellation_t constellationplp1, atsc3_time_interleaver_mode_t timodeplp1, atsc3_time_interleaver_depth_t tidepthplp1, int tiblocksplp1, int tifecblocksmaxplp1, int tifecblocksplp1, int plpsizeplp1, atsc3_lls_insertion_mode_t llsmodeplp1, atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_scattered_pilot_boost_t pilotboost, atsc3_first_sbs_t firstsbs, atsc3_frequency_interleaver_t fimode, atsc3_reduced_carriers_t cred, atsc3_frame_length_mode_t flmode, int flen, atsc3_miso_t misomode, atsc3_papr_t paprmode, atsc3_l1_fec_mode_t l1bmode, atsc3_l1_fec_mode_t l1dmode)
      : gr::block("tdmframemapper_cc",
              gr::io_signature::make(2, 2, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      L1_Basic *l1basicinit = &L1_Signalling[0].l1basic_data;
      L1_Detail *l1detailinit[NUM_SUBFRAMES][NUM_PLPS];
      struct fec_params_t pf;
      double normalization;
      int rateindex, i, j, l1cells, totalcells;
      int fftsamples, gisamples;
      int total_preamble_cells;
      int first_preamble_cells;
      int preamble_cells;
      int data_cells;
      int sbs_cells;
      int sbs_data_cells;
      int papr_cells;
      int plp_size_total;
      int depth;
      int randombits, randomindex;
      int Nextra;
      int xbytes, xbits;
      struct l1_detail_params_t rtn;

      l1detailinit[0][0] = &L1_Signalling[0].l1detail_data[0][0];
      l1detailinit[0][1] = &L1_Signalling[0].l1detail_data[0][1];
      samples = 0;
      cells[0] = cells[1] = 0;
      l1b_mode = l1bmode;
      l1d_mode = l1dmode;
      first_sbs = firstsbs;
      symbols = numpreamblesyms + numpayloadsyms;
      preamble_syms = numpreamblesyms;
      init_fm_randomizer();
      num_parity_bits = 168;
      bch_poly_build_tables();
      q1_val = 3;
      q2_val = 33;
      m1_val = 1080;
      m2_val = 11880;
      ldpc_bf_type_a(ldpc_tab_3_15S);
      q_val = 27;
      ldpc_bf_type_b(ldpc_tab_6_15S);

      normalization = std::sqrt(2.0);
      m_qpsk[0] = gr_complex( 1.0 / normalization,  1.0 / normalization);
      m_qpsk[1] = gr_complex(-1.0 / normalization,  1.0 / normalization);
      m_qpsk[2] = gr_complex( 1.0 / normalization, -1.0 / normalization);
      m_qpsk[3] = gr_complex(-1.0 / normalization, -1.0 / normalization);
      for (i = 0, j = 0; i < 4; i++, j++) {
        m_16qam[i] = mod_table_16QAM[6][j];
      }
      for (i = 4, j = 0; i < 8; i++, j++) {
        m_16qam[i] = -std::conj(mod_table_16QAM[6][j]);
      }
      for (i = 8, j = 0; i < 12; i++, j++) {
        m_16qam[i] = std::conj(mod_table_16QAM[6][j]);
      }
      for (i = 12, j = 0; i < 16; i++, j++) {
        m_16qam[i] = -mod_table_16QAM[6][j];
      }
      for (i = 0, j = 0; i < 16; i++, j++) {
        m_64qam[i] = mod_table_64QAM[7][j];
      }
      for (i = 16, j = 0; i < 32; i++, j++) {
        m_64qam[i] = -std::conj(mod_table_64QAM[7][j]);
      }
      for (i = 32, j = 0; i < 48; i++, j++) {
        m_64qam[i] = std::conj(mod_table_64QAM[7][j]);
      }
      for (i = 48, j = 0; i < 64; i++, j++) {
        m_64qam[i] = -mod_table_64QAM[7][j];
      }
      if (l1bmode == L1_FEC_MODE_6) {
        rateindex = 7;
        for (i = 0, j = 0; i < 64; i++, j++) {
          m_l1b_256qam[i] = mod_table_256QAM[rateindex][j];
        }
        for (i = 64, j = 0; i < 128; i++, j++) {
          m_l1b_256qam[i] = -std::conj(mod_table_256QAM[rateindex][j]);
        }
        for (i = 128, j = 0; i < 192; i++, j++) {
          m_l1b_256qam[i] = std::conj(mod_table_256QAM[rateindex][j]);
        }
        for (i = 192, j = 0; i < 256; i++, j++) {
          m_l1b_256qam[i] = -mod_table_256QAM[rateindex][j];
        }
      }
      else {
        rateindex = 11;
        for (i = 0, j = 0; i < 64; i++, j++) {
          m_l1b_256qam[i] = mod_table_256QAM[rateindex][j];
        }
        for (i = 64, j = 0; i < 128; i++, j++) {
          m_l1b_256qam[i] = -std::conj(mod_table_256QAM[rateindex][j]);
        }
        for (i = 128, j = 0; i < 192; i++, j++) {
          m_l1b_256qam[i] = std::conj(mod_table_256QAM[rateindex][j]);
        }
        for (i = 192, j = 0; i < 256; i++, j++) {
          m_l1b_256qam[i] = -mod_table_256QAM[rateindex][j];
        }
      }
      if (l1dmode == L1_FEC_MODE_6) {
        rateindex = 7;
        for (i = 0, j = 0; i < 64; i++, j++) {
          m_l1d_256qam[i] = mod_table_256QAM[rateindex][j];
        }
        for (i = 64, j = 0; i < 128; i++, j++) {
          m_l1d_256qam[i] = -std::conj(mod_table_256QAM[rateindex][j]);
        }
        for (i = 128, j = 0; i < 192; i++, j++) {
          m_l1d_256qam[i] = std::conj(mod_table_256QAM[rateindex][j]);
        }
        for (i = 192, j = 0; i < 256; i++, j++) {
          m_l1d_256qam[i] = -mod_table_256QAM[rateindex][j];
        }
      }
      else {
        rateindex = 11;
        for (i = 0, j = 0; i < 64; i++, j++) {
          m_l1d_256qam[i] = mod_table_256QAM[rateindex][j];
        }
        for (i = 64, j = 0; i < 128; i++, j++) {
          m_l1d_256qam[i] = -std::conj(mod_table_256QAM[rateindex][j]);
        }
        for (i = 128, j = 0; i < 192; i++, j++) {
          m_l1d_256qam[i] = std::conj(mod_table_256QAM[rateindex][j]);
        }
        for (i = 192, j = 0; i < 256; i++, j++) {
          m_l1d_256qam[i] = -mod_table_256QAM[rateindex][j];
        }
      }

      l1basicinit->version = 0;
      l1basicinit->mimo_scattered_pilot_encoding = MSPE_WALSH_HADAMARD_PILOTS;
      l1basicinit->time_info_flag = TIF_NOT_INCLUDED;
      l1basicinit->return_channel_flag = FALSE;
      l1basicinit->papr_reduction = paprmode;
      l1basicinit->frame_length_mode = flmode;
      if (flmode == FLM_SYMBOL_ALIGNED) {
        l1basicinit->time_offset = 0;
        l1basicinit->additional_samples = 0; /* always 0 */
      }
      else {
        l1basicinit->frame_length = flen / 5;
        l1basicinit->excess_samples_per_symbol = 0;
      }
      l1basicinit->num_subframes = NUM_SUBFRAMES - 1;
      l1basicinit->preamble_num_symbols = numpreamblesyms - 1;
      if (numpreamblesyms == 1) {
        l1basicinit->preamble_reduced_carriers = 0;
      }
      else {
        l1basicinit->preamble_reduced_carriers = cred;
      }
      l1basicinit->L1_Detail_content_tag = 0;
      l1basicinit->L1_Detail_size_bytes = MIN_L1DETAIL_BYTES * 3;
      l1basicinit->L1_Detail_fec_type = l1dmode;
      l1basicinit->L1_Detail_additional_parity_mode = APM_K0;
      l1basicinit->first_sub_mimo = FALSE;
      l1basicinit->first_sub_miso = misomode;
      l1basicinit->first_sub_fft_size = fftsize;
      l1basicinit->first_sub_reduced_carriers = cred;
      l1basicinit->first_sub_guard_interval = guardinterval;
      l1basicinit->first_sub_num_ofdm_symbols = numpayloadsyms - 1;
      l1basicinit->first_sub_scattered_pilot_pattern = pilotpattern;
      l1basicinit->first_sub_scattered_pilot_boost = pilotboost;
      l1basicinit->first_sub_sbs_first = firstsbs;
      l1basicinit->first_sub_sbs_last = SBS_ON;
      l1basicinit->reserved = 0xffffffffffff;

      l1detailinit[0][0]->version = 1;
      l1detailinit[0][0]->num_rf = 0;
      l1detailinit[0][0]->frequency_interleaver = fimode;
      l1detailinit[0][0]->num_plp = NUM_PLPS - 1;
      l1detailinit[0][0]->plp_id = 0;
      if (llsmodeplp0 == LLS_ON) {
        l1detailinit[0][0]->plp_lls_flag = TRUE;
      }
      else {
        l1detailinit[0][0]->plp_lls_flag = FALSE;
      }
      plp_lls_flag[0] = l1detailinit[0][0]->plp_lls_flag;
      l1detailinit[0][0]->plp_layer = 0;
      l1detailinit[0][0]->plp_start = 0;
      l1detailinit[0][0]->plp_scrambler_type = 0;
      if (framesizeplp0 == FECFRAME_SHORT) {
        switch (constellationplp0) {
          case MOD_QPSK:
            fec_cells[0] = 8100;
            break;
          case MOD_16QAM:
            fec_cells[0] = 4050;
            break;
          case MOD_64QAM:
            fec_cells[0] = 2700;
            break;
          case MOD_256QAM:
            fec_cells[0] = 2025;
            break;
          default:
            fec_cells[0] = 0;
            break;
        }
        switch (fecmodeplp0) {
          case PLP_FEC_NONE:
            l1detailinit[0][0]->plp_fec_type = FEC_TYPE_ONLY_16K;
            break;
          case PLP_FEC_CRC32:
            l1detailinit[0][0]->plp_fec_type = FEC_TYPE_CRC_16K;
            break;
          case PLP_FEC_BCH:
            l1detailinit[0][0]->plp_fec_type = FEC_TYPE_BCH_16K;
            break;
          default:
            l1detailinit[0][0]->plp_fec_type = FEC_TYPE_BCH_16K;
            break;
        }
      }
      else {
        switch (constellationplp0) {
          case MOD_QPSK:
            fec_cells[0] = 32400;
            break;
          case MOD_16QAM:
            fec_cells[0] = 16200;
            break;
          case MOD_64QAM:
            fec_cells[0] = 10800;
            break;
          case MOD_256QAM:
            fec_cells[0] = 8100;
            break;
          case MOD_1024QAM:
            fec_cells[0] = 6480;
            break;
          case MOD_4096QAM:
            fec_cells[0] = 5400;
            break;
          default:
            fec_cells[0] = 0;
            break;
        }
        switch (fecmodeplp0) {
          case PLP_FEC_NONE:
            l1detailinit[0][0]->plp_fec_type = FEC_TYPE_ONLY_64K;
            break;
          case PLP_FEC_CRC32:
            l1detailinit[0][0]->plp_fec_type = FEC_TYPE_CRC_64K;
            break;
          case PLP_FEC_BCH:
            l1detailinit[0][0]->plp_fec_type = FEC_TYPE_BCH_64K;
            break;
          default:
            l1detailinit[0][0]->plp_fec_type = FEC_TYPE_BCH_64K;
            break;
        }
      }
      l1detailinit[0][0]->plp_mod = constellationplp0;
      l1detailinit[0][0]->plp_cod = rateplp0;
      l1detailinit[0][0]->plp_TI_mode = timodeplp0;
      l1detailinit[0][0]->plp_TI_extended_interleaving = FALSE;
      l1detailinit[0][0]->plp_HTI_inter_subframe = FALSE;
      l1detailinit[0][0]->plp_HTI_num_ti_blocks = tiblocksplp0 - 1;
      l1detailinit[0][0]->plp_HTI_num_fec_blocks_max = tifecblocksmaxplp0 - 1;
      l1detailinit[0][0]->plp_HTI_num_fec_blocks = tifecblocksplp0 - 1;
      l1detailinit[0][0]->plp_HTI_cell_interleaver = TRUE;
      l1detailinit[0][0]->plp_type = 0;
      if (l1detailinit[0][0]->plp_TI_mode == TI_MODE_CONVOLUTIONAL) {
        if (tidepthplp0 == TI_DEPTH_1254) {
          l1detailinit[0][0]->plp_TI_extended_interleaving = TRUE;
          tidepthplp0 = TI_DEPTH_887;
        }
        else if (tidepthplp0 == TI_DEPTH_1448) {
          l1detailinit[0][0]->plp_TI_extended_interleaving = TRUE;
          tidepthplp0 = TI_DEPTH_1024;
        }
        else {
          l1detailinit[0][0]->plp_TI_extended_interleaving = FALSE;
        }
        l1detailinit[0][0]->plp_CTI_depth = tidepthplp0;
      }

      l1detailinit[0][1]->plp_id = 1;
      if (llsmodeplp1 == LLS_ON) {
        l1detailinit[0][1]->plp_lls_flag = TRUE;
      }
      else {
        l1detailinit[0][1]->plp_lls_flag = FALSE;
      }
      plp_lls_flag[1] = l1detailinit[0][1]->plp_lls_flag;
      l1detailinit[0][1]->plp_layer = 0;
      l1detailinit[0][1]->plp_start = 0;
      l1detailinit[0][1]->plp_scrambler_type = 0;
      if (framesizeplp1 == FECFRAME_SHORT) {
        switch (constellationplp1) {
          case MOD_QPSK:
            fec_cells[1] = 8100;
            break;
          case MOD_16QAM:
            fec_cells[1] = 4050;
            break;
          case MOD_64QAM:
            fec_cells[1] = 2700;
            break;
          case MOD_256QAM:
            fec_cells[1] = 2025;
            break;
          default:
            fec_cells[1] = 0;
            break;
        }
        switch (fecmodeplp1) {
          case PLP_FEC_NONE:
            l1detailinit[0][1]->plp_fec_type = FEC_TYPE_ONLY_16K;
            break;
          case PLP_FEC_CRC32:
            l1detailinit[0][1]->plp_fec_type = FEC_TYPE_CRC_16K;
            break;
          case PLP_FEC_BCH:
            l1detailinit[0][1]->plp_fec_type = FEC_TYPE_BCH_16K;
            break;
          default:
            l1detailinit[0][1]->plp_fec_type = FEC_TYPE_BCH_16K;
            break;
        }
      }
      else {
        switch (constellationplp1) {
          case MOD_QPSK:
            fec_cells[1] = 32400;
            break;
          case MOD_16QAM:
            fec_cells[1] = 16200;
            break;
          case MOD_64QAM:
            fec_cells[1] = 10800;
            break;
          case MOD_256QAM:
            fec_cells[1] = 8100;
            break;
          case MOD_1024QAM:
            fec_cells[1] = 6480;
            break;
          case MOD_4096QAM:
            fec_cells[1] = 5400;
            break;
          default:
            fec_cells[1] = 0;
            break;
        }
        switch (fecmodeplp1) {
          case PLP_FEC_NONE:
            l1detailinit[0][1]->plp_fec_type = FEC_TYPE_ONLY_64K;
            break;
          case PLP_FEC_CRC32:
            l1detailinit[0][1]->plp_fec_type = FEC_TYPE_CRC_64K;
            break;
          case PLP_FEC_BCH:
            l1detailinit[0][1]->plp_fec_type = FEC_TYPE_BCH_64K;
            break;
          default:
            l1detailinit[0][1]->plp_fec_type = FEC_TYPE_BCH_64K;
            break;
        }
      }
      l1detailinit[0][1]->plp_mod = constellationplp1;
      l1detailinit[0][1]->plp_cod = rateplp1;
      l1detailinit[0][1]->plp_TI_mode = timodeplp1;
      l1detailinit[0][1]->plp_TI_extended_interleaving = FALSE;
      l1detailinit[0][1]->plp_HTI_inter_subframe = FALSE;
      l1detailinit[0][1]->plp_HTI_num_ti_blocks = tiblocksplp1 - 1;
      l1detailinit[0][1]->plp_HTI_num_fec_blocks_max = tifecblocksmaxplp1 - 1;
      l1detailinit[0][1]->plp_HTI_num_fec_blocks = tifecblocksplp1 - 1;
      l1detailinit[0][1]->plp_HTI_cell_interleaver = TRUE;
      l1detailinit[0][1]->plp_type = 0;
      if (l1detailinit[0][1]->plp_TI_mode == TI_MODE_CONVOLUTIONAL) {
        if (tidepthplp1 == TI_DEPTH_1254) {
          l1detailinit[0][1]->plp_TI_extended_interleaving = TRUE;
          tidepthplp1 = TI_DEPTH_887;
        }
        else if (tidepthplp1 == TI_DEPTH_1448) {
          l1detailinit[0][1]->plp_TI_extended_interleaving = TRUE;
          tidepthplp1 = TI_DEPTH_1024;
        }
        else {
          l1detailinit[0][1]->plp_TI_extended_interleaving = FALSE;
        }
        l1detailinit[0][1]->plp_CTI_depth = tidepthplp1;
      }
      l1detailinit[0][1]->bsid = 0x8086;
      l1detailinit[0][1]->reserved = 0x7fffffffffffffff;

      rtn = add_l1detail(&l1_dummy[0], 0, 0, 0, 0);
      if (rtn.xbits >= (MIN_L1DETAIL_BYTES * 2 * 8)) {
        l1basicinit->L1_Detail_size_bytes = MIN_L1DETAIL_BYTES;
      }
      else {
        xbits = (MIN_L1DETAIL_BYTES * 2 * 8) - rtn.xbits;
        xbytes = (xbits / 8) + (xbits % 8 ? 1 : 0);
        l1basicinit->L1_Detail_size_bytes = MIN_L1DETAIL_BYTES + xbytes;
      }
      rtn = add_l1detail(&l1_dummy[0], 0, 0, 0, 0);
      l1basicinit->L1_Detail_total_cells = l1cells = rtn.cells;
      printf("L1-Detail cells = %d\n", l1cells);
      l1cells += add_l1basic(&l1_dummy[0], 0, FALSE);
      struct ofdm_params_t p = ofdm_params(fftsize, guardinterval, pilotpattern, pilotboost, cred);
      fftsamples = p.fftsamples;
      papr_cells = p.papr_cells;
      gisamples = p.gisamples;
      first_preamble_cells = p.first_preamble_cells;
      preamble_cells = p.preamble_cells;
      data_cells = p.data_cells;
      sbs_cells = p.sbs_cells;
      sbs_data_cells = p.sbs_data_cells;
      if (paprmode != PAPR_TR) {
        papr_cells = 0;
      }
      Nextra = ((flen * 6912) - BOOTSTRAP_SAMPLES) - numpreamblesyms * (fftsamples + gisamples) - numpayloadsyms * (fftsamples + gisamples);
      l1basicinit->excess_samples_per_symbol = Nextra / numpayloadsyms;
      frame_samples = ((fftsamples + gisamples) * (numpayloadsyms + numpreamblesyms)) + BOOTSTRAP_SAMPLES;
      frame_symbols[0] = first_preamble_cells;
      total_preamble_cells = 0;
      for (int n = 1; n < numpreamblesyms; n++) {
        frame_symbols[n] = (preamble_cells - papr_cells);
        total_preamble_cells += (preamble_cells - papr_cells);
      }
      if (firstsbs == SBS_ON) {
        frame_symbols[numpreamblesyms] = (sbs_cells - papr_cells);
        for (int n = 0; n < numpayloadsyms - 2; n++) {
          frame_symbols[n + numpreamblesyms + 1] = (data_cells - papr_cells);
        }
      }
      else {
        for (int n = 0; n < numpayloadsyms - 1; n++) {
          frame_symbols[n + numpreamblesyms] = (data_cells - papr_cells);
        }
      }
      frame_symbols[numpreamblesyms + numpayloadsyms - 1] = (sbs_cells - papr_cells);
      if (firstsbs == SBS_ON) {
        totalcells = first_preamble_cells + total_preamble_cells + ((numpayloadsyms - 2) * (data_cells - papr_cells)) + ((sbs_cells - papr_cells) * 2);
      }
      else {
        totalcells = first_preamble_cells + total_preamble_cells + ((numpayloadsyms - 1) * (data_cells - papr_cells)) + (sbs_cells - papr_cells);
      }
      total_cells = totalcells;
      l1detailinit[0][0]->sbs_null_cells = sbsnullcells = (sbs_cells - papr_cells) - (sbs_data_cells - papr_cells);
      printf("total cells = %d\n", totalcells);
      if (firstsbs == SBS_ON) {
        printf("SBS null cells = %d\n", sbsnullcells * 2);
        plp_size_total = totalcells - l1cells - (2 * sbsnullcells);
        printf("PLP size total = %d\n", plp_size_total);
      }
      else {
        printf("SBS null cells = %d\n", sbsnullcells);
        plp_size_total = totalcells - l1cells - sbsnullcells;
        printf("PLP size total = %d\n", plp_size_total);
      }
      if (timodeplp0 == TI_MODE_HYBRID && timodeplp1 == TI_MODE_HYBRID) {
        plp_size[0] = tifecblocksplp0 * fec_cells[0];
        plp_size[1] = tifecblocksplp1 * fec_cells[1];
        if ((plp_size[0] + plp_size[1]) > plp_size_total) {
          throw std::runtime_error("Hybrid Time Interleaver PLP size exceeds available cells.");
        }
      }
      else if (timodeplp0 == TI_MODE_HYBRID && timodeplp1 == (TI_MODE_OFF || timodeplp1 == TI_MODE_CONVOLUTIONAL)) {
        plp_size[0] = tifecblocksplp0 * fec_cells[0];
        if (plpsizeplp1 != 0) {
          plp_size[1] = plpsizeplp1;
        }
        else {
          plp_size[1] = plp_size_total - plp_size[0];
        }
      }
      else if ((timodeplp0 == TI_MODE_OFF || timodeplp0 == TI_MODE_CONVOLUTIONAL) && timodeplp1 == TI_MODE_HYBRID) {
        plp_size[1] = tifecblocksplp1 * fec_cells[1];
        if (plpsizeplp0 != 0) {
          plp_size[0] = plpsizeplp0;
        }
        else {
          plp_size[0] = plp_size_total - plp_size[1];
        }
      }
      else {
        if (plpsizeplp0 != 0 && plpsizeplp1 == 0) {
          plp_size[0] = plpsizeplp0;
          plp_size[1] = plp_size_total - plp_size[0];
        }
        else if (plpsizeplp0 == 0 && plpsizeplp1 != 0) {
          plp_size[1] = plpsizeplp1;
          plp_size[0] = plp_size_total - plp_size[1];
        }
        else if (plpsizeplp0 != 0 && plpsizeplp1 != 0) {
          plp_size[0] = plpsizeplp0;
          plp_size[1] = plpsizeplp1;
        }
        else {
          plp_size[0] = plp_size_total / 2;
          plp_size[1] = plp_size_total - plp_size[0];
        }
      }
      l1detailinit[0][0]->plp_size = plp_size[0];
      l1detailinit[0][1]->plp_size = plp_size[1];
      l1detailinit[0][1]->plp_start = plp_size[0];
      printf("PLP size 0 = %d\n", plp_size[0]);
      printf("PLP size 1 = %d\n", plp_size[1]);

      switch(tidepthplp0) {
        case TI_DEPTH_512:
          depth = 512;
          break;
        case TI_DEPTH_724:
          depth = 724;
          break;
        case TI_DEPTH_887:
          depth = 887;
          break;
        case TI_DEPTH_1024:
          depth = 1024;
          break;
        case TI_DEPTH_1254:
          depth = 1254;
          break;
        case TI_DEPTH_1448:
          depth = 1448;
          break;
        default:
          depth = 512;
          break;
      }

      init_ti_randomizer();
      randomindex = 0;
      ti_mode[0] = timodeplp0;
      ti_depth[0] = depth;
      ti_blocks[0] = tiblocksplp0;
      ti_fecblocks[0] = tifecblocksplp0;
      ti_fecblocks_max[0] = tifecblocksmaxplp0;
      commutator[0] = 0;
      pf = fec_params(framesizeplp0, rateplp0);
      rateindex = pf.rate_index;
      switch (constellationplp0) {
        case MOD_QPSK:
          normalization = std::sqrt(2.0);
          ti_qpsk[0] = gr_complex( 1.0 / normalization,  1.0 / normalization);
          ti_qpsk[1] = gr_complex(-1.0 / normalization,  1.0 / normalization);
          ti_qpsk[2] = gr_complex( 1.0 / normalization, -1.0 / normalization);
          ti_qpsk[3] = gr_complex(-1.0 / normalization, -1.0 / normalization);
          break;
        case MOD_16QAM:
          ti_16qam[0] = mod_table_16QAM[rateindex][0];
          ti_16qam[1] = mod_table_16QAM[rateindex][1];
          ti_16qam[2] = mod_table_16QAM[rateindex][2];
          ti_16qam[3] = mod_table_16QAM[rateindex][3];
          ti_16qam[4] = -std::conj(mod_table_16QAM[rateindex][0]);
          ti_16qam[5] = -std::conj(mod_table_16QAM[rateindex][1]);
          ti_16qam[6] = -std::conj(mod_table_16QAM[rateindex][2]);
          ti_16qam[7] = -std::conj(mod_table_16QAM[rateindex][3]);
          ti_16qam[8] = std::conj(mod_table_16QAM[rateindex][0]);
          ti_16qam[9] = std::conj(mod_table_16QAM[rateindex][1]);
          ti_16qam[10] = std::conj(mod_table_16QAM[rateindex][2]);
          ti_16qam[11] = std::conj(mod_table_16QAM[rateindex][3]);
          ti_16qam[12] = -mod_table_16QAM[rateindex][0];
          ti_16qam[13] = -mod_table_16QAM[rateindex][1];
          ti_16qam[14] = -mod_table_16QAM[rateindex][2];
          ti_16qam[15] = -mod_table_16QAM[rateindex][3];
          break;
        case MOD_64QAM:
          for (i = 0, j = 0; i < 16; i++, j++) {
            ti_64qam[i] = mod_table_64QAM[rateindex][j];
          }
          for (i = 16, j = 0; i < 32; i++, j++) {
            ti_64qam[i] = -std::conj(mod_table_64QAM[rateindex][j]);
          }
          for (i = 32, j = 0; i < 48; i++, j++) {
            ti_64qam[i] = std::conj(mod_table_64QAM[rateindex][j]);
          }
          for (i = 48, j = 0; i < 64; i++, j++) {
            ti_64qam[i] = -mod_table_64QAM[rateindex][j];
          }
          break;
        case MOD_256QAM:
          for (i = 0, j = 0; i < 64; i++, j++) {
            ti_256qam[i] = mod_table_256QAM[rateindex][j];
          }
          for (i = 64, j = 0; i < 128; i++, j++) {
            ti_256qam[i] = -std::conj(mod_table_256QAM[rateindex][j]);
          }
          for (i = 128, j = 0; i < 192; i++, j++) {
            ti_256qam[i] = std::conj(mod_table_256QAM[rateindex][j]);
          }
          for (i = 192, j = 0; i < 256; i++, j++) {
            ti_256qam[i] = -mod_table_256QAM[rateindex][j];
          }
          break;
        default:
          normalization = std::sqrt(2.0);
          ti_qpsk[0] = gr_complex( 1.0 / normalization,  1.0 / normalization);
          ti_qpsk[1] = gr_complex(-1.0 / normalization,  1.0 / normalization);
          ti_qpsk[2] = gr_complex( 1.0 / normalization, -1.0 / normalization);
          ti_qpsk[3] = gr_complex(-1.0 / normalization, -1.0 / normalization);
          break;
      }
      delay_line[0].reserve(depth);
      for (int i = 0; i < depth; i++) {
        delay_line[0].emplace_back(i, 0);
      }
      for (int i = 0; i < depth; i++) {
        switch (constellationplp0) {
          case MOD_QPSK:
            for (int j = 0; j < i; j++) {
              randombits = ti_randomize[randomindex] << 1;
              randombits |= ti_randomize[randomindex + 1];
              if (i != 0) {
                delay_line[0][i].push_back(ti_qpsk[randombits]);
                delay_line[0][i].pop_front();
              }
              randomindex += 2;
            }
            break;
          case MOD_16QAM:
            for (int j = 0; j < i; j++) {
              randombits = ti_randomize[randomindex] << 3;
              randombits |= ti_randomize[randomindex + 1] << 2;
              randombits |= ti_randomize[randomindex + 2] << 1;
              randombits |= ti_randomize[randomindex + 3];
              if (i != 0) {
                delay_line[0][i].push_back(ti_16qam[randombits]);
                delay_line[0][i].pop_front();
              }
              randomindex += 4;
            }
            break;
          case MOD_64QAM:
            for (int j = 0; j < i; j++) {
              randombits = ti_randomize[randomindex] << 5;
              randombits |= ti_randomize[randomindex + 1] << 4;
              randombits |= ti_randomize[randomindex + 2] << 3;
              randombits |= ti_randomize[randomindex + 3] << 2;
              randombits |= ti_randomize[randomindex + 4] << 1;
              randombits |= ti_randomize[randomindex + 5];
              if (i != 0) {
                delay_line[0][i].push_back(ti_64qam[randombits]);
                delay_line[0][i].pop_front();
              }
              randomindex += 6;
            }
            break;
          case MOD_256QAM:
            for (int j = 0; j < i; j++) {
              randombits = ti_randomize[randomindex] << 7;
              randombits |= ti_randomize[randomindex + 1] << 6;
              randombits |= ti_randomize[randomindex + 2] << 5;
              randombits |= ti_randomize[randomindex + 3] << 4;
              randombits |= ti_randomize[randomindex + 4] << 3;
              randombits |= ti_randomize[randomindex + 5] << 2;
              randombits |= ti_randomize[randomindex + 6] << 1;
              randombits |= ti_randomize[randomindex + 7];
              if (i != 0) {
                delay_line[0][i].push_back(ti_256qam[randombits]);
                delay_line[0][i].pop_front();
              }
              randomindex += 8;
            }
            break;
          default:
            break;
        }
      }

      switch(tidepthplp1) {
        case TI_DEPTH_512:
          depth = 512;
          break;
        case TI_DEPTH_724:
          depth = 724;
          break;
        case TI_DEPTH_887:
          depth = 887;
          break;
        case TI_DEPTH_1024:
          depth = 1024;
          break;
        case TI_DEPTH_1254:
          depth = 1254;
          break;
        case TI_DEPTH_1448:
          depth = 1448;
          break;
        default:
          depth = 512;
          break;
      }

      randomindex = 0;
      ti_mode[1] = timodeplp1;
      ti_depth[1] = depth;
      ti_blocks[1] = tiblocksplp1;
      ti_fecblocks[1] = tifecblocksplp1;
      ti_fecblocks_max[1] = tifecblocksmaxplp1;
      commutator[1] = 0;
      pf = fec_params(framesizeplp1, rateplp1);
      rateindex = pf.rate_index;
      switch (constellationplp1) {
        case MOD_QPSK:
          normalization = std::sqrt(2.0);
          ti_qpsk[0] = gr_complex( 1.0 / normalization,  1.0 / normalization);
          ti_qpsk[1] = gr_complex(-1.0 / normalization,  1.0 / normalization);
          ti_qpsk[2] = gr_complex( 1.0 / normalization, -1.0 / normalization);
          ti_qpsk[3] = gr_complex(-1.0 / normalization, -1.0 / normalization);
          break;
        case MOD_16QAM:
          ti_16qam[0] = mod_table_16QAM[rateindex][0];
          ti_16qam[1] = mod_table_16QAM[rateindex][1];
          ti_16qam[2] = mod_table_16QAM[rateindex][2];
          ti_16qam[3] = mod_table_16QAM[rateindex][3];
          ti_16qam[4] = -std::conj(mod_table_16QAM[rateindex][0]);
          ti_16qam[5] = -std::conj(mod_table_16QAM[rateindex][1]);
          ti_16qam[6] = -std::conj(mod_table_16QAM[rateindex][2]);
          ti_16qam[7] = -std::conj(mod_table_16QAM[rateindex][3]);
          ti_16qam[8] = std::conj(mod_table_16QAM[rateindex][0]);
          ti_16qam[9] = std::conj(mod_table_16QAM[rateindex][1]);
          ti_16qam[10] = std::conj(mod_table_16QAM[rateindex][2]);
          ti_16qam[11] = std::conj(mod_table_16QAM[rateindex][3]);
          ti_16qam[12] = -mod_table_16QAM[rateindex][0];
          ti_16qam[13] = -mod_table_16QAM[rateindex][1];
          ti_16qam[14] = -mod_table_16QAM[rateindex][2];
          ti_16qam[15] = -mod_table_16QAM[rateindex][3];
          break;
        case MOD_64QAM:
          for (i = 0, j = 0; i < 16; i++, j++) {
            ti_64qam[i] = mod_table_64QAM[rateindex][j];
          }
          for (i = 16, j = 0; i < 32; i++, j++) {
            ti_64qam[i] = -std::conj(mod_table_64QAM[rateindex][j]);
          }
          for (i = 32, j = 0; i < 48; i++, j++) {
            ti_64qam[i] = std::conj(mod_table_64QAM[rateindex][j]);
          }
          for (i = 48, j = 0; i < 64; i++, j++) {
            ti_64qam[i] = -mod_table_64QAM[rateindex][j];
          }
          break;
        case MOD_256QAM:
          for (i = 0, j = 0; i < 64; i++, j++) {
            ti_256qam[i] = mod_table_256QAM[rateindex][j];
          }
          for (i = 64, j = 0; i < 128; i++, j++) {
            ti_256qam[i] = -std::conj(mod_table_256QAM[rateindex][j]);
          }
          for (i = 128, j = 0; i < 192; i++, j++) {
            ti_256qam[i] = std::conj(mod_table_256QAM[rateindex][j]);
          }
          for (i = 192, j = 0; i < 256; i++, j++) {
            ti_256qam[i] = -mod_table_256QAM[rateindex][j];
          }
          break;
        default:
          normalization = std::sqrt(2.0);
          ti_qpsk[0] = gr_complex( 1.0 / normalization,  1.0 / normalization);
          ti_qpsk[1] = gr_complex(-1.0 / normalization,  1.0 / normalization);
          ti_qpsk[2] = gr_complex( 1.0 / normalization, -1.0 / normalization);
          ti_qpsk[3] = gr_complex(-1.0 / normalization, -1.0 / normalization);
          break;
      }
      delay_line[1].reserve(depth);
      for (int i = 0; i < depth; i++) {
        delay_line[1].emplace_back(i, 0);
      }
      for (int i = 0; i < depth; i++) {
        switch (constellationplp1) {
          case MOD_QPSK:
            for (int j = 0; j < i; j++) {
              randombits = ti_randomize[randomindex] << 1;
              randombits |= ti_randomize[randomindex + 1];
              if (i != 0) {
                delay_line[1][i].push_back(ti_qpsk[randombits]);
                delay_line[1][i].pop_front();
              }
              randomindex += 2;
            }
            break;
          case MOD_16QAM:
            for (int j = 0; j < i; j++) {
              randombits = ti_randomize[randomindex] << 3;
              randombits |= ti_randomize[randomindex + 1] << 2;
              randombits |= ti_randomize[randomindex + 2] << 1;
              randombits |= ti_randomize[randomindex + 3];
              if (i != 0) {
                delay_line[1][i].push_back(ti_16qam[randombits]);
                delay_line[1][i].pop_front();
              }
              randomindex += 4;
            }
            break;
          case MOD_64QAM:
            for (int j = 0; j < i; j++) {
              randombits = ti_randomize[randomindex] << 5;
              randombits |= ti_randomize[randomindex + 1] << 4;
              randombits |= ti_randomize[randomindex + 2] << 3;
              randombits |= ti_randomize[randomindex + 3] << 2;
              randombits |= ti_randomize[randomindex + 4] << 1;
              randombits |= ti_randomize[randomindex + 5];
              if (i != 0) {
                delay_line[1][i].push_back(ti_64qam[randombits]);
                delay_line[1][i].pop_front();
              }
              randomindex += 6;
            }
            break;
          case MOD_256QAM:
            for (int j = 0; j < i; j++) {
              randombits = ti_randomize[randomindex] << 7;
              randombits |= ti_randomize[randomindex + 1] << 6;
              randombits |= ti_randomize[randomindex + 2] << 5;
              randombits |= ti_randomize[randomindex + 3] << 4;
              randombits |= ti_randomize[randomindex + 4] << 3;
              randombits |= ti_randomize[randomindex + 5] << 2;
              randombits |= ti_randomize[randomindex + 6] << 1;
              randombits |= ti_randomize[randomindex + 7];
              if (i != 0) {
                delay_line[1][i].push_back(ti_256qam[randombits]);
                delay_line[1][i].pop_front();
              }
              randomindex += 8;
            }
            break;
          default:
            break;
        }
      }
      time_interleaver.resize(plp_size_total);
      for (int plp = 0; plp < NUM_PLPS; plp++) {
        if (ti_mode[plp] == TI_MODE_HYBRID) {
          hybrid_time_interleaver[plp].resize(plp_size[plp]);
          Nfec_ti_max[plp] = (ti_fecblocks_max[plp] / ti_blocks[plp]) + (ti_fecblocks_max[plp] % ti_blocks[plp] != 0);
          HtimeLr[plp].resize(ti_blocks[plp]);
          for (std::vector<std::vector<int>>::size_type x = 0; x != HtimeLr[plp].size(); x++) {
            HtimeLr[plp][x].resize(Nfec_ti_max[plp]);
            for (std::vector<std::vector<int>>::size_type i = 0; i != HtimeLr[plp][x].size(); i++) {
              HtimeLr[plp][x][i].resize(fec_cells[plp]);
            }
          }
          HtimePr[plp].resize(ti_blocks[plp]);
          for (std::vector<std::vector<int>>::size_type i = 0; i != HtimePr[plp].size(); i++) {
            HtimePr[plp][i].resize(Nfec_ti_max[plp]);
          }
          HtimeTBI[plp].resize(ti_blocks[plp]);
          for (std::vector<std::vector<int>>::size_type i = 0; i != HtimeTBI[plp].size(); i++) {
            HtimeTBI[plp][i].resize(fec_cells[plp] * Nfec_ti_max[plp]);
          }
          HtimeNfec[plp].resize(ti_blocks[plp]);
          init_address(plp);
        }
      }

      int sr = 0x18f;
      int b, packed;
      for (int i = 0; i < plp_size_total;) {
        packed = ((sr & 0x4) << 5) | ((sr & 0x8) << 3) | ((sr & 0x10) << 1) | \
                 ((sr & 0x20) >> 1) | ((sr & 0x200) >> 6) | ((sr & 0x1000) >> 10) | \
                 ((sr & 0x2000) >> 12) | ((sr & 0x8000) >> 15);
        for (int n = 7; n >= 0; n--) {
          if (packed & (1 << n)) {
            time_interleaver[i++] = gr_complex(-1, 0);
          }
          else {
            time_interleaver[i++] = gr_complex(1, 0);
          }
          if (i == plp_size_total) {
            break;
          }
        }
        b = sr & 1;
        sr >>= 1;
        if (b) {
          sr ^= POLYNOMIAL;
        }
      }

      set_tag_propagation_policy(TPP_DONT);
      set_output_multiple(totalcells);
    }

    /*
     * Our virtual destructor.
     */
    tdmframemapper_cc_impl::~tdmframemapper_cc_impl()
    {
    }

    void
    tdmframemapper_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = plp_size[0] * (noutput_items / total_cells);
      ninput_items_required[1] = plp_size[1] * (noutput_items / total_cells);
    }

#define CRC_POLY 0x00210801

    int
    tdmframemapper_cc_impl::add_crc32_bits(unsigned char* in, int length)
    {
      int crc = 0xffffffff;
      int b;
      int i = 0;

      for (int n = 0; n < length; n++) {
        b = in[i++] ^ ((crc >> 31) & 0x01);
        crc <<= 1;
        if (b) {
          crc ^= CRC_POLY;
        }
      }

      for (int n = 31; n >= 0; n--) {
        in[i++] = (crc & (1 << n)) ? 1 : 0;
      }
      return 32;
    }

    void
    tdmframemapper_cc_impl::init_fm_randomizer(void)
    {
      int sr = 0x18f;
      int b;

      for (int i = 0; i < MAX_L1DETAIL_MSG_SIZE; i++) {
        fm_randomize[i] = ((sr & 0x4) << 5) | ((sr & 0x8) << 3) | ((sr & 0x10) << 1) | \
                          ((sr & 0x20) >> 1) | ((sr & 0x200) >> 6) | ((sr & 0x1000) >> 10) | \
                          ((sr & 0x2000) >> 12) | ((sr & 0x8000) >> 15);
        b = sr & 1;
        sr >>= 1;
        if (b) {
          sr ^= POLYNOMIAL;
        }
      }
    }

    void
    tdmframemapper_cc_impl::init_ti_randomizer(void)
    {
      int sr = 0x18f;
      int b, packed;

      for (int i = 0; i < (MAX_INTERLEAVER_DEPTH * MAX_INTERLEAVER_DEPTH) * 4;) {
        packed = ((sr & 0x4) << 5) | ((sr & 0x8) << 3) | ((sr & 0x10) << 1) | \
                 ((sr & 0x20) >> 1) | ((sr & 0x200) >> 6) | ((sr & 0x1000) >> 10) | \
                 ((sr & 0x2000) >> 12) | ((sr & 0x8000) >> 15);
        for (int n = 7; n >= 0; n--) {
          ti_randomize[i++] = packed & (1 << n) ? 1 : 0;
        }
        b = sr & 1;
        sr >>= 1;
        if (b) {
          sr ^= POLYNOMIAL;
        }
      }
    }

    int
    tdmframemapper_cc_impl::poly_mult(const int* ina, int lena, const int* inb, int lenb, int* out)
    {
      memset(out, 0, sizeof(int) * (lena + lenb));

      for (int i = 0; i < lena; i++) {
        for (int j = 0; j < lenb; j++) {
          if (ina[i] * inb[j] > 0) {
            out[i + j]++; // count number of terms for this pwr of x
          }
        }
      }
      int max = 0;
      for (int i = 0; i < lena + lenb; i++) {
        out[i] = out[i] & 1; // If even ignore the term
        if (out[i]) {
          max = i;
        }
      }
      // return the size of array to house the result.
      return max + 1;
    }

    // precalculate the crc from:
    // http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html - cf. CRC-32 Lookup

    void
    tdmframemapper_cc_impl::calculate_crc_table(void)
    {
      for (int divident = 0; divident < 256; divident++) {
        std::bitset<MAX_BCH_PARITY_BITS> curByte(divident);
        curByte <<= num_parity_bits - 8;

        for (unsigned char bit = 0; bit < 8; bit++) {
          if ((curByte[num_parity_bits - 1]) != 0) {
            curByte <<= 1;
            curByte ^= polynome;
          }
          else {
            curByte <<= 1;
          }
        }
        crc_table[divident] = curByte;
      }
    }

    void
    tdmframemapper_cc_impl::bch_poly_build_tables(void)
    {
      // Short polynomials
      const int polys01[] = { 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 };
      const int polys02[] = { 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1 };
      const int polys03[] = { 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1 };
      const int polys04[] = { 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1 };
      const int polys05[] = { 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1 };
      const int polys06[] = { 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1 };
      const int polys07[] = { 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1 };
      const int polys08[] = { 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1 };
      const int polys09[] = { 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1 };
      const int polys10[] = { 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1 };
      const int polys11[] = { 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1 };
      const int polys12[] = { 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1 };

      int len;
      int polyout[2][200];

      len = poly_mult(polys01, 15, polys02, 15, polyout[0]);
      len = poly_mult(polys03, 15, polyout[0], len, polyout[1]);
      len = poly_mult(polys04, 15, polyout[1], len, polyout[0]);
      len = poly_mult(polys05, 15, polyout[0], len, polyout[1]);
      len = poly_mult(polys06, 15, polyout[1], len, polyout[0]);
      len = poly_mult(polys07, 15, polyout[0], len, polyout[1]);
      len = poly_mult(polys08, 15, polyout[1], len, polyout[0]);
      len = poly_mult(polys09, 15, polyout[0], len, polyout[1]);
      len = poly_mult(polys10, 15, polyout[1], len, polyout[0]);
      len = poly_mult(polys11, 15, polyout[0], len, polyout[1]);
      len = poly_mult(polys12, 15, polyout[1], len, polyout[0]);

      for (int i = 0; i < num_parity_bits; i++) {
        polynome[i] = polyout[0][i];
      }
      calculate_crc_table();
    }

    void
    tdmframemapper_cc_impl::block_interleaver(unsigned char *l1, const unsigned char *l1t, gr_complex *out, int mode, int rows, int l1select)
    {
      int cell, index, pack, count;
      gr_complex *m_256qam;
      const unsigned char *c1, *c2, *c3, *c4, *c5, *c6, *c7, *c8;

      switch (mode) {
        case L1_FEC_MODE_1:
        case L1_FEC_MODE_2:
        case L1_FEC_MODE_3:
          c1 = &l1t[0];
          c2 = &l1t[rows];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1[index++] = c1[j];
            l1[index++] = c2[j];
          }
          index = 0;
          for (int j = 0; j < rows; j++) {
            cell = l1[index++] << 1;
            cell |= l1[index++];
            *out++ = m_qpsk[cell];
          }
          break;
        case L1_FEC_MODE_4:
          c1 = &l1t[0];
          c2 = &l1t[rows];
          c3 = &l1t[rows * 2];
          c4 = &l1t[rows * 3];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1[index++] = c1[j];
            l1[index++] = c2[j];
            l1[index++] = c3[j];
            l1[index++] = c4[j];
          }
          index = count = 0;
          for (int j = 0; j < rows; j++) {
            pack = 0;
            for (int e = 3; e >= 0; e--) {
              pack |= l1[index++] << e;
            }
            cell = pack << count;
            pack = cell & 0xf;
            cell >>= 4;
            pack |= cell;
            count = (count + 1) & 0x3;
            *out++ = m_16qam[pack & 0xf];
          }
          break;
        case L1_FEC_MODE_5:
          c1 = &l1t[0];
          c2 = &l1t[rows];
          c3 = &l1t[rows * 2];
          c4 = &l1t[rows * 3];
          c5 = &l1t[rows * 4];
          c6 = &l1t[rows * 5];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1[index++] = c1[j];
            l1[index++] = c2[j];
            l1[index++] = c3[j];
            l1[index++] = c4[j];
            l1[index++] = c5[j];
            l1[index++] = c6[j];
          }
          index = count = 0;
          for (int j = 0; j < rows; j++) {
            pack = 0;
            for (int e = 5; e >= 0; e--) {
              pack |= l1[index++] << e;
            }
            cell = pack << count;
            pack = cell & 0x3f;
            cell >>= 6;
            pack |= cell;
            count = (count + 1);
            if (count == 6) {  /* faster than modulo */
              count = 0;
            }
            *out++ = m_64qam[pack & 0x3f];
          }
          break;
        case L1_FEC_MODE_6:
          if (l1select == L1_BASIC) {
            m_256qam = &m_l1b_256qam[0];
          }
          else {
            m_256qam = &m_l1d_256qam[0];
          }
          c1 = &l1t[0];
          c2 = &l1t[rows];
          c3 = &l1t[rows * 2];
          c4 = &l1t[rows * 3];
          c5 = &l1t[rows * 4];
          c6 = &l1t[rows * 5];
          c7 = &l1t[rows * 6];
          c8 = &l1t[rows * 7];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1[index++] = c1[j];
            l1[index++] = c2[j];
            l1[index++] = c3[j];
            l1[index++] = c4[j];
            l1[index++] = c5[j];
            l1[index++] = c6[j];
            l1[index++] = c7[j];
            l1[index++] = c8[j];
          }
          index = count = 0;
          for (int j = 0; j < rows; j++) {
            pack = 0;
            for (int e = 7; e >= 0; e--) {
              pack |= l1[index++] << e;
            }
            cell = pack << count;
            pack = cell & 0xff;
            cell >>= 8;
            pack |= cell;
            count = (count + 1) & 0x7;
            *out++ = m_256qam[pack & 0xff];
          }
          break;
        case L1_FEC_MODE_7:
          if (l1select == L1_BASIC) {
            m_256qam = &m_l1b_256qam[0];
          }
          else {
            m_256qam = &m_l1d_256qam[0];
          }
          c1 = &l1t[0];
          c2 = &l1t[rows];
          c3 = &l1t[rows * 2];
          c4 = &l1t[rows * 3];
          c5 = &l1t[rows * 4];
          c6 = &l1t[rows * 5];
          c7 = &l1t[rows * 6];
          c8 = &l1t[rows * 7];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1[index++] = c1[j];
            l1[index++] = c2[j];
            l1[index++] = c3[j];
            l1[index++] = c4[j];
            l1[index++] = c5[j];
            l1[index++] = c6[j];
            l1[index++] = c7[j];
            l1[index++] = c8[j];
          }
          index = count = 0;
          for (int j = 0; j < rows; j++) {
            pack = 0;
            for (int e = 7; e >= 0; e--) {
              pack |= l1[index++] << e;
            }
            cell = pack << count;
            pack = cell & 0xff;
            cell >>= 8;
            pack |= cell;
            count = (count + 1) & 0x7;
            *out++ = m_256qam[pack & 0xff];
          }
          break;
        default:
          c1 = &l1t[0];
          c2 = &l1t[rows];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1[index++] = c1[j];
            l1[index++] = c2[j];
          }
          index = 0;
          for (int j = 0; j < rows; j++) {
            cell = l1[index++] << 1;
            cell |= l1[index++];
            *out++ = m_qpsk[cell];
          }
          break;
      }
    }

    int
    tdmframemapper_cc_impl::add_l1basic(gr_complex *out, int time_offset, int lls_flag)
    {
      int bits, index, offset_bits = 0;
      int npad, padbits, count, nrepeat;
      int block, indexb, nouter, numbits;
      int npunctemp, npunc, nfectemp, nfec;
      int B, mod, rows;
      long long bitslong;
      std::bitset<MAX_BCH_PARITY_BITS> parity_bits;
      unsigned char b, tempbch, msb;
      unsigned char *l1basic = l1_basic;
      unsigned char *l1temp = l1_temp;
      L1_Basic *l1basicinit = &L1_Signalling[0].l1basic_data;
      const unsigned char* d;
      int plen = FRAME_SIZE_SHORT - NBCH_3_15;
      const int q1 = q1_val;
      const int q2 = q2_val;
      const int m1 = m1_val;

      bits = l1basicinit->version;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      l1basic[offset_bits++] = l1basicinit->mimo_scattered_pilot_encoding;
      l1basic[offset_bits++] = lls_flag;
      bits = l1basicinit->time_info_flag;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      l1basic[offset_bits++] = l1basicinit->return_channel_flag;
      bits = l1basicinit->papr_reduction;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      l1basic[offset_bits++] = l1basicinit->frame_length_mode;
      if (l1basicinit->frame_length_mode == FLM_TIME_ALIGNED) {
        bits = l1basicinit->frame_length;
        for (int n = 9; n >= 0; n--) {
          l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
        }
        bits = l1basicinit->excess_samples_per_symbol;
        for (int n = 12; n >= 0; n--) {
          l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
        }
      }
      else {
        bits = time_offset;
        for (int n = 15; n >= 0; n--) {
          l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
        }
        bits = l1basicinit->additional_samples;
        for (int n = 6; n >= 0; n--) {
          l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
        }
      }
      bits = l1basicinit->num_subframes;
      for (int n = 7; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      bits = l1basicinit->preamble_num_symbols;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      bits = l1basicinit->preamble_reduced_carriers;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      bits = l1basicinit->L1_Detail_content_tag;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      bits = l1basicinit->L1_Detail_size_bytes;
      for (int n = 12; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      bits = l1basicinit->L1_Detail_fec_type;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      bits = l1basicinit->L1_Detail_additional_parity_mode;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      bits = l1basicinit->L1_Detail_total_cells;
      for (int n = 18; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      l1basic[offset_bits++] = l1basicinit->first_sub_mimo;
      bits = l1basicinit->first_sub_miso;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      bits = l1basicinit->first_sub_fft_size;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      bits = l1basicinit->first_sub_reduced_carriers;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      bits = l1basicinit->first_sub_guard_interval;
      for (int n = 3; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      bits = l1basicinit->first_sub_num_ofdm_symbols;
      for (int n = 10; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      bits = l1basicinit->first_sub_scattered_pilot_pattern;
      for (int n = 4; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      bits = l1basicinit->first_sub_scattered_pilot_boost;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      l1basic[offset_bits++] = l1basicinit->first_sub_sbs_first;
      l1basic[offset_bits++] = l1basicinit->first_sub_sbs_last;
      bitslong = l1basicinit->reserved;
      for (int n = 47; n >= 0; n--) {
        l1basic[offset_bits++] = bitslong & (1 << n) ? 1 : 0;
      }
      offset_bits += add_crc32_bits(l1basic, offset_bits);

#if 0
      printf("L1Basic\n");
      for (int i = 0; i < offset_bits; i += 8) {
        bits = index = 0;
        for (int j = 7; j >= 0; j--) {
          bits |= l1basic[i + index] << j;
          index++;
        }
        if ((i % 256) == 0) {
          if (i != 0) {
            printf("\n");
          }
        }
        printf("%02X", bits);
      }
      printf("\n");
#endif

      /* scrambling and BCH encoding */
      for (int i = 0; i < offset_bits; i += 8) {
        bits = index = 0;
        for (int j = 7; j >= 0; j--) {
          bits |= l1basic[i + index++] << j;
        }
        bits ^= fm_randomize[i / 8];
        index = 0;
        for (int n = 7; n >= 0; n--) {
          l1basic[i + index++] = bits & (1 << n) ? 1 : 0;
        }
        b = bits;
        msb = 0;
        for (int n = 1; n <= 8; n++) {
          tempbch = parity_bits[num_parity_bits - n];
          msb |= tempbch << (8 - n);
        }
        /* XOR-in next input byte into MSB of crc and get this MSB, that's our new
         * intermediate divident */
        unsigned char pos = (msb ^ b);
        /* Shift out the MSB used for division per lookuptable and XOR with the
         * remainder */
        parity_bits = (parity_bits << 8) ^ crc_table[pos];
      }

      /* zero padding */
      nouter = offset_bits + num_parity_bits;
      npad = (NBCH_3_15 - nouter) / 360;
      memset(&l1temp[0], 0x55, sizeof(unsigned char) * FRAME_SIZE_SHORT);
      for (int i = 0; i < npad; i++) {
        memset(&l1temp[shortening_table[0][i] * 360], 0, sizeof(unsigned char) * 360);
      }
      padbits = (NBCH_3_15 - nouter) - (360 * npad);
      memset(&l1temp[shortening_table[0][npad] * 360], 0, sizeof(unsigned char) * padbits);
      index = count = 0;
      for (int n = 0; n < NBCH_3_15; n++) {
        if (l1temp[index] == 0x55) {
          l1temp[index++] = l1basic[count++];
          if (count == offset_bits) {
            break;
          }
        }
        else {
          index++;
        }
      }
      index = count = 0;
      for (int n = 0; n < NBCH_3_15; n++) {
        if (l1temp[index] == 0x55) {
          l1temp[index++] = (char)parity_bits[num_parity_bits - 1];
          parity_bits <<= 1;
          count++;
          if (count == num_parity_bits) {
            break;
          }
        }
        else {
          index++;
        }
      }

      /* LDPC encoding */
      memcpy(&l1basic[0], &l1temp[0], sizeof(unsigned char) * NBCH_3_15);
      // now do the parity checking
      d = &l1basic[0];
      for (int i_p = 0; i_p < plen; i_p++) {
        unsigned char pbit = 0;
        for (int i_d = 1; i_d < ldpc_lut_a[i_p][0]; i_d++) {
          pbit ^= d[ldpc_lut_a[i_p][i_d]];
        }
        buffer[i_p] = pbit;
      }
      for (int j = 1; j < m1; j++) {
        buffer[j] ^= buffer[j-1];
      }
      for (int t = 0; t < q1; t++) {
        for (int s = 0; s < 360; s++) {
          l1basic[NBCH_3_15 + (360 * t) + s] = buffer[(q1 * s) + t];
        }
      }
      for (int i_p = 0; i_p < plen; i_p++) {
        unsigned char pbit = 0;
        unsigned int count = 0;
        for (int i_d = 1; i_d < ldpc_lut_a_aux[i_p][0]; i_d++) {
          pbit ^= d[ldpc_lut_a_aux[i_p][i_d]];
          count++;
        }
        if (count) {
          buffer[i_p] ^= pbit;
        }
      }
      for (int t = 0; t < q2; t++) {
        for (int s = 0; s < 360; s++) {
          l1basic[NBCH_3_15 + m1 + (360 * t) + s] = buffer[(m1 + q2 * s) + t];
        }
      }

      /* group-wise interleaver */
      memcpy(&l1temp[0], &l1basic[0], sizeof(unsigned char) * NBCH_3_15);
      index = NBCH_3_15;
      for (int j = 0; j < 36; j++) {
        block = group_table[0][j];
        indexb = block * 360;
        for (int k = 0; k < 360; k++) {
          l1temp[index++] = l1basic[indexb++];;
        }
      }

      /* repetition and parity puncturing */
      switch (l1b_mode) {
        case L1_FEC_MODE_1:
          B = 9360;
          mod = 2;
          break;
        case L1_FEC_MODE_2:
          B = 11460;
          mod = 2;
          break;
        case L1_FEC_MODE_3:
          B = 12360;
          mod = 2;
          break;
        case L1_FEC_MODE_4:
          B = 12292;
          mod = 4;
          break;
        case L1_FEC_MODE_5:
          B = 12350;
          mod = 6;
          break;
        case L1_FEC_MODE_6:
          B = 12432;
          mod = 8;
          break;
        case L1_FEC_MODE_7:
          B = 12766;
          mod = 8;
          break;
        default:
          B = 9360;
          mod = 2;
          break;
      }
      if (l1b_mode == L1_FEC_MODE_1) {
        nrepeat = 2 * (0 * nouter) + 3672;
      }
      else {
        nrepeat = 0;
      }
      npunctemp = (0 * (NBCH_3_15 - nouter)) + B;
      nfectemp = nouter + 12960 - npunctemp;
      nfec = ((nfectemp + mod - 1) / mod) * mod;
      npunc = npunctemp - (nfec - nfectemp);
      numbits = nfec + nrepeat;
      memcpy(&l1basic[0], &l1temp[0], sizeof(unsigned char) * NBCH_3_15);
      memcpy(&l1basic[NBCH_3_15], &l1temp[NBCH_3_15], sizeof(unsigned char) * nrepeat);
      memcpy(&l1basic[NBCH_3_15 + nrepeat], &l1temp[NBCH_3_15], sizeof(unsigned char) * (FRAME_SIZE_SHORT - NBCH_3_15 - npunc));

      /* zero removal */
      for (int i = 0; i < npad; i++) {
        memset(&l1basic[shortening_table[0][i] * 360], 0x55, sizeof(unsigned char) * 360);
      }
      memset(&l1basic[shortening_table[0][npad] * 360], 0x55, sizeof(unsigned char) * padbits);
      index = count = 0;
      for (int i = 0; i < NBCH_3_15; i++) {
        if (l1basic[index] != 0x55) {
          l1temp[count++] = l1basic[index++];
        }
        else {
          index++;
        }
      }
      memcpy(&l1temp[count], &l1basic[NBCH_3_15], sizeof(unsigned char) * (numbits - count));

      /* block interleaver, bit demuxing and constellation mapping */
      rows = numbits / mod;
      block_interleaver(&l1basic[0], &l1temp[0], out, l1b_mode, rows, L1_BASIC);
      return (rows);
    }

    struct l1_detail_params_t
    tdmframemapper_cc_impl::add_l1detail(gr_complex *out, int block_start0, int start_row0, int block_start1, int start_row1)
    {
      int bits, index, offset_bits = 0;
      int npad, padbits, count, nrepeat, table;
      int block, indexb, nouter, numbits;
      int npunctemp, npunc, nfectemp, nfec;
      int Anum, Aden, B, mod, rows, temp = 0;
      long long bitslong;
      std::bitset<MAX_BCH_PARITY_BITS> parity_bits;
      unsigned char b, tempbch, msb;
      unsigned char *l1detail = l1_detail;
      unsigned char *l1temp = l1_temp;
      L1_Detail *l1detailinit[NUM_SUBFRAMES][NUM_PLPS];
      L1_Basic *l1basicinit = &L1_Signalling[0].l1basic_data;
      const unsigned char* d;
      unsigned char* p;
      int plen, nbch, groups;
      const int q1 = q1_val;
      const int q2 = q2_val;
      const int m1 = m1_val;
      struct l1_detail_params_t rtn;

      l1detailinit[0][0] = &L1_Signalling[0].l1detail_data[0][0];
      l1detailinit[0][1] = &L1_Signalling[0].l1detail_data[0][1];
      bits = l1detailinit[0][0]->version;
      for (int n = 3; n >= 0; n--) {
        l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      bits = l1detailinit[0][0]->num_rf;
      for (int n = 2; n >= 0; n--) {
        l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      for (int i = 0; i <= l1basicinit->num_subframes; i++) {
        if (0) {
          l1detail[offset_bits++] = l1detailinit[i][0]->mimo;
          bits = l1detailinit[i][0]->miso;
          for (int n = 1; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
          bits = l1detailinit[i][0]->fft_size;
          for (int n = 1; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
          bits = l1detailinit[i][0]->reduced_carriers;
          for (int n = 2; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
          bits = l1detailinit[i][0]->guard_interval;
          for (int n = 3; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
          bits = l1detailinit[i][0]->num_ofdm_symbols;
          for (int n = 10; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
          bits = l1detailinit[i][0]->scattered_pilot_pattern;
          for (int n = 4; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
          bits = l1detailinit[i][0]->scattered_pilot_boost;
          for (int n = 2; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
          l1detail[offset_bits++] = l1detailinit[i][0]->sbs_first;
          l1detail[offset_bits++] = l1detailinit[i][0]->sbs_last;
        }
        if (l1basicinit->num_subframes > 0) {
          l1detail[offset_bits++] = l1detailinit[i][0]->subframe_multiplex;
        }
        l1detail[offset_bits++] = l1detailinit[i][0]->frequency_interleaver;
        if (l1basicinit->first_sub_sbs_first == SBS_ON || l1basicinit->first_sub_sbs_last == SBS_ON) {
          bits = l1detailinit[i][0]->sbs_null_cells;
          for (int n = 12; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
        }
        bits = l1detailinit[i][0]->num_plp;
        for (int n = 5; n >= 0; n--) {
          l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
        }
        for (int j = 0; j <= l1detailinit[i][0]->num_plp; j++) {
          bits = l1detailinit[i][j]->plp_id;
          for (int n = 5; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
          l1detail[offset_bits++] = l1detailinit[i][j]->plp_lls_flag;
          bits = l1detailinit[i][j]->plp_layer;
          for (int n = 1; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
          bits = l1detailinit[i][j]->plp_start;
          for (int n = 23; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
          bits = l1detailinit[i][j]->plp_size;
          for (int n = 23; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
          bits = l1detailinit[i][j]->plp_scrambler_type;
          for (int n = 1; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
          bits = l1detailinit[i][j]->plp_fec_type;
          for (int n = 3; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
          bits = l1detailinit[i][j]->plp_mod;
          for (int n = 3; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
          bits = l1detailinit[i][j]->plp_cod;
          for (int n = 3; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
          bits = l1detailinit[i][j]->plp_TI_mode;
          for (int n = 1; n >= 0; n--) {
            l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
          }
          if (l1detailinit[i][j]->plp_TI_mode == TI_MODE_OFF) {
            bits = j == 0 ? block_start0 : block_start1;
            for (int n = 14; n >= 0; n--) {
              l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
            }
          }
          else if (l1detailinit[i][j]->plp_TI_mode == TI_MODE_CONVOLUTIONAL) {
            bits = j == 0 ? block_start0 : block_start1;
            for (int n = 21; n >= 0; n--) {
              l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
            }
          }
          if (l1detailinit[i][j]->plp_layer == 0) {
            l1detail[offset_bits++] = l1detailinit[i][j]->plp_type;
            if (l1detailinit[i][j]->plp_type == 1) {
              bits = l1detailinit[i][j]->plp_num_subslices;
              for (int n = 13; n >= 0; n--) {
                l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
              }
              bits = l1detailinit[i][j]->plp_subslice_interval;
              for (int n = 23; n >= 0; n--) {
                l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
              }
            }
            if (l1detailinit[i][j]->plp_TI_mode == TI_MODE_CONVOLUTIONAL || l1detailinit[i][j]->plp_TI_mode == TI_MODE_HYBRID) {
              if (l1detailinit[i][j]->plp_mod == MOD_QPSK) {
                l1detail[offset_bits++] = l1detailinit[i][j]->plp_TI_extended_interleaving;
              }
            }
            if (l1detailinit[i][j]->plp_TI_mode == TI_MODE_CONVOLUTIONAL) {
              bits = l1detailinit[i][j]->plp_CTI_depth;
              for (int n = 2; n >= 0; n--) {
                l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
              }
              bits = j == 0 ? start_row0 : start_row1;
              for (int n = 10; n >= 0; n--) {
                l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
              }
            }
            else if (l1detailinit[i][j]->plp_TI_mode == TI_MODE_HYBRID) {
              l1detail[offset_bits++] = l1detailinit[i][j]->plp_HTI_inter_subframe;
              bits = l1detailinit[i][j]->plp_HTI_num_ti_blocks;
              for (int n = 3; n >= 0; n--) {
                l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
              }
              bits = l1detailinit[i][j]->plp_HTI_num_fec_blocks_max;
              for (int n = 11; n >= 0; n--) {
                l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
              }
              bits = l1detailinit[i][j]->plp_HTI_num_fec_blocks;
              for (int n = 11; n >= 0; n--) {
                l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
              }
              l1detail[offset_bits++] = l1detailinit[i][j]->plp_HTI_cell_interleaver;
            }
          }
          else {
            bits = l1detailinit[i][j]->plp_ldm_injection_level;
            for (int n = 4; n >= 0; n--) {
              l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
            }
          }
        }
      }
      bits = l1detailinit[0][1]->bsid;
      for (int n = 15; n >= 0; n--) {
        l1detail[offset_bits++] = bits & (1 << n) ? 1 : 0;
      }
      if ((((l1basicinit->L1_Detail_size_bytes * 8) - 32) - offset_bits) > 0) {
        bitslong = l1detailinit[0][1]->reserved;
        temp = (((l1basicinit->L1_Detail_size_bytes * 8) - 32) - offset_bits) - 1;
        for (int n = temp; n >= 0; n--) {
          l1detail[offset_bits++] = bitslong & (1 << n) ? 1 : 0;
        }
      }
      offset_bits += add_crc32_bits(l1detail, offset_bits);
      rtn.xbits = temp;

#if 0
      printf("L1Detail\n");
      for (int i = 0; i < offset_bits; i += 8) {
        bits = index = 0;
        for (int j = 7; j >= 0; j--) {
          bits |= l1detail[i + index] << j;
          index++;
        }
        if ((i % 256) == 0) {
          if (i != 0) {
            printf("\n");
          }
        }
        printf("%02X", bits);
      }
      printf("\n");
#endif

      /* scrambling and BCH encoding */
      for (int i = 0; i < offset_bits; i += 8) {
        bits = index = 0;
        for (int j = 7; j >= 0; j--) {
          bits |= l1detail[i + index++] << j;
        }
        bits ^= fm_randomize[i / 8];
        index = 0;
        for (int n = 7; n >= 0; n--) {
          l1detail[i + index++] = bits & (1 << n) ? 1 : 0;
        }
        b = bits;
        msb = 0;
        for (int n = 1; n <= 8; n++) {
          tempbch = parity_bits[num_parity_bits - n];
          msb |= tempbch << (8 - n);
        }
        /* XOR-in next input byte into MSB of crc and get this MSB, that's our new
         * intermediate divident */
        unsigned char pos = (msb ^ b);
        /* Shift out the MSB used for division per lookuptable and XOR with the
         * remainder */
        parity_bits = (parity_bits << 8) ^ crc_table[pos];
      }

      /* zero padding */
      switch (l1d_mode) {
        case L1_FEC_MODE_1:
          plen = FRAME_SIZE_SHORT - NBCH_3_15;
          nbch = NBCH_3_15;
          groups = 36;
          table = 1;
          Anum = 7;
          Aden = 2;
          B = 0;
          mod = 2;
          break;
        case L1_FEC_MODE_2:
          plen = FRAME_SIZE_SHORT - NBCH_3_15;
          nbch = NBCH_3_15;
          groups = 36;
          table = 2;
          Anum = 2;
          Aden = 1;
          B = 6036;
          mod = 2;
          break;
        case L1_FEC_MODE_3:
          plen = FRAME_SIZE_SHORT - NBCH_6_15;
          nbch = NBCH_6_15;
          groups = 27;
          table = 3;
          Anum = 11;
          Aden = 16;
          B = 4653;
          mod = 2;
          break;
        case L1_FEC_MODE_4:
          plen = FRAME_SIZE_SHORT - NBCH_6_15;
          nbch = NBCH_6_15;
          groups = 27;
          table = 4;
          Anum = 29;
          Aden = 32;
          B = 3200;
          mod = 4;
          break;
        case L1_FEC_MODE_5:
          plen = FRAME_SIZE_SHORT - NBCH_6_15;
          nbch = NBCH_6_15;
          groups = 27;
          table = 5;
          Anum = 3;
          Aden = 4;
          B = 4284;
          mod = 6;
          break;
        case L1_FEC_MODE_6:
          plen = FRAME_SIZE_SHORT - NBCH_6_15;
          nbch = NBCH_6_15;
          groups = 27;
          table = 6;
          Anum = 11;
          Aden = 16;
          B = 4900;
          mod = 8;
          break;
        case L1_FEC_MODE_7:
          plen = FRAME_SIZE_SHORT - NBCH_6_15;
          nbch = NBCH_6_15;
          groups = 27;
          table = 7;
          Anum = 49;
          Aden = 256;
          B = 8246;
          mod = 8;
          break;
        default:
          plen = FRAME_SIZE_SHORT - NBCH_3_15;
          nbch = NBCH_3_15;
          groups = 36;
          table = 1;
          Anum = 7;
          Aden = 2;
          B = 0;
          mod = 2;
          break;
      }
      nouter = offset_bits + num_parity_bits;
      npad = ((nbch - nouter) / 360);
      memset(&l1temp[0], 0x55, sizeof(unsigned char) * FRAME_SIZE_SHORT);
      for (int i = 0; i < npad; i++) {
        memset(&l1temp[shortening_table[table][i] * 360], 0, sizeof(unsigned char) * 360);
      }
      padbits = (nbch - nouter) - (360 * npad);
      memset(&l1temp[shortening_table[table][npad] * 360], 0, sizeof(unsigned char) * padbits);

      index = count = 0;
      for (int n = 0; n < nbch; n++) {
        if (l1temp[index] == 0x55) {
          l1temp[index++] = l1detail[count++];
          if (count == offset_bits) {
            break;
          }
        }
        else {
          index++;
        }
      }
      index = count = 0;
      for (int n = 0; n < nbch; n++) {
        if (l1temp[index] == 0x55) {
          l1temp[index++] = (char)parity_bits[num_parity_bits - 1];
          parity_bits <<= 1;
          count++;
          if (count == num_parity_bits) {
            break;
          }
        }
        else {
          index++;
        }
      }

      /* LDPC encoding */
      switch (l1d_mode) {
        case L1_FEC_MODE_1:
        case L1_FEC_MODE_2:
          memcpy(&l1detail[0], &l1temp[0], sizeof(unsigned char) * nbch);
          // now do the parity checking
          d = &l1detail[0];
          for (int i_p = 0; i_p < plen; i_p++) {
            unsigned char pbit = 0;
            for (int i_d = 1; i_d < ldpc_lut_a[i_p][0]; i_d++) {
              pbit ^= d[ldpc_lut_a[i_p][i_d]];
            }
            buffer[i_p] = pbit;
          }
          for (int j = 1; j < m1; j++) {
            buffer[j] ^= buffer[j-1];
          }
          for (int t = 0; t < q1; t++) {
            for (int s = 0; s < 360; s++) {
              l1detail[nbch + (360 * t) + s] = buffer[(q1 * s) + t];
            }
          }
          for (int i_p = 0; i_p < plen; i_p++) {
            unsigned char pbit = 0;
            unsigned int count = 0;
            for (int i_d = 1; i_d < ldpc_lut_a_aux[i_p][0]; i_d++) {
              pbit ^= d[ldpc_lut_a_aux[i_p][i_d]];
              count++;
            }
            if (count) {
              buffer[i_p] ^= pbit;
            }
          }
          for (int t = 0; t < q2; t++) {
            for (int s = 0; s < 360; s++) {
              l1detail[nbch + m1 + (360 * t) + s] = buffer[(m1 + q2 * s) + t];
            }
          }
          break;
        case L1_FEC_MODE_3:
        case L1_FEC_MODE_4:
        case L1_FEC_MODE_5:
        case L1_FEC_MODE_6:
        case L1_FEC_MODE_7:
          memcpy(&buffer[0], &l1temp[0], sizeof(unsigned char) * nbch);
          // now do the parity checking
          d = &l1temp[0];
          p = &buffer[nbch];
          for (int i_p = 0; i_p < plen; i_p++) {
            unsigned char pbit = 0;
            for (int i_d = 1; i_d < ldpc_lut_b[i_p][0]; i_d++) {
              pbit ^= d[ldpc_lut_b[i_p][i_d]];
            }
            p[i_p] = pbit;
          }
          for (int j = 1; j < plen; j++) {
            p[j] ^= p[j - 1];
          }
          memcpy(&l1detail[0], &buffer[0], sizeof(unsigned char) * nbch);
          for (int t = 0; t < q_val; t++) {
            for (int s = 0; s < 360; s++) {
              l1detail[nbch + (360 * t) + s] = buffer[(q_val * s) + t + nbch];
            }
          }
          break;
      }

      /* group-wise interleaver */
      memcpy(&l1temp[0], &l1detail[0], sizeof(unsigned char) * nbch);
      index = nbch;
      for (int j = 0; j < groups; j++) {
        block = group_table[table][j];
        indexb = block * 360;
        for (int k = 0; k < 360; k++) {
          l1temp[index++] = l1detail[indexb++];;
        }
      }

      /* repetition and parity puncturing */
      if (l1d_mode == L1_FEC_MODE_1) {
        nrepeat = 2 * ((61 * nouter) / 16) - 508;
      }
      else {
        nrepeat = 0;
      }
      npunctemp = ((Anum * (nbch - nouter)) / Aden) + B;
      nfectemp = nouter + (FRAME_SIZE_SHORT - nbch) - npunctemp;
      nfec = ((nfectemp + mod - 1) / mod) * mod;
      npunc = npunctemp - (nfec - nfectemp);
      numbits = nfec + nrepeat;
      memcpy(&l1detail[0], &l1temp[0], sizeof(unsigned char) * nbch);
      memcpy(&l1detail[nbch], &l1temp[nbch], sizeof(unsigned char) * nrepeat);
      memcpy(&l1detail[nbch + nrepeat], &l1temp[nbch], sizeof(unsigned char) * (FRAME_SIZE_SHORT - nbch - npunc));

      /* zero removal */
      for (int i = 0; i < npad; i++) {
        memset(&l1detail[shortening_table[table][i] * 360], 0x55, sizeof(unsigned char) * 360);
      }
      memset(&l1detail[shortening_table[table][npad] * 360], 0x55, sizeof(unsigned char) * padbits);
      index = count = 0;
      for (int i = 0; i < nbch; i++) {
        if (l1detail[index] != 0x55) {
          l1temp[count++] = l1detail[index++];
        }
        else {
          index++;
        }
      }
      memcpy(&l1temp[count], &l1detail[nbch], sizeof(unsigned char) * (numbits - count));

      /* block interleaver, bit demuxing and constellation mapping */
      rows = numbits / mod;
      block_interleaver(&l1detail[0], &l1temp[0], out, l1d_mode, rows, L1_DETAIL);
      rtn.cells = rows;
      return (rtn);
    }

    void
    tdmframemapper_cc_impl::init_address(int plp)
    {
      int max_states, xor_size, pn_mask, result;
      int q, k;
      int lfsr;
      int logic11[2] = {0, 3};
      int logic12[2] = {0, 2};
      int logic13[4] = {0, 1, 4, 6};
      int logic14[6] = {0, 1, 4, 5, 9, 11};
      int logic15[4] = {0, 1, 2, 12};
      int* logic;
      int pn_degree;
      int Nd, index;
      long long Pr;
      int Ri, Ti, Ci;
      std::vector<int>& HtimeNfec = this->HtimeNfec[plp];

      for (int x = 0; x < ti_blocks[plp]; x++) {
        if (x < (ti_blocks[plp] - (ti_fecblocks[plp] % ti_blocks[plp]))) {
          HtimeNfec[x] = ti_fecblocks[plp] / ti_blocks[plp];
        }
        else {
          HtimeNfec[x] = (ti_fecblocks[plp] / ti_blocks[plp]) + 1;
        }
      }

      Nd = 0;
      index = fec_cells[plp];
      while (index) {
        index >>= 1;
        Nd++;
      }

      switch (Nd) {
        case 11:
          pn_degree = 10;
          pn_mask = 0x3ff;
          max_states = 2048;
          logic = &logic11[0];
          xor_size = 2;
          break;
        case 12:
          pn_degree = 11;
          pn_mask = 0x7ff;
          max_states = 4096;
          logic = &logic12[0];
          xor_size = 2;
          break;
        case 13:
          pn_degree = 12;
          pn_mask = 0xfff;
          max_states = 8192;
          logic = &logic13[0];
          xor_size = 4;
          break;
        case 14:
          pn_degree = 13;
          pn_mask = 0x1fff;
          max_states = 16384;
          logic = &logic14[0];
          xor_size = 6;
          break;
        case 15:
          pn_degree = 14;
          pn_mask = 0x3fff;
          max_states = 32768;
          logic = &logic15[0];
          xor_size = 4;
          break;
        default:
          pn_degree = 10;
          pn_mask = 0x3ff;
          max_states = 2048;
          logic = &logic11[0];
          xor_size = 2;
          break;
      }

      for (int i = 0; i < ti_blocks[plp]; i++) {
        std::vector<int>& Htime = this->HtimePr[plp][i];
        q = 0;
        k = 0;
        for (int r = 0; r < HtimeNfec[i]; r++) {
          Pr = fec_cells[plp];
          while (Pr >= fec_cells[plp]) {
            Pr = 0;
            for (int j = 0; j < Nd; j++) {
              Pr |= (k & (1 << j)) << ((Nd + 16) - 1 - j * 2);
            }
            Pr >>= 16;
            k = k + 1;
          }
          Htime[q++] = Pr;
        }
      }

      for (int x = 0; x < ti_blocks[plp]; x++) {
        for (int i = 0; i < HtimeNfec[x]; i++) {
          std::vector<int>& Htime = this->HtimeLr[plp][x][i];
          std::vector<int>& HtimePr = this->HtimePr[plp][x];
          q = 0;

          for (int j = 0; j < max_states; j++) {
            if (j == 0 || j == 1) {
              lfsr = 0;
            }
            else if (j == 2) {
              lfsr = 1;
            }
            else {
              result = 0;
              for (int k = 0; k < xor_size; k++) {
                result ^= (lfsr >> logic[k]) & 1;
              }
              lfsr &= pn_mask;
              lfsr >>= 1;
              lfsr |= result << (pn_degree - 1);
            }
            lfsr |= (j % 2) << pn_degree;
            if (lfsr < fec_cells[plp]) {
              Htime[q++] = (lfsr + HtimePr[i]) % fec_cells[plp];
            }
          }
        }
      }
      for (int x = 0; x < ti_blocks[plp]; x++) {
        std::vector<int>& Htime = this->HtimeTBI[plp][x];
        q = 0;
        for (int n = 0; n < fec_cells[plp] * Nfec_ti_max[plp]; n++) {
          Ri = n % fec_cells[plp];
          Ti = Ri % Nfec_ti_max[plp];
          Ci = (Ti + (n / fec_cells[plp])) % Nfec_ti_max[plp];
          if ((fec_cells[plp] * Ci) + Ri >= (Nfec_ti_max[plp] - HtimeNfec[x]) * fec_cells[plp]) {
            Htime[q++] = (fec_cells[plp] * Ci) + Ri;
          }
        }
      }
    }

    const gr_complex zero = gr_complex(0.0, 0.0);

    int
    tdmframemapper_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto inx = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      int indexin[NUM_PLPS] = {0, 0};
      int indexout = 0;
      int indexin_timeint;
      int preamblesyms = preamble_syms;
      int rows, datacells;
      int time_offset;
      int fec_block_start[NUM_PLPS];
      int left_nulls;
      int right_nulls;
      int l1detailcells, l1totalcells;
      int commutator_start[NUM_PLPS] = {0, 0};
      int virtual_offset;
      std::vector<int> H;
      gr_complex *outtimehti;
      gr_complex *outtimeint;

      std::vector<tag_t> tags;
      uint64_t nread;
      uint64_t nread_end;
      uint64_t tagvalue;
      int lls_flag = FALSE;
      struct l1_detail_params_t rtn;

      if (plp_lls_flag[0] == TRUE) {
        nread = this->nitems_read(0); //number of items read on port 0
        nread_end = nread + (plp_size[0] * (noutput_items / total_cells));
        // Read all tags on the input buffer
        this->get_tags_in_range(tags, 0, nread, nread_end, pmt::string_to_symbol("lls"));
        if ((int)tags.size()) {
          tagvalue = pmt::to_uint64(tags[0].value);
          if (tagvalue <= nread_end) {
            lls_flag = TRUE; /* LLS is entirely in the frame */
          }
        }
      }
      if (lls_flag == FALSE && plp_lls_flag[1] == TRUE) {
        nread = this->nitems_read(1); //number of items read on port 1
        nread_end = nread + (plp_size[1] * (noutput_items / total_cells));
        // Read all tags on the input buffer
        this->get_tags_in_range(tags, 1, nread, nread_end, pmt::string_to_symbol("lls"));
        if ((int)tags.size()) {
          tagvalue = pmt::to_uint64(tags[0].value);
          if (tagvalue <= nread_end) {
            lls_flag = TRUE; /* LLS is entirely in the frame */
          }
        }
      }

      if (sbsnullcells & 0x1) {
        left_nulls = (sbsnullcells / 2);
        right_nulls = (sbsnullcells / 2) + 1;
      }
      else {
        left_nulls = sbsnullcells / 2;
        right_nulls = left_nulls;
      }

      for (int i = 0; i < noutput_items; i += noutput_items) {
        outtimeint = &time_interleaver[0];
        for (int plp = 0; plp < NUM_PLPS; plp++) {
          inx = static_cast<const input_type*>(input_items[plp]);
          if (ti_mode[plp] == TI_MODE_HYBRID) {
            std::vector<int>& HtimeNfec = this->HtimeNfec[plp];
            outtimehti = &hybrid_time_interleaver[plp][0];
            for (int x = 0; x < ti_blocks[plp]; x++) {
              for (int j = 0; j < HtimeNfec[x]; j++) {
                H = HtimeLr[plp][x][j];
                for (int n = 0; n < fec_cells[plp]; n++) {
                  *outtimehti++ = inx[H[n]];
                }
                inx += fec_cells[plp];
              }
            }
            indexin[plp] += plp_size[plp];
            in = &hybrid_time_interleaver[plp][0];
            for (int x = 0; x < ti_blocks[plp]; x++) {
              virtual_offset = (Nfec_ti_max[plp] - HtimeNfec[x]) * fec_cells[plp];
              H = HtimeTBI[plp][x];
              for (int n = 0; n < fec_cells[plp] * HtimeNfec[x]; n++) {
                *outtimeint++ = in[H[n] - virtual_offset];
              }
              in += fec_cells[plp] * HtimeNfec[x];
            }
          }
          else if (ti_mode[plp] == TI_MODE_CONVOLUTIONAL) {
            commutator_start[plp] = commutator[plp];
            for (int n = 0; n < plp_size[plp]; n++) {
              delay_line[plp][commutator[plp]].push_front(inx[indexin[plp]++]);
              outtimeint[n] = delay_line[plp][commutator[plp]].back();
              delay_line[plp][commutator[plp]].pop_back();
              commutator[plp] = (commutator[plp] + 1) % ti_depth[plp];
            }
            outtimeint += plp_size[plp];
          }
          else {
            memcpy(&outtimeint[0], &inx[indexin[plp]], sizeof(gr_complex) * plp_size[plp]);
            indexin[plp] += plp_size[plp];
            outtimeint += plp_size[plp];
          }
        }
        in = &time_interleaver[0];
        indexin_timeint = 0;

        time_offset = samples % SAMPLES_PER_MILLISECOND_6MHZ;
        indexout += add_l1basic(&out[0], time_offset, lls_flag);

        for (int k = 0; k < 2; k++) {
          fec_block_start[k] = cells[k] % fec_cells[k];
          if (fec_block_start[k]) {
            fec_block_start[k] = fec_cells[k] - (cells[k] % fec_cells[k]);
          }

          if (ti_mode[k] == TI_MODE_CONVOLUTIONAL) {
            fec_block_start[k] = fec_block_start[k] + ti_depth[k] * ((commutator_start[k] + fec_block_start[k]) % ti_depth[k]);
          }
        }

        rtn = add_l1detail(&l1_dummy[0], fec_block_start[0], commutator_start[0], fec_block_start[1], commutator_start[1]);
        l1detailcells = rtn.cells;
        rows = l1detailcells / preamblesyms;
        for (int i = 0; i < preamblesyms; i++) {
          for (int j = 0; j < rows; j++) {
            out[indexout++] = l1_dummy[j * preamblesyms + i];
          }
        }
        for (int i = rows * preamblesyms; i < l1detailcells; i++) {
          out[indexout++] = l1_dummy[i];
        }

        l1totalcells = indexout;
        datacells = 0;
        for (int n = 0; n < preamblesyms; n++) {
          datacells += frame_symbols[n];
        }
        datacells -= l1totalcells;
        memcpy(&out[indexout], &in[indexin_timeint], sizeof(gr_complex) * datacells);
        indexin_timeint += datacells;
        indexout += datacells;
        if (first_sbs == SBS_ON) {
          for (int n = 0; n < left_nulls; n++) {
            out[indexout++] = zero;
          }
          memcpy(&out[indexout], &in[indexin_timeint], sizeof(gr_complex) * (frame_symbols[preamblesyms] - sbsnullcells));
          indexout += frame_symbols[preamblesyms] - sbsnullcells;
          for (int n = 0; n < right_nulls; n++) {
            out[indexout++] = zero;
          }
          indexin_timeint += frame_symbols[preamblesyms] - sbsnullcells;
          preamblesyms++;
        }
        for (int n = preamblesyms; n < symbols - 1; n++) {
          memcpy(&out[indexout], &in[indexin_timeint], sizeof(gr_complex) * frame_symbols[n]);
          indexin_timeint += frame_symbols[n];
          indexout += frame_symbols[n];
        }
        for (int n = 0; n < left_nulls; n++) {
          out[indexout++] = zero;
        }
        memcpy(&out[indexout], &in[indexin_timeint], sizeof(gr_complex) * (frame_symbols[symbols - 1] - sbsnullcells));
        indexout += frame_symbols[symbols - 1] - sbsnullcells;
        indexin_timeint += frame_symbols[symbols - 1] - sbsnullcells;
        for (int n = 0; n < right_nulls; n++) {
          out[indexout++] = zero;
        }

        samples += frame_samples;
        cells[0] += plp_size[0];
        cells[1] += plp_size[1];
      }

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume (0, indexin[0]);
      consume (1, indexin[1]);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    const int tdmframemapper_cc_impl::shortening_table[8][18] = {
      {4, 1, 5, 2, 8, 6, 0, 7, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {7, 8, 5, 4, 1, 2, 6, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {6, 1, 7, 8, 0, 2, 4, 3, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 12, 15, 13, 2, 5, 7, 9, 8, 6, 16, 10, 14, 1, 17, 11, 4, 3},
      {0, 15, 5, 16, 17, 1, 6, 13, 11, 4, 7, 12, 8, 14, 2, 3, 9, 10},
      {2, 4, 5, 17, 9, 7, 1, 6, 15, 8, 10, 14, 16, 0, 11, 13, 12, 3},
      {0, 15, 5, 16, 17, 1, 6, 13, 11, 4, 7, 12, 8, 14, 2, 3, 9, 10},
      {15, 7, 8, 11, 5, 10, 16, 4, 12, 3, 0, 6, 9, 1, 14, 17, 2, 13}
    };

    const uint16_t tdmframemapper_cc_impl::ldpc_tab_3_15S[12][12] = {
      {11, 8, 372, 841, 4522, 5253, 7430, 8542, 9822, 10550, 11896, 11988},
      {11, 80, 255, 667, 1511, 3549, 5239, 5422, 5497, 7157, 7854, 11267},
      {11, 257, 406, 792, 2916, 3072, 3214, 3638, 4090, 8175, 8892, 9003},
      {11, 80, 150, 346, 1883, 6838, 7818, 9482, 10366, 10514, 11468, 12341},
      {11, 32, 100, 978, 3493, 6751, 7787, 8496, 10170, 10318, 10451, 12561},
      {11, 504, 803, 856, 2048, 6775, 7631, 8110, 8221, 8371, 9443, 10990},
      {11, 152, 283, 696, 1164, 4514, 4649, 7260, 7370, 11925, 11986, 12092},
      {11, 127, 1034, 1044, 1842, 3184, 3397, 5931, 7577, 11898, 12339, 12689},
      {11, 107, 513, 979, 3934, 4374, 4658, 7286, 7809, 8830, 10804, 10893},
      {10, 2045, 2499, 7197, 8887, 9420, 9922, 10132, 10540, 10816, 11876, 0},
      {10, 2932, 6241, 7136, 7835, 8541, 9403, 9817, 11679, 12377, 12810, 0},
      {10, 2211, 2288, 3937, 4310, 5952, 6597, 9692, 10445, 11064, 11272, 0}
    };

    const uint16_t tdmframemapper_cc_impl::ldpc_tab_6_15S[18][31] = {
      {30, 27, 430, 519, 828, 1897, 1943, 2513, 2600, 2640, 3310, 3415, 4266, 5044, 5100, 5328, 5483, 5928, 6204, 6392, 6416, 6602, 7019, 7415, 7623, 8112, 8485, 8724, 8994, 9445, 9667},
      {30, 27, 174, 188, 631, 1172, 1427, 1779, 2217, 2270, 2601, 2813, 3196, 3582, 3895, 3908, 3948, 4463, 4955, 5120, 5809, 5988, 6478, 6604, 7096, 7673, 7735, 7795, 8925, 9613, 9670},
      {30, 27, 370, 617, 852, 910, 1030, 1326, 1521, 1606, 2118, 2248, 2909, 3214, 3413, 3623, 3742, 3752, 4317, 4694, 5300, 5687, 6039, 6100, 6232, 6491, 6621, 6860, 7304, 8542, 8634},
      {4, 990, 1753, 7635, 8540, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {4, 933, 1415, 5666, 8745, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {4, 27, 6567, 8707, 9216, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {4, 2341, 8692, 9580, 9615, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {4, 260, 1092, 5839, 6080, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {4, 352, 3750, 4847, 7726, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {4, 4610, 6580, 9506, 9597, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {4, 2512, 2974, 4814, 9348, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {4, 1461, 4021, 5060, 7009, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {4, 1796, 2883, 5553, 8306, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {3, 1249, 5422, 7057, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {3, 3965, 6968, 9422, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {3, 1498, 2931, 5092, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {3, 27, 1090, 6215, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {3, 26, 4232, 6354, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };

    const int tdmframemapper_cc_impl::group_table[8][36] = {
      {20, 23, 25, 32, 38, 41, 18, 9, 10, 11, 31, 24,
       14, 15, 26, 40, 33, 19, 28, 34, 16, 39, 27, 30,
       21, 44, 43, 35, 42, 36, 12, 13, 29, 22, 37, 17},
      {16, 22, 27, 30, 37, 44, 20, 23, 25, 32, 38, 41,
       9, 10, 17, 18, 21, 33, 35, 14, 28, 12, 15, 19,
       11, 24, 29, 34, 36, 13, 40, 43, 31, 26, 39, 42},
      {9, 31, 23, 10, 11, 25, 43, 29, 36, 16, 27, 34,
       26, 18, 37, 15, 13, 17, 35, 21, 20, 24, 44, 12,
       22, 40, 19, 32, 38, 41, 30, 33, 14, 28, 39, 42},
      {19, 37, 30, 42, 23, 44, 27, 40, 21, 34, 25, 32, 29, 24,
       26, 35, 39, 20, 18, 43, 31, 36, 38, 22, 33, 28, 41},
      {20, 35, 42, 39, 26, 23, 30, 18, 28, 37, 32, 27, 44, 43,
       41, 40, 38, 36, 34, 33, 31, 29, 25, 24, 22, 21, 19},
      {19, 37, 33, 26, 40, 43, 22, 29, 24, 35, 44, 31, 27, 20,
       21, 39, 25, 42, 34, 18, 32, 38, 23, 30, 28, 36, 41},
      {20, 35, 42, 39, 26, 23, 30, 18, 28, 37, 32, 27, 44, 43,
       41, 40, 38, 36, 34, 33, 31, 29, 25, 24, 22, 21, 19},
      {44, 23, 29, 33, 24, 28, 21, 27, 42, 18, 22, 31, 32, 37,
       43, 30, 25, 35, 20, 34, 39, 36, 19, 41, 40, 26, 38}
    };

    const gr_complex tdmframemapper_cc_impl::mod_table_16QAM[12][4] = {
      {gr_complex(0.7062, 0.7075), gr_complex(0.7075, 0.7062), gr_complex(0.7072, 0.7077), gr_complex(0.7077, 0.7072)},
      {gr_complex(0.3620, 0.5534), gr_complex(0.5534, 0.3620), gr_complex(0.5940, 1.1000), gr_complex(1.1000, 0.5940)},
      {gr_complex(0.3412, 0.5241), gr_complex(0.5241, 0.3412), gr_complex(0.5797, 1.1282), gr_complex(1.1282, 0.5797)},
      {gr_complex(0.3192, 0.5011), gr_complex(0.5011, 0.3192), gr_complex(0.5575, 1.1559), gr_complex(1.1559, 0.5575)},
      {gr_complex(0.5115, 1.2092), gr_complex(1.2092, 0.5115), gr_complex(0.2663, 0.4530), gr_complex(0.4530, 0.2663)},
      {gr_complex(0.2592, 0.4888), gr_complex(0.4888, 0.2592), gr_complex(0.5072, 1.1980), gr_complex(1.1980, 0.5072)},
      {gr_complex(0.2535, 0.4923), gr_complex(0.4923, 0.2535), gr_complex(0.4927, 1.2044), gr_complex(1.2044, 0.4927)},
      {gr_complex(0.2386, 0.5296), gr_complex(0.5296, 0.2386), gr_complex(0.4882, 1.1934), gr_complex(1.1934, 0.4882)},
      {gr_complex(0.4487, 1.1657), gr_complex(1.2080, 0.5377), gr_complex(0.2213, 0.4416), gr_complex(0.6186, 0.2544)},
      {gr_complex(0.9342, 0.9847), gr_complex(0.9866, 0.2903), gr_complex(0.2716, 0.9325), gr_complex(0.2901, 0.2695)},
      {gr_complex(0.9555, 0.9555), gr_complex(0.9555, 0.2949), gr_complex(0.2949, 0.9555), gr_complex(0.2949, 0.2949)},
      {gr_complex(0.9517, 0.9511), gr_complex(0.9524, 0.3061), gr_complex(0.3067, 0.9524), gr_complex(0.3061, 0.3067)}
    };

    const gr_complex tdmframemapper_cc_impl::mod_table_64QAM[12][16] = {
      {gr_complex(0.6474, 0.9831), gr_complex(0.6438, 0.9829), gr_complex(0.6471, 0.9767), gr_complex(0.6444, 0.9762),
       gr_complex(0.9839, 0.6475), gr_complex(0.9778, 0.6474), gr_complex(0.9835, 0.6434), gr_complex(0.9777, 0.6433),
       gr_complex(0.4659, 0.6393), gr_complex(0.4643, 0.6386), gr_complex(0.4661, 0.6353), gr_complex(0.4639, 0.6350),
       gr_complex(0.6378, 0.4671), gr_complex(0.6352, 0.4673), gr_complex(0.6385, 0.4656), gr_complex(0.6353, 0.4653)},
      {gr_complex(0.5472, 1.1591), gr_complex(0.5473, 1.1573), gr_complex(0.5467, 1.1599), gr_complex(0.5479, 1.1585),
       gr_complex(1.1578, 0.5478), gr_complex(1.1576, 0.5475), gr_complex(1.1591, 0.5475), gr_complex(1.1591, 0.5475),
       gr_complex(0.3163, 0.5072), gr_complex(0.3163, 0.5072), gr_complex(0.3163, 0.5072), gr_complex(0.3163, 0.5072),
       gr_complex(0.5087, 0.3163), gr_complex(0.5087, 0.3163), gr_complex(0.5087, 0.3163), gr_complex(0.5087, 0.3163)},
      {gr_complex(0.5008, 1.2136), gr_complex(0.4994, 1.2194), gr_complex(0.5313, 1.1715), gr_complex(0.5299, 1.1788),
       gr_complex(1.2107, 0.5037), gr_complex(1.2209, 0.5008), gr_complex(1.1715, 0.5299), gr_complex(1.1802, 0.5270),
       gr_complex(0.2744, 0.4762), gr_complex(0.2729, 0.4762), gr_complex(0.2773, 0.4791), gr_complex(0.2773, 0.4791),
       gr_complex(0.4762, 0.2729), gr_complex(0.4762, 0.2729), gr_complex(0.4791, 0.2773), gr_complex(0.4791, 0.2758)},
      {gr_complex(1.4327, 0.3305), gr_complex(1.0909, 0.2971), gr_complex(1.2484, 0.7803), gr_complex(0.9762, 0.5715),
       gr_complex(0.3309, 1.4326), gr_complex(0.2979, 1.0923), gr_complex(0.7829, 1.2477), gr_complex(0.5739, 0.9763),
       gr_complex(0.3901, 0.2112), gr_complex(0.5317, 0.2475), gr_complex(0.3945, 0.2289), gr_complex(0.5236, 0.2894),
       gr_complex(0.2108, 0.3911), gr_complex(0.2475, 0.5327), gr_complex(0.2287, 0.3955), gr_complex(0.2898, 0.5246)},
      {gr_complex(1.4521, 0.3005), gr_complex(1.2657, 0.8178), gr_complex(1.0666, 0.2744), gr_complex(0.9500, 0.5641),
       gr_complex(0.3011, 1.4529), gr_complex(0.8202, 1.2651), gr_complex(0.2750, 1.0676), gr_complex(0.5656, 0.9499),
       gr_complex(0.3553, 0.1948), gr_complex(0.3569, 0.2094), gr_complex(0.5596, 0.2431), gr_complex(0.5410, 0.3002),
       gr_complex(0.1946, 0.3566), gr_complex(0.2094, 0.3579), gr_complex(0.2430, 0.5607), gr_complex(0.3004, 0.5417)},
      {gr_complex(0.1567, 0.3112), gr_complex(0.1709, 0.3037), gr_complex(0.2093, 0.6562), gr_complex(0.3315, 0.6038),
       gr_complex(0.3112, 0.1567), gr_complex(0.3037, 0.1709), gr_complex(0.6562, 0.2093), gr_complex(0.6038, 0.3315),
       gr_complex(0.2959, 1.4877), gr_complex(0.8427, 1.2612), gr_complex(0.2389, 1.0228), gr_complex(0.5559, 0.8912),
       gr_complex(1.4877, 0.2959), gr_complex(1.2612, 0.8427), gr_complex(1.0228, 0.2389), gr_complex(0.8912, 0.5559)},
      {gr_complex(1.4827, 0.2920), gr_complex(1.2563, 0.8411), gr_complex(1.0211, 0.2174), gr_complex(0.8798, 0.5702),
       gr_complex(0.2920, 1.4827), gr_complex(0.8410, 1.2563), gr_complex(0.2174, 1.0211), gr_complex(0.5702, 0.8798),
       gr_complex(0.3040, 0.1475), gr_complex(0.3028, 0.1691), gr_complex(0.6855, 0.1871), gr_complex(0.6126, 0.3563),
       gr_complex(0.1475, 0.3040), gr_complex(0.1691, 0.3028), gr_complex(0.1871, 0.6855), gr_complex(0.3563, 0.6126)},
      {gr_complex(0.1305, 0.3311), gr_complex(0.1633, 0.3162), gr_complex(0.1622, 0.7113), gr_complex(0.3905, 0.6163),
       gr_complex(0.3311, 0.1305), gr_complex(0.3162, 0.1633), gr_complex(0.7113, 0.1622), gr_complex(0.6163, 0.3905),
       gr_complex(0.2909, 1.4626), gr_complex(0.8285, 1.2399), gr_complex(0.2062, 1.0367), gr_complex(0.5872, 0.8789),
       gr_complex(1.4626, 0.2909), gr_complex(1.2399, 0.8285), gr_complex(1.0367, 0.2062), gr_complex(0.8789, 0.5872)},
      {gr_complex(0.1177, 0.1729), gr_complex(0.1601, 0.3212), gr_complex(0.1352, 0.7279), gr_complex(0.3246, 0.6148),
       gr_complex(0.4192, 0.1179), gr_complex(0.4033, 0.2421), gr_complex(0.7524, 0.1581), gr_complex(0.5996, 0.4330),
       gr_complex(0.2902, 1.4611), gr_complex(0.8180, 1.2291), gr_complex(0.2036, 1.0575), gr_complex(0.5641, 0.8965),
       gr_complex(1.4453, 0.2907), gr_complex(1.2157, 0.8186), gr_complex(1.0447, 0.2242), gr_complex(0.8497, 0.6176)},
      {gr_complex(1.4443, 0.2683), gr_complex(0.7471, 1.2243), gr_complex(1.1749, 0.7734), gr_complex(0.7138, 0.8201),
       gr_complex(0.1638, 1.0769), gr_complex(0.2927, 1.4217), gr_complex(0.1462, 0.7457), gr_complex(0.4134, 0.7408),
       gr_complex(1.0203, 0.1517), gr_complex(0.6653, 0.1357), gr_complex(0.9639, 0.4465), gr_complex(0.6746, 0.4339),
       gr_complex(0.1271, 0.1428), gr_complex(0.3782, 0.1406), gr_complex(0.1311, 0.4288), gr_complex(0.3919, 0.4276)},
      {gr_complex(1.4480, 0.2403), gr_complex(0.6406, 1.1995), gr_complex(1.0952, 0.9115), gr_complex(0.6868, 0.8108),
       gr_complex(1.0500, 0.1642), gr_complex(0.7170, 0.1473), gr_complex(1.0519, 0.5188), gr_complex(0.7146, 0.4532),
       gr_complex(0.1677, 1.0405), gr_complex(0.2402, 1.4087), gr_complex(0.1369, 0.7073), gr_complex(0.4044, 0.7057),
       gr_complex(0.1374, 0.1295), gr_complex(0.4185, 0.1357), gr_complex(0.1325, 0.3998), gr_complex(0.4122, 0.4120)},
      {gr_complex(1.4319, 0.2300), gr_complex(1.0762, 0.9250), gr_complex(0.6290, 1.1820), gr_complex(0.6851, 0.8072),
       gr_complex(1.0443, 0.1688), gr_complex(1.0635, 0.5305), gr_complex(0.7220, 0.1540), gr_complex(0.7151, 0.4711),
       gr_complex(0.2099, 1.4205), gr_complex(0.1190, 0.6677), gr_complex(0.2031, 1.0551), gr_complex(0.3722, 0.7548),
       gr_complex(0.1438, 0.1287), gr_complex(0.1432, 0.3903), gr_complex(0.4298, 0.1384), gr_complex(0.4215, 0.4279)}
    };

    const gr_complex tdmframemapper_cc_impl::mod_table_256QAM[12][64] = {
      {gr_complex(0.5553, 1.1262), gr_complex(0.5673, 1.1336), gr_complex(0.5593, 1.1204), gr_complex(0.5636, 1.1321),
       gr_complex(0.5525, 1.1249), gr_complex(0.5637, 1.1320), gr_complex(0.5598, 1.1181), gr_complex(0.5659, 1.1274),
       gr_complex(0.5579, 1.1381), gr_complex(0.5617, 1.1471), gr_complex(0.5593, 1.1346), gr_complex(0.5672, 1.1430),
       gr_complex(0.5533, 1.1355), gr_complex(0.5632, 1.1421), gr_complex(0.5567, 1.1325), gr_complex(0.5641, 1.1363),
       gr_complex(1.1309, 0.5597), gr_complex(1.1405, 0.5660), gr_complex(1.1348, 0.5588), gr_complex(1.1491, 0.5638),
       gr_complex(1.1245, 0.5615), gr_complex(1.1333, 0.5627), gr_complex(1.1284, 0.5578), gr_complex(1.1436, 0.5636),
       gr_complex(1.1196, 0.5620), gr_complex(1.1347, 0.5665), gr_complex(1.1379, 0.5611), gr_complex(1.1440, 0.5638),
       gr_complex(1.1221, 0.5594), gr_complex(1.1318, 0.5686), gr_complex(1.1302, 0.5619), gr_complex(1.1386, 0.5662),
       gr_complex(0.3394, 0.5381), gr_complex(0.3397, 0.5360), gr_complex(0.3387, 0.5324), gr_complex(0.3400, 0.5335),
       gr_complex(0.3374, 0.5306), gr_complex(0.3405, 0.5343), gr_complex(0.3379, 0.5324), gr_complex(0.3400, 0.5317),
       gr_complex(0.3397, 0.5370), gr_complex(0.3400, 0.5383), gr_complex(0.3381, 0.5347), gr_complex(0.3382, 0.5347),
       gr_complex(0.3379, 0.5342), gr_complex(0.3389, 0.5332), gr_complex(0.3402, 0.5347), gr_complex(0.3384, 0.5340),
       gr_complex(0.5350, 0.3394), gr_complex(0.5363, 0.3397), gr_complex(0.5342, 0.3389), gr_complex(0.5384, 0.3380),
       gr_complex(0.5329, 0.3363), gr_complex(0.5330, 0.3387), gr_complex(0.5311, 0.3389), gr_complex(0.5332, 0.3380),
       gr_complex(0.5313, 0.3397), gr_complex(0.5324, 0.3400), gr_complex(0.5339, 0.3402), gr_complex(0.5360, 0.3405),
       gr_complex(0.5285, 0.3397), gr_complex(0.5317, 0.3379), gr_complex(0.5319, 0.3381), gr_complex(0.5327, 0.3395)},
      {gr_complex(0.5229, 1.1810), gr_complex(0.5384, 1.1625), gr_complex(0.5148, 1.1943), gr_complex(0.5288, 1.1751),
       gr_complex(0.4985, 1.2202), gr_complex(0.5111, 1.1973), gr_complex(0.4889, 1.2357), gr_complex(0.5045, 1.2113),
       gr_complex(0.5222, 1.1817), gr_complex(0.5370, 1.1640), gr_complex(0.5133, 1.1950), gr_complex(0.5303, 1.1751),
       gr_complex(0.4971, 1.2216), gr_complex(0.5126, 1.1995), gr_complex(0.4882, 1.2371), gr_complex(0.5045, 1.2128),
       gr_complex(1.1795, 0.5251), gr_complex(1.1625, 0.5384), gr_complex(1.1914, 0.5133), gr_complex(1.1744, 0.5296),
       gr_complex(1.2209, 0.4993), gr_complex(1.2002, 0.5148), gr_complex(1.2342, 0.4882), gr_complex(1.2142, 0.5052),
       gr_complex(1.1803, 0.5229), gr_complex(1.1640, 0.5399), gr_complex(1.1921, 0.5133), gr_complex(1.1758, 0.5303),
       gr_complex(1.2209, 0.4971), gr_complex(1.2024, 0.5148), gr_complex(1.2349, 0.4889), gr_complex(1.2150, 0.5045),
       gr_complex(0.2740, 0.4771), gr_complex(0.2762, 0.4801), gr_complex(0.2733, 0.4757), gr_complex(0.2748, 0.4779),
       gr_complex(0.2703, 0.4742), gr_complex(0.2725, 0.4764), gr_complex(0.2696, 0.4727), gr_complex(0.2718, 0.4749),
       gr_complex(0.2740, 0.4779), gr_complex(0.2755, 0.4793), gr_complex(0.2725, 0.4757), gr_complex(0.2748, 0.4779),
       gr_complex(0.2711, 0.4734), gr_complex(0.2725, 0.4764), gr_complex(0.2696, 0.4720), gr_complex(0.2711, 0.4742),
       gr_complex(0.4771, 0.2740), gr_complex(0.4786, 0.2762), gr_complex(0.4764, 0.2725), gr_complex(0.4771, 0.2748),
       gr_complex(0.4734, 0.2703), gr_complex(0.4757, 0.2725), gr_complex(0.4734, 0.2696), gr_complex(0.4742, 0.2711),
       gr_complex(0.4771, 0.2740), gr_complex(0.4779, 0.2762), gr_complex(0.4764, 0.2725), gr_complex(0.4771, 0.2748),
       gr_complex(0.4742, 0.2703), gr_complex(0.4749, 0.2725), gr_complex(0.4734, 0.2696), gr_complex(0.4749, 0.2711)},
      {gr_complex(0.2975, 1.0564), gr_complex(0.5862, 0.9617), gr_complex(0.2909, 1.0696), gr_complex(0.5796, 0.9689),
       gr_complex(0.2953, 1.3357), gr_complex(0.7488, 1.2365), gr_complex(0.3004, 1.5114), gr_complex(0.8151, 1.3816),
       gr_complex(0.3004, 1.0535), gr_complex(0.5847, 0.9631), gr_complex(0.2931, 1.0659), gr_complex(0.5825, 0.9668),
       gr_complex(0.2953, 1.3189), gr_complex(0.7466, 1.2168), gr_complex(0.2960, 1.4654), gr_complex(0.8297, 1.3539),
       gr_complex(1.0637, 0.2960), gr_complex(0.9617, 0.5811), gr_complex(1.0732, 0.2931), gr_complex(0.9682, 0.5818),
       gr_complex(1.3619, 0.2997), gr_complex(1.2249, 0.7546), gr_complex(1.5427, 0.3106), gr_complex(1.3969, 0.8523),
       gr_complex(1.0615, 0.2945), gr_complex(0.9631, 0.5818), gr_complex(1.0710, 0.2924), gr_complex(0.9675, 0.5825),
       gr_complex(1.3255, 0.2975), gr_complex(1.1979, 0.7495), gr_complex(1.4560, 0.3040), gr_complex(1.3269, 0.8414),
       gr_complex(0.2493, 0.5585), gr_complex(0.2960, 0.5344), gr_complex(0.2450, 0.5417), gr_complex(0.2873, 0.5191),
       gr_complex(0.2049, 0.3922), gr_complex(0.2173, 0.3806), gr_complex(0.1990, 0.3755), gr_complex(0.2107, 0.3645),
       gr_complex(0.2493, 0.5599), gr_complex(0.2975, 0.5351), gr_complex(0.2450, 0.5439), gr_complex(0.2887, 0.5213),
       gr_complex(0.2056, 0.3937), gr_complex(0.2187, 0.3820), gr_complex(0.1998, 0.3762), gr_complex(0.2122, 0.3667),
       gr_complex(0.5607, 0.2486), gr_complex(0.5381, 0.2960), gr_complex(0.5439, 0.2442), gr_complex(0.5220, 0.2865),
       gr_complex(0.3908, 0.2049), gr_complex(0.3813, 0.2173), gr_complex(0.3740, 0.1998), gr_complex(0.3653, 0.2100),
       gr_complex(0.5643, 0.2486), gr_complex(0.5410, 0.2967), gr_complex(0.5475, 0.2435), gr_complex(0.5257, 0.2880),
       gr_complex(0.3937, 0.2049), gr_complex(0.3850, 0.2187), gr_complex(0.3762, 0.1998), gr_complex(0.3689, 0.2114)},
      {gr_complex(0.1524, 0.3087), gr_complex(0.1525, 0.3087), gr_complex(0.1513, 0.3043), gr_complex(0.1513, 0.3043),
       gr_complex(0.1682, 0.3004), gr_complex(0.1682, 0.3005), gr_complex(0.1663, 0.2964), gr_complex(0.1663, 0.2964),
       gr_complex(0.1964, 0.6584), gr_complex(0.1965, 0.6583), gr_complex(0.1967, 0.6652), gr_complex(0.1968, 0.6652),
       gr_complex(0.3371, 0.5987), gr_complex(0.3370, 0.5987), gr_complex(0.3414, 0.6039), gr_complex(0.3413, 0.6039),
       gr_complex(0.3087, 0.1524), gr_complex(0.3087, 0.1525), gr_complex(0.3043, 0.1513), gr_complex(0.3043, 0.1513),
       gr_complex(0.3004, 0.1682), gr_complex(0.3005, 0.1682), gr_complex(0.2964, 0.1663), gr_complex(0.2964, 0.1663),
       gr_complex(0.6584, 0.1964), gr_complex(0.6583, 0.1965), gr_complex(0.6652, 0.1967), gr_complex(0.6652, 0.1968),
       gr_complex(0.5987, 0.3371), gr_complex(0.5987, 0.3370), gr_complex(0.6039, 0.3414), gr_complex(0.6039, 0.3413),
       gr_complex(0.3183, 1.5992), gr_complex(0.3186, 1.5991), gr_complex(0.2756, 1.3848), gr_complex(0.2759, 1.3847),
       gr_complex(0.9060, 1.3557), gr_complex(0.9058, 1.3559), gr_complex(0.7846, 1.1739), gr_complex(0.7843, 1.1741),
       gr_complex(0.2257, 0.9956), gr_complex(0.2259, 0.9956), gr_complex(0.2276, 1.0326), gr_complex(0.2278, 1.0326),
       gr_complex(0.5446, 0.8635), gr_complex(0.5445, 0.8636), gr_complex(0.5694, 0.8910), gr_complex(0.5692, 0.8911),
       gr_complex(1.5992, 0.3183), gr_complex(1.5991, 0.3186), gr_complex(1.3848, 0.2756), gr_complex(1.3847, 0.2759),
       gr_complex(1.3557, 0.9060), gr_complex(1.3559, 0.9058), gr_complex(1.1739, 0.7846), gr_complex(1.1741, 0.7843),
       gr_complex(0.9956, 0.2257), gr_complex(0.9956, 0.2259), gr_complex(1.0326, 0.2276), gr_complex(1.0326, 0.2278),
       gr_complex(0.8635, 0.5446), gr_complex(0.8636, 0.5445), gr_complex(0.8910, 0.5694), gr_complex(0.8911, 0.5692)},
      {gr_complex(0.1430, 0.3078), gr_complex(0.1430, 0.3077), gr_complex(0.1413, 0.3003), gr_complex(0.1414, 0.3002),
       gr_complex(0.1637, 0.2973), gr_complex(0.1636, 0.2973), gr_complex(0.1604, 0.2905), gr_complex(0.1603, 0.2905),
       gr_complex(0.1768, 0.6686), gr_complex(0.1793, 0.6679), gr_complex(0.1769, 0.6707), gr_complex(0.1793, 0.6700),
       gr_complex(0.3506, 0.5961), gr_complex(0.3484, 0.5974), gr_complex(0.3523, 0.5975), gr_complex(0.3501, 0.5987),
       gr_complex(0.3078, 0.1430), gr_complex(0.3077, 0.1430), gr_complex(0.3003, 0.1413), gr_complex(0.3002, 0.1414),
       gr_complex(0.2973, 0.1637), gr_complex(0.2973, 0.1636), gr_complex(0.2905, 0.1604), gr_complex(0.2905, 0.1603),
       gr_complex(0.6686, 0.1768), gr_complex(0.6679, 0.1793), gr_complex(0.6707, 0.1769), gr_complex(0.6700, 0.1793),
       gr_complex(0.5961, 0.3506), gr_complex(0.5974, 0.3484), gr_complex(0.5975, 0.3523), gr_complex(0.5987, 0.3501),
       gr_complex(0.2071, 1.6690), gr_complex(0.4482, 1.6210), gr_complex(0.2080, 1.3641), gr_complex(0.3307, 1.3397),
       gr_complex(1.0341, 1.3264), gr_complex(0.8297, 1.4630), gr_complex(0.8178, 1.1114), gr_complex(0.7138, 1.1809),
       gr_complex(0.1957, 0.9674), gr_complex(0.2170, 0.9629), gr_complex(0.1977, 1.0341), gr_complex(0.2288, 1.0277),
       gr_complex(0.5458, 0.8224), gr_complex(0.5276, 0.8342), gr_complex(0.5916, 0.8709), gr_complex(0.5651, 0.8883),
       gr_complex(1.6690, 0.2071), gr_complex(1.6210, 0.4482), gr_complex(1.3641, 0.2080), gr_complex(1.3397, 0.3307),
       gr_complex(1.3264, 1.0341), gr_complex(1.4630, 0.8297), gr_complex(1.1114, 0.8178), gr_complex(1.1809, 0.7138),
       gr_complex(0.9674, 0.1957), gr_complex(0.9629, 0.2170), gr_complex(1.0341, 0.1977), gr_complex(1.0277, 0.2288),
       gr_complex(0.8224, 0.5458), gr_complex(0.8342, 0.5276), gr_complex(0.8709, 0.5916), gr_complex(0.8883, 0.5651)},
      {gr_complex(0.1170, 0.3003), gr_complex(0.1171, 0.3003), gr_complex(0.1204, 0.3233), gr_complex(0.1204, 0.3233),
       gr_complex(0.1454, 0.2877), gr_complex(0.1453, 0.2877), gr_complex(0.1566, 0.3074), gr_complex(0.1565, 0.3074),
       gr_complex(0.1427, 0.6856), gr_complex(0.1562, 0.6826), gr_complex(0.1422, 0.6584), gr_complex(0.1529, 0.6560),
       gr_complex(0.3840, 0.5856), gr_complex(0.3723, 0.5931), gr_complex(0.3651, 0.5660), gr_complex(0.3559, 0.5718),
       gr_complex(0.3003, 0.1170), gr_complex(0.3003, 0.1171), gr_complex(0.3233, 0.1204), gr_complex(0.3233, 0.1204),
       gr_complex(0.2877, 0.1454), gr_complex(0.2877, 0.1453), gr_complex(0.3074, 0.1566), gr_complex(0.3074, 0.1565),
       gr_complex(0.6856, 0.1427), gr_complex(0.6826, 0.1562), gr_complex(0.6584, 0.1422), gr_complex(0.6560, 0.1529),
       gr_complex(0.5856, 0.3840), gr_complex(0.5931, 0.3723), gr_complex(0.5660, 0.3651), gr_complex(0.5718, 0.3559),
       gr_complex(0.1683, 1.7041), gr_complex(0.4972, 1.6386), gr_complex(0.1495, 1.3560), gr_complex(0.3814, 1.3099),
       gr_complex(1.0862, 1.3238), gr_complex(0.8074, 1.5101), gr_complex(0.8534, 1.0644), gr_complex(0.6568, 1.1958),
       gr_complex(0.1552, 0.9481), gr_complex(0.2200, 0.9352), gr_complex(0.1577, 1.0449), gr_complex(0.2548, 1.0255),
       gr_complex(0.5609, 0.7800), gr_complex(0.5060, 0.8167), gr_complex(0.6276, 0.8501), gr_complex(0.5452, 0.9052),
       gr_complex(1.7041, 0.1683), gr_complex(1.6386, 0.4972), gr_complex(1.3560, 0.1495), gr_complex(1.3099, 0.3814),
       gr_complex(1.3238, 1.0862), gr_complex(1.5101, 0.8074), gr_complex(1.0644, 0.8534), gr_complex(1.1958, 0.6568),
       gr_complex(0.9481, 0.1552), gr_complex(0.9352, 0.2200), gr_complex(1.0449, 0.1577), gr_complex(1.0255, 0.2548),
       gr_complex(0.7800, 0.5609), gr_complex(0.8167, 0.5060), gr_complex(0.8501, 0.6276), gr_complex(0.9052, 0.5452)},
      {gr_complex(0.0995, 0.2435), gr_complex(0.0996, 0.2434), gr_complex(0.1169, 0.3886), gr_complex(0.1179, 0.3883),
       gr_complex(0.1192, 0.2345), gr_complex(0.1192, 0.2345), gr_complex(0.1953, 0.3558), gr_complex(0.1944, 0.3563),
       gr_complex(0.1293, 0.7217), gr_complex(0.1616, 0.7151), gr_complex(0.1287, 0.6355), gr_complex(0.1456, 0.6318),
       gr_complex(0.4191, 0.6016), gr_complex(0.3916, 0.6198), gr_complex(0.3585, 0.5403), gr_complex(0.3439, 0.5497),
       gr_complex(0.2435, 0.0995), gr_complex(0.2434, 0.0996), gr_complex(0.3886, 0.1169), gr_complex(0.3883, 0.1179),
       gr_complex(0.2345, 0.1192), gr_complex(0.2345, 0.1192), gr_complex(0.3558, 0.1953), gr_complex(0.3563, 0.1944),
       gr_complex(0.7217, 0.1293), gr_complex(0.7151, 0.1616), gr_complex(0.6355, 0.1287), gr_complex(0.6318, 0.1456),
       gr_complex(0.6016, 0.4191), gr_complex(0.6198, 0.3916), gr_complex(0.5403, 0.3585), gr_complex(0.5497, 0.3439),
       gr_complex(0.1665, 1.6859), gr_complex(0.4919, 1.6211), gr_complex(0.1360, 1.3498), gr_complex(0.3914, 1.2989),
       gr_complex(1.0746, 1.3096), gr_complex(0.7987, 1.4940), gr_complex(0.8585, 1.0504), gr_complex(0.6419, 1.1951),
       gr_complex(0.1334, 0.9483), gr_complex(0.2402, 0.9271), gr_complex(0.1323, 1.0786), gr_complex(0.2910, 1.0470),
       gr_complex(0.5764, 0.7648), gr_complex(0.4860, 0.8252), gr_complex(0.6693, 0.8561), gr_complex(0.5348, 0.9459),
       gr_complex(1.6859, 0.1665), gr_complex(1.6211, 0.4919), gr_complex(1.3498, 0.1360), gr_complex(1.2989, 0.3914),
       gr_complex(1.3096, 1.0746), gr_complex(1.4940, 0.7987), gr_complex(1.0504, 0.8585), gr_complex(1.1951, 0.6419),
       gr_complex(0.9483, 0.1334), gr_complex(0.9271, 0.2402), gr_complex(1.0786, 0.1323), gr_complex(1.0470, 0.2910),
       gr_complex(0.7648, 0.5764), gr_complex(0.8252, 0.4860), gr_complex(0.8561, 0.6693), gr_complex(0.9459, 0.5348)},
      {gr_complex(0.0899, 0.1337), gr_complex(0.0910, 0.1377), gr_complex(0.0873, 0.3862), gr_complex(0.0883, 0.3873),
       gr_complex(0.1115, 0.1442), gr_complex(0.1135, 0.1472), gr_complex(0.2067, 0.3591), gr_complex(0.1975, 0.3621),
       gr_complex(0.1048, 0.7533), gr_complex(0.1770, 0.7412), gr_complex(0.1022, 0.5904), gr_complex(0.1191, 0.5890),
       gr_complex(0.4264, 0.6230), gr_complex(0.3650, 0.6689), gr_complex(0.3254, 0.5153), gr_complex(0.2959, 0.5302),
       gr_complex(0.3256, 0.0768), gr_complex(0.3266, 0.0870), gr_complex(0.4721, 0.0994), gr_complex(0.4721, 0.1206),
       gr_complex(0.2927, 0.1267), gr_complex(0.2947, 0.1296), gr_complex(0.3823, 0.2592), gr_complex(0.3944, 0.2521),
       gr_complex(0.7755, 0.1118), gr_complex(0.7513, 0.2154), gr_complex(0.6591, 0.1033), gr_complex(0.6446, 0.1737),
       gr_complex(0.5906, 0.4930), gr_complex(0.6538, 0.4155), gr_complex(0.4981, 0.3921), gr_complex(0.5373, 0.3586),
       gr_complex(0.1630, 1.6621), gr_complex(0.4720, 1.5898), gr_complex(0.1268, 1.3488), gr_complex(0.3752, 1.2961),
       gr_complex(1.0398, 1.2991), gr_complex(0.7733, 1.4772), gr_complex(0.8380, 1.0552), gr_complex(0.6242, 1.2081),
       gr_complex(0.1103, 0.9397), gr_complex(0.2415, 0.9155), gr_complex(0.1118, 1.1163), gr_complex(0.3079, 1.0866),
       gr_complex(0.5647, 0.7638), gr_complex(0.4385, 0.8433), gr_complex(0.6846, 0.8841), gr_complex(0.5165, 1.0034),
       gr_complex(1.6489, 0.1630), gr_complex(1.5848, 0.4983), gr_complex(1.3437, 0.1389), gr_complex(1.2850, 0.4025),
       gr_complex(1.2728, 1.0661), gr_complex(1.4509, 0.7925), gr_complex(1.0249, 0.8794), gr_complex(1.1758, 0.6545),
       gr_complex(0.9629, 0.1113), gr_complex(0.9226, 0.2849), gr_complex(1.1062, 0.1118), gr_complex(1.0674, 0.3393),
       gr_complex(0.7234, 0.6223), gr_complex(0.8211, 0.4860), gr_complex(0.8457, 0.7260), gr_complex(0.9640, 0.5518)},
      {gr_complex(0.0754, 0.2310), gr_complex(0.0768, 0.2305), gr_complex(0.0924, 0.4136), gr_complex(0.1043, 0.4125),
       gr_complex(0.0829, 0.1135), gr_complex(0.0836, 0.1149), gr_complex(0.2682, 0.3856), gr_complex(0.2531, 0.3906),
       gr_complex(0.0836, 0.7817), gr_complex(0.2052, 0.7608), gr_complex(0.0838, 0.6034), gr_complex(0.1394, 0.5961),
       gr_complex(0.4861, 0.6331), gr_complex(0.3661, 0.7034), gr_complex(0.3732, 0.5159), gr_complex(0.3095, 0.5511),
       gr_complex(0.3030, 0.0811), gr_complex(0.3017, 0.0853), gr_complex(0.4758, 0.0932), gr_complex(0.4676, 0.1242),
       gr_complex(0.2425, 0.1081), gr_complex(0.2447, 0.1115), gr_complex(0.3837, 0.2813), gr_complex(0.3959, 0.2642),
       gr_complex(0.7929, 0.0859), gr_complex(0.7652, 0.2324), gr_complex(0.6365, 0.0872), gr_complex(0.6207, 0.1757),
       gr_complex(0.6149, 0.5145), gr_complex(0.6987, 0.3934), gr_complex(0.5063, 0.4029), gr_complex(0.5526, 0.3356),
       gr_complex(0.1598, 1.6262), gr_complex(0.4733, 1.5637), gr_complex(0.1307, 1.3502), gr_complex(0.3877, 1.2983),
       gr_complex(1.0328, 1.2617), gr_complex(0.7675, 1.4398), gr_complex(0.8496, 1.0508), gr_complex(0.6297, 1.1967),
       gr_complex(0.0910, 0.9531), gr_complex(0.2649, 0.9198), gr_complex(0.1080, 1.1340), gr_complex(0.3214, 1.0926),
       gr_complex(0.5941, 0.7527), gr_complex(0.4371, 0.8528), gr_complex(0.7093, 0.8880), gr_complex(0.5235, 1.0090),
       gr_complex(1.6180, 0.1602), gr_complex(1.5540, 0.4734), gr_complex(1.3411, 0.1336), gr_complex(1.2883, 0.3955),
       gr_complex(1.2561, 1.0337), gr_complex(1.4311, 0.7676), gr_complex(1.0362, 0.8626), gr_complex(1.1845, 0.6419),
       gr_complex(0.9546, 0.0957), gr_complex(0.9163, 0.2834), gr_complex(1.1282, 0.1128), gr_complex(1.0838, 0.3340),
       gr_complex(0.7329, 0.6204), gr_complex(0.8428, 0.4615), gr_complex(0.8680, 0.7295), gr_complex(0.9959, 0.5426)},
      {gr_complex(0.0593, 0.2193), gr_complex(0.0690, 0.3047), gr_complex(0.0663, 0.4879), gr_complex(0.1151, 0.4474),
       gr_complex(0.1689, 0.2163), gr_complex(0.1971, 0.2525), gr_complex(0.3096, 0.3796), gr_complex(0.2489, 0.3933),
       gr_complex(0.0790, 0.7970), gr_complex(0.2340, 0.7710), gr_complex(0.0723, 0.6395), gr_complex(0.1896, 0.6163),
       gr_complex(0.5090, 0.6272), gr_complex(0.3787, 0.7126), gr_complex(0.4079, 0.5049), gr_complex(0.3088, 0.5677),
       gr_complex(0.0675, 0.0626), gr_complex(0.3475, 0.0595), gr_complex(0.5482, 0.0626), gr_complex(0.4784, 0.1124),
       gr_complex(0.1674, 0.0751), gr_complex(0.2856, 0.1132), gr_complex(0.4134, 0.3028), gr_complex(0.4235, 0.2289),
       gr_complex(0.8258, 0.0840), gr_complex(0.7936, 0.2483), gr_complex(0.6788, 0.0783), gr_complex(0.6501, 0.2025),
       gr_complex(0.6246, 0.5211), gr_complex(0.7241, 0.3961), gr_complex(0.5144, 0.4089), gr_complex(0.5918, 0.3146),
       gr_complex(0.1631, 1.5801), gr_complex(0.4806, 1.5133), gr_complex(0.1260, 1.3365), gr_complex(0.3750, 1.2897),
       gr_complex(1.0324, 1.2029), gr_complex(0.7737, 1.3837), gr_complex(0.8350, 1.0529), gr_complex(0.6147, 1.1949),
       gr_complex(0.0929, 0.9596), gr_complex(0.2768, 0.9260), gr_complex(0.1095, 1.1349), gr_complex(0.3250, 1.0941),
       gr_complex(0.6086, 0.7556), gr_complex(0.4514, 0.8566), gr_complex(0.7161, 0.8933), gr_complex(0.5294, 1.0121),
       gr_complex(1.5809, 0.1471), gr_complex(1.5253, 0.4385), gr_complex(1.3380, 0.1363), gr_complex(1.2837, 0.4026),
       gr_complex(1.2476, 0.9785), gr_complex(1.4137, 0.7196), gr_complex(1.0246, 0.8681), gr_complex(1.1771, 0.6494),
       gr_complex(0.9782, 0.0985), gr_complex(0.9383, 0.2922), gr_complex(1.1455, 0.1158), gr_complex(1.0972, 0.3418),
       gr_complex(0.7446, 0.6273), gr_complex(0.8573, 0.4721), gr_complex(0.8767, 0.7377), gr_complex(1.0059, 0.5518)},
      {gr_complex(1.1980, 1.1541), gr_complex(0.9192, 1.2082), gr_complex(1.2778, 0.8523), gr_complex(1.0390, 0.9253),
       gr_complex(0.6057, 1.2200), gr_complex(0.7371, 1.4217), gr_complex(0.6678, 1.0021), gr_complex(0.8412, 0.9448),
       gr_complex(1.2128, 0.5373), gr_complex(1.0048, 0.5165), gr_complex(1.4321, 0.6343), gr_complex(1.0245, 0.7152),
       gr_complex(0.6384, 0.6073), gr_complex(0.8175, 0.5684), gr_complex(0.6568, 0.7801), gr_complex(0.8311, 0.7459),
       gr_complex(0.1349, 1.4742), gr_complex(0.1105, 1.2309), gr_complex(0.0634, 0.9796), gr_complex(0.1891, 1.0198),
       gr_complex(0.4142, 1.4461), gr_complex(0.3323, 1.2279), gr_complex(0.4998, 0.9827), gr_complex(0.3467, 1.0202),
       gr_complex(0.0680, 0.6501), gr_complex(0.2016, 0.6464), gr_complex(0.0719, 0.8075), gr_complex(0.2088, 0.8146),
       gr_complex(0.4809, 0.6296), gr_complex(0.3374, 0.6412), gr_complex(0.4955, 0.8008), gr_complex(0.3431, 0.8141),
       gr_complex(1.2731, 0.1108), gr_complex(1.0794, 0.0977), gr_complex(1.5126, 0.1256), gr_complex(0.9029, 0.0853),
       gr_complex(0.5429, 0.0694), gr_complex(0.6795, 0.0559), gr_complex(0.5628, 0.1945), gr_complex(0.7326, 0.1410),
       gr_complex(1.2283, 0.3217), gr_complex(1.0269, 0.3261), gr_complex(1.4663, 0.3716), gr_complex(0.9085, 0.2470),
       gr_complex(0.6160, 0.4549), gr_complex(0.7818, 0.4247), gr_complex(0.5938, 0.3170), gr_complex(0.7600, 0.2850),
       gr_complex(0.0595, 0.0707), gr_complex(0.1722, 0.0706), gr_complex(0.0599, 0.2119), gr_complex(0.1748, 0.2114),
       gr_complex(0.4134, 0.0701), gr_complex(0.2935, 0.0705), gr_complex(0.4231, 0.2066), gr_complex(0.2979, 0.2100),
       gr_complex(0.0638, 0.5002), gr_complex(0.1905, 0.4966), gr_complex(0.0612, 0.3552), gr_complex(0.1810, 0.3533),
       gr_complex(0.4630, 0.4764), gr_complex(0.3231, 0.4895), gr_complex(0.4416, 0.3397), gr_complex(0.3083, 0.3490)},
      {gr_complex(1.2412, 1.0688), gr_complex(1.2668, 0.8034), gr_complex(0.9860, 1.1758), gr_complex(1.0365, 0.9065),
       gr_complex(1.2111, 0.5135), gr_complex(1.4187, 0.6066), gr_complex(1.0103, 0.4879), gr_complex(1.0380, 0.6906),
       gr_complex(0.6963, 1.3442), gr_complex(0.7089, 1.1122), gr_complex(0.1256, 1.4745), gr_complex(0.8331, 0.9455),
       gr_complex(0.6615, 0.6012), gr_complex(0.6894, 0.7594), gr_complex(0.8373, 0.5633), gr_complex(0.8552, 0.7410),
       gr_complex(1.2666, 0.1027), gr_complex(1.4915, 0.1198), gr_complex(1.0766, 0.0945), gr_complex(0.9007, 0.0848),
       gr_complex(1.2454, 0.3064), gr_complex(1.4646, 0.3600), gr_complex(1.0570, 0.2995), gr_complex(0.9140, 0.2530),
       gr_complex(0.5461, 0.0679), gr_complex(0.5681, 0.1947), gr_complex(0.6874, 0.0537), gr_complex(0.7375, 0.1492),
       gr_complex(0.6290, 0.4553), gr_complex(0.6007, 0.3177), gr_complex(0.7885, 0.4231), gr_complex(0.7627, 0.2849),
       gr_complex(0.0816, 1.1632), gr_complex(0.0830, 0.9813), gr_complex(0.2528, 1.2315), gr_complex(0.2502, 1.0100),
       gr_complex(0.0732, 0.6827), gr_complex(0.0811, 0.8293), gr_complex(0.2159, 0.6673), gr_complex(0.2359, 0.8283),
       gr_complex(0.4302, 1.4458), gr_complex(0.5852, 0.9680), gr_complex(0.4528, 1.2074), gr_complex(0.4167, 1.0099),
       gr_complex(0.5035, 0.6307), gr_complex(0.5359, 0.7954), gr_complex(0.3580, 0.6532), gr_complex(0.3841, 0.8207),
       gr_complex(0.0576, 0.0745), gr_complex(0.0581, 0.2241), gr_complex(0.1720, 0.0742), gr_complex(0.1753, 0.2222),
       gr_complex(0.0652, 0.5269), gr_complex(0.0611, 0.3767), gr_complex(0.1972, 0.5178), gr_complex(0.1836, 0.3695),
       gr_complex(0.4145, 0.0709), gr_complex(0.4266, 0.2100), gr_complex(0.2912, 0.0730), gr_complex(0.2982, 0.2177),
       gr_complex(0.4766, 0.4821), gr_complex(0.4497, 0.3448), gr_complex(0.3334, 0.5025), gr_complex(0.3125, 0.3601)}
    };

  } /* namespace atsc3 */
} /* namespace gr */
