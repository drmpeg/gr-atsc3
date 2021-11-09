/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "framemapper_cc_impl.h"

namespace gr {
  namespace atsc3 {

    using input_type = gr_complex;
    using output_type = gr_complex;
    framemapper_cc::sptr
    framemapper_cc::make(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation, atsc3_fftsize_t fftsize, int numpayloadsyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_l1_fec_mode_t l1bmode, atsc3_l1_fec_mode_t l1dmode)
    {
      return gnuradio::make_block_sptr<framemapper_cc_impl>(
        framesize, rate, constellation, fftsize, numpayloadsyms, guardinterval, pilotpattern, l1bmode, l1dmode);
    }


    /*
     * The private constructor
     */
    framemapper_cc_impl::framemapper_cc_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation, atsc3_fftsize_t fftsize, int numpayloadsyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_l1_fec_mode_t l1bmode, atsc3_l1_fec_mode_t l1dmode)
      : gr::block("framemapper_cc",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      L1_Basic *l1basicinit = &L1_Signalling[0].l1basic_data;
      L1_Detail *l1detailinit = &L1_Signalling[0].l1detail_data;
      double normalization;
      int rateindex, i, j, l1cells;
      int cred = 0;

      fft_size = fftsize;
      l1b_mode = l1bmode;
      l1d_mode = l1dmode;
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
      m_16qam[0] = mod_table_16QAM[0];
      m_16qam[1] = mod_table_16QAM[1];
      m_16qam[2] = mod_table_16QAM[2];
      m_16qam[3] = mod_table_16QAM[3];
      m_16qam[4] = -std::conj(mod_table_16QAM[0]);
      m_16qam[5] = -std::conj(mod_table_16QAM[1]);
      m_16qam[6] = -std::conj(mod_table_16QAM[2]);
      m_16qam[7] = -std::conj(mod_table_16QAM[3]);
      m_16qam[8] = std::conj(mod_table_16QAM[0]);
      m_16qam[9] = std::conj(mod_table_16QAM[1]);
      m_16qam[10] = std::conj(mod_table_16QAM[2]);
      m_16qam[11] = std::conj(mod_table_16QAM[3]);
      m_16qam[12] = -mod_table_16QAM[0];
      m_16qam[13] = -mod_table_16QAM[1];
      m_16qam[14] = -mod_table_16QAM[2];
      m_16qam[15] = -mod_table_16QAM[3];
      for (i = 0, j = 0; i < 16; i++, j++) {
        m_64qam[i] = mod_table_64QAM[j];
      }
      for (i = 16, j = 0; i < 32; i++, j++) {
        m_64qam[i] = -std::conj(mod_table_64QAM[j]);
      }
      for (i = 32, j = 0; i < 48; i++, j++) {
        m_64qam[i] = std::conj(mod_table_64QAM[j]);
      }
      for (i = 48, j = 0; i < 64; i++, j++) {
        m_64qam[i] = -mod_table_64QAM[j];
      }
      if (l1bmode == L1_FEC_MODE_6) {
        rateindex = 0;
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
        rateindex = 1;
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
        rateindex = 0;
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
        rateindex = 1;
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
      l1basicinit->lls_flag = FALSE;
      l1basicinit->time_info_flag = TIF_NOT_INCLUDED;
      l1basicinit->return_channel_flag = FALSE;
      l1basicinit->papr_reduction = PAPR_OFF;
      l1basicinit->frame_length_mode = FLM_SYMBOL_ALIGNED;
      l1basicinit->frame_length = 0;
      l1basicinit->excess_samples_per_symbol = 0;
      l1basicinit->time_offset = 0;
      l1basicinit->additional_samples = 0;
      l1basicinit->num_subframes = 0;
      l1basicinit->preamble_reduced_carriers = 0;
      l1basicinit->L1_Detail_content_tag = 0;
      l1basicinit->L1_Detail_size_bytes = 25;
      l1basicinit->L1_Detail_fec_type = l1dmode;
      l1basicinit->L1_Detail_additional_parity_mode = APM_K0;
      l1basicinit->first_sub_mimo = FALSE;
      l1basicinit->first_sub_miso = MISO_OFF;
      l1basicinit->first_sub_fft_size = fftsize;
      l1basicinit->first_sub_reduced_carriers = CRED_0;
      l1basicinit->first_sub_guard_interval = GI_5_1024;
      l1basicinit->first_sub_num_ofdm_symbols = numpayloadsyms - 1;
      l1basicinit->first_sub_scattered_pilot_pattern = PILOT_SP3_4;
      l1basicinit->first_sub_scattered_pilot_boost = 4;
      l1basicinit->first_sub_sbs_first = FALSE;
      l1basicinit->first_sub_sbs_last = TRUE;
      l1basicinit->reserved = 0xffffffffffff;

      l1detailinit->version = 0;
      l1detailinit->num_rf = 0;
      l1detailinit->frequency_interleaver = FALSE;
      l1detailinit->sbs_null_cells = 3026;
      l1detailinit->num_plp = 0;
      l1detailinit->plp_id = 0;
      l1detailinit->plp_lls_flag = FALSE;
      l1detailinit->plp_layer = 0;
      l1detailinit->plp_start = 0;
      l1detailinit->plp_size = 450029;
      l1detailinit->plp_scrambler_type = 0;
      if (framesize == FECFRAME_SHORT) {
        l1detailinit->plp_fec_type = FEC_TYPE_BCH_16K;
      }
      else {
        l1detailinit->plp_fec_type = FEC_TYPE_BCH_64K;
      }
      l1detailinit->plp_mod = constellation;
      l1detailinit->plp_cod = rate;
      l1detailinit->plp_TI_mode = 0;
      l1detailinit->plp_fec_block_start = 0;
      l1detailinit->plp_type = 0;
      l1detailinit->reserved = 0xfffffffffffff;
      l1basicinit->L1_Detail_total_cells = l1cells = add_l1detail(&l1_dummy[0]);
      l1cells += add_l1basic(&l1_dummy[0]);
      printf("l1cells = %d\n", l1cells);
      if (l1cells <= 4307) {
        l1basicinit->preamble_num_symbols = 0;
      }
      else {
        l1basicinit->preamble_num_symbols = 1;
      }
      switch (fftsize) {
        case FFTSIZE_8K:
          first_preamble_cells = 4307;
          switch (guardinterval) {
            case GI_1_192:
              preamble_cells = preamble_cells_table[0][cred];
              break;
            case GI_2_384:
              preamble_cells = preamble_cells_table[1][cred];
              break;
            case GI_3_512:
              preamble_cells = preamble_cells_table[2][cred];
              break;
            case GI_4_768:
              preamble_cells = preamble_cells_table[3][cred];
              break;
            case GI_5_1024:
              preamble_cells = preamble_cells_table[4][cred];
              break;
            case GI_6_1536:
              preamble_cells = preamble_cells_table[5][cred];
              break;
            case GI_7_2048:
              preamble_cells = preamble_cells_table[6][cred];
              break;
            default:
              preamble_cells = preamble_cells_table[0][cred];
              break;
          }
          break;
        case FFTSIZE_16K:
          first_preamble_cells = 8614;
          switch (guardinterval) {
            case GI_1_192:
              preamble_cells = preamble_cells_table[7][cred];
              break;
            case GI_2_384:
              preamble_cells = preamble_cells_table[8][cred];
              break;
            case GI_3_512:
              preamble_cells = preamble_cells_table[9][cred];
              break;
            case GI_4_768:
              preamble_cells = preamble_cells_table[10][cred];
              break;
            case GI_5_1024:
              preamble_cells = preamble_cells_table[11][cred];
              break;
            case GI_6_1536:
              preamble_cells = preamble_cells_table[12][cred];
              break;
            case GI_7_2048:
              preamble_cells = preamble_cells_table[13][cred];
              break;
            case GI_8_2432:
              preamble_cells = preamble_cells_table[14][cred];
              break;
            case GI_9_3072:
              preamble_cells = preamble_cells_table[15][cred];
              break;
            case GI_10_3648:
              preamble_cells = preamble_cells_table[16][cred];
              break;
            case GI_11_4096:
              preamble_cells = preamble_cells_table[17][cred];
              break;
            default:
              preamble_cells = preamble_cells_table[0][cred];
              break;
          }
          break;
        case FFTSIZE_32K:
          first_preamble_cells = 17288;
          switch (guardinterval) {
            case GI_1_192:
              preamble_cells = preamble_cells_table[18][cred];
              break;
            case GI_2_384:
              preamble_cells = preamble_cells_table[19][cred];
              break;
            case GI_3_512:
              preamble_cells = preamble_cells_table[20][cred];
              break;
            case GI_4_768:
              preamble_cells = preamble_cells_table[21][cred];
              break;
            case GI_5_1024:
              preamble_cells = preamble_cells_table[22][cred];
              break;
            case GI_6_1536:
              preamble_cells = preamble_cells_table[23][cred];
              break;
            case GI_7_2048:
              preamble_cells = preamble_cells_table[24][cred];
              break;
            case GI_8_2432:
              preamble_cells = preamble_cells_table[25][cred];
              break;
            case GI_9_3072:
              if (pilotpattern == PILOT_SP8_2 || pilotpattern == PILOT_SP8_4) {
                preamble_cells = preamble_cells_table[26][cred];
              }
              else {
                preamble_cells = preamble_cells_table[27][cred];
              }
              break;
            case GI_10_3648:
              if (pilotpattern == PILOT_SP8_2 || pilotpattern == PILOT_SP8_4) {
                preamble_cells = preamble_cells_table[28][cred];
              }
              else {
                preamble_cells = preamble_cells_table[29][cred];
              }
              break;
            case GI_11_4096:
              preamble_cells = preamble_cells_table[30][cred];
              break;
            case GI_12_4864:
              preamble_cells = preamble_cells_table[31][cred];
              break;
            default:
              preamble_cells = preamble_cells_table[0][cred];
              break;
          }
          break;
        default:
          break;
      }
      switch (fftsize) {
        case FFTSIZE_8K:
          switch (pilotpattern) {
            case PILOT_SP3_2:
              data_cells = data_cells_table_8K[PILOT_SP3_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP3_2][cred];
              break;
            case PILOT_SP3_4:
              data_cells = data_cells_table_8K[PILOT_SP3_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP3_4][cred];
              break;
            case PILOT_SP4_2:
              data_cells = data_cells_table_8K[PILOT_SP4_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP4_2][cred];
              break;
            case PILOT_SP4_4:
              data_cells = data_cells_table_8K[PILOT_SP4_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP4_4][cred];
              break;
            case PILOT_SP6_2:
              data_cells = data_cells_table_8K[PILOT_SP6_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP6_2][cred];
              break;
            case PILOT_SP6_4:
              data_cells = data_cells_table_8K[PILOT_SP6_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP6_4][cred];
              break;
            case PILOT_SP8_2:
              data_cells = data_cells_table_8K[PILOT_SP8_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP8_2][cred];
              break;
            case PILOT_SP8_4:
              data_cells = data_cells_table_8K[PILOT_SP8_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP8_4][cred];
              break;
            case PILOT_SP12_2:
              data_cells = data_cells_table_8K[PILOT_SP12_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP12_2][cred];
              break;
            case PILOT_SP12_4:
              data_cells = data_cells_table_8K[PILOT_SP12_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP12_4][cred];
              break;
            case PILOT_SP16_2:
              data_cells = data_cells_table_8K[PILOT_SP16_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP16_2][cred];
              break;
            case PILOT_SP16_4:
              data_cells = data_cells_table_8K[PILOT_SP16_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP16_4][cred];
              break;
            case PILOT_SP24_2:
              data_cells = data_cells_table_8K[PILOT_SP24_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP24_2][cred];
              break;
            case PILOT_SP24_4:
              data_cells = data_cells_table_8K[PILOT_SP24_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP24_4][cred];
              break;
            case PILOT_SP32_2:
              data_cells = data_cells_table_8K[PILOT_SP32_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP32_2][cred];
              break;
            case PILOT_SP32_4:
              data_cells = data_cells_table_8K[PILOT_SP32_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP32_4][cred];
              break;
            default:
              data_cells = data_cells_table_8K[PILOT_SP3_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP3_2][cred];
              break;
          }
          break;
        case FFTSIZE_16K:
          switch (pilotpattern) {
            case PILOT_SP3_2:
              data_cells = data_cells_table_16K[PILOT_SP3_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP3_2][cred];
              break;
            case PILOT_SP3_4:
              data_cells = data_cells_table_16K[PILOT_SP3_4][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP3_4][cred];
              break;
            case PILOT_SP4_2:
              data_cells = data_cells_table_16K[PILOT_SP4_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP4_2][cred];
              break;
            case PILOT_SP4_4:
              data_cells = data_cells_table_16K[PILOT_SP4_4][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP4_4][cred];
              break;
            case PILOT_SP6_2:
              data_cells = data_cells_table_16K[PILOT_SP6_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP6_2][cred];
              break;
            case PILOT_SP6_4:
              data_cells = data_cells_table_16K[PILOT_SP6_4][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP6_4][cred];
              break;
            case PILOT_SP8_2:
              data_cells = data_cells_table_16K[PILOT_SP8_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP8_2][cred];
              break;
            case PILOT_SP8_4:
              data_cells = data_cells_table_16K[PILOT_SP8_4][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP8_4][cred];
              break;
            case PILOT_SP12_2:
              data_cells = data_cells_table_16K[PILOT_SP12_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP12_2][cred];
              break;
            case PILOT_SP12_4:
              data_cells = data_cells_table_16K[PILOT_SP12_4][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP12_4][cred];
              break;
            case PILOT_SP16_2:
              data_cells = data_cells_table_16K[PILOT_SP16_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP16_2][cred];
              break;
            case PILOT_SP16_4:
              data_cells = data_cells_table_16K[PILOT_SP16_4][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP16_4][cred];
              break;
            case PILOT_SP24_2:
              data_cells = data_cells_table_16K[PILOT_SP24_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP24_2][cred];
              break;
            case PILOT_SP24_4:
              data_cells = data_cells_table_16K[PILOT_SP24_4][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP24_4][cred];
              break;
            case PILOT_SP32_2:
              data_cells = data_cells_table_16K[PILOT_SP32_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP32_2][cred];
              break;
            case PILOT_SP32_4:
              data_cells = data_cells_table_16K[PILOT_SP32_4][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP32_4][cred];
              break;
            default:
              data_cells = data_cells_table_16K[PILOT_SP3_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP3_2][cred];
              break;
          }
          break;
        case FFTSIZE_32K:
          switch (pilotpattern) {
            case PILOT_SP3_2:
              data_cells = data_cells_table_32K[PILOT_SP3_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP3_2][cred];
              break;
            case PILOT_SP3_4:
              data_cells = data_cells_table_32K[PILOT_SP3_4][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP3_4][cred];
              break;
            case PILOT_SP4_2:
              data_cells = data_cells_table_32K[PILOT_SP4_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP4_2][cred];
              break;
            case PILOT_SP4_4:
              data_cells = data_cells_table_32K[PILOT_SP4_4][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP4_4][cred];
              break;
            case PILOT_SP6_2:
              data_cells = data_cells_table_32K[PILOT_SP6_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP6_2][cred];
              break;
            case PILOT_SP6_4:
              data_cells = data_cells_table_32K[PILOT_SP6_4][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP6_4][cred];
              break;
            case PILOT_SP8_2:
              data_cells = data_cells_table_32K[PILOT_SP8_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP8_2][cred];
              break;
            case PILOT_SP8_4:
              data_cells = data_cells_table_32K[PILOT_SP8_4][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP8_4][cred];
              break;
            case PILOT_SP12_2:
              data_cells = data_cells_table_32K[PILOT_SP12_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP12_2][cred];
              break;
            case PILOT_SP12_4:
              data_cells = data_cells_table_32K[PILOT_SP12_4][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP12_4][cred];
              break;
            case PILOT_SP16_2:
              data_cells = data_cells_table_32K[PILOT_SP16_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP16_2][cred];
              break;
            case PILOT_SP16_4:
              data_cells = data_cells_table_32K[PILOT_SP16_4][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP16_4][cred];
              break;
            case PILOT_SP24_2:
              data_cells = data_cells_table_32K[PILOT_SP24_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP24_2][cred];
              break;
            case PILOT_SP24_4:
              data_cells = data_cells_table_32K[PILOT_SP24_4][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP24_4][cred];
              break;
            case PILOT_SP32_2:
              data_cells = data_cells_table_32K[PILOT_SP32_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP32_2][cred];
              break;
            case PILOT_SP32_4:
              data_cells = data_cells_table_32K[PILOT_SP32_4][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP32_4][cred];
              break;
            default:
              data_cells = data_cells_table_32K[PILOT_SP3_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP3_2][cred];
              break;
          }
          break;
        default:
          break;
      }
      printf("preamble cells = %d, data cells = %d, sbs cells = %d\n", preamble_cells, data_cells, sbs_cells);
      printf("total cells = %d\n", first_preamble_cells + preamble_cells + ((numpayloadsyms - 1) * data_cells) + sbs_cells);
      set_output_multiple(first_preamble_cells + preamble_cells + ((numpayloadsyms - 1) * data_cells) + sbs_cells);
    }

    /*
     * Our virtual destructor.
     */
    framemapper_cc_impl::~framemapper_cc_impl()
    {
    }

    void
    framemapper_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = 450029;
    }

#define CRC_POLY 0x00210801

    int
    framemapper_cc_impl::add_crc32_bits(unsigned char* in, int length)
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
    framemapper_cc_impl::init_fm_randomizer(void)
    {
      int sr = 0x18f;
      int b;

      for (int i = 0; i < FRAME_SIZE_SHORT; i++) {
        fm_randomize[i] = ((sr & 0x4) << 5) | ((sr & 0x8 ) << 3) | ((sr & 0x10) << 1) | \
                          ((sr & 0x20) >> 1) | ((sr & 0x200) >> 6) | ((sr & 0x1000) >> 10) | \
                          ((sr & 0x2000) >> 12) | ((sr & 0x8000) >> 15);
        b = sr & 1;
        sr >>= 1;
        if (b) {
          sr ^= POLYNOMIAL;
        }
      }
    }

    int
    framemapper_cc_impl::poly_mult(const int* ina, int lena, const int* inb, int lenb, int* out)
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
    framemapper_cc_impl::calculate_crc_table(void)
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
    framemapper_cc_impl::bch_poly_build_tables(void)
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

    int
    framemapper_cc_impl::add_l1basic(gr_complex *out)
    {
      int temp, index, offset_bits = 0;
      int npad, padbits, count, nrepeat;
      int block, indexb, nouter, numbits;
      int npunctemp, npunc, nfectemp, nfec;
      int B, mod, rows, pack;
      long long templong;
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
      const unsigned char *c1, *c2, *c3, *c4, *c5, *c6, *c7, *c8;

      temp = l1basicinit->version;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      l1basic[offset_bits++] = l1basicinit->mimo_scattered_pilot_encoding;
      l1basic[offset_bits++] = l1basicinit->lls_flag;
      temp = l1basicinit->time_info_flag;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      l1basic[offset_bits++] = l1basicinit->return_channel_flag;
      temp = l1basicinit->papr_reduction;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      l1basic[offset_bits++] = l1basicinit->frame_length_mode;
      if (l1basicinit->frame_length_mode == FALSE) {
        temp = l1basicinit->frame_length;
        for (int n = 9; n >= 0; n--) {
          l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
        }
        temp = l1basicinit->excess_samples_per_symbol;
        for (int n = 12; n >= 0; n--) {
          l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
        }
      }
      else {
        temp = l1basicinit->time_offset;
        for (int n = 15; n >= 0; n--) {
          l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
        }
        temp = l1basicinit->additional_samples;
        for (int n = 6; n >= 0; n--) {
          l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
        }
      }
      temp = l1basicinit->num_subframes;
      for (int n = 7; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->preamble_num_symbols;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->preamble_reduced_carriers;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->L1_Detail_content_tag;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->L1_Detail_size_bytes;
      for (int n = 12; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->L1_Detail_fec_type;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->L1_Detail_additional_parity_mode;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->L1_Detail_total_cells;
      for (int n = 18; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      l1basic[offset_bits++] = l1basicinit->first_sub_mimo;
      temp = l1basicinit->first_sub_miso;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->first_sub_fft_size;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->first_sub_reduced_carriers;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->first_sub_guard_interval;
      for (int n = 3; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->first_sub_num_ofdm_symbols;
      for (int n = 10; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->first_sub_scattered_pilot_pattern;
      for (int n = 4; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->first_sub_scattered_pilot_boost;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      l1basic[offset_bits++] = l1basicinit->first_sub_sbs_first;
      l1basic[offset_bits++] = l1basicinit->first_sub_sbs_last;
      templong = l1basicinit->reserved;
      for (int n = 47; n >= 0; n--) {
        l1basic[offset_bits++] = templong & (1 << n) ? 1 : 0;
      }
      offset_bits += add_crc32_bits(l1basic, offset_bits);

      /* scrambling and BCH encoding */
      for (int i = 0; i < offset_bits; i += 8) {
        temp = index = 0;
        for (int j = 7; j >= 0; j--) {
          temp |= l1basic[i + index++] << j;
        }
        temp ^= fm_randomize[i / 8];
        index = 0;
        for (int n = 7; n >= 0; n--) {
          l1basic[i + index++] = temp & (1 << n) ? 1 : 0;
        }
        b = temp;
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
      // First zero all the parity bits
      memset(buffer, 0, sizeof(unsigned char)*plen);
      // now do the parity checking
      d = &l1basic[0];
      for (int j = 0; j < ldpc_encode_1st.table_length; j++) {
        buffer[ldpc_encode_1st.p[j]] ^= d[ldpc_encode_1st.d[j]];
      }
      for(int j = 1; j < m1; j++) {
        buffer[j] ^= buffer[j-1];
      }
      for (int t = 0; t < q1; t++) {
        for (int s = 0; s < 360; s++) {
          l1basic[NBCH_3_15 + (360 * t) + s] = buffer[(q1 * s) + t];
        }
      }
      for (int j = 0; j < ldpc_encode_2nd.table_length; j++) {
        buffer[ldpc_encode_2nd.p[j]] ^= d[ldpc_encode_2nd.d[j]];
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

#if 0
      for (int i = 0; i < numbits; i += 8) {
        temp = index = 0;
        for (int j = 7; j >= 0; j--) {
          temp |= l1temp[i + index] << j;
          index++;
        }
        if ((i % 128) == 0) {
          if (i != 0) {
            printf("\n");
          }
        }
        printf("%02X", temp);
      }
      printf("\n");
#endif

      /* block interleaver, bit demuxing and constellation mapping */
      rows = numbits / mod;
      switch (l1b_mode) {
        case L1_FEC_MODE_1:
        case L1_FEC_MODE_2:
        case L1_FEC_MODE_3:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1basic[index++] = c1[j];
            l1basic[index++] = c2[j];
          }
          index = 0;
          for (int j = 0; j < rows; j++) {
            temp = l1basic[index++] << 1;
            temp |= l1basic[index++];
            *out++ = m_qpsk[temp];
          }
          break;
        case L1_FEC_MODE_4:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          c3 = &l1temp[rows * 2];
          c4 = &l1temp[rows * 3];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1basic[index++] = c1[j];
            l1basic[index++] = c2[j];
            l1basic[index++] = c3[j];
            l1basic[index++] = c4[j];
          }
          index = count = 0;
          for (int j = 0; j < rows; j++) {
            pack = 0;
            for (int e = 3; e >= 0; e--) {
              pack |= l1basic[index++] << e;
            }
            temp = pack << count;
            pack = temp & 0xf;
            temp >>= 4;
            pack |= temp;
            count = (count + 1) & 0x3;
            *out++ = m_16qam[pack & 0xf];
          }
          break;
        case L1_FEC_MODE_5:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          c3 = &l1temp[rows * 2];
          c4 = &l1temp[rows * 3];
          c5 = &l1temp[rows * 4];
          c6 = &l1temp[rows * 5];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1basic[index++] = c1[j];
            l1basic[index++] = c2[j];
            l1basic[index++] = c3[j];
            l1basic[index++] = c4[j];
            l1basic[index++] = c5[j];
            l1basic[index++] = c6[j];
          }
          index = count = 0;
          for (int j = 0; j < rows; j++) {
            pack = 0;
            for (int e = 5; e >= 0; e--) {
              pack |= l1basic[index++] << e;
            }
            temp = pack << count;
            pack = temp & 0x3f;
            temp >>= 6;
            pack |= temp;
            count = (count + 1);
            if (count == 6) {  /* faster than modulo */
              count = 0;
            }
            *out++ = m_64qam[pack & 0x3f];
          }
          break;
        case L1_FEC_MODE_6:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          c3 = &l1temp[rows * 2];
          c4 = &l1temp[rows * 3];
          c5 = &l1temp[rows * 4];
          c6 = &l1temp[rows * 5];
          c7 = &l1temp[rows * 6];
          c8 = &l1temp[rows * 7];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1basic[index++] = c1[j];
            l1basic[index++] = c2[j];
            l1basic[index++] = c3[j];
            l1basic[index++] = c4[j];
            l1basic[index++] = c5[j];
            l1basic[index++] = c6[j];
            l1basic[index++] = c7[j];
            l1basic[index++] = c8[j];
          }
          index = count = 0;
          for (int j = 0; j < rows; j++) {
            pack = 0;
            for (int e = 7; e >= 0; e--) {
              pack |= l1basic[index++] << e;
            }
            temp = pack << count;
            pack = temp & 0xff;
            temp >>= 8;
            pack |= temp;
            count = (count + 1) & 0x7;
            *out++ = m_l1b_256qam[pack & 0xff];
          }
          break;
        case L1_FEC_MODE_7:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          c3 = &l1temp[rows * 2];
          c4 = &l1temp[rows * 3];
          c5 = &l1temp[rows * 4];
          c6 = &l1temp[rows * 5];
          c7 = &l1temp[rows * 6];
          c8 = &l1temp[rows * 7];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1basic[index++] = c1[j];
            l1basic[index++] = c2[j];
            l1basic[index++] = c3[j];
            l1basic[index++] = c4[j];
            l1basic[index++] = c5[j];
            l1basic[index++] = c6[j];
            l1basic[index++] = c7[j];
            l1basic[index++] = c8[j];
          }
          index = count = 0;
          for (int j = 0; j < rows; j++) {
            pack = 0;
            for (int e = 7; e >= 0; e--) {
              pack |= l1basic[index++] << e;
            }
            temp = pack << count;
            pack = temp & 0xff;
            temp >>= 8;
            pack |= temp;
            count = (count + 1) & 0x7;
            *out++ = m_l1b_256qam[pack & 0xff];
          }
          break;
        default:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1basic[index++] = c1[j];
            l1basic[index++] = c2[j];
          }
          index = 0;
          for (int j = 0; j < rows; j++) {
            temp = l1basic[index++] << 1;
            temp |= l1basic[index++];
            *out++ = m_qpsk[temp];
          }
          break;
      }
      return (numbits / mod);
    }

    int
    framemapper_cc_impl::add_l1detail(gr_complex *out)
    {
      int temp, index, offset_bits = 0;
      int npad, padbits, count, nrepeat, table;
      int block, indexb, nouter, numbits;
      int npunctemp, npunc, nfectemp, nfec;
      int Anum, Aden, B, mod, rows, pack;
      long long templong;
      std::bitset<MAX_BCH_PARITY_BITS> parity_bits;
      unsigned char b, tempbch, msb;
      unsigned char *l1detail = l1_detail;
      unsigned char *l1temp = l1_temp;
      L1_Detail *l1detailinit = &L1_Signalling[0].l1detail_data;
      const unsigned char* d;
      unsigned char* p;
      int plen, nbch, groups;
      const int q1 = q1_val;
      const int q2 = q2_val;
      const int m1 = m1_val;
      const unsigned char *c1, *c2, *c3, *c4, *c5, *c6, *c7, *c8;

      temp = l1detailinit->version;
      for (int n = 3; n >= 0; n--) {
        l1detail[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1detailinit->num_rf;
      for (int n = 2; n >= 0; n--) {
        l1detail[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      l1detail[offset_bits++] = l1detailinit->frequency_interleaver;
      temp = l1detailinit->sbs_null_cells;
      for (int n = 12; n >= 0; n--) {
        l1detail[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1detailinit->num_plp;
      for (int n = 5; n >= 0; n--) {
        l1detail[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1detailinit->plp_id;
      for (int n = 5; n >= 0; n--) {
        l1detail[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      l1detail[offset_bits++] = l1detailinit->plp_lls_flag;
      temp = l1detailinit->plp_layer;
      for (int n = 1; n >= 0; n--) {
        l1detail[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1detailinit->plp_start;
      for (int n = 23; n >= 0; n--) {
        l1detail[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1detailinit->plp_size;
      for (int n = 23; n >= 0; n--) {
        l1detail[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1detailinit->plp_scrambler_type;
      for (int n = 1; n >= 0; n--) {
        l1detail[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1detailinit->plp_fec_type;
      for (int n = 3; n >= 0; n--) {
        l1detail[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1detailinit->plp_mod;
      for (int n = 3; n >= 0; n--) {
        l1detail[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1detailinit->plp_cod;
      for (int n = 3; n >= 0; n--) {
        l1detail[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1detailinit->plp_TI_mode;
      for (int n = 1; n >= 0; n--) {
        l1detail[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1detailinit->plp_fec_block_start;
      for (int n = 14; n >= 0; n--) {
        l1detail[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      l1detail[offset_bits++] = l1detailinit->plp_type;
      templong = l1detailinit->reserved;
      for (int n = 51; n >= 0; n--) {
        l1detail[offset_bits++] = templong & (1 << n) ? 1 : 0;
      }
      offset_bits += add_crc32_bits(l1detail, offset_bits);

      /* scrambling and BCH encoding */
      for (int i = 0; i < offset_bits; i += 8) {
        temp = index = 0;
        for (int j = 7; j >= 0; j--) {
          temp |= l1detail[i + index++] << j;
        }
        temp ^= fm_randomize[i / 8];
        index = 0;
        for (int n = 7; n >= 0; n--) {
          l1detail[i + index++] = temp & (1 << n) ? 1 : 0;
        }
        b = temp;
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
          // First zero all the parity bits
          memset(buffer, 0, sizeof(unsigned char)*plen);
          // now do the parity checking
          d = &l1detail[0];
          for (int j = 0; j < ldpc_encode_1st.table_length; j++) {
            buffer[ldpc_encode_1st.p[j]] ^= d[ldpc_encode_1st.d[j]];
          }
          for(int j = 1; j < m1; j++) {
            buffer[j] ^= buffer[j-1];
          }
          for (int t = 0; t < q1; t++) {
            for (int s = 0; s < 360; s++) {
              l1detail[nbch + (360 * t) + s] = buffer[(q1 * s) + t];
            }
          }
          for (int j = 0; j < ldpc_encode_2nd.table_length; j++) {
            buffer[ldpc_encode_2nd.p[j]] ^= d[ldpc_encode_2nd.d[j]];
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
            for (int i_d = 1; i_d < ldpc_lut[i_p][0]; i_d++) {
              pbit ^= d[ldpc_lut[i_p][i_d]];
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

#if 0
      for (int i = 0; i < numbits; i += 8) {
        temp = index = 0;
        for (int j = 7; j >= 0; j--) {
          temp |= l1temp[i + index] << j;
          index++;
        }
        if ((i % 128) == 0) {
          if (i != 0) {
            printf("\n");
          }
        }
        printf("%02X", temp);
      }
      printf("\n");
#endif

      /* block interleaver, bit demuxing and constellation mapping */
      rows = numbits / mod;
      switch (l1d_mode) {
        case L1_FEC_MODE_1:
        case L1_FEC_MODE_2:
        case L1_FEC_MODE_3:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1detail[index++] = c1[j];
            l1detail[index++] = c2[j];
          }
          index = 0;
          for (int j = 0; j < rows; j++) {
            temp = l1detail[index++] << 1;
            temp |= l1detail[index++];
            *out++ = m_qpsk[temp];
          }
          break;
        case L1_FEC_MODE_4:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          c3 = &l1temp[rows * 2];
          c4 = &l1temp[rows * 3];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1detail[index++] = c1[j];
            l1detail[index++] = c2[j];
            l1detail[index++] = c3[j];
            l1detail[index++] = c4[j];
          }
          index = count = 0;
          for (int j = 0; j < rows; j++) {
            pack = 0;
            for (int e = 3; e >= 0; e--) {
              pack |= l1detail[index++] << e;
            }
            temp = pack << count;
            pack = temp & 0xf;
            temp >>= 4;
            pack |= temp;
            count = (count + 1) & 0x3;
            *out++ = m_16qam[pack & 0xf];
          }
          break;
        case L1_FEC_MODE_5:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          c3 = &l1temp[rows * 2];
          c4 = &l1temp[rows * 3];
          c5 = &l1temp[rows * 4];
          c6 = &l1temp[rows * 5];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1detail[index++] = c1[j];
            l1detail[index++] = c2[j];
            l1detail[index++] = c3[j];
            l1detail[index++] = c4[j];
            l1detail[index++] = c5[j];
            l1detail[index++] = c6[j];
          }
          index = count = 0;
          for (int j = 0; j < rows; j++) {
            pack = 0;
            for (int e = 5; e >= 0; e--) {
              pack |= l1detail[index++] << e;
            }
            temp = pack << count;
            pack = temp & 0x3f;
            temp >>= 6;
            pack |= temp;
            count = (count + 1);
            if (count == 6) {  /* faster than modulo */
              count = 0;
            }
            *out++ = m_64qam[pack & 0x3f];
          }
          break;
        case L1_FEC_MODE_6:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          c3 = &l1temp[rows * 2];
          c4 = &l1temp[rows * 3];
          c5 = &l1temp[rows * 4];
          c6 = &l1temp[rows * 5];
          c7 = &l1temp[rows * 6];
          c8 = &l1temp[rows * 7];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1detail[index++] = c1[j];
            l1detail[index++] = c2[j];
            l1detail[index++] = c3[j];
            l1detail[index++] = c4[j];
            l1detail[index++] = c5[j];
            l1detail[index++] = c6[j];
            l1detail[index++] = c7[j];
            l1detail[index++] = c8[j];
          }
          index = count = 0;
          for (int j = 0; j < rows; j++) {
            pack = 0;
            for (int e = 7; e >= 0; e--) {
              pack |= l1detail[index++] << e;
            }
            temp = pack << count;
            pack = temp & 0xff;
            temp >>= 8;
            pack |= temp;
            count = (count + 1) & 0x7;
            *out++ = m_l1b_256qam[pack & 0xff];
          }
          break;
        case L1_FEC_MODE_7:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          c3 = &l1temp[rows * 2];
          c4 = &l1temp[rows * 3];
          c5 = &l1temp[rows * 4];
          c6 = &l1temp[rows * 5];
          c7 = &l1temp[rows * 6];
          c8 = &l1temp[rows * 7];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1detail[index++] = c1[j];
            l1detail[index++] = c2[j];
            l1detail[index++] = c3[j];
            l1detail[index++] = c4[j];
            l1detail[index++] = c5[j];
            l1detail[index++] = c6[j];
            l1detail[index++] = c7[j];
            l1detail[index++] = c8[j];
          }
          index = count = 0;
          for (int j = 0; j < rows; j++) {
            pack = 0;
            for (int e = 7; e >= 0; e--) {
              pack |= l1detail[index++] << e;
            }
            temp = pack << count;
            pack = temp & 0xff;
            temp >>= 8;
            pack |= temp;
            count = (count + 1) & 0x7;
            *out++ = m_l1b_256qam[pack & 0xff];
          }
          break;
        default:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1detail[index++] = c1[j];
            l1detail[index++] = c2[j];
          }
          index = 0;
          for (int j = 0; j < rows; j++) {
            temp = l1detail[index++] << 1;
            temp |= l1detail[index++];
            *out++ = m_qpsk[temp];
          }
          break;
      }
      return (numbits / mod);
    }

    int
    framemapper_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      int cells = 0;

      for (int i = 0; i < noutput_items; i += noutput_items) {
        printf("noutput_items = %d\n", noutput_items);
        cells += add_l1basic(&out[0]);
        cells += add_l1detail(&out[cells]);
        memcpy(&out[cells], &in[0], sizeof(gr_complex) * 450029);
      }

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    const int framemapper_cc_impl::shortening_table[8][18] = {
      {4, 1, 5, 2, 8, 6, 0, 7, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {7, 8, 5, 4, 1, 2, 6, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {6, 1, 7, 8, 0, 2, 4, 3, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 12, 15, 13, 2, 5, 7, 9, 8, 6, 16, 10, 14, 1, 17, 11, 4, 3},
      {0, 15, 5, 16, 17, 1, 6, 13, 11, 4, 7, 12, 8, 14, 2, 3, 9, 10},
      {2, 4, 5, 17, 9, 7, 1, 6, 15, 8, 10, 14, 16, 0, 11, 13, 12, 3},
      {0, 15, 5, 16, 17, 1, 6, 13, 11, 4, 7, 12, 8, 14, 2, 3, 9, 10},
      {15, 7, 8, 11, 5, 10, 16, 4, 12, 3, 0, 6, 9, 1, 14, 17, 2, 13}
    };

    const uint16_t framemapper_cc_impl::ldpc_tab_3_15S[12][12] = {
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

    const uint16_t framemapper_cc_impl::ldpc_tab_6_15S[18][31] = {
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

    const int framemapper_cc_impl::group_table[8][36] = {
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

    const gr_complex framemapper_cc_impl::mod_table_16QAM[4] = {
      gr_complex(0.2535, 0.4923), gr_complex(0.4923, 0.2535), gr_complex(0.4927, 1.2044), gr_complex(1.2044, 0.4927)
    };

    const gr_complex framemapper_cc_impl::mod_table_64QAM[16] = {
      gr_complex(0.1305, 0.3311), gr_complex(0.1633, 0.3162), gr_complex(0.1622, 0.7113), gr_complex(0.3905, 0.6163),
      gr_complex(0.3311, 0.1305), gr_complex(0.3162, 0.1633), gr_complex(0.7113, 0.1622), gr_complex(0.6163, 0.3905),
      gr_complex(0.2909, 1.4626), gr_complex(0.8285, 1.2399), gr_complex(0.2062, 1.0367), gr_complex(0.5872, 0.8789),
      gr_complex(1.4626, 0.2909), gr_complex(1.2399, 0.8285), gr_complex(1.0367, 0.2062), gr_complex(0.8789, 0.5872)
    };

    const gr_complex framemapper_cc_impl::mod_table_256QAM[2][64] = {
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

    const int framemapper_cc_impl::preamble_cells_table[32][5] = {
      {6432, 6342, 6253, 6164, 6075},
      {6000, 5916, 5833, 5750, 5667},
      {5712, 5632, 5553, 5474, 5395},
      {5136, 5064, 4993, 4922, 4851},
      {4560, 4496, 4433, 4370, 4307},
      {5136, 5064, 4993, 4922, 4851},
      {4560, 4496, 4433, 4370, 4307},
      {13296, 13110, 12927, 12742, 12558},
      {12864, 12684, 12507, 12328, 12150},
      {12576, 12400, 12227, 12052, 11878},
      {12000, 11832, 11667, 11500, 11334},
      {11424, 11264, 11107, 10948, 10790},
      {10272, 10128, 9987, 9844, 9702},
      {9120, 8992, 8867, 8740, 8614},
      {9120, 8992, 8867, 8740, 8614},
      {10272, 10128, 9987, 9844, 9702},
      {10272, 10128, 9987, 9844, 9702},
      {9120, 8992, 8867, 8740, 8614},
      {26592, 26220, 25854, 25484, 25116},
      {26592, 26220, 25854, 25484, 25116},
      {26304, 25936, 25574, 25208, 24844},
      {25728, 25368, 25014, 24656, 24300},
      {25152, 24800, 24454, 24104, 23756},
      {24000, 23664, 23334, 23000, 22668},
      {22848, 22528, 22214, 21896, 21580},
      {22848, 22528, 22214, 21896, 21580},
      {24000, 23664, 23334, 23000, 22668},
      {18240, 17984, 17734, 17480, 17228},
      {24000, 23664, 23334, 23000, 22668},
      {18240, 17984, 17734, 17480, 17228},
      {18240, 17984, 17734, 17480, 17228},
      {18240, 17984, 17734, 17480, 17228}
    };

    const int framemapper_cc_impl::data_cells_table_8K[16][5] = {
      {5711, 5631, 5552, 5473, 5394},
      {6285, 6197, 6110, 6023, 5936},
      {5999, 5915, 5832, 5749, 5666},
      {6429, 6339, 6250, 6161, 6072},
      {6287, 6199, 6112, 6025, 5938},
      {6573, 6481, 6390, 6299, 6208},
      {6431, 6341, 6252, 6163, 6074},
      {6645, 6552, 6460, 6368, 6276},
      {6575, 6483, 6392, 6301, 6210},
      {6717, 6623, 6530, 6437, 6344},
      {6647, 6554, 6462, 6370, 6278},
      {6753, 6660, 6565, 6473, 6378},
      {6719, 6625, 6532, 6439, 6346},
      {6789, 6694, 6600, 6506, 6412},
      {6755, 6661, 6567, 6474, 6380},
      {6807, 6714, 6619, 6524, 6429}
    };

    const int framemapper_cc_impl::data_cells_table_16K[16][5] = {
      {11423, 11263, 11106, 10947, 10789},
      {12573, 12397, 12224, 12049, 11875},
      {11999, 11831, 11666, 11499, 11333},
      {12861, 12681, 12504, 12325, 12147},
      {12575, 12399, 12226, 12051, 11877},
      {13149, 12965, 12784, 12601, 12419},
      {12863, 12683, 12506, 12327, 12149},
      {13293, 13107, 12924, 12739, 12555},
      {13151, 12967, 12786, 12603, 12421},
      {13437, 13249, 13064, 12877, 12691},
      {13295, 13109, 12926, 12741, 12557},
      {13509, 13320, 13134, 12946, 12759},
      {13439, 13251, 13066, 12879, 12693},
      {13581, 13391, 13204, 13015, 12827},
      {13511, 13322, 13136, 12948, 12761},
      {13617, 13428, 13239, 13051, 12861}
    };

    const int framemapper_cc_impl::data_cells_table_32K[16][5] = {
      {22847, 22527, 22213, 21895, 21579},
      {25149, 24797, 24451, 24101, 23753},
      {0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0},
      {25151, 24799, 24453, 24103, 23755},
      {26301, 25933, 25571, 25205, 24841},
      {25727, 25367, 25013, 24655, 24299},
      {26589, 26217, 25851, 25481, 25113},
      {26303, 25935, 25573, 25207, 24843},
      {26877, 26501, 26131, 25757, 25385},
      {26591, 26219, 25853, 25483, 25115},
      {27021, 26643, 26271, 25895, 25521},
      {26879, 26503, 26133, 25759, 25387},
      {27165, 26785, 26411, 26033, 25657},
      {27023, 26645, 26273, 25897, 25523},
      {27237, 26856, 26481, 26102, 25725}
    };

    const int framemapper_cc_impl::sbs_cells_table_8K[16][5] = {
      {4560, 4496, 4433, 4370, 4307},
      {4560, 4496, 4433, 4370, 4307},
      {5136, 5064, 4993, 4922, 4851},
      {5136, 5064, 4993, 4922, 4851},
      {5712, 5632, 5553, 5474, 5395},
      {5712, 5632, 5553, 5474, 5395},
      {6000, 5916, 5833, 5750, 5667},
      {6000, 5916, 5833, 5750, 5667},
      {6288, 6200, 6113, 6026, 5939},
      {6288, 6200, 6113, 6026, 5939},
      {6432, 6342, 6253, 6164, 6075},
      {6432, 6342, 6253, 6164, 6075},
      {6576, 6484, 6393, 6302, 6211},
      {6576, 6484, 6393, 6302, 6211},
      {6648, 6555, 6463, 6371, 6279},
      {6648, 6555, 6463, 6371, 6279},
    };

    const int framemapper_cc_impl::sbs_cells_table_16K[16][5] = {
      {9120, 8992, 8867, 8740, 8614},
      {9120, 8992, 8867, 8740, 8614},
      {10272, 10128, 9987, 9844, 9702},
      {10272, 10128, 9987, 9844, 9702},
      {11424, 11264, 11107, 10948, 10790},
      {11424, 11264, 11107, 10948, 10790},
      {12000, 11832, 11667, 11500, 11334},
      {12000, 11832, 11667, 11500, 11334},
      {12576, 12400, 12227, 12052, 11878},
      {12576, 12400, 12227, 12052, 11878},
      {12864, 12684, 12507, 12328, 12150},
      {12864, 12684, 12507, 12328, 12150},
      {13152, 12968, 12787, 12604, 12422},
      {13152, 12968, 12787, 12604, 12422},
      {13296, 13110, 12927, 12742, 12558},
      {13296, 13110, 12927, 12742, 12558}
    };

    const int framemapper_cc_impl::sbs_cells_table_32K[16][5] = {
      {18240, 17984, 17734, 17480, 17228},
      {18240, 17984, 17734, 17480, 17228},
      {0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0},
      {22848, 22528, 22214, 21896, 21580},
      {22848, 22528, 22214, 21896, 21580},
      {24000, 23664, 23334, 23000, 22668},
      {24000, 23664, 23334, 23000, 22668},
      {25152, 24800, 24454, 24104, 23756},
      {25152, 24800, 24454, 24104, 23756},
      {25728, 25368, 25014, 24656, 24300},
      {25728, 25368, 25014, 24656, 24300},
      {26304, 25936, 25574, 25208, 24844},
      {26304, 25936, 25574, 25208, 24844},
      {26592, 26220, 25854, 25484, 25116},
      {26592, 26220, 25854, 25484, 25116}
    };

  } /* namespace atsc3 */
} /* namespace gr */
