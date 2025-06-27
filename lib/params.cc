/* -*- c++ -*- */
/*
 * Copyright 2025 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "params.h"

namespace gr {
  namespace atsc3 {

    struct fec_params_t fec_params(atsc3_framesize_t framesize, atsc3_code_rate_t rate)
    {
      struct fec_params_t p;
      p.q_val = p.q1_val = p.q2_val = p.m1_val = p.m2_val = 0;
      if (framesize == FECFRAME_NORMAL) {
        switch (rate) {
          case C2_15:
            p.kbch = 8448;
            p.nbch = 8640;
            p.q1_val = 5;
            p.q2_val = 151;
            p.m1_val = 1800;
            p.m2_val = 54360;
            p.ldpc_type = LDPC_TYPE_A;
            p.rate_index = 0;
            break;
          case C3_15:
            p.kbch = 12768;
            p.nbch = 12960;
            p.q1_val = 5;
            p.q2_val = 139;
            p.m1_val = 1800;
            p.m2_val = 50040;
            p.ldpc_type = LDPC_TYPE_A;
            p.rate_index = 1;
            break;
          case C4_15:
            p.kbch = 17088;
            p.nbch = 17280;
            p.q1_val = 5;
            p.q2_val = 127;
            p.m1_val = 1800;
            p.m2_val = 45720;
            p.ldpc_type = LDPC_TYPE_A;
            p.rate_index = 2;
            break;
          case C5_15:
            p.kbch = 21408;
            p.nbch = 21600;
            p.q1_val = 4;
            p.q2_val = 116;
            p.m1_val = 1440;
            p.m2_val = 41760;
            p.ldpc_type = LDPC_TYPE_A;
            p.rate_index = 3;
            break;
          case C6_15:
            p.kbch = 25728;
            p.nbch = 25920;
            p.q_val = 108;
            p.ldpc_type = LDPC_TYPE_B;
            p.rate_index = 4;
            break;
          case C7_15:
            p.kbch = 30048;
            p.nbch = 30240;
            p.q1_val = 3;
            p.q2_val = 93;
            p.m1_val = 1080;
            p.m2_val = 33480;
            p.ldpc_type = LDPC_TYPE_A;
            p.rate_index = 5;
            break;
          case C8_15:
            p.kbch = 34368;
            p.nbch = 34560;
            p.q_val = 84;
            p.ldpc_type = LDPC_TYPE_B;
            p.rate_index = 6;
            break;
          case C9_15:
            p.kbch = 38688;
            p.nbch = 38880;
            p.q_val = 72;
            p.ldpc_type = LDPC_TYPE_B;
            p.rate_index = 7;
            break;
          case C10_15:
            p.kbch = 43008;
            p.nbch = 43200;
            p.q_val = 60;
            p.ldpc_type = LDPC_TYPE_B;
            p.rate_index = 8;
            break;
          case C11_15:
            p.kbch = 47328;
            p.nbch = 47520;
            p.q_val = 48;
            p.ldpc_type = LDPC_TYPE_B;
            p.rate_index = 9;
            break;
          case C12_15:
            p.kbch = 51648;
            p.nbch = 51840;
            p.q_val = 36;
            p.ldpc_type = LDPC_TYPE_B;
            p.rate_index = 10;
            break;
          case C13_15:
            p.kbch = 55968;
            p.nbch = 56160;
            p.q_val = 24;
            p.ldpc_type = LDPC_TYPE_B;
            p.rate_index = 11;
            break;
          default:
            p.nbch = 0;
            break;
        }
      }
      else if (framesize == FECFRAME_SHORT) {
        switch (rate) {
          case C2_15:
            p.kbch = 1992;
            p.nbch = 2160;
            p.q1_val = 9;
            p.q2_val = 30;
            p.m1_val = 3240;
            p.m2_val = 10800;
            p.ldpc_type = LDPC_TYPE_A;
            p.rate_index = 0;
            break;
          case C3_15:
            p.kbch = 3072;
            p.nbch = 3240;
            p.q1_val = 3;
            p.q2_val = 33;
            p.m1_val = 1080;
            p.m2_val = 11880;
            p.ldpc_type = LDPC_TYPE_A;
            p.rate_index = 1;
            break;
          case C4_15:
            p.kbch = 4152;
            p.nbch = 4320;
            p.q1_val = 3;
            p.q2_val = 30;
            p.m1_val = 1080;
            p.m2_val = 10800;
            p.ldpc_type = LDPC_TYPE_A;
            p.rate_index = 2;
            break;
          case C5_15:
            p.kbch = 5232;
            p.nbch = 5400;
            p.q1_val = 2;
            p.q2_val = 28;
            p.m1_val = 720;
            p.m2_val = 10080;
            p.ldpc_type = LDPC_TYPE_A;
            p.rate_index = 3;
            break;
          case C6_15:
            p.kbch = 6312;
            p.nbch = 6480;
            p.q_val = 27;
            p.ldpc_type = LDPC_TYPE_B;
            p.rate_index = 4;
            break;
          case C7_15:
            p.kbch = 7392;
            p.nbch = 7560;
            p.q_val = 24;
            p.ldpc_type = LDPC_TYPE_B;
            p.rate_index = 5;
            break;
          case C8_15:
            p.kbch = 8472;
            p.nbch = 8640;
            p.q_val = 21;
            p.ldpc_type = LDPC_TYPE_B;
            p.rate_index = 6;
            break;
          case C9_15:
            p.kbch = 9552;
            p.nbch = 9720;
            p.q_val = 18;
            p.ldpc_type = LDPC_TYPE_B;
            p.rate_index = 7;
            break;
          case C10_15:
            p.kbch = 10632;
            p.nbch = 10800;
            p.q_val = 15;
            p.ldpc_type = LDPC_TYPE_B;
            p.rate_index = 8;
            break;
          case C11_15:
            p.kbch = 11712;
            p.nbch = 11880;
            p.q_val = 12;
            p.ldpc_type = LDPC_TYPE_B;
            p.rate_index = 9;
            break;
          case C12_15:
            p.kbch = 12792;
            p.nbch = 12960;
            p.q_val = 9;
            p.ldpc_type = LDPC_TYPE_B;
            p.rate_index = 10;
            break;
          case C13_15:
            p.kbch = 13872;
            p.nbch = 14040;
            p.q_val = 6;
            p.ldpc_type = LDPC_TYPE_B;
            p.rate_index = 11;
            break;
          default:
            p.nbch = 0;
            break;
        }
      }
      return p;
    }

    struct ofdm_params_t ofdm_params(atsc3_fftsize_t fftsize, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_scattered_pilot_boost_t pilotboost, atsc3_reduced_carriers_t cred)
    {
      struct ofdm_params_t p;
      switch (fftsize) {
        case FFTSIZE_8K:
          p.fftsamples = 8192;
          p.papr_cells = 72;
          p.carriers = carriers_table[FFTSIZE_8K][cred];
          p.max_carriers = carriers_table[FFTSIZE_8K][0];
          p.preamble_carriers = carriers_table[FFTSIZE_8K][4];
          switch (guardinterval) {
            case GI_1_192:
              p.gisamples = 192;
              p.first_preamble_cells = preamble_cells_table[0][4];
              p.preamble_cells = preamble_cells_table[0][cred];
              p.preamble_dx = preamble_dx_table[0];
              p.preamble_power = preamble_power_table[0];
              p.preamble_ifft_power = preamble_ifft_power_table[0][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[0][4];
              break;
            case GI_2_384:
              p.gisamples = 384;
              p.first_preamble_cells = preamble_cells_table[1][4];
              p.preamble_cells = preamble_cells_table[1][cred];
              p.preamble_dx = preamble_dx_table[1];
              p.preamble_power = preamble_power_table[1];
              p.preamble_ifft_power = preamble_ifft_power_table[1][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[1][4];
              break;
            case GI_3_512:
              p.gisamples = 512;
              p.first_preamble_cells = preamble_cells_table[2][4];
              p.preamble_cells = preamble_cells_table[2][cred];
              p.preamble_dx = preamble_dx_table[2];
              p.preamble_power = preamble_power_table[2];
              p.preamble_ifft_power = preamble_ifft_power_table[2][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[2][4];
              break;
            case GI_4_768:
              p.gisamples = 768;
              p.first_preamble_cells = preamble_cells_table[3][4];
              p.preamble_cells = preamble_cells_table[3][cred];
              p.preamble_dx = preamble_dx_table[3];
              p.preamble_power = preamble_power_table[3];
              p.preamble_ifft_power = preamble_ifft_power_table[3][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[3][4];
              break;
            case GI_5_1024:
              p.gisamples = 1024;
              p.first_preamble_cells = preamble_cells_table[4][4];
              p.preamble_cells = preamble_cells_table[4][cred];
              p.preamble_dx = preamble_dx_table[4];
              p.preamble_power = preamble_power_table[4];
              p.preamble_ifft_power = preamble_ifft_power_table[4][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[4][4];
              break;
            case GI_6_1536:
              p.gisamples = 1536;
              p.first_preamble_cells = preamble_cells_table[5][4];
              p.preamble_cells = preamble_cells_table[5][cred];
              p.preamble_dx = preamble_dx_table[5];
              p.preamble_power = preamble_power_table[5];
              p.preamble_ifft_power = preamble_ifft_power_table[5][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[5][4];
              break;
            case GI_7_2048:
              p.gisamples = 2048;
              p.first_preamble_cells = preamble_cells_table[6][4];
              p.preamble_cells = preamble_cells_table[6][cred];
              p.preamble_dx = preamble_dx_table[6];
              p.preamble_power = preamble_power_table[6];
              p.preamble_ifft_power = preamble_ifft_power_table[6][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[6][4];
              break;
            default:
              p.gisamples = 192;
              p.first_preamble_cells = preamble_cells_table[0][4];
              p.preamble_cells = preamble_cells_table[0][cred];
              p.preamble_dx = preamble_dx_table[0];
              p.preamble_power = preamble_power_table[0];
              p.preamble_ifft_power = preamble_ifft_power_table[0][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[0][4];
              break;
          }
          switch (pilotpattern) {
            case PILOT_SP3_2:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP3_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP3_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP3_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP3_2][cred][pilotboost];
              break;
            case PILOT_SP3_4:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP3_4][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP3_4][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP3_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP3_4][cred][pilotboost];
              break;
            case PILOT_SP4_2:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP4_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP4_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP4_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP4_2][cred][pilotboost];
              break;
            case PILOT_SP4_4:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP4_4][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP4_4][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP4_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP4_4][cred][pilotboost];
              break;
            case PILOT_SP6_2:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP6_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP6_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP6_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP6_2][cred][pilotboost];
              break;
            case PILOT_SP6_4:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP6_4][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP6_4][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP6_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP6_4][cred][pilotboost];
              break;
            case PILOT_SP8_2:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP8_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP8_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP8_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP8_2][cred][pilotboost];
              break;
            case PILOT_SP8_4:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP8_4][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP8_4][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP8_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP8_4][cred][pilotboost];
              break;
            case PILOT_SP12_2:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP12_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP12_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP12_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP12_2][cred][pilotboost];
              break;
            case PILOT_SP12_4:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP12_4][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP12_4][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP12_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP12_4][cred][pilotboost];
              break;
            case PILOT_SP16_2:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP16_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP16_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP16_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP16_2][cred][pilotboost];
              break;
            case PILOT_SP16_4:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP16_4][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP16_4][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP16_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP16_4][cred][pilotboost];
              break;
            case PILOT_SP24_2:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP24_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP24_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP24_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP24_2][cred][pilotboost];
              break;
            case PILOT_SP24_4:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP24_4][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP24_4][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP24_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP24_4][cred][pilotboost];
              break;
            case PILOT_SP32_2:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP32_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP32_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP32_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP32_2][cred][pilotboost];
              break;
            case PILOT_SP32_4:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP32_4][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP32_4][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP32_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP32_4][cred][pilotboost];
              break;
            default:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP3_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP3_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP3_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP3_2][cred][pilotboost];
              break;
          }
          break;
        case FFTSIZE_16K:
          p.fftsamples = 16384;
          p.papr_cells = 144;
          p.carriers = carriers_table[FFTSIZE_16K][cred];
          p.max_carriers = carriers_table[FFTSIZE_16K][0];
          p.preamble_carriers = carriers_table[FFTSIZE_16K][4];
          switch (guardinterval) {
            case GI_1_192:
              p.gisamples = 192;
              p.first_preamble_cells = preamble_cells_table[7][4];
              p.preamble_cells = preamble_cells_table[7][cred];
              p.preamble_dx = preamble_dx_table[7];
              p.preamble_power = preamble_power_table[7];
              p.preamble_ifft_power = preamble_ifft_power_table[7][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[7][4];
              break;
            case GI_2_384:
              p.gisamples = 384;
              p.first_preamble_cells = preamble_cells_table[8][4];
              p.preamble_cells = preamble_cells_table[8][cred];
              p.preamble_dx = preamble_dx_table[8];
              p.preamble_power = preamble_power_table[8];
              p.preamble_ifft_power = preamble_ifft_power_table[8][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[8][4];
              break;
            case GI_3_512:
              p.gisamples = 512;
              p.first_preamble_cells = preamble_cells_table[9][4];
              p.preamble_cells = preamble_cells_table[9][cred];
              p.preamble_dx = preamble_dx_table[9];
              p.preamble_power = preamble_power_table[9];
              p.preamble_ifft_power = preamble_ifft_power_table[9][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[9][4];
              break;
            case GI_4_768:
              p.gisamples = 768;
              p.first_preamble_cells = preamble_cells_table[10][4];
              p.preamble_cells = preamble_cells_table[10][cred];
              p.preamble_dx = preamble_dx_table[10];
              p.preamble_power = preamble_power_table[10];
              p.preamble_ifft_power = preamble_ifft_power_table[10][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[10][4];
              break;
            case GI_5_1024:
              p.gisamples = 1024;
              p.first_preamble_cells = preamble_cells_table[11][4];
              p.preamble_cells = preamble_cells_table[11][cred];
              p.preamble_dx = preamble_dx_table[11];
              p.preamble_power = preamble_power_table[11];
              p.preamble_ifft_power = preamble_ifft_power_table[11][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[11][4];
              break;
            case GI_6_1536:
              p.gisamples = 1536;
              p.first_preamble_cells = preamble_cells_table[12][4];
              p.preamble_cells = preamble_cells_table[12][cred];
              p.preamble_dx = preamble_dx_table[12];
              p.preamble_power = preamble_power_table[12];
              p.preamble_ifft_power = preamble_ifft_power_table[12][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[12][4];
              break;
            case GI_7_2048:
              p.gisamples = 2048;
              p.first_preamble_cells = preamble_cells_table[13][4];
              p.preamble_cells = preamble_cells_table[13][cred];
              p.preamble_dx = preamble_dx_table[13];
              p.preamble_power = preamble_power_table[13];
              p.preamble_ifft_power = preamble_ifft_power_table[13][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[13][4];
              break;
            case GI_8_2432:
              p.gisamples = 2432;
              p.first_preamble_cells = preamble_cells_table[14][4];
              p.preamble_cells = preamble_cells_table[14][cred];
              p.preamble_dx = preamble_dx_table[14];
              p.preamble_power = preamble_power_table[14];
              p.preamble_ifft_power = preamble_ifft_power_table[14][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[14][4];
              break;
            case GI_9_3072:
              p.gisamples = 3072;
              p.first_preamble_cells = preamble_cells_table[15][4];
              p.preamble_cells = preamble_cells_table[15][cred];
              p.preamble_dx = preamble_dx_table[15];
              p.preamble_power = preamble_power_table[15];
              p.preamble_ifft_power = preamble_ifft_power_table[15][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[15][4];
              break;
            case GI_10_3648:
              p.gisamples = 3648;
              p.first_preamble_cells = preamble_cells_table[16][4];
              p.preamble_cells = preamble_cells_table[16][cred];
              p.preamble_dx = preamble_dx_table[16];
              p.preamble_power = preamble_power_table[16];
              p.preamble_ifft_power = preamble_ifft_power_table[16][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[16][4];
              break;
            case GI_11_4096:
              p.gisamples = 4096;
              p.first_preamble_cells = preamble_cells_table[17][4];
              p.preamble_cells = preamble_cells_table[17][cred];
              p.preamble_dx = preamble_dx_table[17];
              p.preamble_power = preamble_power_table[17];
              p.preamble_ifft_power = preamble_ifft_power_table[17][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[17][4];
              break;
            default:
              p.gisamples = 192;
              p.first_preamble_cells = preamble_cells_table[7][4];
              p.preamble_cells = preamble_cells_table[7][cred];
              p.preamble_dx = preamble_dx_table[7];
              p.preamble_power = preamble_power_table[7];
              p.preamble_ifft_power = preamble_ifft_power_table[7][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[7][4];
              break;
          }
          switch (pilotpattern) {
            case PILOT_SP3_2:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP3_2][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP3_2][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP3_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP3_2][cred][pilotboost];
              break;
            case PILOT_SP3_4:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP3_4][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP3_4][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP3_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP3_4][cred][pilotboost];
              break;
            case PILOT_SP4_2:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP4_2][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP4_2][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP4_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP4_2][cred][pilotboost];
              break;
            case PILOT_SP4_4:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP4_4][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP4_4][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP4_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP4_4][cred][pilotboost];
              break;
            case PILOT_SP6_2:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP6_2][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP6_2][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP6_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP6_2][cred][pilotboost];
              break;
            case PILOT_SP6_4:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP6_4][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP6_4][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP6_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP6_4][cred][pilotboost];
              break;
            case PILOT_SP8_2:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP8_2][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP8_2][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP8_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP8_2][cred][pilotboost];
              break;
            case PILOT_SP8_4:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP8_4][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP8_4][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP8_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP8_4][cred][pilotboost];
              break;
            case PILOT_SP12_2:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP12_2][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP12_2][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP12_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP12_2][cred][pilotboost];
              break;
            case PILOT_SP12_4:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP12_4][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP12_4][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP12_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP12_4][cred][pilotboost];
              break;
            case PILOT_SP16_2:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP16_2][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP16_2][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP16_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP16_2][cred][pilotboost];
              break;
            case PILOT_SP16_4:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP16_4][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP16_4][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP16_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP16_4][cred][pilotboost];
              break;
            case PILOT_SP24_2:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP24_2][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP24_2][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP24_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP24_2][cred][pilotboost];
              break;
            case PILOT_SP24_4:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP24_4][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP24_4][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP24_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP24_4][cred][pilotboost];
              break;
            case PILOT_SP32_2:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP32_2][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP32_2][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP32_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP32_2][cred][pilotboost];
              break;
            case PILOT_SP32_4:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP32_4][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP32_4][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP32_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP32_4][cred][pilotboost];
              break;
            default:
              p.data_ifft_power = data_ifft_power_table_16K[PILOT_SP3_2][cred][pilotboost];
              p.data_cells = data_cells_table_16K[PILOT_SP3_2][cred];
              p.sbs_cells = sbs_cells_table_16K[PILOT_SP3_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_16K[PILOT_SP3_2][cred][pilotboost];
              break;
          }
          break;
        case FFTSIZE_32K:
          p.fftsamples = 32768;
          p.papr_cells = 288;
          p.carriers = carriers_table[FFTSIZE_32K][cred];
          p.max_carriers = carriers_table[FFTSIZE_32K][0];
          p.preamble_carriers = carriers_table[FFTSIZE_32K][4];
          switch (guardinterval) {
            case GI_1_192:
              p.gisamples = 192;
              p.first_preamble_cells = preamble_cells_table[18][4];
              p.preamble_cells = preamble_cells_table[18][cred];
              p.preamble_dx = preamble_dx_table[18];
              p.preamble_power = preamble_power_table[18];
              p.preamble_ifft_power = preamble_ifft_power_table[18][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[18][4];
              break;
            case GI_2_384:
              p.gisamples = 384;
              p.first_preamble_cells = preamble_cells_table[19][4];
              p.preamble_cells = preamble_cells_table[19][cred];
              p.preamble_dx = preamble_dx_table[19];
              p.preamble_power = preamble_power_table[19];
              p.preamble_ifft_power = preamble_ifft_power_table[19][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[19][4];
              break;
            case GI_3_512:
              p.gisamples = 512;
              p.first_preamble_cells = preamble_cells_table[20][4];
              p.preamble_cells = preamble_cells_table[20][cred];
              p.preamble_dx = preamble_dx_table[20];
              p.preamble_power = preamble_power_table[20];
              p.preamble_ifft_power = preamble_ifft_power_table[20][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[20][4];
              break;
            case GI_4_768:
              p.gisamples = 768;
              p.first_preamble_cells = preamble_cells_table[21][4];
              p.preamble_cells = preamble_cells_table[21][cred];
              p.preamble_dx = preamble_dx_table[21];
              p.preamble_power = preamble_power_table[21];
              p.preamble_ifft_power = preamble_ifft_power_table[21][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[21][4];
              break;
            case GI_5_1024:
              p.gisamples = 1024;
              p.first_preamble_cells = preamble_cells_table[22][4];
              p.preamble_cells = preamble_cells_table[22][cred];
              p.preamble_dx = preamble_dx_table[22];
              p.preamble_power = preamble_power_table[22];
              p.preamble_ifft_power = preamble_ifft_power_table[22][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[22][4];
              break;
            case GI_6_1536:
              p.gisamples = 1536;
              p.first_preamble_cells = preamble_cells_table[23][4];
              p.preamble_cells = preamble_cells_table[23][cred];
              p.preamble_dx = preamble_dx_table[23];
              p.preamble_power = preamble_power_table[23];
              p.preamble_ifft_power = preamble_ifft_power_table[23][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[23][4];
              break;
            case GI_7_2048:
              p.gisamples = 2048;
              p.first_preamble_cells = preamble_cells_table[24][4];
              p.preamble_cells = preamble_cells_table[24][cred];
              p.preamble_dx = preamble_dx_table[24];
              p.preamble_power = preamble_power_table[24];
              p.preamble_ifft_power = preamble_ifft_power_table[24][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[24][4];
              break;
            case GI_8_2432:
              p.gisamples = 2432;
              p.first_preamble_cells = preamble_cells_table[25][4];
              p.preamble_cells = preamble_cells_table[25][cred];
              p.preamble_dx = preamble_dx_table[25];
              p.preamble_power = preamble_power_table[25];
              p.preamble_ifft_power = preamble_ifft_power_table[25][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[25][4];
              break;
            case GI_9_3072:
              p.gisamples = 3072;
              if (pilotpattern == PILOT_SP8_2 || pilotpattern == PILOT_SP8_4) {
                p.first_preamble_cells = preamble_cells_table[26][4];
                p.preamble_cells = preamble_cells_table[26][cred];
                p.preamble_dx = preamble_dx_table[26];
                p.preamble_power = preamble_power_table[26];
                p.preamble_ifft_power = preamble_ifft_power_table[26][cred];
                p.first_preamble_ifft_power = preamble_ifft_power_table[26][4];
              }
              else {
                p.first_preamble_cells = preamble_cells_table[27][4];
                p.preamble_cells = preamble_cells_table[27][cred];
                p.preamble_dx = preamble_dx_table[27];
                p.preamble_power = preamble_power_table[27];
                p.preamble_ifft_power = preamble_ifft_power_table[27][cred];
                p.first_preamble_ifft_power = preamble_ifft_power_table[27][4];
              }
              break;
            case GI_10_3648:
              p.gisamples = 3648;
              if (pilotpattern == PILOT_SP8_2 || pilotpattern == PILOT_SP8_4) {
                p.first_preamble_cells = preamble_cells_table[28][4];
                p.preamble_cells = preamble_cells_table[28][cred];
                p.preamble_dx = preamble_dx_table[28];
                p.preamble_power = preamble_power_table[28];
                p.preamble_ifft_power = preamble_ifft_power_table[28][cred];
                p.first_preamble_ifft_power = preamble_ifft_power_table[28][4];
              }
              else {
                p.first_preamble_cells = preamble_cells_table[29][4];
                p.preamble_cells = preamble_cells_table[29][cred];
                p.preamble_dx = preamble_dx_table[29];
                p.preamble_power = preamble_power_table[29];
                p.preamble_ifft_power = preamble_ifft_power_table[29][cred];
                p.first_preamble_ifft_power = preamble_ifft_power_table[29][4];
              }
              break;
            case GI_11_4096:
              p.gisamples = 4096;
              p.first_preamble_cells = preamble_cells_table[30][4];
              p.preamble_cells = preamble_cells_table[30][cred];
              p.preamble_dx = preamble_dx_table[30];
              p.preamble_power = preamble_power_table[30];
              p.preamble_ifft_power = preamble_ifft_power_table[30][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[30][4];
              break;
            case GI_12_4864:
              p.gisamples = 4864;
              p.first_preamble_cells = preamble_cells_table[31][4];
              p.preamble_cells = preamble_cells_table[31][cred];
              p.preamble_dx = preamble_dx_table[31];
              p.preamble_power = preamble_power_table[31];
              p.preamble_ifft_power = preamble_ifft_power_table[31][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[31][4];
              break;
            default:
              p.gisamples = 192;
              p.first_preamble_cells = preamble_cells_table[18][4];
              p.preamble_cells = preamble_cells_table[18][cred];
              p.preamble_dx = preamble_dx_table[18];
              p.preamble_power = preamble_power_table[18];
              p.preamble_ifft_power = preamble_ifft_power_table[18][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[18][4];
              break;
          }
          switch (pilotpattern) {
            case PILOT_SP3_2:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP3_2][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP3_2][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP3_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP3_2][cred][pilotboost];
              break;
            case PILOT_SP3_4:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP3_4][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP3_4][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP3_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP3_4][cred][pilotboost];
              break;
            case PILOT_SP4_2:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP4_2][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP4_2][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP4_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP4_2][cred][pilotboost];
              break;
            case PILOT_SP4_4:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP4_4][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP4_4][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP4_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP4_4][cred][pilotboost];
              break;
            case PILOT_SP6_2:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP6_2][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP6_2][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP6_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP6_2][cred][pilotboost];
              break;
            case PILOT_SP6_4:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP6_4][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP6_4][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP6_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP6_4][cred][pilotboost];
              break;
            case PILOT_SP8_2:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP8_2][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP8_2][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP8_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP8_2][cred][pilotboost];
              break;
            case PILOT_SP8_4:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP8_4][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP8_4][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP8_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP8_4][cred][pilotboost];
              break;
            case PILOT_SP12_2:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP12_2][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP12_2][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP12_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP12_2][cred][pilotboost];
              break;
            case PILOT_SP12_4:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP12_4][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP12_4][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP12_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP12_4][cred][pilotboost];
              break;
            case PILOT_SP16_2:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP16_2][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP16_2][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP16_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP16_2][cred][pilotboost];
              break;
            case PILOT_SP16_4:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP16_4][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP16_4][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP16_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP16_4][cred][pilotboost];
              break;
            case PILOT_SP24_2:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP24_2][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP24_2][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP24_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP24_2][cred][pilotboost];
              break;
            case PILOT_SP24_4:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP24_4][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP24_4][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP24_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP24_4][cred][pilotboost];
              break;
            case PILOT_SP32_2:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP32_2][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP32_2][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP32_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP32_2][cred][pilotboost];
              break;
            case PILOT_SP32_4:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP32_4][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP32_4][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP32_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP32_4][cred][pilotboost];
              break;
            default:
              p.data_ifft_power = data_ifft_power_table_32K[PILOT_SP3_2][cred][pilotboost];
              p.data_cells = data_cells_table_32K[PILOT_SP3_2][cred];
              p.sbs_cells = sbs_cells_table_32K[PILOT_SP3_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_32K[PILOT_SP3_2][cred][pilotboost];
              break;
          }
          break;
        default:
          p.papr_cells = 72;
          p.fftsamples = 8192;
          p.carriers = carriers_table[FFTSIZE_8K][cred];
          p.max_carriers = carriers_table[FFTSIZE_8K][0];
          p.preamble_carriers = carriers_table[FFTSIZE_8K][4];
          switch (guardinterval) {
            case GI_1_192:
              p.gisamples = 192;
              p.first_preamble_cells = preamble_cells_table[0][4];
              p.preamble_cells = preamble_cells_table[0][cred];
              p.preamble_dx = preamble_dx_table[0];
              p.preamble_power = preamble_power_table[0];
              p.preamble_ifft_power = preamble_ifft_power_table[0][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[0][4];
              break;
            case GI_2_384:
              p.gisamples = 384;
              p.first_preamble_cells = preamble_cells_table[1][4];
              p.preamble_cells = preamble_cells_table[1][cred];
              p.preamble_dx = preamble_dx_table[1];
              p.preamble_power = preamble_power_table[1];
              p.preamble_ifft_power = preamble_ifft_power_table[1][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[1][4];
              break;
            case GI_3_512:
              p.gisamples = 512;
              p.first_preamble_cells = preamble_cells_table[2][4];
              p.preamble_cells = preamble_cells_table[2][cred];
              p.preamble_dx = preamble_dx_table[2];
              p.preamble_power = preamble_power_table[2];
              p.preamble_ifft_power = preamble_ifft_power_table[2][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[2][4];
              break;
            case GI_4_768:
              p.gisamples = 768;
              p.first_preamble_cells = preamble_cells_table[3][4];
              p.preamble_cells = preamble_cells_table[3][cred];
              p.preamble_dx = preamble_dx_table[3];
              p.preamble_power = preamble_power_table[3];
              p.preamble_ifft_power = preamble_ifft_power_table[3][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[3][4];
              break;
            case GI_5_1024:
              p.gisamples = 1024;
              p.first_preamble_cells = preamble_cells_table[4][4];
              p.preamble_cells = preamble_cells_table[4][cred];
              p.preamble_dx = preamble_dx_table[4];
              p.preamble_power = preamble_power_table[4];
              p.preamble_ifft_power = preamble_ifft_power_table[4][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[4][4];
              break;
            case GI_6_1536:
              p.gisamples = 1536;
              p.first_preamble_cells = preamble_cells_table[5][4];
              p.preamble_cells = preamble_cells_table[5][cred];
              p.preamble_dx = preamble_dx_table[5];
              p.preamble_power = preamble_power_table[5];
              p.preamble_ifft_power = preamble_ifft_power_table[5][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[5][4];
              break;
            case GI_7_2048:
              p.gisamples = 2048;
              p.first_preamble_cells = preamble_cells_table[6][4];
              p.preamble_cells = preamble_cells_table[6][cred];
              p.preamble_dx = preamble_dx_table[6];
              p.preamble_power = preamble_power_table[6];
              p.preamble_ifft_power = preamble_ifft_power_table[6][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[6][4];
              break;
            default:
              p.gisamples = 192;
              p.first_preamble_cells = preamble_cells_table[0][4];
              p.preamble_cells = preamble_cells_table[0][cred];
              p.preamble_dx = preamble_dx_table[0];
              p.preamble_power = preamble_power_table[0];
              p.preamble_ifft_power = preamble_ifft_power_table[0][cred];
              p.first_preamble_ifft_power = preamble_ifft_power_table[0][4];
              break;
          }
          switch (pilotpattern) {
            case PILOT_SP3_2:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP3_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP3_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP3_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP3_2][cred][pilotboost];
              break;
            case PILOT_SP3_4:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP3_4][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP3_4][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP3_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP3_4][cred][pilotboost];
              break;
            case PILOT_SP4_2:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP4_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP4_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP4_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP4_2][cred][pilotboost];
              break;
            case PILOT_SP4_4:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP4_4][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP4_4][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP4_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP4_4][cred][pilotboost];
              break;
            case PILOT_SP6_2:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP6_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP6_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP6_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP6_2][cred][pilotboost];
              break;
            case PILOT_SP6_4:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP6_4][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP6_4][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP6_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP6_4][cred][pilotboost];
              break;
            case PILOT_SP8_2:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP8_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP8_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP8_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP8_2][cred][pilotboost];
              break;
            case PILOT_SP8_4:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP8_4][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP8_4][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP8_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP8_4][cred][pilotboost];
              break;
            case PILOT_SP12_2:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP12_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP12_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP12_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP12_2][cred][pilotboost];
              break;
            case PILOT_SP12_4:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP12_4][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP12_4][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP12_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP12_4][cred][pilotboost];
              break;
            case PILOT_SP16_2:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP16_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP16_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP16_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP16_2][cred][pilotboost];
              break;
            case PILOT_SP16_4:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP16_4][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP16_4][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP16_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP16_4][cred][pilotboost];
              break;
            case PILOT_SP24_2:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP24_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP24_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP24_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP24_2][cred][pilotboost];
              break;
            case PILOT_SP24_4:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP24_4][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP24_4][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP24_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP24_4][cred][pilotboost];
              break;
            case PILOT_SP32_2:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP32_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP32_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP32_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP32_2][cred][pilotboost];
              break;
            case PILOT_SP32_4:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP32_4][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP32_4][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP32_4][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP32_4][cred][pilotboost];
              break;
            default:
              p.data_ifft_power = data_ifft_power_table_8K[PILOT_SP3_2][cred][pilotboost];
              p.data_cells = data_cells_table_8K[PILOT_SP3_2][cred];
              p.sbs_cells = sbs_cells_table_8K[PILOT_SP3_2][cred];
              p.sbs_data_cells = sbs_data_cells_table_8K[PILOT_SP3_2][cred][pilotboost];
              break;
          }
          break;
      }
      return p;
    }

  } /* namespace atsc3 */
} /* namespace gr */
