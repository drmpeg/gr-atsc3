/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "pilotgenerator_cc_impl.h"

namespace gr {
  namespace atsc3 {

    using input_type = gr_complex;
    using output_type = gr_complex;
    pilotgenerator_cc::sptr
    pilotgenerator_cc::make(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation, atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, int plpsize, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_scattered_pilot_boost_t pilotboost, atsc3_first_sbs_t firstsbs, atsc3_l1_fec_mode_t l1bmode, atsc3_l1_fec_mode_t l1dmode)
    {
      return gnuradio::make_block_sptr<pilotgenerator_cc_impl>(
        framesize, rate, constellation, fftsize, numpayloadsyms, numpreamblesyms, plpsize, guardinterval, pilotpattern, pilotboost, firstsbs, l1bmode, l1dmode);
    }


    /*
     * The private constructor
     */
    pilotgenerator_cc_impl::pilotgenerator_cc_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation, atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, int plpsize, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_scattered_pilot_boost_t pilotboost, atsc3_first_sbs_t firstsbs, atsc3_l1_fec_mode_t l1bmode, atsc3_l1_fec_mode_t l1dmode)
      : gr::block("pilotgenerator_cc",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      int fftsamples, gisamples, cred = CRED_0;
      double power, preamble_power, scattered_power;
      int total_preamble_cells, totalcells;
      int first_preamble_cells;
      int preamble_cells;
      int data_cells;
      int sbs_cells;

      fft_size = fftsize;
      symbols = numpreamblesyms + numpayloadsyms;
      preamble_symbols = numpreamblesyms;
      switch (fftsize) {
        case FFTSIZE_8K:
          fftsamples = 8192;
          first_preamble_cells = 4307;
          carriers = carriers_table[FFTSIZE_8K][cred];
          max_carriers = carriers_table[FFTSIZE_8K][0];
          preamble_carriers = carriers_table[FFTSIZE_8K][4];
          switch (guardinterval) {
            case GI_1_192:
              gisamples = 192;
              preamble_cells = preamble_cells_table[0][cred];
              preamble_dx = preamble_dx_table[0];
              preamble_power = preamble_power_table[0];
              break;
            case GI_2_384:
              gisamples = 384;
              preamble_cells = preamble_cells_table[1][cred];
              preamble_dx = preamble_dx_table[1];
              preamble_power = preamble_power_table[1];
              break;
            case GI_3_512:
              gisamples = 512;
              preamble_cells = preamble_cells_table[2][cred];
              preamble_dx = preamble_dx_table[2];
              preamble_power = preamble_power_table[2];
              break;
            case GI_4_768:
              gisamples = 768;
              preamble_cells = preamble_cells_table[3][cred];
              preamble_dx = preamble_dx_table[3];
              preamble_power = preamble_power_table[3];
              break;
            case GI_5_1024:
              gisamples = 1024;
              preamble_cells = preamble_cells_table[4][cred];
              preamble_dx = preamble_dx_table[4];
              preamble_power = preamble_power_table[4];
              break;
            case GI_6_1536:
              gisamples = 1536;
              preamble_cells = preamble_cells_table[5][cred];
              preamble_dx = preamble_dx_table[5];
              preamble_power = preamble_power_table[5];
              break;
            case GI_7_2048:
              gisamples = 2048;
              preamble_cells = preamble_cells_table[6][cred];
              preamble_dx = preamble_dx_table[6];
              preamble_power = preamble_power_table[6];
              break;
            default:
              gisamples = 192;
              preamble_cells = preamble_cells_table[0][cred];
              preamble_dx = preamble_dx_table[0];
              preamble_power = preamble_power_table[0];
              break;
          }
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
          fftsamples = 16384;
          first_preamble_cells = 8614;
          carriers = carriers_table[FFTSIZE_16K][cred];
          max_carriers = carriers_table[FFTSIZE_16K][0];
          preamble_carriers = carriers_table[FFTSIZE_16K][4];
          switch (guardinterval) {
            case GI_1_192:
              gisamples = 192;
              preamble_cells = preamble_cells_table[7][cred];
              preamble_dx = preamble_dx_table[7];
              preamble_power = preamble_power_table[7];
              break;
            case GI_2_384:
              gisamples = 384;
              preamble_cells = preamble_cells_table[8][cred];
              preamble_dx = preamble_dx_table[8];
              preamble_power = preamble_power_table[8];
              break;
            case GI_3_512:
              gisamples = 512;
              preamble_cells = preamble_cells_table[9][cred];
              preamble_dx = preamble_dx_table[9];
              preamble_power = preamble_power_table[9];
              break;
            case GI_4_768:
              gisamples = 768;
              preamble_cells = preamble_cells_table[10][cred];
              preamble_dx = preamble_dx_table[10];
              preamble_power = preamble_power_table[10];
              break;
            case GI_5_1024:
              gisamples = 1024;
              preamble_cells = preamble_cells_table[11][cred];
              preamble_dx = preamble_dx_table[11];
              preamble_power = preamble_power_table[11];
              break;
            case GI_6_1536:
              gisamples = 1536;
              preamble_cells = preamble_cells_table[12][cred];
              preamble_dx = preamble_dx_table[12];
              preamble_power = preamble_power_table[12];
              break;
            case GI_7_2048:
              gisamples = 2048;
              preamble_cells = preamble_cells_table[13][cred];
              preamble_dx = preamble_dx_table[13];
              preamble_power = preamble_power_table[13];
              break;
            case GI_8_2432:
              gisamples = 2432;
              preamble_cells = preamble_cells_table[14][cred];
              preamble_dx = preamble_dx_table[14];
              preamble_power = preamble_power_table[14];
              break;
            case GI_9_3072:
              gisamples = 3072;
              preamble_cells = preamble_cells_table[15][cred];
              preamble_dx = preamble_dx_table[15];
              preamble_power = preamble_power_table[15];
              break;
            case GI_10_3648:
              gisamples = 3648;
              preamble_cells = preamble_cells_table[16][cred];
              preamble_dx = preamble_dx_table[16];
              preamble_power = preamble_power_table[16];
              break;
            case GI_11_4096:
              gisamples = 4096;
              preamble_cells = preamble_cells_table[17][cred];
              preamble_dx = preamble_dx_table[17];
              preamble_power = preamble_power_table[17];
              break;
            default:
              gisamples = 192;
              preamble_cells = preamble_cells_table[7][cred];
              preamble_dx = preamble_dx_table[7];
              preamble_power = preamble_power_table[7];
              break;
          }
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
          fftsamples = 32768;
          first_preamble_cells = 17288;
          carriers = carriers_table[FFTSIZE_32K][cred];
          max_carriers = carriers_table[FFTSIZE_32K][0];
          preamble_carriers = carriers_table[FFTSIZE_32K][4];
          switch (guardinterval) {
            case GI_1_192:
              gisamples = 192;
              preamble_cells = preamble_cells_table[18][cred];
              preamble_dx = preamble_dx_table[18];
              preamble_power = preamble_power_table[18];
              break;
            case GI_2_384:
              gisamples = 384;
              preamble_cells = preamble_cells_table[19][cred];
              preamble_dx = preamble_dx_table[19];
              preamble_power = preamble_power_table[19];
              break;
            case GI_3_512:
              gisamples = 512;
              preamble_cells = preamble_cells_table[20][cred];
              preamble_dx = preamble_dx_table[20];
              preamble_power = preamble_power_table[20];
              break;
            case GI_4_768:
              gisamples = 768;
              preamble_cells = preamble_cells_table[21][cred];
              preamble_dx = preamble_dx_table[21];
              preamble_power = preamble_power_table[21];
              break;
            case GI_5_1024:
              gisamples = 1024;
              preamble_cells = preamble_cells_table[22][cred];
              preamble_dx = preamble_dx_table[22];
              preamble_power = preamble_power_table[22];
              break;
            case GI_6_1536:
              gisamples = 1536;
              preamble_cells = preamble_cells_table[23][cred];
              preamble_dx = preamble_dx_table[23];
              preamble_power = preamble_power_table[23];
              break;
            case GI_7_2048:
              gisamples = 2048;
              preamble_cells = preamble_cells_table[24][cred];
              preamble_dx = preamble_dx_table[24];
              preamble_power = preamble_power_table[24];
              break;
            case GI_8_2432:
              gisamples = 2432;
              preamble_cells = preamble_cells_table[25][cred];
              preamble_dx = preamble_dx_table[25];
              preamble_power = preamble_power_table[25];
              break;
            case GI_9_3072:
              gisamples = 3072;
              if (pilotpattern == PILOT_SP8_2 || pilotpattern == PILOT_SP8_4) {
                preamble_cells = preamble_cells_table[26][cred];
                preamble_dx = preamble_dx_table[26];
                preamble_power = preamble_power_table[26];
              }
              else {
                preamble_cells = preamble_cells_table[27][cred];
                preamble_dx = preamble_dx_table[27];
                preamble_power = preamble_power_table[27];
              }
              break;
            case GI_10_3648:
              gisamples = 3648;
              if (pilotpattern == PILOT_SP8_2 || pilotpattern == PILOT_SP8_4) {
                preamble_cells = preamble_cells_table[28][cred];
                preamble_dx = preamble_dx_table[28];
                preamble_power = preamble_power_table[28];
              }
              else {
                preamble_cells = preamble_cells_table[29][cred];
                preamble_dx = preamble_dx_table[29];
                preamble_power = preamble_power_table[29];
              }
              break;
            case GI_11_4096:
              gisamples = 4096;
              preamble_cells = preamble_cells_table[30][cred];
              preamble_dx = preamble_dx_table[30];
              preamble_power = preamble_power_table[30];
              break;
            case GI_12_4864:
              gisamples = 4864;
              preamble_cells = preamble_cells_table[31][cred];
              preamble_dx = preamble_dx_table[31];
              preamble_power = preamble_power_table[31];
              break;
            default:
              gisamples = 192;
              preamble_cells = preamble_cells_table[18][cred];
              preamble_dx = preamble_dx_table[18];
              preamble_power = preamble_power_table[18];
              break;
          }
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
          fftsamples = 8192;
          first_preamble_cells = 4307;
          carriers = carriers_table[FFTSIZE_8K][cred];
          max_carriers = carriers_table[FFTSIZE_8K][0];
          preamble_carriers = carriers_table[FFTSIZE_8K][4];
          switch (guardinterval) {
            case GI_1_192:
              gisamples = 192;
              preamble_cells = preamble_cells_table[0][cred];
              preamble_dx = preamble_dx_table[0];
              preamble_power = preamble_power_table[0];
              break;
            case GI_2_384:
              gisamples = 384;
              preamble_cells = preamble_cells_table[1][cred];
              preamble_dx = preamble_dx_table[1];
              preamble_power = preamble_power_table[1];
              break;
            case GI_3_512:
              gisamples = 512;
              preamble_cells = preamble_cells_table[2][cred];
              preamble_dx = preamble_dx_table[2];
              preamble_power = preamble_power_table[2];
              break;
            case GI_4_768:
              gisamples = 768;
              preamble_cells = preamble_cells_table[3][cred];
              preamble_dx = preamble_dx_table[3];
              preamble_power = preamble_power_table[3];
              break;
            case GI_5_1024:
              gisamples = 1024;
              preamble_cells = preamble_cells_table[4][cred];
              preamble_dx = preamble_dx_table[4];
              preamble_power = preamble_power_table[4];
              break;
            case GI_6_1536:
              gisamples = 1536;
              preamble_cells = preamble_cells_table[5][cred];
              preamble_dx = preamble_dx_table[5];
              preamble_power = preamble_power_table[5];
              break;
            case GI_7_2048:
              gisamples = 2048;
              preamble_cells = preamble_cells_table[6][cred];
              preamble_dx = preamble_dx_table[6];
              preamble_power = preamble_power_table[6];
              break;
            default:
              gisamples = 192;
              preamble_cells = preamble_cells_table[0][cred];
              preamble_dx = preamble_dx_table[0];
              preamble_power = preamble_power_table[0];
              break;
          }
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
      }
      switch (pilotpattern) {
        case PILOT_SP3_2:
          dx = 3;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP3_2][pilotboost];
          break;
        case PILOT_SP3_4:
          dx = 3;
          dy = 4;
          scattered_power = scattered_power_table[PILOT_SP3_4][pilotboost];
          break;
        case PILOT_SP4_2:
          dx = 4;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP4_2][pilotboost];
          break;
        case PILOT_SP4_4:
          dx = 4;
          dy = 4;
          scattered_power = scattered_power_table[PILOT_SP4_4][pilotboost];
          break;
        case PILOT_SP6_2:
          dx = 6;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP6_2][pilotboost];
          break;
        case PILOT_SP6_4:
          dx = 6;
          dy = 4;
          scattered_power = scattered_power_table[PILOT_SP6_4][pilotboost];
          break;
        case PILOT_SP8_2:
          dx = 8;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP8_2][pilotboost];
          break;
        case PILOT_SP8_4:
          dx = 8;
          dy = 4;
          scattered_power = scattered_power_table[PILOT_SP8_4][pilotboost];
          break;
        case PILOT_SP12_2:
          dx = 12;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP12_2][pilotboost];
          break;
        case PILOT_SP12_4:
          dx = 12;
          dy = 4;
          scattered_power = scattered_power_table[PILOT_SP12_4][pilotboost];
          break;
        case PILOT_SP16_2:
          dx = 16;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP16_2][pilotboost];
          break;
        case PILOT_SP16_4:
          dx = 16;
          dy = 4;
          scattered_power = scattered_power_table[PILOT_SP16_4][pilotboost];
          break;
        case PILOT_SP24_2:
          dx = 24;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP24_2][pilotboost];
          break;
        case PILOT_SP24_4:
          dx = 24;
          dy = 4;
          scattered_power = scattered_power_table[PILOT_SP24_4][pilotboost];
          break;
        case PILOT_SP32_2:
          dx = 32;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP32_2][pilotboost];
          break;
        case PILOT_SP32_4:
          dx = 32;
          dy = 4;
          scattered_power = scattered_power_table[PILOT_SP32_4][pilotboost];
          break;
        default:
          dx = 3;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP3_2][pilotboost];
          break;
      }
      power = pow(10, preamble_power / 20.0);
      printf("preamble power = %f\n", power);
      pr_bpsk[0] = gr_complex(power, 0.0);
      pr_bpsk[1] = gr_complex(-(power), 0.0);
      power = pow(10, scattered_power / 20.0);
      printf("scattered power = %f\n", power);
      sp_bpsk[0] = gr_complex(power, 0.0);
      sp_bpsk[1] = gr_complex(-(power), 0.0);
      power = pow(10, 8.52 / 20.0);
      printf("continual power = %f\n", power);
      cp_bpsk[0] = gr_complex(power, 0.0);
      cp_bpsk[1] = gr_complex(-(power), 0.0);
      init_prbs();
      frame_symbols[0] = PREAMBLE_SYMBOL;
      total_preamble_cells = 0;
      for (int n = 1; n < numpreamblesyms; n++) {
        frame_symbols[n] = PREAMBLE_SYMBOL;
        total_preamble_cells += preamble_cells;
      }
      if (firstsbs == TRUE) {
        frame_symbols[numpreamblesyms] = SBS_SYMBOL;
        for (int n = 0; n < numpayloadsyms; n++) {
          frame_symbols[n + numpreamblesyms + 1] = DATA_SYMBOL;
        }
      }
      else {
        for (int n = 0; n < numpayloadsyms; n++) {
          frame_symbols[n + numpreamblesyms] = DATA_SYMBOL;
        }
      }
      frame_symbols[numpreamblesyms + numpayloadsyms - 1] = SBS_SYMBOL;
      data_carrier_map.resize(symbols);
      for (std::vector<std::vector<int>>::size_type i = 0; i != data_carrier_map.size(); i++) {
        data_carrier_map[i].resize(max_carriers);
      }
      init_pilots();
      if (firstsbs) {
        totalcells = first_preamble_cells + total_preamble_cells + ((numpayloadsyms - 2) * data_cells) + (sbs_cells * 2);
      }
      else {
        totalcells = first_preamble_cells + total_preamble_cells + ((numpayloadsyms - 1) * data_cells) + sbs_cells;
      }
      input_cells = totalcells;
      set_output_multiple((carriers * (symbols - 1)) + preamble_carriers);
    }

    /*
     * Our virtual destructor.
     */
    pilotgenerator_cc_impl::~pilotgenerator_cc_impl()
    {
    }

    void
    pilotgenerator_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = input_cells;
    }

    void
    pilotgenerator_cc_impl::init_prbs(void)
    {
      int sr = 0x1b;

      for (int i = 0; i < MAX_CARRIERS; i++) {
        int b = ((sr) ^ (sr >> 1) ^ (sr >> 3) ^ (sr >> 4)) & 1;
        prbs[i] = sr & 1;
        sr >>= 1;
        if (b) {
          sr |= 0x1000;
        }
      }
    }

    void
    pilotgenerator_cc_impl::init_pilots()
    {
      for (int symbol = 0; symbol < symbols; ++symbol) {
        int remainder, shift, index, preamblecarriers;
        std::vector<int>& data_carrier_map = this->data_carrier_map[symbol];
        for (int i = 0; i < carriers; i++) {
          data_carrier_map[i] = DATA_CARRIER;
        }
        if (frame_symbols[symbol] == PREAMBLE_SYMBOL) {
          index = 0;
          if (symbol == 0) {
            preamblecarriers = preamble_carriers;
            shift = (max_carriers - preamble_carriers) / 2;
          }
          else {
            preamblecarriers = carriers;
            shift = 0;
          }
          for (int i = 0; i < max_carriers; i++) {
            if (continual_pilot_table_8K[index] == i) {
              if (continual_pilot_table_8K[index] > shift) {
                data_carrier_map[i - shift] = CONTINUAL_CARRIER;
              }
              index++;
            }
          }
          for (int i = 0; i < preamblecarriers; i++) {
            if ((i % preamble_dx) == 0) {
              data_carrier_map[i] = PREAMBLE_CARRIER;
            }
          }
        }
        else if (frame_symbols[symbol] == SBS_SYMBOL) {
          index = shift = 0;
          for (int i = 0; i < max_carriers; i++) {
            if (continual_pilot_table_8K[index] == i) {
              if (continual_pilot_table_8K[index] > shift) {
                data_carrier_map[i] = CONTINUAL_CARRIER;
              }
              index++;
            }
          }
          for (int i = 0; i < carriers; i++) {
            if ((i % dx) == 0) {
              data_carrier_map[i] = SCATTERED_CARRIER;
            }
          }
          data_carrier_map[0] = SCATTERED_CARRIER;
          data_carrier_map[carriers - 1] = SCATTERED_CARRIER;
          data_carrier_map[1731] = SCATTERED_CARRIER; /* fix me */
          data_carrier_map[2886] = SCATTERED_CARRIER;
          data_carrier_map[5733] = SCATTERED_CARRIER;
        }
        else {
          index = shift = 0;
          for (int i = 0; i < max_carriers; i++) {
            if (continual_pilot_table_8K[index] == i) {
              if (continual_pilot_table_8K[index] > shift) {
                data_carrier_map[i] = CONTINUAL_CARRIER;
              }
              index++;
            }
          }
          for (int i = 0; i < carriers; i++) {
            remainder = i % (dx * dy);
            if (remainder == (dx * ((symbol - preamble_symbols) % dy))) {
              data_carrier_map[i] = SCATTERED_CARRIER;
            }
          }
          data_carrier_map[0] = SCATTERED_CARRIER;
          data_carrier_map[carriers - 1] = SCATTERED_CARRIER;
          data_carrier_map[1731] = SCATTERED_CARRIER; /* fix me */
          data_carrier_map[2886] = SCATTERED_CARRIER;
          data_carrier_map[5733] = SCATTERED_CARRIER;
        }
      }
    }

    int
    pilotgenerator_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      int indexin = 0;
      int indexout = 0;
      int preamblecarriers;

      for (int i = 0; i < noutput_items; i += (carriers * (symbols - 1)) + preamble_carriers) {
        for (int j = 0; j < symbols; j++) {
          if (frame_symbols[j] == PREAMBLE_SYMBOL) {
            if (j == 0) {
              preamblecarriers = preamble_carriers;
            }
            else {
              preamblecarriers = carriers;
            }
            for (int n = 0; n < preamblecarriers; n++) {
              if (data_carrier_map[j][n] == PREAMBLE_CARRIER) {
                out[indexout++] = pr_bpsk[prbs[n]];
              }
              else if (data_carrier_map[j][n] == CONTINUAL_CARRIER) {
                out[indexout++] = cp_bpsk[prbs[n]];
              }
              else {
                out[indexout++] = in[indexin++];
              }
            }
          }
          else if (frame_symbols[j] == SBS_SYMBOL) {
            for (int n = 0; n < carriers; n++) {
              if (data_carrier_map[j][n] == SCATTERED_CARRIER) {
                out[indexout++] = sp_bpsk[prbs[n]];
              }
              else if (data_carrier_map[j][n] == CONTINUAL_CARRIER) {
                out[indexout++] = cp_bpsk[prbs[n]];
              }
              else {
                out[indexout++] = in[indexin++];
              }
            }
          }
          else {
            for (int n = 0; n < carriers; n++) {
              if (data_carrier_map[j][n] == SCATTERED_CARRIER) {
                out[indexout++] = sp_bpsk[prbs[n]];
              }
              else if (data_carrier_map[j][n] == CONTINUAL_CARRIER) {
                out[indexout++] = cp_bpsk[prbs[n]];
              }
              else {
                out[indexout++] = in[indexin++];
              }
            }
          }
        }
      }

      printf("indexin = %d, indexout = %d\n", indexin, indexout);
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (input_cells);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    const int pilotgenerator_cc_impl::carriers_table[3][5] = {
      {6913, 6817, 6721, 6625, 6529},
      {13825, 13633, 13441, 13249, 13057},
      {27649, 27265, 26881, 26497, 26113}
    };

    const int pilotgenerator_cc_impl::preamble_dx_table[32] = {
      16, 8, 6, 4, 3, 4, 3, 32, 16, 12, 8, 6, 4, 3, 3, 4,
      4, 3, 32, 32, 24, 16, 12, 8, 6, 6, 8, 3, 8, 3, 3, 3
    };

    const double pilotgenerator_cc_impl::preamble_power_table[32] = {
      5.30, 3.60, 2.90, 1.80, 0.90, 1.80, 0.90, 6.80, 5.30, 4.60, 3.60, 2.90, 2.10, 1.30, 1.30, 2.10,
      2.10, 1.30, 6.80, 6.80, 6.20, 5.30, 4.60, 4.00, 3.20, 3.20, 4.00, 1.30, 4.00, 1.30, 1.30, 1.30
    };

    const double pilotgenerator_cc_impl::scattered_power_table[16][5] = {
      {0.00 , 0.00 , 1.40 , 2.20 , 2.90},
      {0.00 , 1.40 , 2.90 , 3.80 , 4.40},
      {0.00 , 0.60 , 2.10 , 3.00 , 3.60},
      {0.00 , 2.10 , 3.60 , 4.40 , 5.10},
      {0.00 , 1.60 , 3.10 , 4.00 , 4.60},
      {0.00 , 3.00 , 4.50 , 5.40 , 6.00},
      {0.00 , 2.20 , 3.80 , 4.60 , 5.30},
      {0.00 , 3.60 , 5.10 , 6.00 , 6.60},
      {0.00 , 3.20 , 4.70 , 5.60 , 6.20},
      {0.00 , 4.50 , 6.00 , 6.90 , 7.50},
      {0.00 , 3.80 , 5.30 , 6.20 , 6.80},
      {0.00 , 5.20 , 6.70 , 7.60 , 8.20},
      {0.00 , 4.70 , 6.20 , 7.10 , 7.70},
      {0.00 , 6.10 , 7.60 , 8.50 , 9.10},
      {0.00 , 5.40 , 6.90 , 7.70 , 8.40},
      {0.00 , 6.70 , 8.20 , 9.10 , 9.70}
    };

    const int pilotgenerator_cc_impl::continual_pilot_table_8K[48] = {
      59, 167, 307, 469, 637, 751, 865, 1031, 1159, 1333, 1447, 1607, 1811, 1943, 2041, 2197,
      2323, 2519, 2605, 2767, 2963, 3029, 3175, 3325, 3467, 3665, 3833, 3901, 4073, 4235, 4325, 4511,
      4627, 4825, 4907, 5051, 5227, 5389, 5531, 5627, 5833, 5905, 6053, 6197, 6353, 6563, 6637, 6809
    };

    const int pilotgenerator_cc_impl::preamble_cells_table[32][5] = {
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

    const int pilotgenerator_cc_impl::data_cells_table_8K[16][5] = {
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

    const int pilotgenerator_cc_impl::data_cells_table_16K[16][5] = {
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

    const int pilotgenerator_cc_impl::data_cells_table_32K[16][5] = {
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

    const int pilotgenerator_cc_impl::sbs_cells_table_8K[16][5] = {
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

    const int pilotgenerator_cc_impl::sbs_cells_table_16K[16][5] = {
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

    const int pilotgenerator_cc_impl::sbs_cells_table_32K[16][5] = {
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
