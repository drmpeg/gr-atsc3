/* -*- c++ -*- */
/*
 * Copyright 2021-2023 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "pilotgenerator_cc_impl.h"
#include <volk/volk.h>

namespace gr {
  namespace atsc3 {

    using input_type = gr_complex;
    using output_type = gr_complex;
    pilotgenerator_cc::sptr
    pilotgenerator_cc::make(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_scattered_pilot_boost_t pilotboost, atsc3_first_sbs_t firstsbs, atsc3_first_sbs_t lastsbs, atsc3_reduced_carriers_t cred, atsc3_miso_t misomode, atsc3_papr_t paprmode, atsc3_pilotgenerator_mode_t outputmode, unsigned int fftlength, unsigned int vlength)
    {
      return gnuradio::make_block_sptr<pilotgenerator_cc_impl>(
        fftsize, numpayloadsyms, numpreamblesyms, guardinterval, pilotpattern, pilotboost, firstsbs, lastsbs, cred, misomode, paprmode, outputmode, fftlength, vlength);
    }


    /*
     * The private constructor
     */
    pilotgenerator_cc_impl::pilotgenerator_cc_impl(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_scattered_pilot_boost_t pilotboost, atsc3_first_sbs_t firstsbs, atsc3_first_sbs_t lastsbs, atsc3_reduced_carriers_t cred, atsc3_miso_t misomode, atsc3_papr_t paprmode, atsc3_pilotgenerator_mode_t outputmode, unsigned int fftlength, unsigned int vlength)
      : gr::block("pilotgenerator_cc",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type) * vlength)),
        ofdm_fft(fftlength, 1),
        miso_fft(fftlength, 1),
        ofdm_fft_size(fftlength)
    {
      double power, preamble_power, scattered_power;
      double preamble_ifft_power, data_ifft_power;
      double first_preamble_ifft_power;
      int total_preamble_cells, totalcells;
      int first_preamble_cells;
      int preamble_cells;
      int data_cells;
      int sbs_cells;
      int papr_cells;

      fft_size = fftsize;
      pilot_pattern = pilotpattern;
      papr_mode = paprmode;
      miso_mode = misomode;
      output_mode = outputmode;
      cred_coeff = cred;
      symbols = numpreamblesyms + numpayloadsyms;
      preamble_symbols = numpreamblesyms;
      switch (fftsize) {
        case FFTSIZE_8K:
          papr_cells = 72;
          carriers = carriers_table[FFTSIZE_8K][cred];
          max_carriers = carriers_table[FFTSIZE_8K][0];
          preamble_carriers = carriers_table[FFTSIZE_8K][4];
          switch (guardinterval) {
            case GI_1_192:
              first_preamble_cells = preamble_cells_table[0][4];
              preamble_cells = preamble_cells_table[0][cred];
              preamble_dx = preamble_dx_table[0];
              preamble_power = preamble_power_table[0];
              preamble_ifft_power = preamble_ifft_power_table[0][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[0][0];
              break;
            case GI_2_384:
              first_preamble_cells = preamble_cells_table[1][4];
              preamble_cells = preamble_cells_table[1][cred];
              preamble_dx = preamble_dx_table[1];
              preamble_power = preamble_power_table[1];
              preamble_ifft_power = preamble_ifft_power_table[1][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[1][0];
              break;
            case GI_3_512:
              first_preamble_cells = preamble_cells_table[2][4];
              preamble_cells = preamble_cells_table[2][cred];
              preamble_dx = preamble_dx_table[2];
              preamble_power = preamble_power_table[2];
              preamble_ifft_power = preamble_ifft_power_table[2][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[2][0];
              break;
            case GI_4_768:
              first_preamble_cells = preamble_cells_table[3][4];
              preamble_cells = preamble_cells_table[3][cred];
              preamble_dx = preamble_dx_table[3];
              preamble_power = preamble_power_table[3];
              preamble_ifft_power = preamble_ifft_power_table[3][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[3][0];
              break;
            case GI_5_1024:
              first_preamble_cells = preamble_cells_table[4][4];
              preamble_cells = preamble_cells_table[4][cred];
              preamble_dx = preamble_dx_table[4];
              preamble_power = preamble_power_table[4];
              preamble_ifft_power = preamble_ifft_power_table[4][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[4][0];
              break;
            case GI_6_1536:
              first_preamble_cells = preamble_cells_table[5][4];
              preamble_cells = preamble_cells_table[5][cred];
              preamble_dx = preamble_dx_table[5];
              preamble_power = preamble_power_table[5];
              preamble_ifft_power = preamble_ifft_power_table[5][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[5][0];
              break;
            case GI_7_2048:
              first_preamble_cells = preamble_cells_table[6][4];
              preamble_cells = preamble_cells_table[6][cred];
              preamble_dx = preamble_dx_table[6];
              preamble_power = preamble_power_table[6];
              preamble_ifft_power = preamble_ifft_power_table[6][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[6][0];
              break;
            default:
              first_preamble_cells = preamble_cells_table[0][4];
              preamble_cells = preamble_cells_table[0][cred];
              preamble_dx = preamble_dx_table[0];
              preamble_power = preamble_power_table[0];
              preamble_ifft_power = preamble_ifft_power_table[0][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[0][0];
              break;
          }
          switch (pilotpattern) {
            case PILOT_SP3_2:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP3_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP3_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP3_2][cred];
              break;
            case PILOT_SP3_4:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP3_4][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP3_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP3_4][cred];
              break;
            case PILOT_SP4_2:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP4_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP4_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP4_2][cred];
              break;
            case PILOT_SP4_4:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP4_4][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP4_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP4_4][cred];
              break;
            case PILOT_SP6_2:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP6_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP6_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP6_2][cred];
              break;
            case PILOT_SP6_4:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP6_4][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP6_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP6_4][cred];
              break;
            case PILOT_SP8_2:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP8_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP8_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP8_2][cred];
              break;
            case PILOT_SP8_4:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP8_4][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP8_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP8_4][cred];
              break;
            case PILOT_SP12_2:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP12_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP12_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP12_2][cred];
              break;
            case PILOT_SP12_4:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP12_4][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP12_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP12_4][cred];
              break;
            case PILOT_SP16_2:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP16_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP16_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP16_2][cred];
              break;
            case PILOT_SP16_4:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP16_4][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP16_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP16_4][cred];
              break;
            case PILOT_SP24_2:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP24_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP24_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP24_2][cred];
              break;
            case PILOT_SP24_4:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP24_4][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP24_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP24_4][cred];
              break;
            case PILOT_SP32_2:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP32_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP32_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP32_2][cred];
              break;
            case PILOT_SP32_4:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP32_4][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP32_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP32_4][cred];
              break;
            default:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP3_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP3_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP3_2][cred];
              break;
          }
          break;
        case FFTSIZE_16K:
          papr_cells = 144;
          carriers = carriers_table[FFTSIZE_16K][cred];
          max_carriers = carriers_table[FFTSIZE_16K][0];
          preamble_carriers = carriers_table[FFTSIZE_16K][4];
          switch (guardinterval) {
            case GI_1_192:
              first_preamble_cells = preamble_cells_table[7][4];
              preamble_cells = preamble_cells_table[7][cred];
              preamble_dx = preamble_dx_table[7];
              preamble_power = preamble_power_table[7];
              preamble_ifft_power = preamble_ifft_power_table[7][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[7][0];
              break;
            case GI_2_384:
              first_preamble_cells = preamble_cells_table[8][4];
              preamble_cells = preamble_cells_table[8][cred];
              preamble_dx = preamble_dx_table[8];
              preamble_power = preamble_power_table[8];
              preamble_ifft_power = preamble_ifft_power_table[8][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[8][0];
              break;
            case GI_3_512:
              first_preamble_cells = preamble_cells_table[9][4];
              preamble_cells = preamble_cells_table[9][cred];
              preamble_dx = preamble_dx_table[9];
              preamble_power = preamble_power_table[9];
              preamble_ifft_power = preamble_ifft_power_table[9][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[9][0];
              break;
            case GI_4_768:
              first_preamble_cells = preamble_cells_table[10][4];
              preamble_cells = preamble_cells_table[10][cred];
              preamble_dx = preamble_dx_table[10];
              preamble_power = preamble_power_table[10];
              preamble_ifft_power = preamble_ifft_power_table[10][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[10][0];
              break;
            case GI_5_1024:
              first_preamble_cells = preamble_cells_table[11][4];
              preamble_cells = preamble_cells_table[11][cred];
              preamble_dx = preamble_dx_table[11];
              preamble_power = preamble_power_table[11];
              preamble_ifft_power = preamble_ifft_power_table[11][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[11][0];
              break;
            case GI_6_1536:
              first_preamble_cells = preamble_cells_table[12][4];
              preamble_cells = preamble_cells_table[12][cred];
              preamble_dx = preamble_dx_table[12];
              preamble_power = preamble_power_table[12];
              preamble_ifft_power = preamble_ifft_power_table[12][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[12][0];
              break;
            case GI_7_2048:
              first_preamble_cells = preamble_cells_table[13][4];
              preamble_cells = preamble_cells_table[13][cred];
              preamble_dx = preamble_dx_table[13];
              preamble_power = preamble_power_table[13];
              preamble_ifft_power = preamble_ifft_power_table[13][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[13][0];
              break;
            case GI_8_2432:
              first_preamble_cells = preamble_cells_table[14][4];
              preamble_cells = preamble_cells_table[14][cred];
              preamble_dx = preamble_dx_table[14];
              preamble_power = preamble_power_table[14];
              preamble_ifft_power = preamble_ifft_power_table[14][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[14][0];
              break;
            case GI_9_3072:
              first_preamble_cells = preamble_cells_table[15][4];
              preamble_cells = preamble_cells_table[15][cred];
              preamble_dx = preamble_dx_table[15];
              preamble_power = preamble_power_table[15];
              preamble_ifft_power = preamble_ifft_power_table[15][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[15][0];
              break;
            case GI_10_3648:
              first_preamble_cells = preamble_cells_table[16][4];
              preamble_cells = preamble_cells_table[16][cred];
              preamble_dx = preamble_dx_table[16];
              preamble_power = preamble_power_table[16];
              preamble_ifft_power = preamble_ifft_power_table[16][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[16][0];
              break;
            case GI_11_4096:
              first_preamble_cells = preamble_cells_table[17][4];
              preamble_cells = preamble_cells_table[17][cred];
              preamble_dx = preamble_dx_table[17];
              preamble_power = preamble_power_table[17];
              preamble_ifft_power = preamble_ifft_power_table[17][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[17][0];
              break;
            default:
              first_preamble_cells = preamble_cells_table[7][4];
              preamble_cells = preamble_cells_table[7][cred];
              preamble_dx = preamble_dx_table[7];
              preamble_power = preamble_power_table[7];
              preamble_ifft_power = preamble_ifft_power_table[7][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[7][0];
              break;
          }
          switch (pilotpattern) {
            case PILOT_SP3_2:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP3_2][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP3_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP3_2][cred];
              break;
            case PILOT_SP3_4:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP3_4][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP3_4][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP3_4][cred];
              break;
            case PILOT_SP4_2:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP4_2][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP4_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP4_2][cred];
              break;
            case PILOT_SP4_4:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP4_4][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP4_4][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP4_4][cred];
              break;
            case PILOT_SP6_2:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP6_2][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP6_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP6_2][cred];
              break;
            case PILOT_SP6_4:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP6_4][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP6_4][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP6_4][cred];
              break;
            case PILOT_SP8_2:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP8_2][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP8_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP8_2][cred];
              break;
            case PILOT_SP8_4:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP8_4][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP8_4][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP8_4][cred];
              break;
            case PILOT_SP12_2:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP12_2][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP12_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP12_2][cred];
              break;
            case PILOT_SP12_4:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP12_4][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP12_4][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP12_4][cred];
              break;
            case PILOT_SP16_2:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP16_2][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP16_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP16_2][cred];
              break;
            case PILOT_SP16_4:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP16_4][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP16_4][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP16_4][cred];
              break;
            case PILOT_SP24_2:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP24_2][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP24_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP24_2][cred];
              break;
            case PILOT_SP24_4:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP24_4][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP24_4][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP24_4][cred];
              break;
            case PILOT_SP32_2:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP32_2][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP32_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP32_2][cred];
              break;
            case PILOT_SP32_4:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP32_4][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP32_4][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP32_4][cred];
              break;
            default:
              data_ifft_power = data_ifft_power_table_16K[PILOT_SP3_2][cred][pilotboost];
              data_cells = data_cells_table_16K[PILOT_SP3_2][cred];
              sbs_cells = sbs_cells_table_16K[PILOT_SP3_2][cred];
              break;
          }
          break;
        case FFTSIZE_32K:
          papr_cells = 288;
          carriers = carriers_table[FFTSIZE_32K][cred];
          max_carriers = carriers_table[FFTSIZE_32K][0];
          preamble_carriers = carriers_table[FFTSIZE_32K][4];
          switch (guardinterval) {
            case GI_1_192:
              first_preamble_cells = preamble_cells_table[18][4];
              preamble_cells = preamble_cells_table[18][cred];
              preamble_dx = preamble_dx_table[18];
              preamble_power = preamble_power_table[18];
              preamble_ifft_power = preamble_ifft_power_table[18][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[18][0];
              break;
            case GI_2_384:
              first_preamble_cells = preamble_cells_table[19][4];
              preamble_cells = preamble_cells_table[19][cred];
              preamble_dx = preamble_dx_table[19];
              preamble_power = preamble_power_table[19];
              preamble_ifft_power = preamble_ifft_power_table[19][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[19][0];
              break;
            case GI_3_512:
              first_preamble_cells = preamble_cells_table[20][4];
              preamble_cells = preamble_cells_table[20][cred];
              preamble_dx = preamble_dx_table[20];
              preamble_power = preamble_power_table[20];
              preamble_ifft_power = preamble_ifft_power_table[20][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[20][0];
              break;
            case GI_4_768:
              first_preamble_cells = preamble_cells_table[21][4];
              preamble_cells = preamble_cells_table[21][cred];
              preamble_dx = preamble_dx_table[21];
              preamble_power = preamble_power_table[21];
              preamble_ifft_power = preamble_ifft_power_table[21][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[21][0];
              break;
            case GI_5_1024:
              first_preamble_cells = preamble_cells_table[22][4];
              preamble_cells = preamble_cells_table[22][cred];
              preamble_dx = preamble_dx_table[22];
              preamble_power = preamble_power_table[22];
              preamble_ifft_power = preamble_ifft_power_table[22][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[22][0];
              break;
            case GI_6_1536:
              first_preamble_cells = preamble_cells_table[23][4];
              preamble_cells = preamble_cells_table[23][cred];
              preamble_dx = preamble_dx_table[23];
              preamble_power = preamble_power_table[23];
              preamble_ifft_power = preamble_ifft_power_table[23][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[23][0];
              break;
            case GI_7_2048:
              first_preamble_cells = preamble_cells_table[24][4];
              preamble_cells = preamble_cells_table[24][cred];
              preamble_dx = preamble_dx_table[24];
              preamble_power = preamble_power_table[24];
              preamble_ifft_power = preamble_ifft_power_table[24][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[24][0];
              break;
            case GI_8_2432:
              first_preamble_cells = preamble_cells_table[25][4];
              preamble_cells = preamble_cells_table[25][cred];
              preamble_dx = preamble_dx_table[25];
              preamble_power = preamble_power_table[25];
              preamble_ifft_power = preamble_ifft_power_table[25][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[25][0];
              break;
            case GI_9_3072:
              if (pilotpattern == PILOT_SP8_2 || pilotpattern == PILOT_SP8_4) {
                first_preamble_cells = preamble_cells_table[26][4];
                preamble_cells = preamble_cells_table[26][cred];
                preamble_dx = preamble_dx_table[26];
                preamble_power = preamble_power_table[26];
                preamble_ifft_power = preamble_ifft_power_table[26][cred];
                first_preamble_ifft_power = preamble_ifft_power_table[26][0];
              }
              else {
                first_preamble_cells = preamble_cells_table[27][4];
                preamble_cells = preamble_cells_table[27][cred];
                preamble_dx = preamble_dx_table[27];
                preamble_power = preamble_power_table[27];
                preamble_ifft_power = preamble_ifft_power_table[27][cred];
                first_preamble_ifft_power = preamble_ifft_power_table[27][0];
              }
              break;
            case GI_10_3648:
              if (pilotpattern == PILOT_SP8_2 || pilotpattern == PILOT_SP8_4) {
                first_preamble_cells = preamble_cells_table[28][4];
                preamble_cells = preamble_cells_table[28][cred];
                preamble_dx = preamble_dx_table[28];
                preamble_power = preamble_power_table[28];
                preamble_ifft_power = preamble_ifft_power_table[28][cred];
                first_preamble_ifft_power = preamble_ifft_power_table[28][0];
              }
              else {
                first_preamble_cells = preamble_cells_table[29][4];
                preamble_cells = preamble_cells_table[29][cred];
                preamble_dx = preamble_dx_table[29];
                preamble_power = preamble_power_table[29];
                preamble_ifft_power = preamble_ifft_power_table[29][cred];
                first_preamble_ifft_power = preamble_ifft_power_table[29][0];
              }
              break;
            case GI_11_4096:
              first_preamble_cells = preamble_cells_table[30][4];
              preamble_cells = preamble_cells_table[30][cred];
              preamble_dx = preamble_dx_table[30];
              preamble_power = preamble_power_table[30];
              preamble_ifft_power = preamble_ifft_power_table[30][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[30][0];
              break;
            case GI_12_4864:
              first_preamble_cells = preamble_cells_table[31][4];
              preamble_cells = preamble_cells_table[31][cred];
              preamble_dx = preamble_dx_table[31];
              preamble_power = preamble_power_table[31];
              preamble_ifft_power = preamble_ifft_power_table[31][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[31][0];
              break;
            default:
              first_preamble_cells = preamble_cells_table[18][4];
              preamble_cells = preamble_cells_table[18][cred];
              preamble_dx = preamble_dx_table[18];
              preamble_power = preamble_power_table[18];
              preamble_ifft_power = preamble_ifft_power_table[18][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[18][0];
              break;
          }
          switch (pilotpattern) {
            case PILOT_SP3_2:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP3_2][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP3_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP3_2][cred];
              break;
            case PILOT_SP3_4:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP3_4][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP3_4][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP3_4][cred];
              break;
            case PILOT_SP4_2:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP4_2][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP4_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP4_2][cred];
              break;
            case PILOT_SP4_4:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP4_4][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP4_4][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP4_4][cred];
              break;
            case PILOT_SP6_2:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP6_2][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP6_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP6_2][cred];
              break;
            case PILOT_SP6_4:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP6_4][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP6_4][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP6_4][cred];
              break;
            case PILOT_SP8_2:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP8_2][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP8_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP8_2][cred];
              break;
            case PILOT_SP8_4:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP8_4][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP8_4][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP8_4][cred];
              break;
            case PILOT_SP12_2:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP12_2][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP12_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP12_2][cred];
              break;
            case PILOT_SP12_4:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP12_4][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP12_4][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP12_4][cred];
              break;
            case PILOT_SP16_2:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP16_2][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP16_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP16_2][cred];
              break;
            case PILOT_SP16_4:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP16_4][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP16_4][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP16_4][cred];
              break;
            case PILOT_SP24_2:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP24_2][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP24_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP24_2][cred];
              break;
            case PILOT_SP24_4:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP24_4][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP24_4][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP24_4][cred];
              break;
            case PILOT_SP32_2:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP32_2][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP32_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP32_2][cred];
              break;
            case PILOT_SP32_4:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP32_4][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP32_4][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP32_4][cred];
              break;
            default:
              data_ifft_power = data_ifft_power_table_32K[PILOT_SP3_2][cred][pilotboost];
              data_cells = data_cells_table_32K[PILOT_SP3_2][cred];
              sbs_cells = sbs_cells_table_32K[PILOT_SP3_2][cred];
              break;
          }
          break;
        default:
          papr_cells = 72;
          carriers = carriers_table[FFTSIZE_8K][cred];
          max_carriers = carriers_table[FFTSIZE_8K][0];
          preamble_carriers = carriers_table[FFTSIZE_8K][4];
          switch (guardinterval) {
            case GI_1_192:
              first_preamble_cells = preamble_cells_table[0][4];
              preamble_cells = preamble_cells_table[0][cred];
              preamble_dx = preamble_dx_table[0];
              preamble_power = preamble_power_table[0];
              preamble_ifft_power = preamble_ifft_power_table[0][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[0][0];
              break;
            case GI_2_384:
              first_preamble_cells = preamble_cells_table[1][4];
              preamble_cells = preamble_cells_table[1][cred];
              preamble_dx = preamble_dx_table[1];
              preamble_power = preamble_power_table[1];
              preamble_ifft_power = preamble_ifft_power_table[1][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[1][0];
              break;
            case GI_3_512:
              first_preamble_cells = preamble_cells_table[2][4];
              preamble_cells = preamble_cells_table[2][cred];
              preamble_dx = preamble_dx_table[2];
              preamble_power = preamble_power_table[2];
              preamble_ifft_power = preamble_ifft_power_table[2][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[2][0];
              break;
            case GI_4_768:
              first_preamble_cells = preamble_cells_table[3][4];
              preamble_cells = preamble_cells_table[3][cred];
              preamble_dx = preamble_dx_table[3];
              preamble_power = preamble_power_table[3];
              preamble_ifft_power = preamble_ifft_power_table[3][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[3][0];
              break;
            case GI_5_1024:
              first_preamble_cells = preamble_cells_table[4][4];
              preamble_cells = preamble_cells_table[4][cred];
              preamble_dx = preamble_dx_table[4];
              preamble_power = preamble_power_table[4];
              preamble_ifft_power = preamble_ifft_power_table[4][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[4][0];
              break;
            case GI_6_1536:
              first_preamble_cells = preamble_cells_table[5][4];
              preamble_cells = preamble_cells_table[5][cred];
              preamble_dx = preamble_dx_table[5];
              preamble_power = preamble_power_table[5];
              preamble_ifft_power = preamble_ifft_power_table[5][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[5][0];
              break;
            case GI_7_2048:
              first_preamble_cells = preamble_cells_table[6][4];
              preamble_cells = preamble_cells_table[6][cred];
              preamble_dx = preamble_dx_table[6];
              preamble_power = preamble_power_table[6];
              preamble_ifft_power = preamble_ifft_power_table[6][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[6][0];
              break;
            default:
              first_preamble_cells = preamble_cells_table[0][4];
              preamble_cells = preamble_cells_table[0][cred];
              preamble_dx = preamble_dx_table[0];
              preamble_power = preamble_power_table[0];
              preamble_ifft_power = preamble_ifft_power_table[0][cred];
              first_preamble_ifft_power = preamble_ifft_power_table[0][0];
              break;
          }
          switch (pilotpattern) {
            case PILOT_SP3_2:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP3_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP3_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP3_2][cred];
              break;
            case PILOT_SP3_4:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP3_4][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP3_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP3_4][cred];
              break;
            case PILOT_SP4_2:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP4_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP4_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP4_2][cred];
              break;
            case PILOT_SP4_4:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP4_4][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP4_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP4_4][cred];
              break;
            case PILOT_SP6_2:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP6_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP6_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP6_2][cred];
              break;
            case PILOT_SP6_4:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP6_4][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP6_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP6_4][cred];
              break;
            case PILOT_SP8_2:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP8_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP8_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP8_2][cred];
              break;
            case PILOT_SP8_4:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP8_4][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP8_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP8_4][cred];
              break;
            case PILOT_SP12_2:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP12_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP12_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP12_2][cred];
              break;
            case PILOT_SP12_4:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP12_4][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP12_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP12_4][cred];
              break;
            case PILOT_SP16_2:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP16_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP16_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP16_2][cred];
              break;
            case PILOT_SP16_4:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP16_4][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP16_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP16_4][cred];
              break;
            case PILOT_SP24_2:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP24_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP24_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP24_2][cred];
              break;
            case PILOT_SP24_4:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP24_4][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP24_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP24_4][cred];
              break;
            case PILOT_SP32_2:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP32_2][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP32_2][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP32_2][cred];
              break;
            case PILOT_SP32_4:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP32_4][cred][pilotboost];
              data_cells = data_cells_table_8K[PILOT_SP32_4][cred];
              sbs_cells = sbs_cells_table_8K[PILOT_SP32_4][cred];
              break;
            default:
              data_ifft_power = data_ifft_power_table_8K[PILOT_SP3_2][cred][pilotboost];
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
      pr_bpsk[0] = gr_complex(power, 0.0);
      pr_bpsk[1] = gr_complex(-(power), 0.0);
      power = pow(10, scattered_power / 20.0);
      sp_bpsk[0] = gr_complex(power, 0.0);
      sp_bpsk[1] = gr_complex(-(power), 0.0);
      power = pow(10, 8.52 / 20.0);
      cp_bpsk[0] = gr_complex(power, 0.0);
      cp_bpsk[1] = gr_complex(-(power), 0.0);
      init_prbs();
      if (paprmode != PAPR_TR) {
        papr_cells = 0;
      }
      frame_symbols[0] = PREAMBLE_SYMBOL;
      total_preamble_cells = 0;
      for (int n = 1; n < numpreamblesyms; n++) {
        frame_symbols[n] = PREAMBLE_SYMBOL;
        total_preamble_cells += (preamble_cells - papr_cells);
      }
      if (firstsbs == SBS_ON) {
        frame_symbols[numpreamblesyms] = SBS_SYMBOL;
        for (int n = 0; n < numpayloadsyms - 2; n++) {
          frame_symbols[n + numpreamblesyms + 1] = DATA_SYMBOL;
        }
      }
      else {
        for (int n = 0; n < numpayloadsyms - 1; n++) {
          frame_symbols[n + numpreamblesyms] = DATA_SYMBOL;
        }
      }
      if (lastsbs == SBS_ON) {
        frame_symbols[numpreamblesyms + numpayloadsyms - 1] = SBS_SYMBOL;
      }
      else {
        frame_symbols[numpreamblesyms + numpayloadsyms - 1] = DATA_SYMBOL;
      }
      data_carrier_map.resize(symbols);
      for (std::vector<std::vector<int>>::size_type i = 0; i != data_carrier_map.size(); i++) {
        data_carrier_map[i].resize(max_carriers);
      }
      init_pilots();
      if (numpreamblesyms == 0) {
        first_preamble_cells = 0;
      }
      if (firstsbs == SBS_ON) {
        if (lastsbs == SBS_ON) {
          totalcells = first_preamble_cells + total_preamble_cells + ((numpayloadsyms - 2) * (data_cells - papr_cells)) + ((sbs_cells - papr_cells) * 2);
        }
        else {
          totalcells = first_preamble_cells + total_preamble_cells + ((numpayloadsyms - 1) * (data_cells - papr_cells)) + ((sbs_cells - papr_cells) * 1);
        }
      }
      else {
        if (lastsbs == SBS_ON) {
          totalcells = first_preamble_cells + total_preamble_cells + ((numpayloadsyms - 1) * (data_cells - papr_cells)) + (sbs_cells - papr_cells);
        }
        else {
          totalcells = first_preamble_cells + total_preamble_cells + ((numpayloadsyms) * (data_cells - papr_cells));
        }
      }
      input_cells = totalcells;
      printf("input cells = %d\n", input_cells);
      /* -1.98 is a tweak factor to match verification files */
      first_preamble_normalization = 1.0 / std::sqrt((first_preamble_ifft_power * ((double)preamble_carriers / (double)max_carriers) - 1.98));
      preamble_normalization = 1.0 / std::sqrt(preamble_ifft_power);
      data_normalization = 1.0 / std::sqrt(data_ifft_power);
      if (outputmode == PILOTGENERATOR_FREQ) {
        insertion_items = symbols;
      }
      else {
        insertion_items = ((carriers * (symbols - 1)) + preamble_carriers);
      }
      set_output_multiple(insertion_items);
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
      ninput_items_required[0] = input_cells * (noutput_items / insertion_items);
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
        int remainder, shift, index, preamblecarriers, trshift;
        std::vector<int>& data_carrier_map = this->data_carrier_map[symbol];
        for (int i = 0; i < carriers; i++) {
          data_carrier_map[i] = DATA_CARRIER;
        }
        trshift = dx * ((symbol - (preamble_symbols - dy)) % dy);
        if (frame_symbols[symbol] == SBS_SYMBOL || frame_symbols[symbol] == PREAMBLE_SYMBOL) {
          trshift = 0;
        }
        switch (fft_size) {
          case FFTSIZE_8K:
            if (frame_symbols[symbol] == PREAMBLE_SYMBOL) {
              index = 0;
              if (symbol == 0) {
                preamblecarriers = preamble_carriers;
                shift = (max_carriers - preamble_carriers) / 2;
              }
              else {
                preamblecarriers = carriers;
                shift = (max_carriers - carriers) / 2;
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
              if (symbol != 0 && papr_mode == PAPR_TR) {
                index = 0;
                if (preamble_dx == 3 || preamble_dx == 4 || preamble_dx == 8) {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_alt_table_8K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
                else {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_table_8K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
              }
            }
            else if (frame_symbols[symbol] == SBS_SYMBOL) {
              index = 0;
              shift = (max_carriers - carriers) / 2;
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_8K[index] == i) {
                  if (continual_pilot_table_8K[index] > shift) {
                    data_carrier_map[i - shift] = CONTINUAL_CARRIER;
                  }
                  index++;
                }
              }
              for (int i = 0; i < carriers; i++) {
                if ((i % dx) == 0) {
                  data_carrier_map[i] = SCATTERED_CARRIER;
                }
              }
              if (papr_mode == PAPR_TR) {
                index = 0;
                if (dx == 3 || dx == 4 || dx == 8) {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_alt_table_8K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
                else {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_table_8K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
              }
            }
            else {
              index = 0;
              shift = (max_carriers - carriers) / 2;
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_8K[index] == i) {
                  if (continual_pilot_table_8K[index] > shift) {
                    data_carrier_map[i - shift] = CONTINUAL_CARRIER;
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
              if (papr_mode == PAPR_TR) {
                index = 0;
                for (int i = 0; i < max_carriers; i++) {
                  if ((trpapr_table_8K[index] + trshift) == i) {
                    data_carrier_map[i - shift] = TRPAPR_CARRIER;
                    index++;
                  }
                }
              }
            }
            if ((frame_symbols[symbol] == SBS_SYMBOL) || (frame_symbols[symbol] == DATA_SYMBOL)) {
              data_carrier_map[0] = SCATTERED_CARRIER;
              data_carrier_map[carriers - 1] = SCATTERED_CARRIER;
              switch (pilot_pattern) {
                case PILOT_SP3_2:
                  data_carrier_map[1731] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP3_4:
                  data_carrier_map[1731] = SCATTERED_CARRIER;
                  data_carrier_map[2886] = SCATTERED_CARRIER;
                  data_carrier_map[5733] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP4_2:
                  data_carrier_map[1732] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP4_4:
                  data_carrier_map[1732] = SCATTERED_CARRIER;
                  data_carrier_map[2888] = SCATTERED_CARRIER;
                  data_carrier_map[5724] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP6_2:
                  data_carrier_map[1734] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP6_4:
                  data_carrier_map[1734] = SCATTERED_CARRIER;
                  data_carrier_map[2892] = SCATTERED_CARRIER;
                  data_carrier_map[5730] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP8_2:
                  data_carrier_map[1736] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP8_4:
                  data_carrier_map[1736] = SCATTERED_CARRIER;
                  data_carrier_map[2896] = SCATTERED_CARRIER;
                  data_carrier_map[5720] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP12_2:
                  data_carrier_map[1740] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP12_4:
                  data_carrier_map[1740] = SCATTERED_CARRIER;
                  data_carrier_map[2904] = SCATTERED_CARRIER;
                  data_carrier_map[5748] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP16_2:
                  data_carrier_map[1744] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP16_4:
                  data_carrier_map[1744] = SCATTERED_CARRIER;
                  if ((cred_coeff & 0x1) == 0) {
                    data_carrier_map[2912] = SCATTERED_CARRIER;
                    data_carrier_map[5744] = SCATTERED_CARRIER;
                  }
                  break;
                case PILOT_SP24_2:
                  break;
                case PILOT_SP24_4:
                  break;
                case PILOT_SP32_2:
                  if ((cred_coeff & 0x1) == 0) {
                    data_carrier_map[1696] = SCATTERED_CARRIER;
                  }
                  break;
                case PILOT_SP32_4:
                  switch (cred_coeff) {
                    case CRED_0:
                      data_carrier_map[1696] = SCATTERED_CARRIER;
                      data_carrier_map[2880] = SCATTERED_CARRIER;
                      data_carrier_map[5728] = SCATTERED_CARRIER;
                      break;
                    case CRED_1:
                      break;
                    case CRED_2:
                      data_carrier_map[1696] = SCATTERED_CARRIER;
                      break;
                    case CRED_3:
                      data_carrier_map[1696] = SCATTERED_CARRIER;
                      data_carrier_map[2880] = SCATTERED_CARRIER;
                      break;
                    case CRED_4:
                      data_carrier_map[1696] = SCATTERED_CARRIER;
                      data_carrier_map[2880] = SCATTERED_CARRIER;
                      data_carrier_map[5728] = SCATTERED_CARRIER;
                      break;
                    default:
                      break;
                  }
                  break;
                default:
                  break;
              }
            }
            break;
          case FFTSIZE_16K:
            if (frame_symbols[symbol] == PREAMBLE_SYMBOL) {
              index = 0;
              if (symbol == 0) {
                preamblecarriers = preamble_carriers;
                shift = (max_carriers - preamble_carriers) / 2;
              }
              else {
                preamblecarriers = carriers;
                shift = (max_carriers - carriers) / 2;
              }
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_16K[index] == i) {
                  if (continual_pilot_table_16K[index] > shift) {
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
              if (symbol != 0 && papr_mode == PAPR_TR) {
                index = 0;
                if (preamble_dx == 3 || preamble_dx == 4 || preamble_dx == 8) {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_alt_table_16K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
                else {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_table_16K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
              }
            }
            else if (frame_symbols[symbol] == SBS_SYMBOL) {
              index = 0;
              shift = (max_carriers - carriers) / 2;
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_16K[index] == i) {
                  if (continual_pilot_table_16K[index] > shift) {
                    data_carrier_map[i - shift] = CONTINUAL_CARRIER;
                  }
                  index++;
                }
              }
              for (int i = 0; i < carriers; i++) {
                if ((i % dx) == 0) {
                  data_carrier_map[i] = SCATTERED_CARRIER;
                }
              }
              if (papr_mode == PAPR_TR) {
                index = 0;
                if (dx == 3 || dx == 4 || dx == 8) {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_alt_table_16K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
                else {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_table_16K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
              }
            }
            else {
              index = 0;
              shift = (max_carriers - carriers) / 2;
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_16K[index] == i) {
                  if (continual_pilot_table_16K[index] > shift) {
                    data_carrier_map[i - shift] = CONTINUAL_CARRIER;
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
              if (papr_mode == PAPR_TR) {
                index = 0;
                for (int i = 0; i < max_carriers; i++) {
                  if ((trpapr_table_16K[index] + trshift) == i) {
                    data_carrier_map[i - shift] = TRPAPR_CARRIER;
                    index++;
                  }
                }
              }
            }
            if ((frame_symbols[symbol] == SBS_SYMBOL) || (frame_symbols[symbol] == DATA_SYMBOL)) {
              data_carrier_map[0] = SCATTERED_CARRIER;
              data_carrier_map[carriers - 1] = SCATTERED_CARRIER;
              switch (pilot_pattern) {
                case PILOT_SP3_2:
                  data_carrier_map[3471] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP3_4:
                  data_carrier_map[3471] = SCATTERED_CARRIER;
                  data_carrier_map[5778] = SCATTERED_CARRIER;
                  data_carrier_map[11469] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP4_2:
                  data_carrier_map[3460] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP4_4:
                  data_carrier_map[3460] = SCATTERED_CARRIER;
                  data_carrier_map[5768] = SCATTERED_CARRIER;
                  data_carrier_map[11452] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP6_2:
                  data_carrier_map[3462] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP6_4:
                  data_carrier_map[3462] = SCATTERED_CARRIER;
                  data_carrier_map[5772] = SCATTERED_CARRIER;
                  data_carrier_map[11466] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP8_2:
                  data_carrier_map[3464] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP8_4:
                  data_carrier_map[3464] = SCATTERED_CARRIER;
                  data_carrier_map[5776] = SCATTERED_CARRIER;
                  data_carrier_map[11448] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP12_2:
                  data_carrier_map[3468] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP12_4:
                  data_carrier_map[3468] = SCATTERED_CARRIER;
                  data_carrier_map[5784] = SCATTERED_CARRIER;
                  data_carrier_map[11460] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP16_2:
                  data_carrier_map[3472] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP16_4:
                  data_carrier_map[3472] = SCATTERED_CARRIER;
                  data_carrier_map[5792] = SCATTERED_CARRIER;
                  data_carrier_map[11440] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP24_2:
                  data_carrier_map[3480] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP24_4:
                  data_carrier_map[3480] = SCATTERED_CARRIER;
                  data_carrier_map[5808] = SCATTERED_CARRIER;
                  data_carrier_map[11496] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP32_2:
                  data_carrier_map[3488] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP32_4:
                  data_carrier_map[3488] = SCATTERED_CARRIER;
                  if ((cred_coeff & 0x1) == 0) {
                    data_carrier_map[5824] = SCATTERED_CARRIER;
                    data_carrier_map[11488] = SCATTERED_CARRIER;
                  }
                  break;
                default:
                  break;
              }
            }
            break;
          case FFTSIZE_32K:
            if (frame_symbols[symbol] == PREAMBLE_SYMBOL) {
              index = 0;
              if (symbol == 0) {
                preamblecarriers = preamble_carriers;
                shift = (max_carriers - preamble_carriers) / 2;
              }
              else {
                preamblecarriers = carriers;
                shift = (max_carriers - carriers) / 2;
              }
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_32K[index] == i) {
                  if (continual_pilot_table_32K[index] > shift) {
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
              if (symbol != 0 && papr_mode == PAPR_TR) {
                index = 0;
                if (preamble_dx == 3 || preamble_dx == 4 || preamble_dx == 8) {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_alt_table_32K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
                else {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_table_32K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
              }
            }
            else if (frame_symbols[symbol] == SBS_SYMBOL) {
              index = 0;
              shift = (max_carriers - carriers) / 2;
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_32K[index] == i) {
                  if (continual_pilot_table_32K[index] > shift) {
                    data_carrier_map[i - shift] = CONTINUAL_CARRIER;
                  }
                  index++;
                }
              }
              for (int i = 0; i < carriers; i++) {
                if ((i % dx) == 0) {
                  data_carrier_map[i] = SCATTERED_CARRIER;
                }
              }
              if (papr_mode == PAPR_TR) {
                index = 0;
                if (dx == 3 || dx == 4 || dx == 8) {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_alt_table_32K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
                else {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_table_32K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
              }
            }
            else {
              index = 0;
              shift = (max_carriers - carriers) / 2;
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_32K[index] == i) {
                  if (continual_pilot_table_32K[index] > shift) {
                    data_carrier_map[i - shift] = CONTINUAL_CARRIER;
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
              if (papr_mode == PAPR_TR) {
                index = 0;
                for (int i = 0; i < max_carriers; i++) {
                  if ((trpapr_table_32K[index] + trshift) == i) {
                    data_carrier_map[i - shift] = TRPAPR_CARRIER;
                    index++;
                  }
                }
              }
            }
            if ((frame_symbols[symbol] == SBS_SYMBOL) || (frame_symbols[symbol] == DATA_SYMBOL)) {
              data_carrier_map[0] = SCATTERED_CARRIER;
              data_carrier_map[carriers - 1] = SCATTERED_CARRIER;
              switch (pilot_pattern) {
                case PILOT_SP3_2:
                  data_carrier_map[6939] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP3_4:
                  break;
                case PILOT_SP4_2:
                  break;
                case PILOT_SP4_4:
                  break;
                case PILOT_SP6_2:
                  data_carrier_map[6942] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP6_4:
                  break;
                case PILOT_SP8_2:
                  data_carrier_map[6920] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP8_4:
                  break;
                case PILOT_SP12_2:
                  data_carrier_map[6924] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP12_4:
                  break;
                case PILOT_SP16_2:
                  data_carrier_map[6928] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP16_4:
                  break;
                case PILOT_SP24_2:
                  data_carrier_map[6936] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP24_4:
                  break;
                case PILOT_SP32_2:
                  data_carrier_map[6944] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP32_4:
                  break;
                default:
                  break;
              }
            }
            break;
        }
      }
    }

    const gr_complex zero = gr_complex(0.0, 0.0);

    int
    pilotgenerator_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      int indexin = 0;
      int preamblecarriers;
      int left_nulls, right_nulls;
      float normalization;
      gr_complex* dst;
      gr_complex* src;
      float angle;
      gr_complex temp;

      for (int i = 0; i < noutput_items; i += insertion_items) {
        for (int j = 0; j < symbols; j++) {
          if (frame_symbols[j] == PREAMBLE_SYMBOL) {
            if (j == 0) {
              preamblecarriers = preamble_carriers;
              normalization = first_preamble_normalization;
            }
            else {
              preamblecarriers = carriers;
              normalization = preamble_normalization;
            }
            left_nulls = !output_mode ? ((ofdm_fft_size - preamblecarriers) / 2) + 1 : 0;
            right_nulls = !output_mode ? (ofdm_fft_size - preamblecarriers) / 2 : 0;
            for (int n = 0; n < left_nulls; n++) {
              *out++ = zero;
            }
            for (int n = 0; n < preamblecarriers; n++) {
              if (data_carrier_map[j][n] == PREAMBLE_CARRIER) {
                *out++ = pr_bpsk[prbs[n]];
              }
              else if (data_carrier_map[j][n] == CONTINUAL_CARRIER) {
                *out++ = cp_bpsk[prbs[n]];
              }
              else if (data_carrier_map[j][n] == TRPAPR_CARRIER) {
                *out++ = zero;
              }
              else {
                *out++ = in[indexin++];
              }
            }
            for (int n = 0; n < right_nulls; n++) {
              *out++ = zero;
            }
          }
          else {
            normalization = data_normalization;
            left_nulls = !output_mode ? ((ofdm_fft_size - carriers) / 2) + 1 : 0;
            right_nulls = !output_mode ? (ofdm_fft_size - carriers) / 2 : 0;
            for (int n = 0; n < left_nulls; n++) {
              *out++ = zero;
            }
            for (int n = 0; n < carriers; n++) {
              if (data_carrier_map[j][n] == SCATTERED_CARRIER) {
                *out++ = sp_bpsk[prbs[n]];
              }
              else if (data_carrier_map[j][n] == CONTINUAL_CARRIER) {
                *out++ = cp_bpsk[prbs[n]];
              }
              else if (data_carrier_map[j][n] == TRPAPR_CARRIER) {
                *out++ = zero;
              }
              else {
                *out++ = in[indexin++];
              }
            }
            for (int n = 0; n < right_nulls; n++) {
              *out++ = zero;
            }
          }
          if (miso_mode == MISO_64) {
            if (frame_symbols[j] != PREAMBLE_SYMBOL) {
              dst = miso_fft.get_inbuf();
              memcpy(&dst[0], &miso_coefficients_64[0], sizeof(gr_complex) * 64);
              std::fill_n(&dst[64], ofdm_fft_size - 64, 0);
              miso_fft.execute();
              src = miso_fft.get_outbuf();
              out -= carriers;
              for (int n = 0; n < carriers; n++) {
                angle = std::arg(src[n]);
                if (j == 2 && n < 10)
                  printf("%f, ", angle);
                angle = miso_angle[n];
                if (j == 2 && n < 10)
                  printf("%f, %d\n", angle, n);
                temp = std::exp(gr_complexd(0.0, angle));
                *out++ *= temp;
              }
            }
          }
          if (output_mode == PILOTGENERATOR_FREQ) {
            out -= ofdm_fft_size;
#if 0
            if (equalization_enable == EQUALIZATION_ON) {
              volk_32fc_x2_multiply_32fc(out, out, inverse_sinc, ofdm_fft_size);
            }
#endif
            dst = ofdm_fft.get_inbuf();
            memcpy(&dst[ofdm_fft_size / 2], &out[0], sizeof(gr_complex) * ofdm_fft_size / 2);
            memcpy(&dst[0], &out[ofdm_fft_size / 2], sizeof(gr_complex) * ofdm_fft_size / 2);
            ofdm_fft.execute();
            volk_32fc_s32fc_multiply_32fc(out, ofdm_fft.get_outbuf(), normalization, ofdm_fft_size);
            out += ofdm_fft_size;
          }
        }
      }

      if (indexin != input_cells) {
        printf("input_cells = %d, indexin = %d\n", input_cells, indexin);
      }
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (indexin);

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

    const double pilotgenerator_cc_impl::preamble_ifft_power_table[32][5] = {
      {8240.53, 8130.20, 8013.76, 7897.31, 7780.87},
      {8322.93, 8211.44, 8093.84, 7976.24, 7858.64},
      {8301.50, 8190.31, 8073.00, 7955.69, 7838.38},
      {8094.28, 7985.96, 7871.52, 7757.08, 7642.65},
      {7737.10, 7633.73, 7524.25, 7414.77, 7305.30},
      {8094.28, 7985.96, 7871.52, 7757.08, 7642.65},
      {7737.10, 7633.73, 7524.25, 7414.77, 7305.30},
      {16051.13, 15836.42, 15603.37, 15382.54, 15155.60},
      {16477.67, 16257.01, 16018.01, 15791.24, 15558.36},
      {16583.95, 16361.81, 16121.33, 15893.08, 15658.71},
      {16643.58, 16420.60, 16179.28, 15950.19, 15714.99},
      {16601.06, 16378.66, 16137.94, 15909.43, 15674.81},
      {16561.26, 16339.42, 16099.24, 15871.28, 15637.21},
      {16020.04, 15805.70, 15573.04, 15352.59, 15126.04},
      {16020.04, 15805.70, 15573.04, 15352.59, 15126.04},
      {16561.26, 16339.42, 16099.24, 15871.28, 15637.21},
      {16561.26, 16339.42, 16099.24, 15871.28, 15637.21},
      {16020.04, 15805.70, 15573.04, 15352.59, 15126.04},
      {32097.48, 31668.05, 31201.95, 30760.29, 30306.41},
      {32097.48, 31668.05, 31201.95, 30760.29, 30306.41},
      {32475.84, 32041.14, 31569.77, 31122.85, 30663.71},
      {32951.95, 32510.63, 32032.64, 31579.09, 31113.33},
      {33165.03, 32720.74, 32239.78, 31783.27, 31314.54},
      {34048.92, 33592.35, 33099.12, 32630.32, 32149.31},
      {33842.90, 33389.18, 32898.80, 32432.87, 31954.71},
      {33842.90, 33389.18, 32898.80, 32432.87, 31954.71},
      {34048.92, 33592.35, 33099.12, 32630.32, 32149.31},
      {32038.72, 31610.06, 31144.72, 30703.83, 30250.72},
      {34048.92, 33592.35, 33099.12, 32630.32, 32149.31},
      {32038.72, 31610.06, 31144.72, 30703.83, 30250.72},
      {32038.72, 31610.06, 31144.72, 30703.83, 30250.72},
      {32038.72, 31610.06, 31144.72, 30703.83, 30250.72}
    };

    const double pilotgenerator_cc_impl::data_ifft_power_table_8K[16][5][5] = {
      {{7206.33, 7206.33, 7646.12, 7967.68, 8302.73}, {7110.33, 7110.33, 7543.95, 7861.57, 8191.33}, {7008.22, 7008.22, 7435.66, 7748.36, 8074.82}, {6906.11, 6906.11, 7327.38, 7636.14, 7957.32}, {6804.00, 6804.00, 7219.10, 7523.92, 7839.81}},
      {{7206.33, 7427.12, 7757.73, 8018.64, 8223.83}, {7110.33, 7327.95, 7654.33, 7910.88, 8114.70}, {7008.22, 7223.66, 7544.82, 7798.01, 7998.45}, {6906.11, 7118.38, 7434.32, 7684.13, 7882.20}, {6804.00, 7013.10, 7324.81, 7571.26, 7765.96}},
      {{7206.33, 7335.49, 7745.44, 8069.14, 8324.24}, {7110.33, 7236.94, 7641.52, 7961.26, 8213.26}, {7008.22, 7133.27, 7532.48, 7846.26, 8095.17}, {6906.11, 7029.60, 7422.45, 7732.26, 7978.08}, {6804.00, 6925.93, 7313.42, 7618.26, 7859.99}},
      {{7206.33, 7477.44, 7769.24, 7971.39, 8181.27}, {7110.33, 7378.52, 7666.26, 7865.29, 8072.61}, {7008.22, 7272.48, 7556.17, 7752.08, 7956.83}, {6906.11, 7166.45, 7446.08, 7639.87, 7841.06}, {6804.00, 7060.42, 7335.99, 7527.66, 7725.28}},
      {{7206.33, 7463.93, 7809.46, 8080.54, 8295.62}, {7110.33, 7364.80, 7704.79, 7972.35, 8184.48}, {7008.22, 7258.56, 7594.01, 7858.05, 8067.22}, {6906.11, 7153.32, 7484.23, 7743.75, 7949.97}, {6804.00, 7048.08, 7373.45, 7630.44, 7832.71}},
      {{7206.33, 7497.87, 7737.93, 7927.21, 8077.51}, {7110.33, 7397.95, 7634.83, 7821.73, 7969.81}, {7008.22, 7290.91, 7525.63, 7709.14, 7855.00}, {6906.11, 7184.88, 7415.42, 7597.55, 7741.20}, {6804.00, 7078.84, 7306.22, 7485.96, 7627.39}},
      {{7206.33, 7492.88, 7814.32, 8024.02, 8243.34}, {7110.33, 7392.96, 7709.54, 7917.41, 8132.67}, {7008.22, 7286.93, 7598.64, 7803.69, 8016.90}, {6906.11, 7180.91, 7488.74, 7689.97, 7900.13}, {6804.00, 7074.88, 7377.85, 7577.25, 7783.36}},
      {{7206.33, 7490.93, 7698.42, 7862.96, 7992.15}, {7110.33, 7391.44, 7595.59, 7758.19, 7885.30}, {7008.22, 7284.84, 7487.64, 7646.30, 7773.33}, {6906.11, 7179.24, 7378.70, 7535.42, 7660.37}, {6804.00, 7072.64, 7269.76, 7424.54, 7547.41}},
      {{7206.33, 7522.86, 7773.18, 7969.29, 8125.67}, {7110.33, 7422.14, 7668.57, 7863.25, 8017.32}, {7008.22, 7316.32, 7558.85, 7751.09, 7901.86}, {6906.11, 7209.49, 7449.13, 7637.93, 7787.40}, {6804.00, 7102.67, 7339.41, 7525.78, 7672.94}},
      {{7206.33, 7475.54, 7648.41, 7783.36, 7891.04}, {7110.33, 7375.99, 7546.56, 7680.17, 7786.06}, {7008.22, 7270.34, 7437.60, 7569.88, 7674.96}, {6906.11, 7164.68, 7329.64, 7460.59, 7562.86}, {6804.00, 7059.02, 7221.68, 7350.29, 7451.76}},
      {{7206.33, 7512.03, 7727.53, 7897.38, 8031.80}, {7110.33, 7411.63, 7624.20, 7792.37, 7925.08}, {7008.22, 7305.13, 7514.76, 7680.24, 7811.25}, {6906.11, 7198.63, 7405.31, 7569.12, 7698.43}, {6804.00, 7093.12, 7296.87, 7457.00, 7584.60}},
      {{7206.33, 7466.13, 7618.63, 7738.99, 7835.14}, {7110.33, 7363.26, 7511.56, 7629.46, 7721.49}, {7008.22, 7260.28, 7409.39, 7526.82, 7619.74}, {6906.11, 7151.31, 7296.21, 7410.19, 7500.99}, {6804.00, 7049.33, 7194.04, 7308.55, 7399.24}},
      {{7206.33, 7491.23, 7669.09, 7809.50, 7920.09}, {7110.33, 7391.43, 7567.41, 7704.99, 7814.54}, {7008.22, 7285.51, 7458.63, 7595.36, 7702.87}, {6906.11, 7179.60, 7349.84, 7484.74, 7591.21}, {6804.00, 7073.68, 7242.05, 7374.11, 7479.54}},
      {{7206.33, 7440.66, 7568.35, 7669.30, 7748.41}, {7110.33, 7341.37, 7467.34, 7566.98, 7645.90}, {7008.22, 7235.96, 7360.21, 7458.55, 7536.28}, {6906.11, 7130.55, 7254.08, 7350.12, 7426.65}, {6804.00, 7026.15, 7146.95, 7242.69, 7318.03}},
      {{7206.33, 7477.75, 7635.15, 7744.12, 7857.61}, {7110.33, 7377.35, 7531.46, 7638.46, 7749.85}, {7008.22, 7272.84, 7425.66, 7531.68, 7641.99}, {6906.11, 7165.32, 7315.85, 7419.91, 7528.12}, {6804.00, 7060.81, 7210.05, 7313.13, 7420.25}},
      {{7206.33, 7420.32, 7532.04, 7620.18, 7690.50}, {7110.33, 7313.29, 7419.22, 7502.79, 7569.50}, {7008.22, 7211.14, 7317.29, 7400.29, 7467.39}, {6906.11, 7109.00, 7215.35, 7298.80, 7365.28}, {6804.00, 7006.86, 7112.42, 7196.30, 7263.17}}
    };

    const double pilotgenerator_cc_impl::data_ifft_power_table_16K[16][5][5] = {
      {{14411.67, 14411.67, 15288.86, 15932.70, 16602.50}, {14219.67, 14219.67, 15085.51, 15720.49, 16379.71}, {14009.33, 14009.33, 14862.84, 15488.94, 16139.59}, {13811.22, 13811.22, 14652.38, 15269.62, 15910.69}, {13607.00, 13607.00, 14435.81, 15044.18, 15676.67}},
      {{14411.67, 14851.86, 15510.50, 16028.89, 16439.91}, {14219.67, 14653.51, 15302.71, 15814.36, 16219.64}, {14009.33, 14437.84, 15077.59, 15582.50, 15982.03}, {13811.22, 14233.38, 14863.69, 15361.87, 15755.65}, {13607.00, 14022.81, 14644.67, 15135.12, 15523.16}},
      {{14411.67, 14668.83, 15488.26, 16134.29, 16645.20}, {14219.67, 14472.72, 15281.42, 15918.52, 16422.23}, {14009.33, 14259.28, 15055.24, 15683.41, 16180.94}, {13811.22, 14057.06, 14842.28, 15461.53, 15951.87}, {13607.00, 13849.72, 14623.21, 15233.53, 15716.68}},
      {{14411.67, 14952.26, 15532.20, 15935.04, 16353.30}, {14219.67, 14752.42, 15325.23, 15721.83, 16133.97}, {14009.33, 14534.24, 15098.94, 15490.30, 15897.32}, {13811.22, 14329.28, 14885.87, 15270.98, 15671.88}, {13607.00, 14117.21, 14665.68, 15045.56, 15441.33}},
      {{14411.67, 14926.41, 15613.87, 16156.56, 16586.36}, {14219.67, 14727.15, 15405.54, 15941.18, 16364.07}, {14009.33, 14509.56, 15178.87, 15706.47, 16123.45}, {13811.22, 14304.20, 14963.42, 15483.98, 15895.05}, {13607.00, 14092.72, 14742.86, 15255.38, 15661.54}},
      {{14411.67, 14989.75, 15467.04, 15842.95, 16141.04}, {14219.67, 14789.90, 15259.85, 15632.00, 15925.64}, {14009.33, 14570.72, 15035.33, 15401.71, 15690.92}, {13811.22, 14364.76, 14823.03, 15183.64, 15469.41}, {13607.00, 14152.69, 14603.62, 14959.46, 15240.79}},
      {{14411.67, 14983.09, 15623.25, 16044.16, 16480.28}, {14219.67, 14783.26, 15414.68, 15828.94, 16259.96}, {14009.33, 14565.10, 15187.77, 15596.39, 16021.30}, {13811.22, 14359.16, 14973.09, 15375.06, 15793.87}, {13607.00, 14147.11, 14751.29, 15148.62, 15561.33}},
      {{14411.67, 14974.58, 15386.60, 15711.94, 15968.72}, {14219.67, 14775.60, 15181.94, 15502.39, 15756.02}, {14009.33, 14557.28, 14957.94, 15273.51, 15523.99}, {13811.22, 14351.19, 14746.17, 15057.86, 15304.17}, {13607.00, 14138.99, 14528.28, 14836.09, 15078.25}},
      {{14411.67, 15041.63, 15540.41, 15932.96, 16244.17}, {14219.67, 14841.20, 15332.19, 15719.86, 16026.47}, {14009.33, 14622.43, 15106.64, 15488.44, 15790.44}, {13811.22, 14414.89, 14892.31, 15269.23, 15567.63}, {13607.00, 14202.24, 14672.87, 15043.92, 15337.71}},
      {{14411.67, 14943.26, 15282.84, 15550.82, 15762.46}, {14219.67, 14744.17, 15079.15, 15342.45, 15551.49}, {14009.33, 14526.74, 14856.11, 15116.75, 15323.18}, {13811.22, 14320.54, 14646.31, 14903.28, 15106.09}, {13607.00, 14109.22, 14430.39, 14683.69, 14883.90}},
      {{14411.67, 15019.66, 15448.67, 15787.59, 16055.82}, {14219.67, 14818.87, 15242.01, 15576.56, 15840.38}, {14009.33, 14599.75, 15018.01, 15347.20, 15607.61}, {13811.22, 14393.85, 14805.24, 15130.07, 15387.07}, {13607.00, 14180.85, 14586.36, 14906.82, 15159.41}},
      {{14411.67, 14920.95, 15221.58, 15458.22, 15645.66}, {14219.67, 14722.22, 15018.45, 15252.17, 15436.38}, {14009.33, 14504.15, 14796.99, 15026.78, 15209.77}, {13811.22, 14299.30, 14587.75, 14814.62, 14994.37}, {13607.00, 14088.34, 14372.40, 14596.34, 14773.87}},
      {{14411.67, 14978.51, 15331.00, 15609.88, 15830.29}, {14219.67, 14777.90, 15126.65, 15400.85, 15618.19}, {14009.33, 14559.96, 14902.97, 15174.49, 15388.75}, {13811.22, 14354.24, 14692.51, 14959.35, 15170.53}, {13607.00, 14142.41, 14475.94, 14739.09, 14947.20}},
      {{14411.67, 14867.25, 15115.96, 15311.51, 15466.70}, {14219.67, 14668.66, 14913.92, 15107.88, 15260.67}, {14009.33, 14452.74, 14694.55, 14884.91, 15036.31}, {13811.22, 14248.04, 14486.41, 14675.16, 14824.17}, {13607.00, 14038.22, 14273.15, 14458.30, 14605.93}},
      {{14411.67, 14950.04, 15261.41, 15477.36, 15702.29}, {14219.67, 14750.23, 15058.02, 15271.03, 15492.78}, {14009.33, 14533.10, 14836.30, 15046.37, 15264.94}, {13811.22, 14327.18, 14626.80, 14832.92, 15048.32}, {13607.00, 14116.15, 14410.20, 14614.37, 14826.59}},
      {{14411.67, 14823.96, 15040.47, 15210.22, 15345.66}, {14219.67, 14620.90, 14830.83, 14997.45, 15128.66}, {14009.33, 14410.50, 14620.85, 14786.35, 14918.33}, {13811.22, 14201.32, 14406.10, 14567.47, 14695.23}, {13607.00, 13997.04, 14202.24, 14363.48, 14491.01}}
    };

    const double pilotgenerator_cc_impl::data_ifft_power_table_32K[16][5][5] = {
      {{28822.33, 28822.33, 30576.34, 31863.75, 33202.05}, {28438.33, 28438.33, 30167.65, 31437.32, 32756.47}, {28017.67, 28017.67, 29723.29, 30974.22, 32275.22}, {27621.44, 27621.44, 29302.38, 30535.58, 31818.42}, {27213.00, 27213.00, 28869.25, 30085.70, 31349.40}},
      {{28822.33, 29700.34, 31015.05, 32051.38, 32872.06}, {28438.33, 29304.65, 30600.47, 31622.33, 32431.52}, {28017.67, 28871.29, 30149.22, 31157.61, 31954.31}, {27621.44, 28463.38, 29723.42, 30716.34, 31502.55}, {27213.00, 28042.25, 29284.40, 30262.84, 31037.56}},
      {{0.00, 0.00, 0.00, 0.00, 0.00}, {0.00, 0.00, 0.00, 0.00, 0.00}, {0.00, 0.00, 0.00, 0.00, 0.00}, {0.00, 0.00, 0.00, 0.00, 0.00}, {0.00, 0.00, 0.00, 0.00, 0.00}},
      {{0.00, 0.00, 0.00, 0.00, 0.00}, {0.00, 0.00, 0.00, 0.00, 0.00}, {0.00, 0.00, 0.00, 0.00, 0.00}, {0.00, 0.00, 0.00, 0.00, 0.00}, {0.00, 0.00, 0.00, 0.00, 0.00}},
      {{28822.33, 29850.37, 31224.70, 32309.62, 33167.83}, {28438.33, 29451.86, 30808.03, 31876.86, 32723.26}, {28017.67, 29016.68, 30353.69, 31407.43, 32242.01}, {27621.44, 28605.95, 29923.80, 30963.45, 31785.21}, {27213.00, 28184.00, 29482.69, 30506.24, 31317.19}},
      {{28822.33, 29973.50, 30925.26, 31675.43, 32269.09}, {28438.33, 29573.80, 30511.88, 31251.52, 31837.30}, {28017.67, 29136.44, 30061.84, 30791.94, 31368.85}, {27621.44, 28724.52, 29636.24, 30355.81, 30924.84}, {27213.00, 28300.38, 29199.42, 29907.46, 30468.61}},
      {{28822.33, 29963.53, 31243.10, 32082.43, 32955.18}, {28438.33, 29563.87, 30824.95, 31653.00, 32513.53}, {28017.67, 29127.54, 30371.14, 31186.90, 32035.22}, {27621.44, 28715.66, 29940.78, 30745.24, 31582.35}, {27213.00, 28291.55, 29499.19, 30292.36, 31116.26}},
      {{28822.33, 29942.86, 30763.97, 31410.90, 31921.87}, {28438.33, 29543.90, 30352.64, 30990.81, 31495.47}, {28017.67, 29107.27, 29905.65, 30534.05, 31032.40}, {27621.44, 28696.09, 29482.10, 30101.74, 30592.78}, {27213.00, 28271.68, 29047.33, 29658.20, 30141.93}},
      {{28822.33, 30080.16, 31074.87, 31858.28, 32479.17}, {28438.33, 29678.30, 30659.43, 31433.10, 32044.77}, {28017.67, 29240.78, 30207.33, 30970.25, 31573.71}, {27621.44, 28826.70, 29779.67, 30531.84, 31126.09}, {27213.00, 28400.40, 29340.78, 30081.21, 30667.25}},
      {{28822.33, 29877.71, 30551.70, 31083.74, 31504.30}, {28438.33, 29478.52, 30144.31, 30668.01, 31083.35}, {28017.67, 29043.66, 29699.25, 30216.61, 30625.74}, {27621.44, 28633.25, 29279.63, 29788.66, 30192.56}, {27213.00, 28209.62, 28846.79, 29349.48, 29747.17}},
      {{28822.33, 30033.92, 30890.95, 31567.00, 32101.85}, {28438.33, 29633.34, 30478.63, 31144.96, 31671.98}, {28017.67, 29196.10, 30029.64, 30686.24, 31206.44}, {27621.44, 28783.31, 29604.09, 30251.97, 30764.35}, {27213.00, 28357.29, 29167.33, 29805.48, 30311.03}},
      {{28822.33, 29830.59, 30426.47, 30895.69, 31267.72}, {28438.33, 29433.12, 30020.22, 30483.58, 30850.16}, {28017.67, 28997.98, 29577.29, 30033.81, 30395.92}, {27621.44, 28588.29, 29158.82, 29609.48, 29966.13}, {27213.00, 28165.37, 28728.11, 29171.93, 29523.12}},
      {{28822.33, 29951.08, 30654.84, 31209.63, 31648.70}, {28438.33, 29550.86, 30245.14, 30792.57, 31225.49}, {28017.67, 29114.97, 29798.77, 30338.84, 30765.60}, {27621.44, 28702.53, 29377.85, 29909.56, 30330.17}, {27213.00, 28278.87, 28943.71, 29468.06, 29882.51}},
      {{28822.33, 29720.43, 30211.16, 30597.95, 30904.27}, {28438.33, 29324.25, 29808.09, 30189.68, 30492.22}, {28017.67, 28891.40, 29368.35, 29744.74, 30042.50}, {27621.44, 28483.00, 28953.06, 29324.25, 29618.22}, {27213.00, 28061.37, 28525.54, 28891.53, 29180.72}},
      {{28822.33, 29893.61, 30514.92, 30944.83, 31391.67}, {28438.33, 29495.00, 30107.15, 30531.17, 30971.65}, {28017.67, 29059.72, 29662.71, 30080.84, 30515.97}, {27621.44, 28647.89, 29243.71, 29655.96, 30083.72}, {27213.00, 28224.84, 28811.49, 29217.85, 29640.26}},
      {{28822.33, 29632.24, 30056.33, 30391.32, 30655.98}, {28438.33, 29237.11, 29655.05, 29985.78, 30246.99}, {28017.67, 28805.32, 29218.10, 29543.57, 29801.34}, {27621.44, 28397.97, 28804.59, 29125.81, 29380.12}, {27213.00, 27978.40, 28379.87, 28695.83, 28946.69}}
    };

    const int pilotgenerator_cc_impl::continual_pilot_table_8K[48] = {
      59, 167, 307, 469, 637, 751, 865, 1031, 1159, 1333, 1447, 1607, 1811, 1943, 2041, 2197,
      2323, 2519, 2605, 2767, 2963, 3029, 3175, 3325, 3467, 3665, 3833, 3901, 4073, 4235, 4325, 4511,
      4627, 4825, 4907, 5051, 5227, 5389, 5531, 5627, 5833, 5905, 6053, 6197, 6353, 6563, 6637, 6809
    };

    const int pilotgenerator_cc_impl::continual_pilot_table_16K[96] = {
      118, 178, 334, 434, 614, 670, 938, 1070, 1274, 1358, 1502, 1618, 1730, 1918, 2062, 2078,
      2318, 2566, 2666, 2750, 2894, 3010, 3214, 3250, 3622, 3686, 3886, 3962, 4082, 4166, 4394, 4558,
      4646, 4718, 5038, 5170, 5210, 5342, 5534, 5614, 5926, 5942, 6058, 6134, 6350, 6410, 6650, 6782,
      6934, 7154, 7330, 7438, 7666, 7742, 7802, 7894, 8146, 8258, 8470, 8494, 8650, 8722, 9022, 9118,
      9254, 9422, 9650, 9670, 9814, 9902, 10102, 10166, 10454, 10598, 10778, 10822, 11062, 11138,
      11254, 11318, 11666, 11758, 11810, 11974, 12106, 12242, 12394, 12502, 12706, 12866, 13126, 13190,
      13274, 13466, 13618, 13666
    };

    const int pilotgenerator_cc_impl::continual_pilot_table_32K[192] = {
      236, 316, 356, 412, 668, 716, 868, 1100, 1228, 1268, 1340, 1396, 1876, 1916, 2140, 2236,
      2548, 2644, 2716, 2860, 3004, 3164, 3236, 3436, 3460, 3700, 3836, 4028, 4124, 4132, 4156, 4316,
      4636, 5012, 5132, 5140, 5332, 5372, 5500, 5524, 5788, 6004, 6020, 6092, 6428, 6452, 6500, 6740,
      7244, 7316, 7372, 7444, 7772, 7844, 7924, 8020, 8164, 8308, 8332, 8348, 8788, 8804, 9116, 9140,
      9292, 9412, 9436, 9604, 10076, 10204, 10340, 10348, 10420, 10660, 10684, 10708, 11068, 11132,
      11228, 11356, 11852, 11860, 11884, 12044, 12116, 12164, 12268, 12316, 12700, 12772, 12820, 12988,
      13300, 13340, 13564, 13780, 13868, 14084, 14308, 14348, 14660, 14828, 14876, 14948, 15332, 15380,
      15484, 15532, 15604, 15764, 15788, 15796, 16292, 16420, 16516, 16580, 16940, 16964, 16988, 17228,
      17300, 17308, 17444, 17572, 18044, 18212, 18236, 18356, 18508, 18532, 18844, 18860, 19300, 19316,
      19340, 19484, 19628, 19724, 19804, 19876, 20204, 20276, 20332, 20404, 20908, 21148, 21196, 21220,
      21556, 21628, 21644, 21860, 22124, 22148, 22276, 22316, 22508, 22516, 22636, 23012, 23332, 23492,
      23516, 23524, 23620, 23812, 23948, 24188, 24212, 24412, 24484, 24644, 24788, 24932, 25004, 25100,
      25412, 25508, 25732, 25772, 26252, 26308, 26380, 26420, 26548, 26780, 26932, 26980, 27236, 27292,
      27332, 27412
    };

    const int pilotgenerator_cc_impl::trpapr_table_8K[72] = {
      250, 386, 407, 550, 591, 717, 763, 787, 797, 839, 950, 1090, 1105, 1199, 1738, 1867,
      1903, 1997, 2114, 2260, 2356, 2427, 2428, 2444, 2452, 2475, 2564, 2649, 2663, 2678, 2740, 2777,
      2819, 2986, 3097, 3134, 3253, 3284, 3323, 3442, 3596, 3694, 3719, 3751, 3763, 3836, 4154, 4257,
      4355, 4580, 4587, 4678, 4805, 5084, 5126, 5161, 5229, 5321, 5445, 5649, 5741, 5746, 5885, 5918,
      6075, 6093, 6319, 6421, 6463, 6511, 6517, 6577
    };

    const int pilotgenerator_cc_impl::trpapr_table_16K[144] = {
      421, 548, 589, 621, 644, 727, 770, 813, 857, 862, 1113, 1187, 1201, 1220, 1393, 1517,
      1821, 1899, 1924, 2003, 2023, 2143, 2146, 2290, 2474, 2482, 2597, 2644, 2749, 2818, 2951, 3014,
      3212, 3237, 3363, 3430, 3515, 3517, 3745, 3758, 4049, 4165, 4354, 4399, 4575, 4763, 4789, 4802,
      4834, 4970, 5260, 5386, 5395, 5402, 5579, 5716, 5734, 5884, 5895, 6073, 6123, 6158, 6212, 6243,
      6521, 6593, 6604, 6607, 6772, 6842, 6908, 6986, 7220, 7331, 7396, 7407, 7588, 7635, 7665, 7893,
      7925, 7949, 8019, 8038, 8167, 8289, 8295, 8338, 8549, 8555, 8660, 8857, 8925, 9007, 9057, 9121,
      9364, 9375, 9423, 9446, 9479, 9502, 9527, 9860, 9919, 9938, 10138, 10189, 10191, 10275, 10333,
      10377, 10988, 11109, 11261, 11266, 11362, 11390, 11534, 11623, 11893, 11989, 12037, 12101, 12119,
      12185, 12254, 12369, 12371, 12380, 12401, 12586, 12597, 12638, 12913, 12974, 13001, 13045, 13052,
      13111, 13143, 13150, 13151, 13300
    };

    const int pilotgenerator_cc_impl::trpapr_table_32K[288] = {
      803, 805, 811, 901, 1001, 1027, 1245, 1258, 1318, 1478, 1507, 1509, 1556, 1577, 1655, 1742,
      1978, 2001, 2056, 2110, 2164, 2227, 2305, 2356, 2408, 2522, 2563, 2780, 2805, 2879, 3010, 3019,
      3128, 3389, 3649, 3730, 3873, 4027, 4066, 4087, 4181, 4246, 4259, 4364, 4406, 4515, 4690, 4773,
      4893, 4916, 4941, 4951, 4965, 5165, 5222, 5416, 5638, 5687, 5729, 5930, 5997, 6005, 6161, 6218,
      6292, 6344, 6370, 6386, 6505, 6974, 7079, 7114, 7275, 7334, 7665, 7765, 7868, 7917, 7966, 8023,
      8055, 8089, 8091, 8191, 8374, 8495, 8651, 8690, 8755, 8821, 9139, 9189, 9274, 9561, 9611, 9692,
      9711, 9782, 9873, 9964, 10011, 10209, 10575, 10601, 10623, 10690, 10967, 11045, 11083, 11084,
      11090, 11128, 11153, 11530, 11737, 11829, 11903, 11907, 11930, 11942, 12356, 12429, 12484, 12547,
      12562, 12605, 12767, 12863, 13019, 13052, 13053, 13167, 13210, 13244, 13259, 13342, 13370, 13384,
      13447, 13694, 13918, 14002, 14077, 14111, 14216, 14243, 14270, 14450, 14451, 14456, 14479, 14653,
      14692, 14827, 14865, 14871, 14908, 15215, 15227, 15284, 15313, 15333, 15537, 15643, 15754, 15789,
      16065, 16209, 16213, 16217, 16259, 16367, 16369, 16646, 16780, 16906, 16946, 17012, 17167, 17192,
      17325, 17414, 17629, 17687, 17746, 17788, 17833, 17885, 17913, 18067, 18089, 18316, 18337, 18370,
      18376, 18440, 18550, 18680, 18910, 18937, 19047, 19052, 19117, 19383, 19396, 19496, 19601, 19778,
      19797, 20038, 20357, 20379, 20455, 20669, 20707, 20708, 20751, 20846, 20853, 20906, 21051, 21079,
      21213, 21267, 21308, 21355, 21523, 21574, 21815, 21893, 21973, 22084, 22172, 22271, 22713, 22905,
      23039, 23195, 23303, 23635, 23732, 23749, 23799, 23885, 23944, 24149, 24311, 24379, 24471, 24553,
      24585, 24611, 24616, 24621, 24761, 24789, 24844, 24847, 24977, 25015, 25160, 25207, 25283, 25351,
      25363, 25394, 25540, 25603, 25647, 25747, 25768, 25915, 25928, 26071, 26092, 26139, 26180, 26209,
      26270, 26273, 26278, 26326, 26341, 26392, 26559, 26642, 26776, 26842
    };

    const int pilotgenerator_cc_impl::trpapr_alt_table_8K[72] = {
      295, 329, 347, 365, 463, 473, 481, 553, 578, 602, 742, 749, 829, 922, 941, 1115,
      1123, 1174, 1363, 1394, 1402, 1615, 1657, 1702, 1898, 1910, 1997, 2399, 2506, 2522, 2687, 2735,
      3043, 3295, 3389, 3454, 3557, 3647, 3719, 3793, 3794, 3874, 3898, 3970, 4054, 4450, 4609, 4666,
      4829, 4855, 4879, 4961, 4969, 5171, 5182, 5242, 5393, 5545, 5567, 5618, 5630, 5734, 5861, 5897,
      5987, 5989, 6002, 6062, 6074, 6205, 6334, 6497
    };

    const int pilotgenerator_cc_impl::trpapr_alt_table_16K[144] = {
      509, 739, 770, 890, 970, 989, 1031, 1033, 1121, 1223, 1231, 1285, 1526, 1559, 1603, 1615,
      1690, 1771, 1903, 1910, 1958, 2033, 2146, 2225, 2302, 2306, 2345, 2447, 2477, 2561, 2578, 2597,
      2635, 2654, 2687, 2891, 2938, 3029, 3271, 3479, 3667, 3713, 3791, 3977, 4067, 4150, 4217, 4387,
      4501, 4541, 4657, 4733, 4742, 4963, 5011, 5149, 5311, 5362, 5491, 5531, 5609, 5722, 5747, 5798,
      5842, 5881, 5959, 5983, 6059, 6166, 6178, 6214, 6230, 6382, 6557, 6625, 6811, 6881, 6994, 7261,
      7535, 7546, 7711, 7897, 7898, 7918, 7997, 8125, 8398, 8483, 8530, 8686, 8731, 8855, 9001, 9026,
      9110, 9206, 9223, 9325, 9466, 9493, 9890, 9893, 10537, 10570, 10691, 10835, 10837, 11098, 11126,
      11146, 11198, 11270, 11393, 11629, 11657, 11795, 11867, 11909, 11983, 12046, 12107, 12119, 12353,
      12482, 12569, 12575, 12662, 12691, 12739, 12787, 12902, 12917, 12985, 13010, 13022, 13073, 13102,
      13141, 13159, 13225, 13255, 13303
    };

    const int pilotgenerator_cc_impl::trpapr_alt_table_32K[288] = {
      793, 884, 899, 914, 1004, 1183, 1198, 1276, 1300, 1339, 1348, 1444, 1487, 1490, 1766, 1870,
      1903, 1909, 1961, 2053, 2092, 2099, 2431, 2572, 2578, 2618, 2719, 2725, 2746, 2777, 2798, 2891,
      2966, 2972, 3023, 3037, 3076, 3257, 3284, 3326, 3389, 3425, 3454, 3523, 3602, 3826, 3838, 3875,
      3955, 4094, 4126, 4261, 4349, 4357, 4451, 4646, 4655, 4913, 5075, 5083, 5306, 5317, 5587, 5821,
      6038, 6053, 6062, 6137, 6268, 6286, 6490, 6517, 6529, 6554, 6593, 6671, 6751, 6827, 6845, 7043,
      7111, 7147, 7196, 7393, 7451, 7475, 7517, 7750, 7769, 7780, 8023, 8081, 8263, 8290, 8425, 8492,
      8939, 8986, 9113, 9271, 9298, 9343, 9455, 9476, 9637, 9821, 9829, 9913, 9953, 9988, 10001, 10007,
      10018, 10082, 10172, 10421, 10553, 10582, 10622, 10678, 10843, 10885, 10901, 11404, 11674, 11959,
      12007, 12199, 12227, 12290, 12301, 12629, 12631, 12658, 12739, 12866, 12977, 13121, 13294, 13843,
      13849, 13852, 13933, 14134, 14317, 14335, 14342, 14407, 14651, 14758, 14815, 14833, 14999, 15046,
      15097, 15158, 15383, 15503, 15727, 15881, 16139, 16238, 16277, 16331, 16444, 16490, 16747, 16870,
      16981, 17641, 17710, 17714, 17845, 18011, 18046, 18086, 18097, 18283, 18334, 18364, 18431, 18497,
      18527, 18604, 18686, 18709, 18731, 18740, 18749, 18772, 18893, 19045, 19075, 19087, 19091, 19099,
      19127, 19169, 19259, 19427, 19433, 19450, 19517, 19526, 19610, 19807, 19843, 19891, 20062, 20159,
      20246, 20420, 20516, 20530, 20686, 20801, 20870, 20974, 21131, 21158, 21565, 21635, 21785, 21820,
      21914, 21926, 22046, 22375, 22406, 22601, 22679, 22699, 22772, 22819, 22847, 22900, 22982, 22987,
      23063, 23254, 23335, 23357, 23561, 23590, 23711, 23753, 23902, 24037, 24085, 24101, 24115, 24167,
      24182, 24361, 24374, 24421, 24427, 24458, 24463, 24706, 24748, 24941, 25079, 25127, 25195, 25285,
      25444, 25492, 25505, 25667, 25682, 25729, 25741, 25765, 25973, 26171, 26180, 26227, 26353, 26381,
      26542, 26603, 26651, 26671, 26759, 26804, 26807, 26827
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

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_64[64] = {
      gr_complex(-0.0082, 0.0031), gr_complex( 0.0041, 0.0092), gr_complex(-0.0192, 0.0795),
      gr_complex(-0.0321, 0.0025), gr_complex(-0.0234, 0.0615), gr_complex(-0.0257, 0.0544),
      gr_complex(-0.0599, 0.0362), gr_complex(-0.0472, 0.0203), gr_complex( 0.0925, 0.0460),
      gr_complex( 0.0217, 0.0165), gr_complex(-0.1060, 0.0672), gr_complex( 0.0356, 0.0135),
      gr_complex(-0.0765, 0.0388), gr_complex(-0.1158, 0.1144), gr_complex( 0.0197, 0.1516),
      gr_complex(-0.0359, 0.0376), gr_complex(-0.0932, 0.0818), gr_complex(-0.0454, 0.1093),
      gr_complex( 0.0692, 0.0114), gr_complex(-0.0801, 0.0271), gr_complex(-0.1156, 0.0505),
      gr_complex( 0.2071, 0.0987), gr_complex( 0.0217, 0.1298), gr_complex( 0.0305, 0.1189),
      gr_complex( 0.1325, 0.1816), gr_complex( 0.0220, 0.1673), gr_complex( 0.2034, 0.1647),
      gr_complex( 0.1139, 0.0092), gr_complex(-0.0485, 0.2120), gr_complex( 0.0006, 0.0205),
      gr_complex(-0.0667, 0.0071), gr_complex(-0.2077, 0.2282), gr_complex(-0.1265, 0.0798),
      gr_complex( 0.0276, 0.1123), gr_complex(-0.0043, 0.1419), gr_complex( 0.0147, 0.0002),
      gr_complex( 0.0066, 0.0353), gr_complex(-0.1775, 0.1532), gr_complex(-0.0491, 0.0762),
      gr_complex(-0.1185, 0.1370), gr_complex( 0.1381, 0.1197), gr_complex( 0.0402, 0.1385),
      gr_complex(-0.0414, 0.0215), gr_complex( 0.0425, 0.2408), gr_complex( 0.0778, 0.1461),
      gr_complex(-0.2283, 0.0223), gr_complex( 0.0196, 0.0578), gr_complex(-0.0309, 0.1078),
      gr_complex( 0.0340, 0.0156), gr_complex( 0.0431, 0.0505), gr_complex(-0.0656, 0.1008),
      gr_complex( 0.0719, 0.1280), gr_complex(-0.0367, 0.0387), gr_complex( 0.0549, 0.0875),
      gr_complex(-0.0199, 0.0339), gr_complex( 0.0247, 0.0080), gr_complex(-0.0482, 0.0159),
      gr_complex( 0.0474, 0.0310), gr_complex(-0.0262, 0.0055), gr_complex(-0.0185, 0.0061),
      gr_complex( 0.0547, 0.0236), gr_complex(-0.0084, 0.0174), gr_complex( 0.0030, 0.0153),
      gr_complex(-0.0339, 0.0449)
    };

    const float pilotgenerator_cc_impl::miso_angle[6913] = {
      0.723570, 0.708126, 0.692728, 0.677373, 0.662061, 0.646787, 0.631552, 0.616353,
      0.601188, 0.586054, 0.570951, 0.555876, 0.540828, 0.525804, 0.510803, 0.495824,
      0.480865, 0.465923, 0.450998, 0.436089, 0.421192, 0.406308, 0.391435, 0.376572,
      0.361717, 0.346868, 0.332026, 0.317189, 0.302355, 0.287525, 0.272696, 0.257868,
      0.243040, 0.228212, 0.213383, 0.198551, 0.183717, 0.168880, 0.154039, 0.139194,
      0.124344, 0.109489, 0.094630, 0.079765, 0.064894, 0.050017, 0.035134, 0.020246,
      0.005352, 6.273636, 6.258729, 6.243816, 6.228898, 6.213974, 6.199045, 6.184110,
      6.169170, 6.154227, 6.139278, 6.124326, 6.109370, 6.094411, 6.079450, 6.064487,
      6.049520, 6.034554, 6.019588, 6.004620, 5.989653, 5.974688, 5.959723, 5.944761,
      5.929802, 5.914846, 5.899895, 5.884949, 5.870008, 5.855072, 5.840144, 5.825223,
      5.810310, 5.795406, 5.780511, 5.765627, 5.750753, 5.735890, 5.721040, 5.706203,
      5.691379, 5.676569, 5.661773, 5.646993, 5.632228, 5.617480, 5.602749, 5.588035,
      5.573339, 5.558663, 5.544004, 5.529367, 5.514748, 5.500151, 5.485574, 5.471018,
      5.456484, 5.441972, 5.427483, 5.413016, 5.398572, 5.384151, 5.369753, 5.355380,
      5.341029, 5.326703, 5.312401, 5.298122, 5.283868, 5.269637, 5.255431, 5.241249,
      5.227090, 5.212955, 5.198844, 5.184755, 5.170691, 5.156649, 5.142630, 5.128633,
      5.114656, 5.100703, 5.086770, 5.072858, 5.058966, 5.045093, 5.031240, 5.017404,
      5.003587, 4.989787, 4.976003, 4.962234, 4.948482, 4.934743, 4.921018, 4.907305,
      4.893604, 4.879915, 4.866235, 4.852565, 4.838902, 4.825248, 4.811601, 4.797958,
      4.784320, 4.770686, 4.757054, 4.743423, 4.729794, 4.716164, 4.702533, 4.688899,
      4.675261, 4.661619, 4.647972, 4.634318, 4.620657, 4.606987, 4.593308, 4.579618,
      4.565917, 4.552203, 4.538476, 4.524735, 4.510979, 4.497206, 4.483417, 4.469609,
      4.455783, 4.441938, 4.428073, 4.414186, 4.400278, 4.386347, 4.372393, 4.358416,
      4.344413, 4.330386, 4.316332, 4.302253, 4.288147, 4.274014, 4.259853, 4.245664,
      4.231445, 4.217198, 4.202923, 4.188617, 4.174282, 4.159915, 4.145519, 4.131092,
      4.116635, 4.102147, 4.087626, 4.073075, 4.058494, 4.043880, 4.029236, 4.014560,
      3.999853, 3.985115, 3.970346, 3.955545, 3.940715, 3.925853, 3.910961, 3.896039,
      3.881086, 3.866104, 3.851092, 3.836051, 3.820980, 3.805881, 3.790754, 3.775598,
      3.760414, 3.745203, 3.729965, 3.714700, 3.699409, 3.684092, 3.668749, 3.653381,
      3.637988, 3.622571, 3.607129, 3.591665, 3.576177, 3.560666, 3.545132, 3.529577,
      3.514001, 3.498403, 3.482784, 3.467145, 3.451486, 3.435807, 3.420109, 3.404392,
      3.388657, 3.372903, 3.357132, 3.341343, 3.325537, 3.309714, 3.293874, 3.278018,
      3.262146, 3.246258, 3.230353, 3.214434, 3.198499, 3.182549, 3.166585, 3.150605,
      3.134611, 3.118602, 3.102579, 3.086540, 3.070487, 3.054420, 3.038338, 3.022242,
      3.006131, 2.990005, 2.973864, 2.957708, 2.941536, 2.925350, 2.909147, 2.892929,
      2.876695, 2.860444, 2.844176, 2.827891, 2.811589, 2.795268, 2.778930, 2.762572,
      2.746195, 2.729798, 2.713381, 2.696943, 2.680483, 2.664002, 2.647497, 2.630969,
      2.614418, 2.597841, 2.581239, 2.564611, 2.547956, 2.531273, 2.514561, 2.497820,
      2.481049, 2.464247, 2.447413, 2.430546, 2.413645, 2.396710, 2.379740, 2.362733,
      2.345688, 2.328606, 2.311484, 2.294322, 2.277118, 2.259873, 2.242584, 2.225251,
      2.207872, 2.190448, 2.172977, 2.155457, 2.137889, 2.120270, 2.102601, 2.084879,
      2.067104, 2.049276, 2.031393, 2.013454, 1.995458, 1.977404, 1.959292, 1.941121,
      1.922889, 1.904596, 1.886241, 1.867823, 1.849342, 1.830796, 1.812185, 1.793509,
      1.774765, 1.755955, 1.737075, 1.718128, 1.699111, 1.680024, 1.660867, 1.641638,
      1.622339, 1.602966, 1.583521, 1.564003, 1.544412, 1.524747, 1.505007, 1.485193,
      1.465304, 1.445339, 1.425299, 1.405184, 1.384992, 1.364725, 1.344380, 1.323960,
      1.303463, 1.282890, 1.262240, 1.241515, 1.220712, 1.199833, 1.178878, 1.157846,
      1.136739, 1.115556, 1.094297, 1.072963, 1.051554, 1.030070, 1.008512, 0.986880,
      0.965174, 0.943396, 0.921545, 0.899621, 0.877625, 0.855559, 0.833422, 0.811215,
      0.788939, 0.766594, 0.744182, 0.721702, 0.699156, 0.676544, 0.653868, 0.631128,
      0.608324, 0.585458, 0.562531, 0.539544, 0.516498, 0.493393, 0.470230, 0.447012,
      0.423738, 0.400411, 0.377030, 0.353598, 0.330115, 0.306582, 0.283002, 0.259374,
      0.235701, 0.211983, 0.188223, 0.164421, 0.140578, 0.116697, 0.092777, 0.068822,
      0.044832, 0.020808, 6.279939, 6.255853, 6.231739, 6.207597, 6.183429, 6.159237,
      6.135023, 6.110786, 6.086532, 6.062258, 6.037968, 6.013663, 5.989346, 5.965016,
      5.940677, 5.916329, 5.891974, 5.867615, 5.843252, 5.818888, 5.794523, 5.770160,
      5.745800, 5.721446, 5.697097, 5.672757, 5.648427, 5.624108, 5.599803, 5.575512,
      5.551238, 5.526982, 5.502746, 5.478530, 5.454338, 5.430170, 5.406028, 5.381914,
      5.357830, 5.333776, 5.309754, 5.285766, 5.261813, 5.237898, 5.214020, 5.190183,
      5.166387, 5.142633, 5.118924, 5.095260, 5.071643, 5.048074, 5.024555, 5.001087,
      4.977670, 4.954308, 4.931000, 4.907748, 4.884553, 4.861416, 4.838339, 4.815323,
      4.792367, 4.769476, 4.746648, 4.723884, 4.701187, 4.678556, 4.655993, 4.633500,
      4.611075, 4.588721, 4.566438, 4.544228, 4.522089, 4.500025, 4.478035, 4.456119,
      4.434279, 4.412515, 4.390827, 4.369217, 4.347685, 4.326230, 4.304854, 4.283556,
      4.262338, 4.241199, 4.220140, 4.199161, 4.178261, 4.157441, 4.136703, 4.116043,
      4.095464, 4.074965, 4.054546, 4.034207, 4.013947, 3.993767, 3.973665, 3.953643,
      3.933698, 3.913831, 3.894042, 3.874329, 3.854691, 3.835130, 3.815643, 3.796230,
      3.776890, 3.757621, 3.738424, 3.719297, 3.700239, 3.681250, 3.662327, 3.643469,
      3.624676, 3.605945, 3.587277, 3.568668, 3.550118, 3.531625, 3.513188, 3.494804,
      3.476472, 3.458191, 3.439959, 3.421773, 3.403632, 3.385535, 3.367477, 3.349460,
      3.331478, 3.313532, 3.295619, 3.277736, 3.259881, 3.242053, 3.224248, 3.206465,
      3.188702, 3.170955, 3.153223, 3.135504, 3.117794, 3.100092, 3.082395, 3.064701,
      3.047008, 3.029312, 3.011612, 2.993906, 2.976189, 2.958462, 2.940721, 2.922963,
      2.905186, 2.887388, 2.869567, 2.851721, 2.833846, 2.815941, 2.798004, 2.780032,
      2.762023, 2.743976, 2.725887, 2.707755, 2.689577, 2.671353, 2.653079, 2.634753,
      2.616375, 2.597941, 2.579451, 2.560902, 2.542292, 2.523621, 2.504885, 2.486084,
      2.467215, 2.448278, 2.429270, 2.410192, 2.391039, 2.371812, 2.352509, 2.333129,
      2.313670, 2.294131, 2.274510, 2.254808, 2.235023, 2.215152, 2.195195, 2.175153,
      2.155022, 2.134803, 2.114493, 2.094094, 2.073602, 2.053018, 2.032341, 2.011569,
      1.990704, 1.969742, 1.948684, 1.927528, 1.906275, 1.884923, 1.863473, 1.841923,
      1.820272, 1.798521, 1.776667, 1.754713, 1.732655, 1.710494, 1.688230, 1.665862,
      1.643389, 1.620812, 1.598130, 1.575341, 1.552447, 1.529447, 1.506340, 1.483126,
      1.459804, 1.436376, 1.412839, 1.389195, 1.365443, 1.341582, 1.317613, 1.293535,
      1.269349, 1.245054, 1.220650, 1.196138, 1.171517, 1.146787, 1.121949, 1.097002,
      1.071948, 1.046785, 1.021514, 0.996136, 0.970652, 0.945060, 0.919363, 0.893558,
      0.867649, 0.841636, 0.815519, 0.789298, 0.762975, 0.736550, 0.710025, 0.683399,
      0.656675, 0.629854, 0.602935, 0.575923, 0.548816, 0.521616, 0.494326, 0.466946,
      0.439479, 0.411926, 0.384288, 0.356568, 0.328768, 0.300890, 0.272936, 0.244909,
      0.216810, 0.188642, 0.160407, 0.132109, 0.103750, 0.075332, 0.046860, 0.018334,
      6.272944, 6.244324, 6.215659, 6.186955, 6.158214, 6.129440, 6.100637, 6.071807,
      6.042954, 6.014083, 5.985197, 5.956299, 5.927393, 5.898483, 5.869573, 5.840666,
      5.811768, 5.782880, 5.754008, 5.725155, 5.696326, 5.667523, 5.638752, 5.610016,
      5.581318, 5.552662, 5.524053, 5.495495, 5.466990, 5.438542, 5.410155, 5.381834,
      5.353580, 5.325398, 5.297292, 5.269264, 5.241318, 5.213456, 5.185683, 5.158001,
      5.130413, 5.102922, 5.075531, 5.048242, 5.021059, 4.993983, 4.967018, 4.940165,
      4.913427, 4.886807, 4.860306, 4.833927, 4.807671, 4.781540, 4.755537, 4.729663,
      4.703919, 4.678308, 4.652830, 4.627488, 4.602283, 4.577216, 4.552288, 4.527501,
      4.502855, 4.478353, 4.453994, 4.429780, 4.405712, 4.381790, 4.358016, 4.334391,
      4.310913, 4.287586, 4.264409, 4.241383, 4.218508, 4.195786, 4.173215, 4.150798,
      4.128534, 4.106424, 4.084468, 4.062667, 4.041020, 4.019529, 3.998194, 3.977014,
      3.955990, 3.935122, 3.914410, 3.893855, 3.873457, 3.853216, 3.833131, 3.813204,
      3.793433, 3.773820, 3.754364, 3.735065, 3.715923, 3.696939, 3.678111, 3.659441,
      3.640928, 3.622571, 3.604372, 3.586329, 3.568443, 3.550713, 3.533139, 3.515721,
      3.498459, 3.481351, 3.464399, 3.447602, 3.430958, 3.414469, 3.398133, 3.381948,
      3.365918, 3.350038, 3.334310, 3.318732, 3.303304, 3.288026, 3.272895, 3.257912,
      3.243075, 3.228384, 3.213838, 3.199435, 3.185175, 3.171056, 3.157078, 3.143239,
      3.129537, 3.115972, 3.102541, 3.089244, 3.076079, 3.063045, 3.050139, 3.037360,
      3.024707, 3.012177, 2.999768, 2.987480, 2.975309, 2.963254, 2.951313, 2.939483,
      2.927762, 2.916149, 2.904640, 2.893235, 2.881929, 2.870722, 2.859610, 2.848590,
      2.837662, 2.826822, 2.816067, 2.805396, 2.794805, 2.784292, 2.773855, 2.763490,
      2.753196, 2.742970, 2.732809, 2.722711, 2.712672, 2.702692, 2.692766, 2.682894,
      2.673072, 2.663298, 2.653570, 2.643885, 2.634241, 2.624636, 2.615068, 2.605534,
      2.596033, 2.586563, 2.577121, 2.567706, 2.558316, 2.548949, 2.539603, 2.530278,
      2.520971, 2.511681, 2.502407, 2.493147, 2.483900, 2.474665, 2.465441, 2.456226,
      2.447021, 2.437824, 2.428634, 2.419450, 2.410273, 2.401100, 2.391933, 2.382771,
      2.373612, 2.364457, 2.355306, 2.346159, 2.337014, 2.327874, 2.318736, 2.309602,
      2.300472, 2.291346, 2.282223, 2.273106, 2.263993, 2.254885, 2.245783, 2.236686,
      2.227597, 2.218515, 2.209441, 2.200374, 2.191317, 2.182269, 2.173232, 2.164207,
      2.155192, 2.146192, 2.137203, 2.128229, 2.119271, 2.110327, 2.101401, 2.092493,
      2.083602, 2.074731, 2.065879, 2.057048, 2.048239, 2.039452, 2.030689, 2.021949,
      2.013234, 2.004544, 1.995880, 1.987243, 1.978634, 1.970053, 1.961501, 1.952978,
      1.944486, 1.936023, 1.927592, 1.919193, 1.910825, 1.902489, 1.894186, 1.885917,
      1.877680, 1.869477, 1.861308, 1.853173, 1.845072, 1.837004, 1.828971, 1.820971,
      1.813006, 1.805074, 1.797175, 1.789310, 1.781478, 1.773679, 1.765912, 1.758176,
      1.750472, 1.742799, 1.735156, 1.727542, 1.719957, 1.712400, 1.704871, 1.697368,
      1.689890, 1.682437, 1.675008, 1.667602, 1.660217, 1.652853, 1.645508, 1.638182,
      1.630873, 1.623580, 1.616302, 1.609037, 1.601785, 1.594544, 1.587312, 1.580089,
      1.572873, 1.565663, 1.558457, 1.551255, 1.544055, 1.536854, 1.529653, 1.522450,
      1.515242, 1.508031, 1.500814, 1.493589, 1.486356, 1.479113, 1.471859, 1.464593,
      1.457314, 1.450021, 1.442712, 1.435386, 1.428044, 1.420683, 1.413303, 1.405903,
      1.398481, 1.391038, 1.383573, 1.376085, 1.368572, 1.361035, 1.353473, 1.345885,
      1.338272, 1.330632, 1.322965, 1.315271, 1.307550, 1.299800, 1.292024, 1.284219,
      1.276386, 1.268525, 1.260636, 1.252718, 1.244773, 1.236800, 1.228798, 1.220769,
      1.212713, 1.204629, 1.196519, 1.188382, 1.180219, 1.172030, 1.163816, 1.155576,
      1.147313, 1.139024, 1.130713, 1.122378, 1.114020, 1.105641, 1.097240, 1.088818,
      1.080375, 1.071913, 1.063432, 1.054931, 1.046413, 1.037876, 1.029323, 1.020754,
      1.012168, 1.003567, 0.994950, 0.986320, 0.977675, 0.969017, 0.960346, 0.951661,
      0.942965, 0.934256, 0.925537, 0.916804, 0.908062, 0.899308, 0.890543, 0.881768,
      0.872982, 0.864186, 0.855379, 0.846562, 0.837734, 0.828895, 0.820045, 0.811183,
      0.802311, 0.793426, 0.784529, 0.775619, 0.766696, 0.757758, 0.748806, 0.739839,
      0.730855, 0.721855, 0.712837, 0.703799, 0.694743, 0.685664, 0.676565, 0.667442,
      0.658294, 0.649121, 0.639920, 0.630691, 0.621431, 0.612140, 0.602816, 0.593457,
      0.584061, 0.574627, 0.565154, 0.555637, 0.546077, 0.536472, 0.526820, 0.517117,
      0.507363, 0.497555, 0.487692, 0.477771, 0.467790, 0.457747, 0.447640, 0.437467,
      0.427226, 0.416914, 0.406529, 0.396069, 0.385533, 0.374917, 0.364220, 0.353439,
      0.342573, 0.331620, 0.320576, 0.309441, 0.298212, 0.286887, 0.275464, 0.263941,
      0.252317, 0.240589, 0.228755, 0.216814, 0.204764, 0.192603, 0.180330, 0.167943,
      0.155439, 0.142817, 0.130077, 0.117217, 0.104234, 0.091128, 0.077897, 0.064540,
      0.051056, 0.037443, 0.023700, 0.009826, 6.279006, 6.264867, 6.250594, 6.236186,
      6.221642, 6.206961, 6.192142, 6.177184, 6.162087, 6.146851, 6.131473, 6.115954,
      6.100293, 6.084489, 6.068542, 6.052451, 6.036216, 6.019837, 6.003313, 5.986643,
      5.969827, 5.952866, 5.935757, 5.918503, 5.901102, 5.883554, 5.865858, 5.848016,
      5.830026, 5.811888, 5.793603, 5.775171, 5.756591, 5.737864, 5.718989, 5.699967,
      5.680799, 5.661483, 5.642020, 5.622412, 5.602657, 5.582756, 5.562710, 5.542518,
      5.522182, 5.501702, 5.481078, 5.460311, 5.439402, 5.418350, 5.397158, 5.375824,
      5.354352, 5.332740, 5.310990, 5.289104, 5.267081, 5.244923, 5.222631, 5.200207,
      5.177652, 5.154966, 5.132152, 5.109210, 5.086143, 5.062951, 5.039637, 5.016202,
      4.992648, 4.968977, 4.945191, 4.921292, 4.897282, 4.873164, 4.848938, 4.824609,
      4.800179, 4.775648, 4.751022, 4.726302, 4.701490, 4.676591, 4.651607, 4.626540,
      4.601394, 4.576172, 4.550879, 4.525515, 4.500085, 4.474593, 4.449042, 4.423436,
      4.397777, 4.372071, 4.346320, 4.320529, 4.294700, 4.268839, 4.242949, 4.217033,
      4.191097, 4.165143, 4.139176, 4.113200, 4.087219, 4.061236, 4.035255, 4.009283,
      3.983320, 3.957372, 3.931443, 3.905536, 3.879655, 3.853805, 3.827989, 3.802210,
      3.776473, 3.750780, 3.725137, 3.699545, 3.674010, 3.648533, 3.623119, 3.597770,
      3.572491, 3.547283, 3.522150, 3.497096, 3.472122, 3.447232, 3.422429, 3.397714,
      3.373092, 3.348564, 3.324132, 3.299800, 3.275569, 3.251442, 3.227421, 3.203506,
      3.179702, 3.156009, 3.132430, 3.108966, 3.085618, 3.062389, 3.039281, 3.016293,
      2.993429, 2.970689, 2.948075, 2.925588, 2.903228, 2.880998, 2.858898, 2.836930,
      2.815094, 2.793392, 2.771824, 2.750391, 2.729094, 2.707934, 2.686912, 2.666028,
      2.645283, 2.624679, 2.604214, 2.583891, 2.563709, 2.543670, 2.523773, 2.504020,
      2.484411, 2.464946, 2.445625, 2.426450, 2.407421, 2.388537, 2.369800, 2.351210,
      2.332767, 2.314472, 2.296324, 2.278324, 2.260473, 2.242770, 2.225217, 2.207812,
      2.190557, 2.173451, 2.156495, 2.139688, 2.123031, 2.106525, 2.090168, 2.073961,
      2.057904, 2.041997, 2.026240, 2.010633, 1.995175, 1.979867, 1.964708, 1.949699,
      1.934838, 1.920125, 1.905560, 1.891143, 1.876873, 1.862749, 1.848771, 1.834939,
      1.821251, 1.807706, 1.794305, 1.781046, 1.767928, 1.754950, 1.742111, 1.729409,
      1.716844, 1.704415, 1.692119, 1.679955, 1.667922, 1.656019, 1.644242, 1.632592,
      1.621065, 1.609660, 1.598374, 1.587207, 1.576155, 1.565216, 1.554388, 1.543668,
      1.533055, 1.522545, 1.512136, 1.501825, 1.491610, 1.481487, 1.471455, 1.461508,
      1.451646, 1.441865, 1.432161, 1.422533, 1.412976, 1.403487, 1.394064, 1.384703,
      1.375402, 1.366155, 1.356962, 1.347818, 1.338720, 1.329666, 1.320650, 1.311672,
      1.302728, 1.293814, 1.284928, 1.276066, 1.267226, 1.258405, 1.249600, 1.240808,
      1.232027, 1.223254, 1.214486, 1.205722, 1.196958, 1.188192, 1.179423, 1.170648,
      1.161864, 1.153071, 1.144267, 1.135448, 1.126615, 1.117765, 1.108897, 1.100010,
      1.091103, 1.082173, 1.073220, 1.064245, 1.055244, 1.046217, 1.037166, 1.028087,
      1.018981, 1.009848, 1.000687, 0.991498, 0.982280, 0.973035, 0.963762, 0.954460,
      0.945131, 0.935774, 0.926390, 0.916978, 0.907541, 0.898078, 0.888588, 0.879075,
      0.869537, 0.859976, 0.850393, 0.840788, 0.831163, 0.821517, 0.811853, 0.802170,
      0.792471, 0.782756, 0.773027, 0.763284, 0.753529, 0.743762, 0.733986, 0.724201,
      0.714408, 0.704609, 0.694804, 0.684997, 0.675187, 0.665374, 0.655563, 0.645753,
      0.635945, 0.626141, 0.616342, 0.606550, 0.596765, 0.586989, 0.577223, 0.567468,
      0.557725, 0.547997, 0.538283, 0.528585, 0.518903, 0.509240, 0.499595, 0.489971,
      0.480367, 0.470785, 0.461226, 0.451690, 0.442178, 0.432692, 0.423231, 0.413796,
      0.404388, 0.395007, 0.385654, 0.376329, 0.367033, 0.357766, 0.348527, 0.339318,
      0.330138, 0.320987, 0.311865, 0.302773, 0.293709, 0.284673, 0.275665, 0.266685,
      0.257732, 0.248806, 0.239905, 0.231029, 0.222177, 0.213349, 0.204542, 0.195756,
      0.186990, 0.178242, 0.169512, 0.160797, 0.152096, 0.143408, 0.134730, 0.126062,
      0.117400, 0.108744, 0.100091, 0.091439, 0.082787, 0.074131, 0.065470, 0.056801,
      0.048123, 0.039431, 0.030725, 0.022002, 0.013259, 0.004493, 6.278888, 6.270070,
      6.261221, 6.252339, 6.243422, 6.234467, 6.225470, 6.216429, 6.207342, 6.198206,
      6.189019, 6.179777, 6.170477, 6.161119, 6.151698, 6.142212, 6.132659, 6.123035,
      6.113340, 6.103571, 6.093724, 6.083799, 6.073792, 6.063700, 6.053523, 6.043259,
      6.032905, 6.022458, 6.011918, 6.001283, 5.990550, 5.979718, 5.968786, 5.957751,
      5.946613, 5.935370, 5.924021, 5.912563, 5.900998, 5.889321, 5.877534, 5.865634,
      5.853622, 5.841496, 5.829254, 5.816896, 5.804423, 5.791832, 5.779124, 5.766297,
      5.753352, 5.740288, 5.727103, 5.713799, 5.700375, 5.686830, 5.673164, 5.659378,
      5.645470, 5.631440, 5.617290, 5.603018, 5.588625, 5.574111, 5.559475, 5.544718,
      5.529840, 5.514842, 5.499722, 5.484482, 5.469121, 5.453640, 5.438040, 5.422319,
      5.406480, 5.390522, 5.374444, 5.358248, 5.341934, 5.325503, 5.308954, 5.292288,
      5.275506, 5.258608, 5.241594, 5.224464, 5.207221, 5.189862, 5.172390, 5.154805,
      5.137106, 5.119295, 5.101373, 5.083339, 5.065195, 5.046939, 5.028575, 5.010101,
      4.991519, 4.972830, 4.954032, 4.935129, 4.916119, 4.897004, 4.877785, 4.858460,
      4.839035, 4.819505, 4.799874, 4.780143, 4.760311, 4.740380, 4.720350, 4.700223,
      4.679999, 4.659679, 4.639265, 4.618757, 4.598156, 4.577463, 4.556680, 4.535806,
      4.514844, 4.493795, 4.472660, 4.451440, 4.430137, 4.408751, 4.387283, 4.365737,
      4.344112, 4.322411, 4.300634, 4.278783, 4.256861, 4.234867, 4.212806, 4.190677,
      4.168482, 4.146224, 4.123904, 4.101524, 4.079087, 4.056593, 4.034045, 4.011445,
      3.988795, 3.966097, 3.943354, 3.920566, 3.897738, 3.874870, 3.851966, 3.829027,
      3.806055, 3.783054, 3.760026, 3.736972, 3.713896, 3.690799, 3.667685, 3.644556,
      3.621414, 3.598261, 3.575101, 3.551936, 3.528768, 3.505601, 3.482436, 3.459276,
      3.436123, 3.412980, 3.389850, 3.366736, 3.343638, 3.320561, 3.297507, 3.274477,
      3.251475, 3.228503, 3.205563, 3.182657, 3.159789, 3.136959, 3.114171, 3.091427,
      3.068727, 3.046077, 3.023476, 3.000927, 2.978432, 2.955993, 2.933612, 2.911292,
      2.889033, 2.866837, 2.844707, 2.822643, 2.800648, 2.778723, 2.756870, 2.735090,
      2.713386, 2.691757, 2.670205, 2.648733, 2.627341, 2.606029, 2.584801, 2.563655,
      2.542595, 2.521621, 2.500733, 2.479933, 2.459221, 2.438599, 2.418068, 2.397627,
      2.377279, 2.357023, 2.336860, 2.316792, 2.296818, 2.276939, 2.257156, 2.237469,
      2.217879, 2.198387, 2.178991, 2.159694, 2.140496, 2.121396, 2.102395, 2.083494,
      2.064692, 2.045990, 2.027389, 2.008889, 1.990489, 1.972190, 1.953991, 1.935894,
      1.917899, 1.900005, 1.882213, 1.864522, 1.846934, 1.829447, 1.812063, 1.794780,
      1.777599, 1.760521, 1.743544, 1.726670, 1.709898, 1.693229, 1.676661, 1.660196,
      1.643833, 1.627573, 1.611413, 1.595356, 1.579402, 1.563549, 1.547798, 1.532149,
      1.516602, 1.501156, 1.485812, 1.470569, 1.455428, 1.440388, 1.425449, 1.410611,
      1.395874, 1.381237, 1.366701, 1.352265, 1.337929, 1.323693, 1.309557, 1.295520,
      1.281583, 1.267744, 1.254004, 1.240362, 1.226819, 1.213372, 1.200024, 1.186772,
      1.173617, 1.160558, 1.147595, 1.134727, 1.121955, 1.109276, 1.096692, 1.084201,
      1.071802, 1.059495, 1.047281, 1.035157, 1.023123, 1.011179, 0.999323, 0.987556,
      0.975876, 0.964282, 0.952774, 0.941350, 0.930010, 0.918752, 0.907577, 0.896482,
      0.885467, 0.874530, 0.863671, 0.852888, 0.842180, 0.831545, 0.820983, 0.810493,
      0.800073, 0.789721, 0.779436, 0.769216, 0.759061, 0.748969, 0.738938, 0.728967,
      0.719054, 0.709197, 0.699395, 0.689646, 0.679949, 0.670301, 0.660702, 0.651149,
      0.641641, 0.632175, 0.622751, 0.613366, 0.604018, 0.594706, 0.585428, 0.576182,
      0.566967, 0.557780, 0.548619, 0.539483, 0.530370, 0.521279, 0.512207, 0.503153,
      0.494114, 0.485091, 0.476079, 0.467079, 0.458087, 0.449104, 0.440126, 0.431152,
      0.422182, 0.413213, 0.404243, 0.395272, 0.386298, 0.377320, 0.368335, 0.359344,
      0.350346, 0.341337, 0.332317, 0.323287, 0.314243, 0.305185, 0.296113, 0.287025,
      0.277921, 0.268799, 0.259659, 0.250500, 0.241321, 0.232123, 0.222903, 0.213662,
      0.204400, 0.195115, 0.185807, 0.176477, 0.167123, 0.157746, 0.148345, 0.138920,
      0.129471, 0.119998, 0.110501, 0.100979, 0.091434, 0.081865, 0.072271, 0.062654,
      0.053014, 0.043351, 0.033664, 0.023955, 0.014223, 0.004469, 6.277879, 6.268083,
      6.258266, 6.248429, 6.238571, 6.228695, 6.218800, 6.208887, 6.198956, 6.189008,
      6.179044, 6.169064, 6.159069, 6.149060, 6.139036, 6.129000, 6.118951, 6.108891,
      6.098818, 6.088737, 6.078644, 6.068543, 6.058434, 6.048316, 6.038192, 6.028061,
      6.017925, 6.007783, 5.997638, 5.987488, 5.977335, 5.967179, 5.957021, 5.946862,
      5.936702, 5.926540, 5.916379, 5.906219, 5.896059, 5.885900, 5.875744, 5.865589,
      5.855435, 5.845285, 5.835137, 5.824993, 5.814850, 5.804712, 5.794576, 5.784443,
      5.774313, 5.764186, 5.754063, 5.743941, 5.733823, 5.723706, 5.713592, 5.703478,
      5.693365, 5.683253, 5.673141, 5.663027, 5.652912, 5.642795, 5.632675, 5.622551,
      5.612422, 5.602287, 5.592144, 5.581994, 5.571835, 5.561665, 5.551483, 5.541288,
      5.531078, 5.520852, 5.510609, 5.500346, 5.490062, 5.479756, 5.469426, 5.459069,
      5.448684, 5.438270, 5.427824, 5.417343, 5.406827, 5.396273, 5.385679, 5.375044,
      5.364364, 5.353637, 5.342862, 5.332036, 5.321157, 5.310223, 5.299230, 5.288178,
      5.277064, 5.265885, 5.254641, 5.243326, 5.231940, 5.220481, 5.208945, 5.197332,
      5.185639, 5.173862, 5.162002, 5.150054, 5.138016, 5.125888, 5.113667, 5.101350,
      5.088936, 5.076422, 5.063807, 5.051089, 5.038266, 5.025336, 5.012298, 4.999148,
      4.985888, 4.972513, 4.959023, 4.945415, 4.931690, 4.917845, 4.903879, 4.889790,
      4.875578, 4.861240, 4.846776, 4.832184, 4.817464, 4.802614, 4.787634, 4.772521,
      4.757277, 4.741899, 4.726387, 4.710740, 4.694957, 4.679038, 4.662981, 4.646787,
      4.630455, 4.613983, 4.597373, 4.580623, 4.563732, 4.546701, 4.529530, 4.512217,
      4.494762, 4.477166, 4.459428, 4.441548, 4.423526, 4.405361, 4.387054, 4.368605,
      4.350013, 4.331279, 4.312402, 4.293384, 4.274222, 4.254919, 4.235473, 4.215886,
      4.196157, 4.176287, 4.156276, 4.136124, 4.115831, 4.095399, 4.074826, 4.054114,
      4.033264, 4.012275, 3.991148, 3.969883, 3.948482, 3.926944, 3.905270, 3.883462,
      3.861519, 3.839443, 3.817234, 3.794893, 3.772420, 3.749818, 3.727086, 3.704225,
      3.681237, 3.658123, 3.634883, 3.611519, 3.588032, 3.564423, 3.540694, 3.516845,
      3.492878, 3.468795, 3.444597, 3.420286, 3.395862, 3.371328, 3.346686, 3.321937,
      3.297082, 3.272125, 3.247066, 3.221907, 3.196651, 3.171299, 3.145855, 3.120319,
      3.094694, 3.068983, 3.043186, 3.017309, 2.991351, 2.965316, 2.939208, 2.913027,
      2.886776, 2.860460, 2.834079, 2.807637, 2.781136, 2.754580, 2.727971, 2.701312,
      2.674606, 2.647856, 2.621065, 2.594236, 2.567373, 2.540477, 2.513552, 2.486601,
      2.459628, 2.432635, 2.405625, 2.378602, 2.351568, 2.324528, 2.297482, 2.270435,
      2.243390, 2.216350, 2.189318, 2.162296, 2.135288, 2.108297, 2.081325, 2.054376,
      2.027451, 2.000555, 1.973690, 1.946857, 1.920061, 1.893304, 1.866587, 1.839915,
      1.813289, 1.786711, 1.760184, 1.733711, 1.707293, 1.680933, 1.654633, 1.628394,
      1.602220, 1.576111, 1.550071, 1.524100, 1.498201, 1.472374, 1.446624, 1.420949,
      1.395353, 1.369836, 1.344400, 1.319047, 1.293778, 1.268594, 1.243496, 1.218486,
      1.193564, 1.168733, 1.143992, 1.119344, 1.094788, 1.070326, 1.045958, 1.021686,
      0.997510, 0.973431, 0.949449, 0.925565, 0.901780, 0.878095, 0.854509, 0.831023,
      0.807638, 0.784354, 0.761171, 0.738090, 0.715111, 0.692235, 0.669460, 0.646788,
      0.624219, 0.601752, 0.579388, 0.557127, 0.534968, 0.512911, 0.490958, 0.469106,
      0.447356, 0.425709, 0.404163, 0.382718, 0.361374, 0.340130, 0.318986, 0.297942,
      0.276997, 0.256150, 0.235400, 0.214747, 0.194191, 0.173730, 0.153364, 0.133091,
      0.112911, 0.092823, 0.072825, 0.052917, 0.033097, 0.013365, 6.276904, 6.257341,
      6.237862, 6.218464, 6.199146, 6.179907, 6.160745, 6.141657, 6.122643, 6.103700,
      6.084826, 6.066021, 6.047280, 6.028603, 6.009987, 5.991429, 5.972929, 5.954482,
      5.936087, 5.917742, 5.899443, 5.881188, 5.862974, 5.844799, 5.826661, 5.808555,
      5.790479, 5.772430, 5.754406, 5.736403, 5.718419, 5.700449, 5.682491, 5.664543,
      5.646600, 5.628659, 5.610718, 5.592772, 5.574819, 5.556855, 5.538878, 5.520884,
      5.502869, 5.484830, 5.466764, 5.448668, 5.430538, 5.412372, 5.394166, 5.375916,
      5.357619, 5.339273, 5.320874, 5.302420, 5.283906, 5.265331, 5.246690, 5.227982,
      5.209203, 5.190351, 5.171422, 5.152414, 5.133324, 5.114150, 5.094889, 5.075538,
      5.056095, 5.036558, 5.016924, 4.997190, 4.977356, 4.957418, 4.937375, 4.917224,
      4.896964, 4.876592, 4.856106, 4.835505, 4.814787, 4.793951, 4.772994, 4.751915,
      4.730713, 4.709385, 4.687932, 4.666350, 4.644640, 4.622799, 4.600826, 4.578721,
      4.556481, 4.534107, 4.511597, 4.488949, 4.466163, 4.443239, 4.420174, 4.396969,
      4.373622, 4.350133, 4.326501, 4.302725, 4.278805, 4.254739, 4.230528, 4.206171,
      4.181666, 4.157014, 4.132215, 4.107267, 4.082170, 4.056925, 4.031529, 4.005983,
      3.980288, 3.954442, 3.928445, 3.902297, 3.875998, 3.849547, 3.822945, 3.796191,
      3.769284, 3.742226, 3.715016, 3.687654, 3.660140, 3.632473, 3.604655, 3.576685,
      3.548564, 3.520291, 3.491866, 3.463291, 3.434564, 3.405688, 3.376661, 3.347485,
      3.318159, 3.288684, 3.259062, 3.229291, 3.199374, 3.169310, 3.139102, 3.108747,
      3.078250, 3.047608, 3.016826, 2.985901, 2.954837, 2.923634, 2.892292, 2.860815,
      2.829201, 2.797453, 2.765574, 2.733562, 2.701422, 2.669152, 2.636757, 2.604237,
      2.571593, 2.538829, 2.505944, 2.472943, 2.439825, 2.406595, 2.373252, 2.339800,
      2.306241, 2.272577, 2.238810, 2.204943, 2.170978, 2.136916, 2.102761, 2.068515,
      2.034181, 1.999761, 1.965257, 1.930672, 1.896008, 1.861269, 1.826455, 1.791571,
      1.756619, 1.721601, 1.686520, 1.651378, 1.616177, 1.580921, 1.545611, 1.510251,
      1.474842, 1.439387, 1.403888, 1.368348, 1.332768, 1.297152, 1.261500, 1.225815,
      1.190100, 1.154356, 1.118585, 1.082789, 1.046971, 1.011130, 0.975269, 0.939391,
      0.903495, 0.867585, 0.831661, 0.795723, 0.759775, 0.723816, 0.687848, 0.651872,
      0.615889, 0.579900, 0.543904, 0.507904, 0.471900, 0.435892, 0.399881, 0.363867,
      0.327850, 0.291832, 0.255811, 0.219788, 0.183764, 0.147738, 0.111710, 0.075681,
      0.039649, 0.003615, 6.250764, 6.214725, 6.178682, 6.142636, 6.106586, 6.070530,
      6.034470, 5.998404, 5.962332, 5.926252, 5.890166, 5.854071, 5.817966, 5.781851,
      5.745726, 5.709589, 5.673439, 5.637276, 5.601099, 5.564908, 5.528700, 5.492475,
      5.456233, 5.419971, 5.383690, 5.347389, 5.311066, 5.274721, 5.238352, 5.201960,
      5.165542, 5.129097, 5.092627, 5.056129, 5.019602, 4.983046, 4.946459, 4.909842,
      4.873193, 4.836511, 4.799797, 4.763049, 4.726266, 4.689449, 4.652596, 4.615707,
      4.578782, 4.541820, 4.504820, 4.467783, 4.430708, 4.393594, 4.356442, 4.319252,
      4.282022, 4.244754, 4.207448, 4.170102, 4.132718, 4.095295, 4.057834, 4.020335,
      3.982798, 3.945224, 3.907613, 3.869965, 3.832282, 3.794563, 3.756809, 3.719022,
      3.681201, 3.643348, 3.605463, 3.567548, 3.529603, 3.491630, 3.453629, 3.415601,
      3.377548, 3.339471, 3.301372, 3.263250, 3.225109, 3.186948, 3.148770, 3.110577,
      3.072368, 3.034146, 2.995913, 2.957669, 2.919416, 2.881157, 2.842892, 2.804623,
      2.766352, 2.728080, 2.689808, 2.651539, 2.613274, 2.575013, 2.536760, 2.498516,
      2.460281, 2.422058, 2.383848, 2.345652, 2.307471, 2.269308, 2.231164, 2.193038,
      2.154934, 2.116853, 2.078794, 2.040761, 2.002753, 1.964771, 1.926818, 1.888893,
      1.850997, 1.813132, 1.775299, 1.737498, 1.699729, 1.661994, 1.624293, 1.586626,
      1.548995, 1.511398, 1.473839, 1.436315, 1.398827, 1.361377, 1.323963, 1.286587,
      1.249248, 1.211946, 1.174681, 1.137454, 1.100264, 1.063110, 1.025994, 0.988914,
      0.951871, 0.914864, 0.877892, 0.840956, 0.804055, 0.767188, 0.730356, 0.693557,
      0.656792, 0.620059, 0.583357, 0.546688, 0.510050, 0.473441, 0.436863, 0.400314,
      0.363793, 0.327299, 0.290833, 0.254394, 0.217979, 0.181591, 0.145226, 0.108886,
      0.072568, 0.036273, 6.283185, 6.246933, 6.210701, 6.174489, 6.138297, 6.102123,
      6.065967, 6.029829, 5.993708, 5.957603, 5.921515, 5.885443, 5.849385, 5.813342,
      5.777314, 5.741300, 5.705299, 5.669312, 5.633339, 5.597379, 5.561432, 5.525497,
      5.489576, 5.453668, 5.417772, 5.381889, 5.346020, 5.310163, 5.274321, 5.238492,
      5.202677, 5.166876, 5.131090, 5.095320, 5.059566, 5.023828, 4.988107, 4.952404,
      4.916719, 4.881053, 4.845408, 4.809784, 4.774181, 4.738602, 4.703046, 4.667516,
      4.632012, 4.596535, 4.561087, 4.525668, 4.490281, 4.454926, 4.419606, 4.384321,
      4.349073, 4.313864, 4.278694, 4.243566, 4.208481, 4.173441, 4.138448, 4.103503,
      4.068607, 4.033764, 3.998974, 3.964239, 3.929561, 3.894943, 3.860384, 3.825888,
      3.791456, 3.757090, 3.722792, 3.688564, 3.654407, 3.620324, 3.586315, 3.552384,
      3.518531, 3.484758, 3.451067, 3.417459, 3.383938, 3.350503, 3.317157, 3.283902,
      3.250738, 3.217667, 3.184692, 3.151813, 3.119032, 3.086349, 3.053768, 3.021288,
      2.988912, 2.956640, 2.924473, 2.892414, 2.860462, 2.828620, 2.796887, 2.765266,
      2.733757, 2.702361, 2.671078, 2.639911, 2.608859, 2.577923, 2.547104, 2.516403,
      2.485821, 2.455357, 2.425013, 2.394788, 2.364684, 2.334701, 2.304839, 2.275099,
      2.245481, 2.215985, 2.186612, 2.157361, 2.128233, 2.099228, 2.070346, 2.041588,
      2.012953, 1.984441, 1.956052, 1.927786, 1.899644, 1.871624, 1.843728, 1.815953,
      1.788302, 1.760772, 1.733364, 1.706078, 1.678913, 1.651869, 1.624945, 1.598141,
      1.571457, 1.544892, 1.518446, 1.492117, 1.465906, 1.439812, 1.413834, 1.387971,
      1.362223, 1.336590, 1.311069, 1.285661, 1.260365, 1.235180, 1.210104, 1.185138,
      1.160280, 1.135529, 1.110884, 1.086344, 1.061908, 1.037574, 1.013343, 0.989211,
      0.965179, 0.941245, 0.917408, 0.893666, 0.870018, 0.846462, 0.822997, 0.799623,
      0.776335, 0.753135, 0.730020, 0.706987, 0.684037, 0.661166, 0.638374, 0.615658,
      0.593016, 0.570447, 0.547949, 0.525520, 0.503157, 0.480860, 0.458625, 0.436451,
      0.414336, 0.392278, 0.370274, 0.348323, 0.326421, 0.304568, 0.282760, 0.260995,
      0.239272, 0.217587, 0.195938, 0.174323, 0.152740, 0.131187, 0.109660, 0.088157,
      0.066677, 0.045216, 0.023772, 0.002343, 6.264112, 6.242705, 6.221305, 6.199911,
      6.178518, 6.157126, 6.135732, 6.114332, 6.092927, 6.071510, 6.050083, 6.028641,
      6.007183, 5.985706, 5.964208, 5.942687, 5.921139, 5.899565, 5.877960, 5.856323,
      5.834652, 5.812945, 5.791201, 5.769415, 5.747587, 5.725716, 5.703798, 5.681832,
      5.659817, 5.637751, 5.615630, 5.593455, 5.571224, 5.548935, 5.526586, 5.504176,
      5.481703, 5.459166, 5.436563, 5.413894, 5.391157, 5.368350, 5.345472, 5.322523,
      5.299501, 5.276405, 5.253233, 5.229986, 5.206662, 5.183259, 5.159779, 5.136217,
      5.112576, 5.088853, 5.065048, 5.041160, 5.017189, 4.993134, 4.968994, 4.944769,
      4.920459, 4.896062, 4.871579, 4.847009, 4.822351, 4.797606, 4.772773, 4.747850,
      4.722840, 4.697741, 4.672553, 4.647276, 4.621910, 4.596454, 4.570909, 4.545274,
      4.519549, 4.493736, 4.467833, 4.441840, 4.415758, 4.389587, 4.363327, 4.336978,
      4.310541, 4.284015, 4.257401, 4.230700, 4.203910, 4.177034, 4.150070, 4.123020,
      4.095884, 4.068663, 4.041356, 4.013964, 3.986488, 3.958929, 3.931286, 3.903561,
      3.875754, 3.847866, 3.819896, 3.791847, 3.763719, 3.735512, 3.707227, 3.678865,
      3.650427, 3.621913, 3.593325, 3.564663, 3.535928, 3.507122, 3.478244, 3.449296,
      3.420279, 3.391193, 3.362041, 3.332823, 3.303540, 3.274192, 3.244782, 3.215311,
      3.185778, 3.156187, 3.126536, 3.096829, 3.067066, 3.037248, 3.007377, 2.977454,
      2.947480, 2.917456, 2.887383, 2.857265, 2.827099, 2.796890, 2.766638, 2.736344,
      2.706009, 2.675636, 2.645224, 2.614777, 2.584295, 2.553779, 2.523231, 2.492652,
      2.462044, 2.431408, 2.400745, 2.370057, 2.339345, 2.308611, 2.277856, 2.247081,
      2.216288, 2.185478, 2.154652, 2.123812, 2.092959, 2.062094, 2.031220, 2.000336,
      1.969445, 1.938548, 1.907645, 1.876738, 1.845829, 1.814918, 1.784008, 1.753098,
      1.722191, 1.691287, 1.660388, 1.629495, 1.598609, 1.567730, 1.536861, 1.506002,
      1.475154, 1.444318, 1.413496, 1.382688, 1.351895, 1.321118, 1.290359, 1.259618,
      1.228896, 1.198194, 1.167513, 1.136853, 1.106216, 1.075603, 1.045013, 1.014450,
      0.983911, 0.953399, 0.922914, 0.892457, 0.862030, 0.831631, 0.801263, 0.770926,
      0.740620, 0.710347, 0.680106, 0.649899, 0.619726, 0.589588, 0.559485, 0.529417,
      0.499387, 0.469393, 0.439438, 0.409520, 0.379641, 0.349801, 0.320002, 0.290242,
      0.260524, 0.230846, 0.201211, 0.171619, 0.142069, 0.112563, 0.083100, 0.053682,
      0.024310, 6.278168, 6.248886, 6.219651, 6.190463, 6.161322, 6.132229, 6.103184,
      6.074189, 6.045242, 6.016346, 5.987500, 5.958705, 5.929961, 5.901269, 5.872629,
      5.844042, 5.815509, 5.787029, 5.758604, 5.730233, 5.701918, 5.673658, 5.645455,
      5.617310, 5.589221, 5.561190, 5.533217, 5.505304, 5.477450, 5.449656, 5.421923,
      5.394251, 5.366640, 5.339091, 5.311606, 5.284183, 5.256824, 5.229530, 5.202300,
      5.175136, 5.148037, 5.121005, 5.094040, 5.067143, 5.040314, 5.013553, 4.986861,
      4.960239, 4.933687, 4.907207, 4.880796, 4.854458, 4.828193, 4.802000, 4.775880,
      4.749835, 4.723863, 4.697967, 4.672146, 4.646400, 4.620731, 4.595140, 4.569625,
      4.544188, 4.518829, 4.493548, 4.468348, 4.443226, 4.418184, 4.393223, 4.368342,
      4.343543, 4.318825, 4.294190, 4.269636, 4.245166, 4.220778, 4.196473, 4.172253,
      4.148116, 4.124063, 4.100096, 4.076213, 4.052415, 4.028703, 4.005075, 3.981535,
      3.958080, 3.934711, 3.911429, 3.888233, 3.865124, 3.842102, 3.819167, 3.796319,
      3.773558, 3.750885, 3.728298, 3.705799, 3.683388, 3.661063, 3.638826, 3.616677,
      3.594615, 3.572639, 3.550752, 3.528951, 3.507237, 3.485610, 3.464070, 3.442616,
      3.421249, 3.399968, 3.378772, 3.357663, 3.336639, 3.315701, 3.294847, 3.274078,
      3.253393, 3.232792, 3.212275, 3.191842, 3.171491, 3.151223, 3.131038, 3.110933,
      3.090911, 3.070969, 3.051108, 3.031327, 3.011625, 2.992002, 2.972458, 2.952991,
      2.933602, 2.914290, 2.895054, 2.875894, 2.856809, 2.837798, 2.818862, 2.799998,
      2.781208, 2.762489, 2.743842, 2.725266, 2.706759, 2.688322, 2.669954, 2.651654,
      2.633421, 2.615255, 2.597155, 2.579120, 2.561149, 2.543243, 2.525399, 2.507618,
      2.489899, 2.472240, 2.454642, 2.437103, 2.419624, 2.402202, 2.384837, 2.367529,
      2.350277, 2.333081, 2.315938, 2.298850, 2.281814, 2.264832, 2.247900, 2.231019,
      2.214189, 2.197408, 2.180676, 2.163992, 2.147356, 2.130767, 2.114223, 2.097726,
      2.081273, 2.064865, 2.048500, 2.032178, 2.015899, 1.999662, 1.983466, 1.967311,
      1.951196, 1.935120, 1.919083, 1.903085, 1.887125, 1.871202, 1.855316, 1.839467,
      1.823654, 1.807876, 1.792133, 1.776425, 1.760750, 1.745110, 1.729503, 1.713928,
      1.698387, 1.682877, 1.667399, 1.651952, 1.636537, 1.621152, 1.605797, 1.590473,
      1.575178, 1.559912, 1.544675, 1.529468, 1.514288, 1.499137, 1.484014, 1.468919,
      1.453851, 1.438810, 1.423796, 1.408810, 1.393849, 1.378915, 1.364007, 1.349125,
      1.334269, 1.319438, 1.304632, 1.289851, 1.275096, 1.260365, 1.245659, 1.230978,
      1.216320, 1.201687, 1.187077, 1.172492, 1.157930, 1.143391, 1.128875, 1.114383,
      1.099913, 1.085466, 1.071042, 1.056640, 1.042260, 1.027902, 1.013566, 0.999251,
      0.984957, 0.970685, 0.956434, 0.942203, 0.927993, 0.913803, 0.899633, 0.885482,
      0.871351, 0.857239, 0.843146, 0.829071, 0.815015, 0.800977, 0.786955, 0.772952,
      0.758965, 0.744994, 0.731039, 0.717101, 0.703177, 0.689268, 0.675374, 0.661493,
      0.647626, 0.633772, 0.619930, 0.606100, 0.592282, 0.578473, 0.564676, 0.550887,
      0.537108, 0.523337, 0.509574, 0.495817, 0.482067, 0.468323, 0.454583, 0.440847,
      0.427115, 0.413386, 0.399658, 0.385931, 0.372204, 0.358477, 0.344748, 0.331017,
      0.317282, 0.303543, 0.289798, 0.276048, 0.262290, 0.248524, 0.234749, 0.220964,
      0.207168, 0.193359, 0.179538, 0.165702, 0.151850, 0.137982, 0.124097, 0.110193,
      0.096268, 0.082323, 0.068356, 0.054366, 0.040352, 0.026311, 0.012244, 6.281335,
      6.267211, 6.253057, 6.238872, 6.224653, 6.210402, 6.196115, 6.181792, 6.167431,
      6.153032, 6.138594, 6.124114, 6.109592, 6.095027, 6.080418, 6.065763, 6.051061,
      6.036312, 6.021513, 6.006664, 5.991764, 5.976810, 5.961804, 5.946743, 5.931627,
      5.916453, 5.901221, 5.885931, 5.870580, 5.855168, 5.839695, 5.824158, 5.808558,
      5.792891, 5.777159, 5.761360, 5.745493, 5.729557, 5.713551, 5.697475, 5.681327,
      5.665107, 5.648813, 5.632446, 5.616004, 5.599486, 5.582891, 5.566219, 5.549469,
      5.532641, 5.515733, 5.498745, 5.481677, 5.464526, 5.447295, 5.429979, 5.412580,
      5.395098, 5.377532, 5.359879, 5.342142, 5.324317, 5.306407, 5.288409, 5.270323,
      5.252150, 5.233887, 5.215535, 5.197094, 5.178563, 5.159942, 5.141229, 5.122426,
      5.103530, 5.084543, 5.065464, 5.046293, 5.027029, 5.007671, 4.988220, 4.968675,
      4.949037, 4.929304, 4.909477, 4.889556, 4.869540, 4.849428, 4.829222, 4.808920,
      4.788523, 4.768031, 4.747442, 4.726758, 4.705979, 4.685102, 4.664131, 4.643064,
      4.621900, 4.600640, 4.579284, 4.557832, 4.536284, 4.514640, 4.492901, 4.471066,
      4.449134, 4.427107, 4.404985, 4.382768, 4.360455, 4.338047, 4.315545, 4.292948,
      4.270257, 4.247472, 4.224593, 4.201620, 4.178555, 4.155396, 4.132146, 4.108803,
      4.085369, 4.061844, 4.038228, 4.014522, 3.990725, 3.966840, 3.942866, 3.918804,
      3.894654, 3.870418, 3.846095, 3.821687, 3.797194, 3.772617, 3.747956, 3.723213,
      3.698388, 3.673482, 3.648496, 3.623431, 3.598288, 3.573068, 3.547771, 3.522398,
      3.496952, 3.471432, 3.445841, 3.420177, 3.394445, 3.368643, 3.342774, 3.316838,
      3.290838, 3.264774, 3.238647, 3.212458, 3.186211, 3.159904, 3.133541, 3.107121,
      3.080648, 3.054121, 3.027543, 3.000916, 2.974240, 2.947516, 2.920749, 2.893937,
      2.867082, 2.840188, 2.813254, 2.786283, 2.759275, 2.732234, 2.705160, 2.678055,
      2.650921, 2.623759, 2.596571, 2.569358, 2.542123, 2.514867, 2.487591, 2.460297,
      2.432987, 2.405663, 2.378326, 2.350976, 2.323618, 2.296250, 2.268877, 2.241498,
      2.214115, 2.186731, 2.159346, 2.131962, 2.104580, 2.077202, 2.049829, 2.022463,
      1.995105, 1.967757, 1.940419, 1.913095, 1.885782, 1.858485, 1.831203, 1.803939,
      1.776694, 1.749467, 1.722262, 1.695077, 1.667916, 1.640779, 1.613667, 1.586580,
      1.559521, 1.532489, 1.505486, 1.478513, 1.451570, 1.424658, 1.397779, 1.370933,
      1.344120, 1.317341, 1.290598, 1.263890, 1.237218, 1.210584, 1.183987, 1.157428,
      1.130907, 1.104426, 1.077985, 1.051583, 1.025222, 0.998902, 0.972623, 0.946386,
      0.920191, 0.894039, 0.867928, 0.841861, 0.815836, 0.789856, 0.763919, 0.738025,
      0.712176, 0.686371, 0.660610, 0.634893, 0.609221, 0.583594, 0.558011, 0.532473,
      0.506979, 0.481531, 0.456127, 0.430768, 0.405454, 0.380185, 0.354960, 0.329780,
      0.304644, 0.279553, 0.254506, 0.229504, 0.204545, 0.179631, 0.154760, 0.129933,
      0.105149, 0.080409, 0.055712, 0.031057, 0.006445, 6.265061, 6.240534, 6.216049,
      6.191605, 6.167202, 6.142840, 6.118519, 6.094239, 6.069998, 6.045798, 6.021636,
      5.997514, 5.973430, 5.949384, 5.925376, 5.901406, 5.877473, 5.853576, 5.829715,
      5.805890, 5.782100, 5.758345, 5.734624, 5.710937, 5.687282, 5.663661, 5.640071,
      5.616513, 5.592987, 5.569489, 5.546023, 5.522585, 5.499176, 5.475795, 5.452441,
      5.429114, 5.405813, 5.382537, 5.359285, 5.336058, 5.312853, 5.289671, 5.266510,
      5.243371, 5.220251, 5.197150, 5.174068, 5.151004, 5.127957, 5.104926, 5.081910,
      5.058908, 5.035919, 5.012943, 4.989979, 4.967026, 4.944082, 4.921146, 4.898220,
      4.875299, 4.852386, 4.829477, 4.806572, 4.783670, 4.760771, 4.737872, 4.714974,
      4.692075, 4.669174, 4.646269, 4.623361, 4.600448, 4.577529, 4.554602, 4.531668,
      4.508724, 4.485770, 4.462805, 4.439827, 4.416836, 4.393830, 4.370810, 4.347771,
      4.324717, 4.301642, 4.278548, 4.255434, 4.232298, 4.209138, 4.185956, 4.162748,
      4.139514, 4.116253, 4.092965, 4.069647, 4.046300, 4.022922, 3.999512, 3.976069,
      3.952592, 3.929081, 3.905535, 3.881951, 3.858330, 3.834671, 3.810973, 3.787235,
      3.763455, 3.739635, 3.715771, 3.691864, 3.667913, 3.643917, 3.619876, 3.595788,
      3.571652, 3.547469, 3.523237, 3.498955, 3.474624, 3.450243, 3.425809, 3.401323,
      3.376786, 3.352195, 3.327549, 3.302850, 3.278095, 3.253286, 3.228420, 3.203497,
      3.178518, 3.153481, 3.128386, 3.103232, 3.078020, 3.052748, 3.027416, 3.002025,
      2.976572, 2.951059, 2.925484, 2.899847, 2.874149, 2.848387, 2.822563, 2.796677,
      2.770726, 2.744712, 2.718634, 2.692492, 2.666285, 2.640013, 2.613676, 2.587274,
      2.560806, 2.534272, 2.507673, 2.481007, 2.454275, 2.427476, 2.400610, 2.373678,
      2.346678, 2.319611, 2.292476, 2.265273, 2.238003, 2.210664, 2.183257, 2.155782,
      2.128239, 2.100626, 2.072945, 2.045196, 2.017377, 1.989488, 1.961532, 1.933505,
      1.905409, 1.877244, 1.849009, 1.820704, 1.792329, 1.763885, 1.735370, 1.706786,
      1.678131, 1.649407, 1.620611, 1.591746, 1.562810, 1.533804, 1.504728, 1.475581,
      1.446364, 1.417076, 1.387718, 1.358289, 1.328790, 1.299220, 1.269580, 1.239870,
      1.210089, 1.180238, 1.150316, 1.120325, 1.090264, 1.060131, 1.029930, 0.999659,
      0.969318, 0.938907, 0.908428, 0.877879, 0.847261, 0.816574, 0.785819, 0.754995,
      0.724103, 0.693143, 0.662116, 0.631021, 0.599860, 0.568632, 0.537337, 0.505977,
      0.474551, 0.443060, 0.411504, 0.379884, 0.348200, 0.316454, 0.284644, 0.252773,
      0.220839, 0.188845, 0.156791, 0.124676, 0.092502, 0.060270, 0.027981, 6.278820,
      6.246417, 6.213959, 6.181446, 6.148879, 6.116260, 6.083588, 6.050865, 6.018092,
      5.985270, 5.952399, 5.919481, 5.886518, 5.853508, 5.820454, 5.787357, 5.754219,
      5.721039, 5.687819, 5.654561, 5.621265, 5.587933, 5.554566, 5.521165, 5.487732,
      5.454267, 5.420771, 5.387247, 5.353695, 5.320117, 5.286512, 5.252885, 5.219235,
      5.185564, 5.151872, 5.118161, 5.084434, 5.050689, 5.016930, 4.983157, 4.949371,
      4.915575, 4.881768, 4.847953, 4.814130, 4.780301, 4.746466, 4.712627, 4.678786,
      4.644943, 4.611099, 4.577256, 4.543414, 4.509574, 4.475739, 4.441907, 4.408082,
      4.374262, 4.340451, 4.306647, 4.272853, 4.239069, 4.205295, 4.171534, 4.137784,
      4.104046, 4.070323, 4.036615, 4.002922, 3.969244, 3.935582, 3.901937, 3.868309,
      3.834698, 3.801105, 3.767531, 3.733977, 3.700441, 3.666924, 3.633428, 3.599951,
      3.566494, 3.533058, 3.499643, 3.466248, 3.432874, 3.399521, 3.366189, 3.332878,
      3.299588, 3.266319, 3.233070, 3.199843, 3.166636, 3.133451, 3.100285, 3.067140,
      3.034015, 3.000911, 2.967826, 2.934761, 2.901716, 2.868689, 2.835682, 2.802693,
      2.769722, 2.736770, 2.703836, 2.670919, 2.638020, 2.605138, 2.572272, 2.539423,
      2.506590, 2.473772, 2.440970, 2.408184, 2.375412, 2.342654, 2.309911, 2.277182,
      2.244467, 2.211765, 2.179077, 2.146402, 2.113739, 2.081089, 2.048451, 2.015826,
      1.983212, 1.950611, 1.918021, 1.885443, 1.852876, 1.820322, 1.787778, 1.755247,
      1.722727, 1.690218, 1.657721, 1.625237, 1.592764, 1.560303, 1.527855, 1.495420,
      1.462997, 1.430588, 1.398192, 1.365811, 1.333443, 1.301092, 1.268754, 1.236433,
      1.204129, 1.171842, 1.139574, 1.107323, 1.075092, 1.042881, 1.010691, 0.978523,
      0.946378, 0.914256, 0.882160, 0.850089, 0.818046, 0.786030, 0.754044, 0.722088,
      0.690164, 0.658273, 0.626416, 0.594595, 0.562812, 0.531067, 0.499362, 0.467698,
      0.436078, 0.404503, 0.372973, 0.341492, 0.310060, 0.278680, 0.247353, 0.216080,
      0.184864, 0.153706, 0.122608, 0.091572, 0.060599, 0.029692, 6.282037, 6.251266,
      6.220567, 6.189939, 6.159387, 6.128911, 6.098514, 6.068196, 6.037960, 6.007809,
      5.977742, 5.947763, 5.917874, 5.888076, 5.858370, 5.828759, 5.799244, 5.769828,
      5.740510, 5.711295, 5.682182, 5.653174, 5.624272, 5.595478, 5.566793, 5.538219,
      5.509758, 5.481411, 5.453178, 5.425062, 5.397065, 5.369186, 5.341429, 5.313793,
      5.286280, 5.258891, 5.231629, 5.204493, 5.177485, 5.150605, 5.123855, 5.097236,
      5.070749, 5.044395, 5.018174, 4.992088, 4.966137, 4.940323, 4.914645, 4.889105,
      4.863704, 4.838442, 4.813319, 4.788337, 4.763495, 4.738795, 4.714237, 4.689822,
      4.665549, 4.641420, 4.617435, 4.593594, 4.569898, 4.546346, 4.522940, 4.499679,
      4.476563, 4.453595, 4.430772, 4.408094, 4.385565, 4.363181, 4.340945, 4.318854,
      4.296911, 4.275115, 4.253466, 4.231963, 4.210607, 4.189398, 4.168335, 4.147418,
      4.126649, 4.106025, 4.085547, 4.065215, 4.045028, 4.024986, 4.005090, 3.985337,
      3.965728, 3.946264, 3.926943, 3.907764, 3.888728, 3.869833, 3.851080, 3.832468,
      3.813996, 3.795663, 3.777469, 3.759413, 3.741495, 3.723713, 3.706068, 3.688557,
      3.671180, 3.653937, 3.636826, 3.619847, 3.602998, 3.586279, 3.569688, 3.553224,
      3.536887, 3.520675, 3.504586, 3.488621, 3.472777, 3.457054, 3.441449, 3.425962,
      3.410592, 3.395336, 3.380194, 3.365164, 3.350245, 3.335435, 3.320733, 3.306136,
      3.291645, 3.277256, 3.262969, 3.248782, 3.234694, 3.220702, 3.206805, 3.193002,
      3.179290, 3.165668, 3.152134, 3.138688, 3.125327, 3.112049, 3.098852, 3.085735,
      3.072697, 3.059736, 3.046849, 3.034036, 3.021294, 3.008623, 2.996020, 2.983485,
      2.971014, 2.958607, 2.946262, 2.933978, 2.921754, 2.909587, 2.897476, 2.885421,
      2.873419, 2.861469, 2.849570, 2.837721, 2.825921, 2.814167, 2.802460, 2.790798,
      2.779180, 2.767604, 2.756071, 2.744578, 2.733125, 2.721712, 2.710336, 2.698998,
      2.687697, 2.676432, 2.665203, 2.654007, 2.642847, 2.631720, 2.620625, 2.609564,
      2.598534, 2.587537, 2.576571, 2.565636, 2.554733, 2.543860, 2.533018, 2.522206,
      2.511425, 2.500674, 2.489953, 2.479262, 2.468601, 2.457971, 2.447371, 2.436802,
      2.426263, 2.415755, 2.405279, 2.394833, 2.384418, 2.374036, 2.363685, 2.353367,
      2.343081, 2.332829, 2.322609, 2.312423, 2.302271, 2.292154, 2.282071, 2.272023,
      2.262010, 2.252034, 2.242094, 2.232191, 2.222324, 2.212495, 2.202705, 2.192953,
      2.183238, 2.173564, 2.163929, 2.154334, 2.144779, 2.135264, 2.125791, 2.116358,
      2.106967, 2.097619, 2.088311, 2.079047, 2.069825, 2.060645, 2.051509, 2.042416,
      2.033366, 2.024359, 2.015396, 2.006475, 1.997599, 1.988766, 1.979977, 1.971231,
      1.962528, 1.953868, 1.945251, 1.936677, 1.928146, 1.919657, 1.911210, 1.902804,
      1.894440, 1.886117, 1.877835, 1.869592, 1.861388, 1.853224, 1.845099, 1.837011,
      1.828960, 1.820946, 1.812968, 1.805025, 1.797116, 1.789241, 1.781398, 1.773588,
      1.765809, 1.758060, 1.750340, 1.742650, 1.734985, 1.727348, 1.719737, 1.712150,
      1.704587, 1.697045, 1.689526, 1.682027, 1.674547, 1.667086, 1.659643, 1.652215,
      1.644804, 1.637406, 1.630021, 1.622649, 1.615288, 1.607937, 1.600594, 1.593261,
      1.585934, 1.578614, 1.571299, 1.563988, 1.556681, 1.549377, 1.542074, 1.534772,
      1.527471, 1.520168, 1.512865, 1.505559, 1.498251, 1.490939, 1.483624, 1.476304,
      1.468979, 1.461648, 1.454311, 1.446969, 1.439619, 1.432261, 1.424897, 1.417524,
      1.410145, 1.402756, 1.395359, 1.387954, 1.380540, 1.373117, 1.365686, 1.358246,
      1.350798, 1.343341, 1.335876, 1.328403, 1.320921, 1.313433, 1.305936, 1.298432,
      1.290922, 1.283405, 1.275882, 1.268353, 1.260820, 1.253280, 1.245737, 1.238190,
      1.230640, 1.223087, 1.215531, 1.207974, 1.200416, 1.192858, 1.185299, 1.177742,
      1.170186, 1.162632, 1.155082, 1.147534, 1.139991, 1.132452, 1.124919, 1.117393,
      1.109873, 1.102360, 1.094856, 1.087361, 1.079876, 1.072400, 1.064936, 1.057483,
      1.050042, 1.042614, 1.035199, 1.027799, 1.020412, 1.013041, 1.005685, 0.998346,
      0.991022, 0.983716, 0.976427, 0.969156, 0.961903, 0.954668, 0.947453, 0.940256,
      0.933079, 0.925921, 0.918782, 0.911664, 0.904565, 0.897486, 0.890428, 0.883389,
      0.876371, 0.869372, 0.862393, 0.855433, 0.848493, 0.841571, 0.834669, 0.827785,
      0.820919, 0.814070, 0.807239, 0.800425, 0.793626, 0.786843, 0.780075, 0.773321,
      0.766581, 0.759853, 0.753138, 0.746434, 0.739740, 0.733056, 0.726380, 0.719713,
      0.713051, 0.706396, 0.699746, 0.693099, 0.686455, 0.679812, 0.673170, 0.666527,
      0.659882, 0.653235, 0.646584, 0.639927, 0.633264, 0.626594, 0.619915, 0.613226,
      0.606527, 0.599815, 0.593089, 0.586350, 0.579595, 0.572823, 0.566034, 0.559226,
      0.552398, 0.545549, 0.538679, 0.531785, 0.524867, 0.517925, 0.510957, 0.503963,
      0.496942, 0.489891, 0.482813, 0.475704, 0.468565, 0.461395, 0.454194, 0.446960,
      0.439693, 0.432393, 0.425060, 0.417692, 0.410289, 0.402852, 0.395380, 0.387873,
      0.380329, 0.372750, 0.365135, 0.357484, 0.349797, 0.342074, 0.334315, 0.326520,
      0.318690, 0.310823, 0.302922, 0.294985, 0.287013, 0.279006, 0.270965, 0.262890,
      0.254782, 0.246640, 0.238466, 0.230259, 0.222021, 0.213751, 0.205451, 0.197121,
      0.188761, 0.180372, 0.171955, 0.163510, 0.155038, 0.146540, 0.138016, 0.129467,
      0.120894, 0.112298, 0.103678, 0.095037, 0.086374, 0.077690, 0.068986, 0.060263,
      0.051521, 0.042761, 0.033984, 0.025191, 0.016381, 0.007556, 6.281902, 6.273049,
      6.264182, 6.255302, 6.246410, 6.237506, 6.228591, 6.219665, 6.210729, 6.201782,
      6.192826, 6.183861, 6.174887, 6.165903, 6.156912, 6.147912, 6.138904, 6.129888,
      6.120863, 6.111832, 6.102792, 6.093743, 6.084687, 6.075622, 6.066548, 6.057466,
      6.048373, 6.039271, 6.030159, 6.021036, 6.011901, 6.002755, 5.993596, 5.984423,
      5.975236, 5.966033, 5.956815, 5.947580, 5.938326, 5.929052, 5.919759, 5.910444,
      5.901105, 5.891742, 5.882354, 5.872938, 5.863493, 5.854018, 5.844512, 5.834971,
      5.825396, 5.815783, 5.806131, 5.796439, 5.786704, 5.776925, 5.767099, 5.757225,
      5.747301, 5.737325, 5.727293, 5.717206, 5.707060, 5.696853, 5.686584, 5.676250,
      5.665849, 5.655378, 5.644837, 5.634222, 5.623532, 5.612764, 5.601916, 5.590987,
      5.579973, 5.568874, 5.557687, 5.546410, 5.535040, 5.523577, 5.512018, 5.500361,
      5.488605, 5.476747, 5.464785, 5.452719, 5.440546, 5.428264, 5.415872, 5.403368,
      5.390751, 5.378019, 5.365171, 5.352205, 5.339120, 5.325914, 5.312587, 5.299136,
      5.285562, 5.271861, 5.258035, 5.244081, 5.229998, 5.215785, 5.201442, 5.186968,
      5.172360, 5.157621, 5.142747, 5.127739, 5.112596, 5.097316, 5.081900, 5.066347,
      5.050656, 5.034828, 5.018861, 5.002754, 4.986509, 4.970123, 4.953598, 4.936933,
      4.920126, 4.903180, 4.886092, 4.868863, 4.851493, 4.833982, 4.816329, 4.798534,
      4.780598, 4.762521, 4.744302, 4.725941, 4.707439, 4.688797, 4.670013, 4.651088,
      4.632023, 4.612817, 4.593471, 4.573984, 4.554358, 4.534593, 4.514688, 4.494645,
      4.474464, 4.454145, 4.433689, 4.413095, 4.392366, 4.371500, 4.350500, 4.329364,
      4.308095, 4.286693, 4.265158, 4.243491, 4.221693, 4.199765, 4.177708, 4.155522,
      4.133209, 4.110770, 4.088204, 4.065516, 4.042703, 4.019769, 3.996713, 3.973538,
      3.950245, 3.926835, 3.903310, 3.879670, 3.855918, 3.832055, 3.808083, 3.784004,
      3.759819, 3.735529, 3.711138, 3.686646, 3.662056, 3.637370, 3.612590, 3.587718,
      3.562756, 3.537706, 3.512572, 3.487354, 3.462056, 3.436680, 3.411229, 3.385704,
      3.360110, 3.334449, 3.308722, 3.282933, 3.257085, 3.231181, 3.205224, 3.179216,
      3.153160, 3.127060, 3.100919, 3.074739, 3.048523, 3.022275, 2.995998, 2.969696,
      2.943370, 2.917025, 2.890663, 2.864288, 2.837902, 2.811510, 2.785113, 2.758716,
      2.732321, 2.705931, 2.679551, 2.653181, 2.626827, 2.600490, 2.574173, 2.547881,
      2.521615, 2.495378, 2.469174, 2.443004, 2.416873, 2.390781, 2.364733, 2.338731,
      2.312776, 2.286873, 2.261023, 2.235227, 2.209491, 2.183814, 2.158199, 2.132649,
      2.107166, 2.081751, 2.056406, 2.031135, 2.005937, 1.980815, 1.955771, 1.930807,
      1.905924, 1.881123, 1.856406, 1.831775, 1.807231, 1.782774, 1.758407, 1.734131,
      1.709946, 1.685854, 1.661857, 1.637953, 1.614146, 1.590435, 1.566822, 1.543307,
      1.519891, 1.496574, 1.473359, 1.450243, 1.427229, 1.404317, 1.381506, 1.358800,
      1.336196, 1.313696, 1.291299, 1.269007, 1.246819, 1.224735, 1.202755, 1.180881,
      1.159111, 1.137446, 1.115885, 1.094430, 1.073079, 1.051832, 1.030689, 1.009651,
      0.988716, 0.967885, 0.947157, 0.926532, 0.906009, 0.885588, 0.865268, 0.845050,
      0.824932, 0.804913, 0.784994, 0.765173, 0.745450, 0.725824, 0.706294, 0.686859,
      0.667518, 0.648271, 0.629115, 0.610051, 0.591077, 0.572192, 0.553394, 0.534683,
      0.516057, 0.497514, 0.479054, 0.460674, 0.442374, 0.424151, 0.406004, 0.387931,
      0.369931, 0.352002, 0.334141, 0.316347, 0.298618, 0.280952, 0.263346, 0.245800,
      0.228309, 0.210873, 0.193488, 0.176154, 0.158866, 0.141623, 0.124421, 0.107260,
      0.090136, 0.073046, 0.055988, 0.038959, 0.021957, 0.004978, 6.271205, 6.254265,
      6.237340, 6.220428, 6.203525, 6.186628, 6.169735, 6.152843, 6.135948, 6.119048,
      6.102140, 6.085221, 6.068288, 6.051337, 6.034368, 6.017375, 6.000356, 5.983308,
      5.966228, 5.949115, 5.931964, 5.914773, 5.897538, 5.880259, 5.862931, 5.845551,
      5.828119, 5.810629, 5.793081, 5.775471, 5.757797, 5.740057, 5.722249, 5.704369,
      5.686415, 5.668386, 5.650279, 5.632092, 5.613822, 5.595469, 5.577029, 5.558501,
      5.539883, 5.521173, 5.502369, 5.483470, 5.464474, 5.445379, 5.426183, 5.406886,
      5.387486, 5.367980, 5.348368, 5.328650, 5.308821, 5.288884, 5.268835, 5.248673,
      5.228398, 5.208009, 5.187505, 5.166883, 5.146145, 5.125288, 5.104312, 5.083217,
      5.062001, 5.040664, 5.019205, 4.997622, 4.975917, 4.954089, 4.932136, 4.910058,
      4.887856, 4.865527, 4.843073, 4.820492, 4.797785, 4.774951, 4.751991, 4.728903,
      4.705688, 4.682345, 4.658875, 4.635278, 4.611553, 4.587701, 4.563721, 4.539615,
      4.515381, 4.491020, 4.466533, 4.441920, 4.417181, 4.392317, 4.367328, 4.342214,
      4.316977, 4.291615, 4.266132, 4.240527, 4.214801, 4.188954, 4.162988, 4.136904,
      4.110702, 4.084384, 4.057950, 4.031403, 4.004743, 3.977971, 3.951089, 3.924099,
      3.897002, 3.869799, 3.842493, 3.815084, 3.787575, 3.759969, 3.732266, 3.704469,
      3.676579, 3.648600, 3.620533, 3.592380, 3.564145, 3.535830, 3.507436, 3.478967,
      3.450426, 3.421814, 3.393136, 3.364394, 3.335590, 3.306728, 3.277811, 3.248842,
      3.219824, 3.190760, 3.161654, 3.132510, 3.103328, 3.074115, 3.044872, 3.015604,
      2.986314, 2.957004, 2.927680, 2.898343, 2.868999, 2.839649, 2.810298, 2.780949,
      2.751605, 2.722271, 2.692949, 2.663643, 2.634356, 2.605092, 2.575853, 2.546643,
      2.517467, 2.488326, 2.459224, 2.430163, 2.401147, 2.372180, 2.343263, 2.314400,
      2.285594, 2.256847, 2.228163, 2.199543, 2.170991, 2.142508, 2.114098, 2.085762,
      2.057503, 2.029323, 2.001225, 1.973210, 1.945281, 1.917439, 1.889686, 1.862024,
      1.834455, 1.806981, 1.779603, 1.752322, 1.725141, 1.698060, 1.671081, 1.644205,
      1.617434, 1.590769, 1.564211, 1.537760, 1.511419, 1.485187, 1.459067, 1.433058,
      1.407162, 1.381379, 1.355710, 1.330156, 1.304718, 1.279396, 1.254191, 1.229103,
      1.204133, 1.179281, 1.154548, 1.129934, 1.105440, 1.081065, 1.056811, 1.032678,
      1.008665, 0.984773, 0.961003, 0.937354, 0.913827, 0.890422, 0.867138, 0.843977,
      0.820937, 0.798020, 0.775224, 0.752551, 0.730000, 0.707570, 0.685263, 0.663077,
      0.641012, 0.619069, 0.597247, 0.575546, 0.553965, 0.532505, 0.511164, 0.489943,
      0.468840, 0.447856, 0.426991, 0.406242, 0.385611, 0.365095, 0.344694, 0.324409,
      0.304237, 0.284178, 0.264231, 0.244396, 0.224670, 0.205053, 0.185544, 0.166142,
      0.146844, 0.127651, 0.108560, 0.089570, 0.070680, 0.051887, 0.033190, 0.014588,
      6.279263, 6.260843, 6.242512, 6.224267, 6.206107, 6.188028, 6.170029, 6.152107,
      6.134259, 6.116484, 6.098778, 6.081139, 6.063564, 6.046050, 6.028594, 6.011193,
      5.993845, 5.976545, 5.959291, 5.942080, 5.924907, 5.907771, 5.890667, 5.873591,
      5.856542, 5.839514, 5.822504, 5.805509, 5.788525, 5.771549, 5.754576, 5.737603,
      5.720627, 5.703643, 5.686648, 5.669639, 5.652611, 5.635562, 5.618486, 5.601381,
      5.584244, 5.567069, 5.549855, 5.532598, 5.515294, 5.497940, 5.480532, 5.463067,
      5.445542, 5.427954, 5.410300, 5.392577, 5.374781, 5.356911, 5.338962, 5.320933,
      5.302821, 5.284623, 5.266336, 5.247959, 5.229489, 5.210924, 5.192261, 5.173498,
      5.154634, 5.135666, 5.116593, 5.097414, 5.078124, 5.058725, 5.039213, 5.019588,
      4.999848, 4.979991, 4.960017, 4.939924, 4.919712, 4.899378, 4.878923, 4.858345,
      4.837642, 4.816815, 4.795863, 4.774785, 4.753580, 4.732247, 4.710786, 4.689198,
      4.667480, 4.645632, 4.623655, 4.601548, 4.579311, 4.556943, 4.534444, 4.511814,
      4.489052, 4.466160, 4.443136, 4.419981, 4.396694, 4.373275, 4.349726, 4.326045,
      4.302232, 4.278288, 4.254213, 4.230008, 4.205672, 4.181206, 4.156609, 4.131883,
      4.107027, 4.082042, 4.056928, 4.031686, 4.006316, 3.980818, 3.955194, 3.929442,
      3.903565, 3.877563, 3.851435, 3.825184, 3.798809, 3.772312, 3.745692, 3.718951,
      3.692090, 3.665109, 3.638010, 3.610793, 3.583459, 3.556009, 3.528445, 3.500767,
      3.472977, 3.445076, 3.417064, 3.388945, 3.360717, 3.332384, 3.303946, 3.275405,
      3.246763, 3.218021, 3.189180, 3.160243, 3.131212, 3.102087, 3.072871, 3.043566,
      3.014174, 2.984696, 2.955135, 2.925494, 2.895773, 2.865976, 2.836104, 2.806160,
      2.776147, 2.746067, 2.715921, 2.685714, 2.655447, 2.625123, 2.594744, 2.564313,
      2.533834, 2.503308, 2.472738, 2.442128, 2.411480, 2.380797, 2.350081, 2.319336,
      2.288564, 2.257769, 2.226954, 2.196120, 2.165272, 2.134411, 2.103542, 2.072666,
      2.041787, 2.010908, 1.980031, 1.949160, 1.918296, 1.887444, 1.856605, 1.825782,
      1.794979, 1.764197, 1.733439, 1.702708, 1.672007, 1.641337, 1.610702, 1.580103,
      1.549543, 1.519024, 1.488549, 1.458120, 1.427737, 1.397404, 1.367123, 1.336896,
      1.306724, 1.276609, 1.246553, 1.216558, 1.186625, 1.156755, 1.126951, 1.097213,
      1.067543, 1.037942, 1.008412, 0.978953, 0.949567, 0.920255, 0.891016, 0.861854,
      0.832768, 0.803759, 0.774828, 0.745976, 0.717202, 0.688509, 0.659896, 0.631363,
      0.602912, 0.574542, 0.546254, 0.518048, 0.489924, 0.461883, 0.433924, 0.406048,
      0.378254, 0.350543, 0.322915, 0.295369, 0.267906, 0.240524, 0.213225, 0.186007,
      0.158871, 0.131815, 0.104841, 0.077946, 0.051132, 0.024396, 6.280926, 6.254347,
      6.227847, 6.201424, 6.175077, 6.148806, 6.122610, 6.096488, 6.070440, 6.044465,
      6.018562, 5.992730, 5.966968, 5.941275, 5.915651, 5.890094, 5.864603, 5.839177,
      5.813816, 5.788518, 5.763282, 5.738106, 5.712991, 5.687933, 5.662932, 5.637988,
      5.613098, 5.588261, 5.563476, 5.538741, 5.514055, 5.489417, 5.464824, 5.440276,
      5.415771, 5.391308, 5.366884, 5.342499, 5.318150, 5.293836, 5.269554, 5.245305,
      5.221085, 5.196892, 5.172726, 5.148584, 5.124464, 5.100366, 5.076284, 5.052221,
      5.028172, 5.004136, 4.980111, 4.956095, 4.932085, 4.908080, 4.884079, 4.860078,
      4.836076, 4.812071, 4.788061, 4.764043, 4.740015, 4.715977, 4.691925, 4.667856,
      4.643771, 4.619665, 4.595538, 4.571387, 4.547209, 4.523004, 4.498769, 4.474501,
      4.450199, 4.425861, 4.401485, 4.377069, 4.352611, 4.328108, 4.303560, 4.278963,
      4.254317, 4.229619, 4.204867, 4.180060, 4.155197, 4.130274, 4.105290, 4.080244,
      4.055134, 4.029959, 4.004716, 3.979404, 3.954022, 3.928568, 3.903040, 3.877438,
      3.851759, 3.826003, 3.800167, 3.774252, 3.748254, 3.722174, 3.696010, 3.669760,
      3.643425, 3.617002, 3.590491, 3.563890, 3.537199, 3.510416, 3.483541, 3.456574,
      3.429513, 3.402356, 3.375105, 3.347757, 3.320313, 3.292772, 3.265132, 3.237394,
      3.209558, 3.181622, 3.153585, 3.125449, 3.097212, 3.068874, 3.040435, 3.011894,
      2.983251, 2.954506, 2.925660, 2.896711, 2.867659, 2.838505, 2.809249, 2.779890,
      2.750428, 2.720864, 2.691198, 2.661429, 2.631558, 2.601585, 2.571510, 2.541334,
      2.511056, 2.480677, 2.450197, 2.419615, 2.388934, 2.358153, 2.327272, 2.296292,
      2.265213, 2.234036, 2.202760, 2.171387, 2.139917, 2.108350, 2.076688, 2.044929,
      2.013075, 1.981128, 1.949086, 1.916950, 1.884722, 1.852401, 1.819990, 1.787486,
      1.754893, 1.722209, 1.689437, 1.656576, 1.623627, 1.590591, 1.557469, 1.524260,
      1.490967, 1.457589, 1.424127, 1.390582, 1.356954, 1.323245, 1.289454, 1.255583,
      1.221632, 1.187602, 1.153494, 1.119308, 1.085045, 1.050706, 1.016291, 0.981800,
      0.947236, 0.912598, 0.877887, 0.843103, 0.808247, 0.773321, 0.738324, 0.703257,
      0.668121, 0.632916, 0.597643, 0.562303, 0.526896, 0.491423, 0.455884, 0.420280,
      0.384612, 0.348879, 0.313083, 0.277225, 0.241304, 0.205321, 0.169278, 0.133173,
      0.097009, 0.060784, 0.024500, 6.271344, 6.234944, 6.198485, 6.161970, 6.125399,
      6.088770, 6.052086, 6.015347, 5.978553, 5.941704, 5.904802, 5.867846, 5.830837,
      5.793776, 5.756662, 5.719497, 5.682281, 5.645014, 5.607697, 5.570330, 5.532913,
      5.495448, 5.457934, 5.420372, 5.382763, 5.345107, 5.307404, 5.269655, 5.231861,
      5.194021, 5.156137, 5.118208, 5.080236, 5.042222, 5.004164, 4.966065, 4.927924,
      4.889742, 4.851520, 4.813257, 4.774956, 4.736616, 4.698237, 4.659822, 4.621368,
      4.582880, 4.544354, 4.505794, 4.467199, 4.428571, 4.389908, 4.351214, 4.312487,
      4.273729, 4.234940, 4.196121, 4.157272, 4.118395, 4.079490, 4.040557, 4.001597,
      3.962612, 3.923601, 3.884565, 3.845505, 3.806422, 3.767316, 3.728188, 3.689038,
      3.649868, 3.610677, 3.571467, 3.532238, 3.492991, 3.453727, 3.414445, 3.375148,
      3.335834, 3.296505, 3.257162, 3.217805, 3.178435, 3.139051, 3.099656, 3.060248,
      3.020829, 2.981400, 2.941960, 2.902510, 2.863051, 2.823582, 2.784106, 2.744620,
      2.705127, 2.665627, 2.626119, 2.586604, 2.547083, 2.507555, 2.468021, 2.428482,
      2.388936, 2.349386, 2.309829, 2.270268, 2.230702, 2.191131, 2.151555, 2.111974,
      2.072388, 2.032797, 1.993202, 1.953602, 1.913998, 1.874388, 1.834774, 1.795153,
      1.755529, 1.715899, 1.676264, 1.636623, 1.596976, 1.557323, 1.517664, 1.477999,
      1.438327, 1.398649, 1.358963, 1.319270, 1.279570, 1.239861, 1.200144, 1.160419,
      1.120686, 1.080943, 1.041191, 1.001429, 0.961657, 0.921875, 0.882083, 0.842279,
      0.802465, 0.762639, 0.722801, 0.682952, 0.643090, 0.603215, 0.563327, 0.523427,
      0.483513, 0.443586, 0.403645, 0.363690, 0.323721, 0.283738, 0.243740, 0.203728,
      0.163701, 0.123659, 0.083603, 0.043531, 0.003445, 6.246530, 6.206413, 6.166283,
      6.126137, 6.085977, 6.045803, 6.005613, 5.965410, 5.925193, 5.884963, 5.844718,
      5.804461, 5.764191, 5.723908, 5.683614, 5.643308, 5.602991, 5.562662, 5.522324,
      5.481976, 5.441619, 5.401253, 5.360879, 5.320498, 5.280110, 5.239717, 5.199317,
      5.158914, 5.118507, 5.078097, 5.037684, 4.997271, 4.956856, 4.916442, 4.876029,
      4.835619, 4.795211, 4.754807, 4.714407, 4.674014, 4.633626, 4.593247, 4.552876,
      4.512514, 4.472162, 4.431823, 4.391495, 4.351179, 4.310879, 4.270593, 4.230322,
      4.190069, 4.149833, 4.109616, 4.069417, 4.029240, 3.989082, 3.948947, 3.908834,
      3.868744, 3.828679, 3.788637, 3.748622, 3.708632, 3.668668, 3.628732, 3.588824,
      3.548944, 3.509092, 3.469270, 3.429478, 3.389715, 3.349983, 3.310281, 3.270611,
      3.230971, 3.191364, 3.151788, 3.112244, 3.072732, 3.033252, 2.993804, 2.954389,
      2.915005, 2.875654, 2.836335, 2.797048, 2.757792, 2.718569, 2.679377, 2.640216,
      2.601086, 2.561987, 2.522918, 2.483880, 2.444871, 2.405893, 2.366942, 2.328021,
      2.289128, 2.250263, 2.211425, 2.172614, 2.133829, 2.095070, 2.056336, 2.017627,
      1.978943, 1.940283, 1.901645, 1.863030, 1.824438, 1.785867, 1.747317, 1.708788,
      1.670278, 1.631788, 1.593317, 1.554865, 1.516430, 1.478014, 1.439613, 1.401229,
      1.362861, 1.324509, 1.286173, 1.247851, 1.209544, 1.171251, 1.132972, 1.094706,
      1.056455, 1.018216, 0.979990, 0.941778, 0.903578, 0.865391, 0.827217, 0.789056,
      0.750908, 0.712773, 0.674651, 0.636542, 0.598448, 0.560367, 0.522301, 0.484250,
      0.446214, 0.408194, 0.370190, 0.332203, 0.294235, 0.256284, 0.218353, 0.180443,
      0.142553, 0.104685, 0.066840, 0.029020, 6.274410, 6.236642, 6.198901, 6.161189,
      6.123507, 6.085858, 6.048242, 6.010660, 5.973115, 5.935607, 5.898140, 5.860713,
      5.823330, 5.785991, 5.748699, 5.711455, 5.674261, 5.637120, 5.600033, 5.563002,
      5.526028, 5.489115, 5.452263, 5.415477, 5.378755, 5.342103, 5.305520, 5.269010,
      5.232574, 5.196214, 5.159933, 5.123733, 5.087615, 5.051582, 5.015636, 4.979779,
      4.944012, 4.908338, 4.872759, 4.837276, 4.801892, 4.766608, 4.731426, 4.696349,
      4.661378, 4.626514, 4.591760, 4.557117, 4.522585, 4.488170, 4.453869, 4.419685,
      4.385621, 4.351677, 4.317855, 4.284156, 4.250581, 4.217133, 4.183810, 4.150617,
      4.117552, 4.084617, 4.051814, 4.019143, 3.986606, 3.954203, 3.921935, 3.889803,
      3.857808, 3.825951, 3.794232, 3.762652, 3.731212, 3.699913, 3.668754, 3.637737,
      3.606861, 3.576128, 3.545538, 3.515092, 3.484789, 3.454631, 3.424617, 3.394747,
      3.365023, 3.335443, 3.306010, 3.276722, 3.247581, 3.218585, 3.189735, 3.161031,
      3.132474, 3.104063, 3.075799, 3.047681, 3.019709, 2.991884, 2.964205, 2.936671,
      2.909285, 2.882044, 2.854949, 2.828000, 2.801196, 2.774537, 2.748024, 2.721654,
      2.695430, 2.669349, 2.643412, 2.617620, 2.591969, 2.566462, 2.541097, 2.515873,
      2.490791, 2.465849, 2.441047, 2.416385, 2.391862, 2.367478, 2.343230, 2.319119,
      2.295145, 2.271306, 2.247601, 2.224030, 2.200592, 2.177284, 2.154108, 2.131061,
      2.108142, 2.085351, 2.062686, 2.040146, 2.017729, 1.995435, 1.973262, 1.951208,
      1.929272, 1.907452, 1.885747, 1.864155, 1.842675, 1.821304, 1.800041, 1.778884,
      1.757831, 1.736880, 1.716029, 1.695276, 1.674619, 1.654056, 1.633584, 1.613201,
      1.592905, 1.572693, 1.552564, 1.532514, 1.512541, 1.492642, 1.472816, 1.453058,
      1.433368, 1.413741, 1.394176, 1.374669, 1.355218, 1.335819, 1.316471, 1.297171,
      1.277915, 1.258701, 1.239526, 1.220387, 1.201281, 1.182206, 1.163158, 1.144135,
      1.125135, 1.106153, 1.087188, 1.068238, 1.049298, 1.030367, 1.011441, 0.992519,
      0.973598, 0.954674, 0.935748, 0.916813, 0.897869, 0.878915, 0.859947, 0.840963,
      0.821960, 0.802938, 0.783894, 0.764825, 0.745730, 0.726607, 0.707455, 0.688271,
      0.669055, 0.649803, 0.630515, 0.611189, 0.591824, 0.572419, 0.552972, 0.533482,
      0.513948, 0.494369, 0.474744, 0.455071, 0.435351, 0.415581, 0.395762, 0.375893,
      0.355973, 0.336001, 0.315978, 0.295901, 0.275772, 0.255589, 0.235353, 0.215062,
      0.194718, 0.174319, 0.153866, 0.133358, 0.112795, 0.092178, 0.071507, 0.050781,
      0.030000, 0.009166, 6.271462, 6.250520, 6.229525, 6.208476, 6.187374, 6.166219,
      6.145013, 6.123755, 6.102446, 6.081085, 6.059675, 6.038214, 6.016705, 5.995146,
      5.973539, 5.951885, 5.930184, 5.908437, 5.886643, 5.864804, 5.842922, 5.820996,
      5.799026, 5.777015, 5.754962, 5.732868, 5.710733, 5.688560, 5.666348, 5.644098,
      5.621811, 5.599488, 5.577129, 5.554736, 5.532310, 5.509849, 5.487357, 5.464833,
      5.442279, 5.419695, 5.397082, 5.374441, 5.351772, 5.329078, 5.306357, 5.283612,
      5.260843, 5.238051, 5.215237, 5.192400, 5.169544, 5.146667, 5.123772, 5.100858,
      5.077927, 5.054978, 5.032015, 5.009037, 4.986044, 4.963038, 4.940019, 4.916988,
      4.893946, 4.870894, 4.847832, 4.824761, 4.801682, 4.778595, 4.755502, 4.732402,
      4.709297, 4.686188, 4.663074, 4.639956, 4.616836, 4.593713, 4.570589, 4.547463,
      4.524337, 4.501211, 4.478085, 4.454960, 4.431836, 4.408714, 4.385595, 4.362478,
      4.339365, 4.316256, 4.293149, 4.270047, 4.246950, 4.223857, 4.200769, 4.177687,
      4.154610, 4.131539, 4.108474, 4.085414, 4.062361, 4.039313, 4.016273, 3.993238,
      3.970210, 3.947188, 3.924172, 3.901162, 3.878159, 3.855160, 3.832168, 3.809182,
      3.786201, 3.763225, 3.740254, 3.717287, 3.694325, 3.671367, 3.648412, 3.625461,
      3.602512, 3.579566, 3.556621, 3.533678, 3.510736, 3.487794, 3.464852, 3.441910,
      3.418965, 3.396019, 3.373071, 3.350119, 3.327163, 3.304203, 3.281237, 3.258265,
      3.235287, 3.212300, 3.189306, 3.166303, 3.143289, 3.120265, 3.097229, 3.074182,
      3.051120, 3.028045, 3.004956, 2.981850, 2.958727, 2.935588, 2.912430, 2.889252,
      2.866055, 2.842837, 2.819596, 2.796333, 2.773046, 2.749735, 2.726398, 2.703035,
      2.679645, 2.656226, 2.632779, 2.609302, 2.585794, 2.562254, 2.538682, 2.515076,
      2.491436, 2.467762, 2.444051, 2.420303, 2.396518, 2.372694, 2.348832, 2.324929,
      2.300985, 2.277000, 2.252972, 2.228901, 2.204786, 2.180626, 2.156421, 2.132170,
      2.107871, 2.083525, 2.059131, 2.034688, 2.010194, 1.985651, 1.961056, 1.936409,
      1.911710, 1.886957, 1.862151, 1.837291, 1.812376, 1.787405, 1.762378, 1.737294,
      1.712153, 1.686954, 1.661696, 1.636380, 1.611004, 1.585568, 1.560072, 1.534514,
      1.508895, 1.483214, 1.457471, 1.431664, 1.405795, 1.379861, 1.353863, 1.327801,
      1.301674, 1.275481, 1.249223, 1.222899, 1.196509, 1.170051, 1.143527, 1.116936,
      1.090277, 1.063550, 1.036755, 1.009893, 0.982962, 0.955963, 0.928895, 0.901758,
      0.874552, 0.847278, 0.819935, 0.792522, 0.765042, 0.737492, 0.709873, 0.682186,
      0.654430, 0.626606, 0.598714, 0.570754, 0.542726, 0.514631, 0.486469, 0.458241,
      0.429946, 0.401587, 0.373162, 0.344672, 0.316119, 0.287502, 0.258823, 0.230082,
      0.201280, 0.172418, 0.143497, 0.114519, 0.085483, 0.056390, 0.027244, 6.281229,
      6.251976, 6.222672, 6.193317, 6.163916, 6.134467, 6.104973, 6.075436, 6.045856,
      6.016236, 5.986578, 5.956883, 5.927155, 5.897393, 5.867601, 5.837780, 5.807933,
      5.778062, 5.748170, 5.718257, 5.688328, 5.658384, 5.628428, 5.598462, 5.568489,
      5.538511, 5.508531, 5.478553, 5.448577, 5.418608, 5.388648, 5.358700, 5.328766,
      5.298850, 5.268953, 5.239081, 5.209235, 5.179418, 5.149632, 5.119882, 5.090169,
      5.060498, 5.030869, 5.001288, 4.971756, 4.942276, 4.912851, 4.883485, 4.854179,
      4.824938, 4.795762, 4.766656, 4.737622, 4.708663, 4.679780, 4.650978, 4.622258,
      4.593623, 4.565076, 4.536618, 4.508252, 4.479982, 4.451808, 4.423733, 4.395760,
      4.367890, 4.340127, 4.312470, 4.284924, 4.257488, 4.230166, 4.202960, 4.175871,
      4.148901, 4.122052, 4.095325, 4.068721, 4.042243, 4.015891, 3.989668, 3.963574,
      3.937612, 3.911782, 3.886085, 3.860523, 3.835097, 3.809807, 3.784656, 3.759644,
      3.734772, 3.710041, 3.685452, 3.661005, 3.636702, 3.612544, 3.588531, 3.564662,
      3.540942, 3.517368, 3.493942, 3.470664, 3.447536, 3.424556, 3.401726, 3.379047,
      3.356519, 3.334141, 3.311915, 3.289841, 3.267919, 3.246149, 3.224531, 3.203066,
      3.181754, 3.160595, 3.139589, 3.118736, 3.098036, 3.077489, 3.057095, 3.036855,
      3.016766, 2.996830, 2.977047, 2.957417, 2.937937, 2.918609, 2.899433, 2.880407,
      2.861532, 2.842806, 2.824230, 2.805802, 2.787522, 2.769390, 2.751404, 2.733564,
      2.715869, 2.698319, 2.680911, 2.663645, 2.646521, 2.629537, 2.612691, 2.595983,
      2.579411, 2.562975, 2.546672, 2.530502, 2.514462, 2.498551, 2.482768, 2.467111,
      2.451578, 2.436168, 2.420878, 2.405707, 2.390654, 2.375715, 2.360888, 2.346174,
      2.331568, 2.317068, 2.302673, 2.288380, 2.274188, 2.260093, 2.246093, 2.232187,
      2.218372, 2.204645, 2.191003, 2.177446, 2.163970, 2.150573, 2.137252, 2.124005,
      2.110829, 2.097723, 2.084684, 2.071709, 2.058796, 2.045943, 2.033146, 2.020405,
      2.007717, 1.995080, 1.982491, 1.969948, 1.957449, 1.944993, 1.932576, 1.920198,
      1.907856, 1.895549, 1.883274, 1.871031, 1.858817, 1.846630, 1.834470, 1.822335,
      1.810223, 1.798134, 1.786066, 1.774018, 1.761987, 1.749976, 1.737981, 1.726002,
      1.714038, 1.702090, 1.690155, 1.678233, 1.666324, 1.654428, 1.642545, 1.630673,
      1.618813, 1.606965, 1.595128, 1.583303, 1.571490, 1.559689, 1.547899, 1.536122,
      1.524358, 1.512606, 1.500869, 1.489145, 1.477435, 1.465741, 1.454062, 1.442400,
      1.430755, 1.419128, 1.407519, 1.395930, 1.384360, 1.372813, 1.361287, 1.349784,
      1.338305, 1.326851, 1.315423, 1.304021, 1.292648, 1.281303, 1.269989, 1.258705,
      1.247454, 1.236235, 1.225051, 1.213902, 1.202789, 1.191713, 1.180674, 1.169676,
      1.158716, 1.147799, 1.136922, 1.126089, 1.115299, 1.104553, 1.093853, 1.083198,
      1.072590, 1.062028, 1.051515, 1.041050, 1.030633, 1.020266, 1.009948, 0.999680,
      0.989463};

  } /* namespace atsc3 */
} /* namespace gr */
