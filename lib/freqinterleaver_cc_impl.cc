/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "freqinterleaver_cc_impl.h"

namespace gr {
  namespace atsc3 {

    using input_type = gr_complex;
    using output_type = gr_complex;
    freqinterleaver_cc::sptr
    freqinterleaver_cc::make(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_first_sbs_t firstsbs, atsc3_frequency_interleaver_t mode, atsc3_reduced_carriers_t cred)
    {
      return gnuradio::make_block_sptr<freqinterleaver_cc_impl>(
        fftsize, numpayloadsyms, numpreamblesyms, guardinterval, pilotpattern, firstsbs, mode, cred);
    }


    /*
     * The private constructor
     */
    freqinterleaver_cc_impl::freqinterleaver_cc_impl(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_first_sbs_t firstsbs, atsc3_frequency_interleaver_t mode, atsc3_reduced_carriers_t cred)
      : gr::sync_block("freqinterleaver_cc",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      int total_preamble_cells, totalcells;
      int first_preamble_cells;
      int preamble_cells;
      int data_cells;
      int sbs_cells;
      int maxstates;

      fft_size = fftsize;
      interleaver_mode = mode;
      symbols = numpreamblesyms + numpayloadsyms;
      switch (fftsize) {
        case FFTSIZE_8K:
          maxstates = 8192;
          switch (guardinterval) {
            case GI_1_192:
              first_preamble_cells = preamble_cells_table[0][4];
              preamble_cells = preamble_cells_table[0][cred];
              break;
            case GI_2_384:
              first_preamble_cells = preamble_cells_table[1][4];
              preamble_cells = preamble_cells_table[1][cred];
              break;
            case GI_3_512:
              first_preamble_cells = preamble_cells_table[2][4];
              preamble_cells = preamble_cells_table[2][cred];
              break;
            case GI_4_768:
              first_preamble_cells = preamble_cells_table[3][4];
              preamble_cells = preamble_cells_table[3][cred];
              break;
            case GI_5_1024:
              first_preamble_cells = preamble_cells_table[4][4];
              preamble_cells = preamble_cells_table[4][cred];
              break;
            case GI_6_1536:
              first_preamble_cells = preamble_cells_table[5][4];
              preamble_cells = preamble_cells_table[5][cred];
              break;
            case GI_7_2048:
              first_preamble_cells = preamble_cells_table[6][4];
              preamble_cells = preamble_cells_table[6][cred];
              break;
            default:
              first_preamble_cells = preamble_cells_table[0][4];
              preamble_cells = preamble_cells_table[0][cred];
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
          maxstates = 16384;
          switch (guardinterval) {
            case GI_1_192:
              first_preamble_cells = preamble_cells_table[7][4];
              preamble_cells = preamble_cells_table[7][cred];
              break;
            case GI_2_384:
              first_preamble_cells = preamble_cells_table[8][4];
              preamble_cells = preamble_cells_table[8][cred];
              break;
            case GI_3_512:
              first_preamble_cells = preamble_cells_table[9][4];
              preamble_cells = preamble_cells_table[9][cred];
              break;
            case GI_4_768:
              first_preamble_cells = preamble_cells_table[10][4];
              preamble_cells = preamble_cells_table[10][cred];
              break;
            case GI_5_1024:
              first_preamble_cells = preamble_cells_table[11][4];
              preamble_cells = preamble_cells_table[11][cred];
              break;
            case GI_6_1536:
              first_preamble_cells = preamble_cells_table[12][4];
              preamble_cells = preamble_cells_table[12][cred];
              break;
            case GI_7_2048:
              first_preamble_cells = preamble_cells_table[13][4];
              preamble_cells = preamble_cells_table[13][cred];
              break;
            case GI_8_2432:
              first_preamble_cells = preamble_cells_table[14][4];
              preamble_cells = preamble_cells_table[14][cred];
              break;
            case GI_9_3072:
              first_preamble_cells = preamble_cells_table[15][4];
              preamble_cells = preamble_cells_table[15][cred];
              break;
            case GI_10_3648:
              first_preamble_cells = preamble_cells_table[16][4];
              preamble_cells = preamble_cells_table[16][cred];
              break;
            case GI_11_4096:
              first_preamble_cells = preamble_cells_table[17][4];
              preamble_cells = preamble_cells_table[17][cred];
              break;
            default:
              first_preamble_cells = preamble_cells_table[7][4];
              preamble_cells = preamble_cells_table[7][cred];
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
          maxstates = 32768;
          switch (guardinterval) {
            case GI_1_192:
              first_preamble_cells = preamble_cells_table[18][4];
              preamble_cells = preamble_cells_table[18][cred];
              break;
            case GI_2_384:
              first_preamble_cells = preamble_cells_table[19][4];
              preamble_cells = preamble_cells_table[19][cred];
              break;
            case GI_3_512:
              first_preamble_cells = preamble_cells_table[20][4];
              preamble_cells = preamble_cells_table[20][cred];
              break;
            case GI_4_768:
              first_preamble_cells = preamble_cells_table[21][4];
              preamble_cells = preamble_cells_table[21][cred];
              break;
            case GI_5_1024:
              first_preamble_cells = preamble_cells_table[22][4];
              preamble_cells = preamble_cells_table[22][cred];
              break;
            case GI_6_1536:
              first_preamble_cells = preamble_cells_table[23][4];
              preamble_cells = preamble_cells_table[23][cred];
              break;
            case GI_7_2048:
              first_preamble_cells = preamble_cells_table[24][4];
              preamble_cells = preamble_cells_table[24][cred];
              break;
            case GI_8_2432:
              first_preamble_cells = preamble_cells_table[25][4];
              preamble_cells = preamble_cells_table[25][cred];
              break;
            case GI_9_3072:
              if (pilotpattern == PILOT_SP8_2 || pilotpattern == PILOT_SP8_4) {
                first_preamble_cells = preamble_cells_table[26][4];
                preamble_cells = preamble_cells_table[26][cred];
              }
              else {
                first_preamble_cells = preamble_cells_table[27][4];
                preamble_cells = preamble_cells_table[27][cred];
              }
              break;
            case GI_10_3648:
              if (pilotpattern == PILOT_SP8_2 || pilotpattern == PILOT_SP8_4) {
                first_preamble_cells = preamble_cells_table[28][4];
                preamble_cells = preamble_cells_table[28][cred];
              }
              else {
                first_preamble_cells = preamble_cells_table[29][4];
                preamble_cells = preamble_cells_table[29][cred];
              }
              break;
            case GI_11_4096:
              first_preamble_cells = preamble_cells_table[30][4];
              preamble_cells = preamble_cells_table[30][cred];
              break;
            case GI_12_4864:
              first_preamble_cells = preamble_cells_table[31][4];
              preamble_cells = preamble_cells_table[31][cred];
              break;
            default:
              first_preamble_cells = preamble_cells_table[18][4];
              preamble_cells = preamble_cells_table[18][cred];
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
          maxstates = 8192;
          switch (guardinterval) {
            case GI_1_192:
              first_preamble_cells = preamble_cells_table[0][4];
              preamble_cells = preamble_cells_table[0][cred];
              break;
            case GI_2_384:
              first_preamble_cells = preamble_cells_table[1][4];
              preamble_cells = preamble_cells_table[1][cred];
              break;
            case GI_3_512:
              first_preamble_cells = preamble_cells_table[2][4];
              preamble_cells = preamble_cells_table[2][cred];
              break;
            case GI_4_768:
              first_preamble_cells = preamble_cells_table[3][4];
              preamble_cells = preamble_cells_table[3][cred];
              break;
            case GI_5_1024:
              first_preamble_cells = preamble_cells_table[4][4];
              preamble_cells = preamble_cells_table[4][cred];
              break;
            case GI_6_1536:
              first_preamble_cells = preamble_cells_table[5][4];
              preamble_cells = preamble_cells_table[5][cred];
              break;
            case GI_7_2048:
              first_preamble_cells = preamble_cells_table[6][4];
              preamble_cells = preamble_cells_table[6][cred];
              break;
            default:
              first_preamble_cells = preamble_cells_table[0][4];
              preamble_cells = preamble_cells_table[0][cred];
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

      frame_cells[0] = first_preamble_cells;
      total_preamble_cells = 0;
      for (int n = 1; n < numpreamblesyms; n++) {
        frame_cells[n] = preamble_cells;
        total_preamble_cells += preamble_cells;
      }
      if (firstsbs == TRUE) {
        frame_cells[numpreamblesyms] = sbs_cells;
        for (int n = 0; n < numpayloadsyms; n++) {
          frame_cells[n + numpreamblesyms + 1] = data_cells;
        }
      }
      else {
        for (int n = 0; n < numpayloadsyms; n++) {
          frame_cells[n + numpreamblesyms] = data_cells;
        }
      }
      frame_cells[numpreamblesyms + numpayloadsyms - 1] = sbs_cells;

      HevenFP.resize(symbols);
      HoddFP.resize(symbols);
      HevenP.resize(symbols);
      HoddP.resize(symbols);
      HevenSBS.resize(symbols);
      HoddSBS.resize(symbols);
      Heven.resize(symbols);
      Hodd.resize(symbols);
      for (std::vector<std::vector<int>>::size_type i = 0; i != Heven.size(); i++) {
        HevenFP[i].resize(maxstates);
        HoddFP[i].resize(maxstates);
        HevenP[i].resize(maxstates);
        HoddP[i].resize(maxstates);
        HevenSBS[i].resize(maxstates);
        HoddSBS[i].resize(maxstates);
        Heven[i].resize(maxstates);
        Hodd[i].resize(maxstates);
      }
      init_address(first_preamble_cells, preamble_cells, sbs_cells, data_cells);

      if (firstsbs) {
        totalcells = first_preamble_cells + total_preamble_cells + ((numpayloadsyms - 2) * data_cells) + (sbs_cells * 2);
      }
      else {
        totalcells = first_preamble_cells + total_preamble_cells + ((numpayloadsyms - 1) * data_cells) + sbs_cells;
      }
      output_cells = totalcells;
      printf("output cells = %d\n", totalcells);
      set_output_multiple(totalcells);
    }

    /*
     * Our virtual destructor.
     */
    freqinterleaver_cc_impl::~freqinterleaver_cc_impl()
    {
    }

    void
    freqinterleaver_cc_impl::init_address(int firstpreamblecells, int preamblecells, int sbscells, int datacells)
    {
      int max_states, xor_size, xor_size2, pn_mask, result;
      int q_evenFP;
      int q_oddFP;
      int q_evenP;
      int q_oddP;
      int q_evenSBS;
      int q_oddSBS;
      int q_even;
      int q_odd;
      int lfsr;
      int lfsr2 = 0;
      int logic8k[4] = {0, 1, 4, 6};
      int logic16k[6] = {0, 1, 4, 5, 9, 11};
      int logic32k[4] = {0, 1, 2, 12};
      int logic8k2[6] = {0, 1, 4, 5, 9, 11};
      int logic16k2[4] = {0, 1, 2, 12};
      int logic32k2[2] = {0, 1};
      int* logic;
      int* logic2;
      const int *bitpermeven, *bitpermodd;
      int pn_degree, even, odd;

      switch (fft_size) {
        case FFTSIZE_8K:
          pn_degree = 12;
          pn_mask = 0xfff;
          max_states = 8192;
          logic = &logic8k[0];
          logic2 = &logic8k2[0];
          xor_size = 4;
          xor_size2 = 6;
          bitpermeven = &bitperm8keven[0];
          bitpermodd = &bitperm8kodd[0];
          break;
        case FFTSIZE_16K:
          pn_degree = 13;
          pn_mask = 0x1fff;
          max_states = 16384;
          logic = &logic16k[0];
          logic2 = &logic16k2[0];
          xor_size = 6;
          xor_size2 = 4;
          bitpermeven = &bitperm16keven[0];
          bitpermodd = &bitperm16kodd[0];
          break;
        case FFTSIZE_32K:
          pn_degree = 14;
          pn_mask = 0x3fff;
          max_states = 32768;
          logic = &logic32k[0];
          logic2 = &logic32k2[0];
          xor_size = 4;
          xor_size2 = 2;
          bitpermeven = &bitperm32k[0];
          bitpermodd = &bitperm32k[0];
          break;
        default:
          pn_degree = 12;
          pn_mask = 0xfff;
          max_states = 8192;
          logic = &logic8k[0];
          logic2 = &logic8k2[0];
          xor_size = 4;
          xor_size2 = 6;
          bitpermeven = &bitperm8keven[0];
          bitpermodd = &bitperm8kodd[0];
          break;
      }

      for (int i = 0; i < symbols; i++) {
        std::vector<int>& HevenFP = this->HevenFP[i];
        std::vector<int>& HoddFP = this->HoddFP[i];
        std::vector<int>& HevenP = this->HevenP[i];
        std::vector<int>& HoddP = this->HoddP[i];
        std::vector<int>& HevenSBS = this->HevenSBS[i];
        std::vector<int>& HoddSBS = this->HoddSBS[i];
        std::vector<int>& Heven = this->Heven[i];
        std::vector<int>& Hodd = this->Hodd[i];
        q_evenFP = 0;
        q_oddFP = 0;
        q_evenP = 0;
        q_oddP = 0;
        q_evenSBS = 0;
        q_oddSBS = 0;
        q_even = 0;
        q_odd = 0;
        if ((i % 2) == 0) {
          if (i == 0) {
            lfsr2 = (pn_mask << 1) | 0x1;
          }
          else {
            result = 0;
            for (int k = 0; k < xor_size2; k++) {
              result ^= (lfsr2 >> logic2[k]) & 1;
            }
            lfsr2 &= (pn_mask << 1) | 0x1;
            lfsr2 >>= 1;
            lfsr2 |= result << (pn_degree);
          }
        }

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
          even = 0;
          odd = 0;
          for (int n = 0; n < pn_degree; n++) {
            even |= ((lfsr >> n) & 0x1) << bitpermeven[n];
          }
          for (int n = 0; n < pn_degree; n++) {
            odd |= ((lfsr >> n) & 0x1) << bitpermodd[n];
          }
          even = even + ((j % 2) * (max_states / 2));
          odd = odd + ((j % 2) * (max_states / 2));
          even ^= lfsr2;
          odd ^= lfsr2;
          if (even < firstpreamblecells) {
            HevenFP[q_evenFP++] = even;
          }
          if (odd < firstpreamblecells) {
            HoddFP[q_oddFP++] = odd;
          }
          if (even < preamblecells) {
            HevenP[q_evenP++] = even;
          }
          if (odd < preamblecells) {
            HoddP[q_oddP++] = odd;
          }
          if (even < sbscells) {
            HevenSBS[q_evenSBS++] = even;
          }
          if (odd < sbscells) {
            HoddSBS[q_oddSBS++] = odd;
          }
          if (even < datacells) {
            Heven[q_even++] = even;
          }
          if (odd < datacells) {
            Hodd[q_odd++] = odd;
          }
        }
        if (fft_size == FFTSIZE_32K) {
          for (int n = 0; n < q_oddFP; n++) {
            int a;
            a = HoddFP[n];
            HevenFP[a] = n;
          }
          for (int n = 0; n < q_oddP; n++) {
            int a;
            a = HoddP[n];
            HevenP[a] = n;
          }
          for (int n = 0; n < q_oddSBS; n++) {
            int a;
            a = HoddSBS[n];
            HevenSBS[a] = n;
          }
          for (int n = 0; n < q_odd; n++) {
            int a;
            a = Hodd[n];
            Heven[a] = n;
          }
        }
      }
    }

    int
    freqinterleaver_cc_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      std::vector<int> H;
      int index;

      for (int i = 0; i < noutput_items; i += output_cells) {
        for (int j = 0; j < symbols; j++) {
          if (frame_symbols[j] == PREAMBLE_SYMBOL) {
            if (j == 0) {
              if ((j % 2) == 0) {
                H = HevenFP[j];
              }
              else {
                H = HoddFP[j];
              }
            }
            else {
              if ((j % 2) == 0) {
                H = HevenP[j];
              }
              else {
                H = HoddP[j];
              }
            }
            for (int n = 0; n < frame_cells[j]; n++) {
              *out++ = in[H[n]];
            }
            in += frame_cells[j];
          }
          else if (interleaver_mode == FREQ_ALL_SYMBOLS) {
            if (frame_symbols[j] == SBS_SYMBOL) {
              if ((j % 2) == 0) {
                H = HevenSBS[j];
              }
              else {
                H = HoddSBS[j];
              }
            }
            else {
              if ((j % 2) == 0) {
                H = Heven[j];
              }
              else {
                H = Hodd[j];
              }
            }
            for (int n = 0; n < frame_cells[j]; n++) {
              *out++ = in[H[n]];
            }
            in += frame_cells[j];
          }
          else {
            index = 0;
            for (int n = 0; n < frame_cells[j]; n++) {
              *out++ = in[index++];
            }
            in += frame_cells[j];
          }
        }
      }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    const int freqinterleaver_cc_impl::bitperm8keven[12] = {7, 1, 4, 2, 9, 6, 8, 10, 0, 3, 11, 5
    };

    const int freqinterleaver_cc_impl::bitperm8kodd[12] = {11, 4, 9, 3, 1, 2, 5, 0, 6, 7, 10, 8
    };

    const int freqinterleaver_cc_impl::bitperm16keven[13] = {9, 7, 6, 10, 12, 5, 1, 11, 0, 2, 3, 4, 8
    };

    const int freqinterleaver_cc_impl::bitperm16kodd[13] = {6, 8, 10, 12, 2, 0, 4, 1, 11, 3, 5, 9, 7
    };

    const int freqinterleaver_cc_impl::bitperm32k[14] = {7, 13, 3, 4, 9, 2, 12, 11, 1, 8, 10, 0, 5, 6
    };

    const int freqinterleaver_cc_impl::preamble_cells_table[32][5] = {
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

    const int freqinterleaver_cc_impl::data_cells_table_8K[16][5] = {
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

    const int freqinterleaver_cc_impl::data_cells_table_16K[16][5] = {
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

    const int freqinterleaver_cc_impl::data_cells_table_32K[16][5] = {
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

    const int freqinterleaver_cc_impl::sbs_cells_table_8K[16][5] = {
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

    const int freqinterleaver_cc_impl::sbs_cells_table_16K[16][5] = {
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

    const int freqinterleaver_cc_impl::sbs_cells_table_32K[16][5] = {
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
