/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "bootstrap_cc_impl.h"
#include <gnuradio/math.h>
#include <gnuradio/fft/window.h>
#include <gnuradio/filter/firdes.h>

namespace gr {
  namespace atsc3 {

    using input_type = gr_complex;
    using output_type = gr_complex;
    bootstrap_cc::sptr
    bootstrap_cc::make(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_min_time_to_next_t frameinterval, atsc3_l1_fec_mode_t l1bmode, atsc3_bootstrap_mode_t outputmode, atsc3_showlevels_t showlevels)
    {
      return gnuradio::make_block_sptr<bootstrap_cc_impl>(
        fftsize, numpayloadsyms, numpreamblesyms, guardinterval, pilotpattern, frameinterval, l1bmode, outputmode, showlevels);
    }


    /*
     * The private constructor
     */
    bootstrap_cc_impl::bootstrap_cc_impl(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_min_time_to_next_t frameinterval, atsc3_l1_fec_mode_t l1bmode, atsc3_bootstrap_mode_t outputmode, atsc3_showlevels_t showlevels)
      : gr::block("bootstrap_cc",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type))),
        show_levels(showlevels),
        real_positive(0.0),
        real_negative(0.0),
        imag_positive(0.0),
        imag_negative(0.0),
        real_positive_threshold(1.0),
        real_negative_threshold(-1.0),
        imag_positive_threshold(1.0),
        imag_negative_threshold(-1.0),
        real_positive_threshold_count(0),
        real_negative_threshold_count(0),
        imag_positive_threshold_count(0),
        imag_negative_threshold_count(0),
        bootstrap_fft(BOOTSTRAP_FFT_SIZE, 1)
    {
      int symbols;
      int symbol_size;
      int guard_interval;
      int bootstrap_fft_size = BOOTSTRAP_FFT_SIZE;
      int zcindex, pnindex;
      int reverse;
      int left_nulls = 275;
      int relative_cyclic_shift;
      int absolute_cyclic_shift;
      int preamble_structure;
      gr_complex phase_shift;
      gr_complex zero = gr_complex(0.0, 0.0);
      gr_complex* dst;
      gr_complex* in;
      gr_complex* out;
      unsigned char bootstrap_signal[3] = {};
      unsigned int interpolation = 9;
      unsigned int decimation = 8;
      unsigned int ctr;
      float fractional_bw = 0.4;
      std::vector<gr_complex> staps;

      staps = design_resampler_filter(interpolation, decimation, fractional_bw);
      d_decimation = decimation;
      d_firs.reserve(interpolation);
      for (unsigned i = 0; i < interpolation; i++) {
        d_firs.emplace_back(std::vector<gr_complex>());
      }
      set_taps(staps);
      install_taps(d_new_taps);

      output_mode = outputmode;
      symbols = numpreamblesyms + numpayloadsyms;
      bootstrap_signal[0] = (frameinterval << 2) | SYSTEM_BANDWIDTH_6MHZ;
      bootstrap_signal[1] = BSR_COEFFICIENT;
      switch (fftsize) {
        case FFTSIZE_8K:
          symbol_size = 8192;
          switch (guardinterval) {
            case GI_1_192:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 0;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 1;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 2;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 3;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 4;
                  break;
                default:
                  preamble_structure = 0;
                  break;
              }
              break;
            case GI_2_384:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 5;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 6;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 7;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 8;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 9;
                  break;
                default:
                  preamble_structure = 5;
                  break;
              }
              break;
            case GI_3_512:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 10;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 11;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 12;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 13;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 14;
                  break;
                default:
                  preamble_structure = 10;
                  break;
              }
              break;
            case GI_4_768:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 15;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 16;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 17;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 18;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 19;
                  break;
                default:
                  preamble_structure = 15;
                  break;
              }
              break;
            case GI_5_1024:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 20;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 21;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 22;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 23;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 24;
                  break;
                default:
                  preamble_structure = 20;
                  break;
              }
              break;
            case GI_6_1536:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 25;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 26;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 27;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 28;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 29;
                  break;
                default:
                  preamble_structure = 25;
                  break;
              }
              break;
            case GI_7_2048:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 30;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 31;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 32;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 33;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 34;
                  break;
                default:
                  preamble_structure = 30;
                  break;
              }
              break;
            default:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 0;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 1;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 2;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 3;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 4;
                  break;
                default:
                  preamble_structure = 0;
                  break;
              }
              break;
          }
          break;
        case FFTSIZE_16K:
          symbol_size = 16384;
          switch (guardinterval) {
            case GI_1_192:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 35;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 36;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 37;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 38;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 39;
                  break;
                default:
                  preamble_structure = 35;
                  break;
              }
              break;
            case GI_2_384:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 40;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 41;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 42;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 43;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 44;
                  break;
                default:
                  preamble_structure = 40;
                  break;
              }
              break;
            case GI_3_512:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 45;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 46;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 47;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 48;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 49;
                  break;
                default:
                  preamble_structure = 45;
                  break;
              }
              break;
            case GI_4_768:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 50;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 51;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 52;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 53;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 54;
                  break;
                default:
                  preamble_structure = 50;
                  break;
              }
              break;
            case GI_5_1024:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 55;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 56;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 57;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 58;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 59;
                  break;
                default:
                  preamble_structure = 55;
                  break;
              }
              break;
            case GI_6_1536:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 60;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 61;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 62;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 63;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 64;
                  break;
                default:
                  preamble_structure = 60;
                  break;
              }
              break;
            case GI_7_2048:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 65;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 66;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 67;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 68;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 69;
                  break;
                default:
                  preamble_structure = 65;
                  break;
              }
              break;
            case GI_8_2432:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 70;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 71;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 72;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 73;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 74;
                  break;
                default:
                  preamble_structure = 70;
                  break;
              }
              break;
            case GI_9_3072:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 75;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 76;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 77;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 78;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 79;
                  break;
                default:
                  preamble_structure = 75;
                  break;
              }
              break;
            case GI_10_3648:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 80;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 81;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 82;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 83;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 84;
                  break;
                default:
                  preamble_structure = 80;
                  break;
              }
              break;
            case GI_11_4096:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 85;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 86;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 87;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 88;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 89;
                  break;
                default:
                  preamble_structure = 85;
                  break;
              }
              break;
            default:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 35;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 36;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 37;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 38;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 39;
                  break;
                default:
                  preamble_structure = 35;
                  break;
              }
          }
          break;
        case FFTSIZE_32K:
          symbol_size = 32768;
          switch (guardinterval) {
            case GI_1_192:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 90;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 91;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 92;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 93;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 94;
                  break;
                default:
                  preamble_structure = 90;
                  break;
              }
              break;
            case GI_2_384:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 95;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 96;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 97;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 98;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 99;
                  break;
                default:
                  preamble_structure = 95;
                  break;
              }
              break;
            case GI_3_512:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 100;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 101;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 102;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 103;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 104;
                  break;
                default:
                  preamble_structure = 100;
                  break;
              }
              break;
            case GI_4_768:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 105;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 106;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 107;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 108;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 109;
                  break;
                default:
                  preamble_structure = 105;
                  break;
              }
              break;
            case GI_5_1024:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 110;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 111;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 112;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 113;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 114;
                  break;
                default:
                  preamble_structure = 110;
                  break;
              }
              break;
            case GI_6_1536:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 115;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 116;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 117;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 118;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 119;
                  break;
                default:
                  preamble_structure = 115;
                  break;
              }
              break;
            case GI_7_2048:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 120;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 121;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 122;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 123;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 124;
                  break;
                default:
                  preamble_structure = 120;
                  break;
              }
              break;
            case GI_8_2432:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 125;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 126;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 127;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 128;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 129;
                  break;
                default:
                  preamble_structure = 125;
                  break;
              }
              break;
            case GI_9_3072:
              if (pilotpattern == PILOT_SP8_2 || pilotpattern == PILOT_SP8_4) {
                switch (l1bmode) {
                  case L1_FEC_MODE_1:
                    preamble_structure = 130;
                    break;
                  case L1_FEC_MODE_2:
                    preamble_structure = 131;
                    break;
                  case L1_FEC_MODE_3:
                    preamble_structure = 132;
                    break;
                  case L1_FEC_MODE_4:
                    preamble_structure = 133;
                    break;
                  case L1_FEC_MODE_5:
                    preamble_structure = 134;
                    break;
                  default:
                    preamble_structure = 130;
                    break;
                }
              }
              else {
                switch (l1bmode) {
                  case L1_FEC_MODE_1:
                    preamble_structure = 135;
                    break;
                  case L1_FEC_MODE_2:
                    preamble_structure = 136;
                    break;
                  case L1_FEC_MODE_3:
                    preamble_structure = 137;
                    break;
                  case L1_FEC_MODE_4:
                    preamble_structure = 138;
                    break;
                  case L1_FEC_MODE_5:
                    preamble_structure = 139;
                    break;
                  default:
                    preamble_structure = 135;
                    break;
                }
              }
              break;
            case GI_10_3648:
              if (pilotpattern == PILOT_SP8_2 || pilotpattern == PILOT_SP8_4) {
                switch (l1bmode) {
                  case L1_FEC_MODE_1:
                    preamble_structure = 140;
                    break;
                  case L1_FEC_MODE_2:
                    preamble_structure = 141;
                    break;
                  case L1_FEC_MODE_3:
                    preamble_structure = 142;
                    break;
                  case L1_FEC_MODE_4:
                    preamble_structure = 143;
                    break;
                  case L1_FEC_MODE_5:
                    preamble_structure = 144;
                    break;
                  default:
                    preamble_structure = 140;
                    break;
                }
              }
              else {
                switch (l1bmode) {
                  case L1_FEC_MODE_1:
                    preamble_structure = 145;
                    break;
                  case L1_FEC_MODE_2:
                    preamble_structure = 146;
                    break;
                  case L1_FEC_MODE_3:
                    preamble_structure = 147;
                    break;
                  case L1_FEC_MODE_4:
                    preamble_structure = 148;
                    break;
                  case L1_FEC_MODE_5:
                    preamble_structure = 149;
                    break;
                  default:
                    preamble_structure = 145;
                    break;
                }
              }
              break;
            case GI_11_4096:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 150;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 151;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 152;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 153;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 154;
                  break;
                default:
                  preamble_structure = 150;
                  break;
              }
              break;
            case GI_12_4864:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 155;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 156;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 157;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 158;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 159;
                  break;
                default:
                  preamble_structure = 155;
                  break;
              }
              break;
            default:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 90;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 91;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 92;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 93;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 94;
                  break;
                default:
                  preamble_structure = 90;
                  break;
              }
              break;
          }
          break;
        default:
          symbol_size = 8192;
          switch (guardinterval) {
            case GI_1_192:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 0;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 1;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 2;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 3;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 4;
                  break;
                default:
                  preamble_structure = 0;
                  break;
              }
              break;
            case GI_2_384:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 5;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 6;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 7;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 8;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 9;
                  break;
                default:
                  preamble_structure = 5;
                  break;
              }
              break;
            case GI_3_512:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 10;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 11;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 12;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 13;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 14;
                  break;
                default:
                  preamble_structure = 10;
                  break;
              }
              break;
            case GI_4_768:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 15;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 16;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 17;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 18;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 19;
                  break;
                default:
                  preamble_structure = 15;
                  break;
              }
              break;
            case GI_5_1024:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 20;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 21;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 22;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 23;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 24;
                  break;
                default:
                  preamble_structure = 20;
                  break;
              }
              break;
            case GI_6_1536:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 25;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 26;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 27;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 28;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 29;
                  break;
                default:
                  preamble_structure = 25;
                  break;
              }
              break;
            case GI_7_2048:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 30;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 31;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 32;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 33;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 34;
                  break;
                default:
                  preamble_structure = 30;
                  break;
              }
              break;
            default:
              switch (l1bmode) {
                case L1_FEC_MODE_1:
                  preamble_structure = 0;
                  break;
                case L1_FEC_MODE_2:
                  preamble_structure = 1;
                  break;
                case L1_FEC_MODE_3:
                  preamble_structure = 2;
                  break;
                case L1_FEC_MODE_4:
                  preamble_structure = 3;
                  break;
                case L1_FEC_MODE_5:
                  preamble_structure = 4;
                  break;
                default:
                  preamble_structure = 0;
                  break;
              }
              break;
          }
          break;
      }
      bootstrap_signal[2] = preamble_structure;
      switch (guardinterval) {
        case GI_1_192:
          guard_interval = 192;
          break;
        case GI_2_384:
          guard_interval = 384;
          break;
        case GI_3_512:
          guard_interval = 512;
          break;
        case GI_4_768:
          guard_interval = 768;
          break;
        case GI_5_1024:
          guard_interval = 1024;
          break;
        case GI_6_1536:
          guard_interval = 1536;
          break;
        case GI_7_2048:
          guard_interval = 2048;
          break;
        case GI_8_2432:
          guard_interval = 2432;
          break;
        case GI_9_3072:
          guard_interval = 3072;
          break;
        case GI_10_3648:
          guard_interval = 3648;
          break;
        case GI_11_4096:
          guard_interval = 4096;
          break;
        case GI_12_4864:
          guard_interval = 4864;
          break;
        default:
          guard_interval = 192;
          break;
      }
      init_pseudo_noise_sequence();
      init_zadoff_chu_sequence();
      absolute_cyclic_shift = 0;
      pnindex = 0;
      for (int k = 0; k < NUM_BOOTSTRAP_SYMBOLS; k++) {
        zcindex = 0;
        std::fill_n(&bootstrap_freq[0], BOOTSTRAP_FFT_SIZE, 0);
        for (int i = 0; i < ZADOFF_CHU_LENGTH / 2; i++) {
          if (pnseq[pnindex]) {
            bootstrap_freq[zcindex + left_nulls] = -zcseq[zcindex];
          }
          else {
            bootstrap_freq[zcindex + left_nulls] = zcseq[zcindex];
          }
          zcindex++;
          pnindex++;
        }
        bootstrap_freq[zcindex + left_nulls] = zero;
        zcindex++;
        reverse = pnindex - 1;
        for (int i = 0; i < ZADOFF_CHU_LENGTH / 2; i++) {
          if (pnseq[reverse--]) {
            bootstrap_freq[zcindex + left_nulls] = -zcseq[zcindex];
          }
          else {
            bootstrap_freq[zcindex + left_nulls] = zcseq[zcindex];
          }
          zcindex++;
        }
        in = &bootstrap_freq[0];
        dst = bootstrap_fft.get_inbuf();
        if (k == 0) {
          out = &bootstrap_time[k][0];
        }
        else {
          out = &bootstrap_partb[k][0]; /* use as temporary storage */
        }
        memcpy(&dst[bootstrap_fft_size / 2], &in[0], sizeof(gr_complex) * bootstrap_fft_size / 2);
        memcpy(&dst[0], &in[bootstrap_fft_size / 2], sizeof(gr_complex) * bootstrap_fft_size / 2);
        bootstrap_fft.execute();
        memcpy(out, bootstrap_fft.get_outbuf(), sizeof(gr_complex) * bootstrap_fft_size);
        if (k != 0) {
          relative_cyclic_shift = gray_code_cyclic_shift(reversebits(bootstrap_signal[k - 1]));
          absolute_cyclic_shift = (absolute_cyclic_shift - relative_cyclic_shift) % BOOTSTRAP_FFT_SIZE;
          if (absolute_cyclic_shift < 0) {
            absolute_cyclic_shift += BOOTSTRAP_FFT_SIZE;
          }
          for (int i = 0; i < BOOTSTRAP_FFT_SIZE - absolute_cyclic_shift; i++) {
            bootstrap_time[k][i + absolute_cyclic_shift] = bootstrap_partb[k][i];
          }
          for (int i = 0; i < absolute_cyclic_shift; i++) {
            bootstrap_time[k][i] = bootstrap_partb[k][i + (BOOTSTRAP_FFT_SIZE - absolute_cyclic_shift)];
          }
        }
        if (k == 3) {
          for (int i = 0; i < BOOTSTRAP_FFT_SIZE; i++) {
            bootstrap_time[k][i] *= -1.0 / std::sqrt(1498.0);
          }
        }
        else {
          for (int i = 0; i < BOOTSTRAP_FFT_SIZE; i++) {
            bootstrap_time[k][i] *= 1.0 / std::sqrt(1498.0);
          }
        }
        if (k == 0) {
          for (int n = 0; n < B_SIZE; n++) {
            bootstrap_partb[k][n] = bootstrap_time[k][n + (BOOTSTRAP_FFT_SIZE - B_SIZE)] * std::exp(gr_complex(0.0, 2 * GR_M_PI * float(n + C_SIZE) / 2048.0));
          }
        }
        else {
          for (int n = 0; n < B_SIZE; n++) {
            bootstrap_partb[k][n] = bootstrap_time[k][n + (BOOTSTRAP_FFT_SIZE - C_SIZE)] * std::exp(gr_complex(0.0, -2 * GR_M_PI * float(n - C_SIZE) / 2048.0));
          }
        }
      }

      out = &bootstrap_symbol[0];
      for (int j = 0; j < NUM_BOOTSTRAP_SYMBOLS; j++) {
        if (j == 0) {
          for (int n = 0; n < C_SIZE; n++) {
            *out++ = bootstrap_time[j][n + (BOOTSTRAP_FFT_SIZE - C_SIZE)];
          }
          for (int n = 0; n < BOOTSTRAP_FFT_SIZE; n++) {
            *out++ = bootstrap_time[j][n];
          }
          for (int n = 0; n < B_SIZE; n++) {
            *out++ = bootstrap_partb[j][n];
          }
        }
        else {
          for (int n = 0; n < B_SIZE; n++) {
            *out++ = bootstrap_partb[j][n];
          }
          for (int n = 0; n < C_SIZE; n++) {
            *out++ = bootstrap_time[j][n + (BOOTSTRAP_FFT_SIZE - C_SIZE)];
          }
          for (int n = 0; n < BOOTSTRAP_FFT_SIZE; n++) {
            *out++ = bootstrap_time[j][n];
          }
        }
      }

      printf("interpolation = %d\n", this->interpolation());
      printf("decimation = %d\n", this->decimation());
      ctr = 0;
      unsigned int index;
      in = &bootstrap_symbol[0];
      out = &bootstrap_resample[0];
      for (index = 0; index < (((BOOTSTRAP_FFT_SIZE + B_SIZE + C_SIZE) * NUM_BOOTSTRAP_SYMBOLS * interpolation) / decimation); index++) {
        out[index] = d_firs[ctr].filter(in);
        ctr += this->decimation();
        while (ctr >= this->interpolation()) {
          ctr -= this->interpolation();
          in++;
        }
      }
      printf("index = %d\n", index);
      printf("in delta = %ld\n", in - &bootstrap_symbol[0]);

      out = &bootstrap_resample[0];
      for (int n = 0; n < 16; n++) {
        printf("%f, %f\n", out[n].real(), out[n].imag());
      }

      frame_items = (symbols * symbol_size) + (symbols * guard_interval);
      if (outputmode) {
        insertion_items = ((frame_items + ((BOOTSTRAP_FFT_SIZE + guard_interval) * NUM_BOOTSTRAP_SYMBOLS) * interpolation) / decimation);
      }
      else {
        insertion_items = frame_items + ((BOOTSTRAP_FFT_SIZE + guard_interval) * NUM_BOOTSTRAP_SYMBOLS);
      }
      set_output_multiple(insertion_items);
    }

    /*
     * Our virtual destructor.
     */
    bootstrap_cc_impl::~bootstrap_cc_impl()
    {
    }

    void
    bootstrap_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = frame_items * (noutput_items / insertion_items);
    }

    void
    bootstrap_cc_impl::init_pseudo_noise_sequence(void)
    {
      int sr = 0x19d;

      for (int i = 0; i < ZADOFF_CHU_LENGTH * (NUM_BOOTSTRAP_SYMBOLS / 2); i++) {
        int b = ((sr) ^ (sr >> 1) ^ (sr >> 14) ^ (sr >> 15) ^ (sr >> 16)) & 1;
        pnseq[i] = sr & 1;
        sr >>= 1;
        if (b) {
          sr |= 0x8000;
        }
      }
    }

    void
    bootstrap_cc_impl::init_zadoff_chu_sequence(void)
    {
      int q = 137;

      for (int n = 0; n < ZADOFF_CHU_LENGTH; n++) {
        zcseq[n] = std::exp(gr_complexd(0.0, 1.0) * gr_complexd(GR_M_PI * q * double(-1 * n * (n + 1)) / 1499.0, 0.0));
      }
    }

    int
    bootstrap_cc_impl::gray_code_cyclic_shift(int signal_bits)
    {
      int m[11];
      int sum, weight;

      for (int i = 0; i < 11; i++) {
        if (i < (10 - VALID_SIGNALLING_BITS)) {
          m[i] = 0;
        }
        else if (i == 10 - (VALID_SIGNALLING_BITS)) {
          m[i] = 1;
        }
        else {
          sum = 0;
          for (int k = 0; k <= (10 - i); k++) {
            sum += ((signal_bits) >> k) & 0x1;
          }
          m[i] = sum % 2;
        }
      }
      sum = 0;
      weight = 1;
      for (int n = 0; n < 11; n++) {
        sum += m[n] * weight;
        weight <<= 1;
      }
      return sum;
    }

    unsigned char
    bootstrap_cc_impl::reversebits(unsigned char b)
    {
      b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
      b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
      b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
      return b;
    }


    std::vector<float>
    bootstrap_cc_impl::design_resampler_filter_float(const unsigned interpolation, const unsigned decimation, const float fractional_bw)
    {
      // These are default values used to generate the filter when no taps are known
      float beta = 7.0;
      float halfband = 0.5;
      float rate = float(interpolation) / float(decimation);
      float trans_width, mid_transition_band;

      if (rate >= 1.0) {
        trans_width = halfband - fractional_bw;
        mid_transition_band = halfband - trans_width / 2.0;
      } else {
        trans_width = rate * (halfband - fractional_bw);
        mid_transition_band = rate * halfband - trans_width / 2.0;
      }

      return gr::filter::firdes::low_pass(interpolation,       /* gain */
                                          interpolation,       /* Fs */
                                          mid_transition_band, /* trans mid point */
                                          trans_width,         /* transition width */
                                          gr::fft::window::WIN_KAISER,
                                          beta); /* beta*/
    }

    std::vector<gr_complex>
    bootstrap_cc_impl::design_resampler_filter(const unsigned interpolation, const unsigned decimation, const float fractional_bw)
    {
      auto real_taps = design_resampler_filter_float(interpolation, decimation, fractional_bw);

      std::vector<gr_complex> cplx_taps(real_taps.size());
      for (size_t i = 0; i < real_taps.size(); i++) {
        cplx_taps[i] = real_taps[i];
      }

      return cplx_taps;
    }

    void
    bootstrap_cc_impl::set_taps(const std::vector<gr_complex>& taps)
    {
      d_new_taps = taps;
      // round up length to a multiple of the interpolation factor
      int n = taps.size() % this->interpolation();
      if (n > 0) {
        n = this->interpolation() - n;
        while (n-- > 0) {
          d_new_taps.insert(d_new_taps.end(), 0);
        }
      }
      assert(d_new_taps.size() % this->interpolation() == 0);
    }

    void
    bootstrap_cc_impl::install_taps(const std::vector<gr_complex>& taps)
    {
      int nfilters = this->interpolation();
      int nt = taps.size() / nfilters;

      assert(nt * nfilters == (int)taps.size());

      std::vector<std::vector<gr_complex>> xtaps(nfilters);

      for (int n = 0; n < nfilters; n++)
        xtaps[n].resize(nt);

      for (int i = 0; i < (int)taps.size(); i++)
        xtaps[i % nfilters][i / nfilters] = taps[i];

      for (int n = 0; n < nfilters; n++)
        d_firs[n].set_taps(xtaps[n]);

//      set_history(nt);
      printf("nt = %d\n", nt);
    }

    int
    bootstrap_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      gr_complex* level;

      for (int i = 0; i < noutput_items; i += insertion_items) {
        level = out;
        if (output_mode) {
          memcpy(out, &bootstrap_resample[0], sizeof(gr_complex) * ((BOOTSTRAP_FFT_SIZE + B_SIZE + C_SIZE) * NUM_BOOTSTRAP_SYMBOLS * 9) / 8);
          out += ((BOOTSTRAP_FFT_SIZE + B_SIZE + C_SIZE) * NUM_BOOTSTRAP_SYMBOLS * 9) / 8;
        }
        else {
          memcpy(out, &bootstrap_symbol[0], sizeof(gr_complex) * (BOOTSTRAP_FFT_SIZE + B_SIZE + C_SIZE) * NUM_BOOTSTRAP_SYMBOLS);
          out += (BOOTSTRAP_FFT_SIZE + B_SIZE + C_SIZE) * NUM_BOOTSTRAP_SYMBOLS;
        }
        memcpy(out, in, sizeof(gr_complex) * frame_items);
        if (show_levels == SHOWLEVELS_ON) {
          for (int j = 0; j < insertion_items; j++) {
            if (level[j].real() > real_positive) {
              real_positive = level[j].real();
            }
            if (level[j].real() < real_negative) {
              real_negative = level[j].real();
            }
            if (level[j].imag() > imag_positive) {
              imag_positive = level[j].imag();
            }
            if (level[j].imag() < imag_negative) {
              imag_negative = level[j].imag();
            }
            if (level[j].real() > real_positive_threshold) {
              real_positive_threshold_count++;
            }
            if (level[j].real() < real_negative_threshold) {
              real_negative_threshold_count++;
            }
            if (level[j].imag() > imag_positive_threshold) {
              imag_positive_threshold_count++;
            }
            if (level[j].imag() < imag_negative_threshold) {
              imag_negative_threshold_count++;
            }
          }
          printf("peak real = %+e, %+e, %d, %d\n",
                 real_positive,
                 real_negative,
                 real_positive_threshold_count,
                 real_negative_threshold_count);
          printf("peak imag = %+e, %+e, %d, %d\n",
                 imag_positive,
                 imag_negative,
                 imag_positive_threshold_count,
                 imag_negative_threshold_count);
        }
      }
      out += frame_items;
      in += frame_items;

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (frame_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace atsc3 */
} /* namespace gr */
