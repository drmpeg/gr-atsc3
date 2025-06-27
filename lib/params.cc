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

  } /* namespace atsc3 */
} /* namespace gr */
