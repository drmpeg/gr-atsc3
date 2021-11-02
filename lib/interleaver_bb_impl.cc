/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "interleaver_bb_impl.h"

namespace gr {
  namespace atsc3 {

    using input_type = unsigned char;
    using output_type = unsigned char;
    interleaver_bb::sptr
    interleaver_bb::make(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation)
    {
      return gnuradio::make_block_sptr<interleaver_bb_impl>(
        framesize, rate, constellation);
    }


    /*
     * The private constructor
     */
    interleaver_bb_impl::interleaver_bb_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation)
      : gr::block("interleaver_bb",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      signal_constellation = constellation;
      code_rate = rate;
      if (framesize == FECFRAME_NORMAL) {
        frame_size = FRAME_SIZE_NORMAL;
        group_size = frame_size / 360;
        switch (rate) {
          case C2_15:
            nbch = 8640;
            ldpc_type = LDPC_TYPE_A;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_2_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_2_15N_16QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_64QAM:
                group_table = &group_tab_2_15N_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_2_15N_256QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              default:
                group_table = &group_tab_2_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C3_15:
            nbch = 12960;
            ldpc_type = LDPC_TYPE_A;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_3_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_3_15N_16QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_64QAM:
                group_table = &group_tab_3_15N_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_3_15N_256QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              default:
                group_table = &group_tab_3_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C4_15:
            nbch = 17280;
            ldpc_type = LDPC_TYPE_A;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_4_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_4_15N_16QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_64QAM:
                group_table = &group_tab_4_15N_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_4_15N_256QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              default:
                group_table = &group_tab_4_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C5_15:
            nbch = 21600;
            ldpc_type = LDPC_TYPE_A;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_5_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_5_15N_16QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_64QAM:
                group_table = &group_tab_5_15N_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_5_15N_256QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              default:
                group_table = &group_tab_5_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C6_15:
            nbch = 25920;
            q_val = 108;
            ldpc_type = LDPC_TYPE_B;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_6_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_6_15N_16QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_64QAM:
                group_table = &group_tab_6_15N_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_6_15N_256QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              default:
                group_table = &group_tab_6_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C7_15:
            nbch = 30240;
            ldpc_type = LDPC_TYPE_A;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_7_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_7_15N_16QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_64QAM:
                group_table = &group_tab_7_15N_64QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_256QAM:
                group_table = &group_tab_7_15N_256QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              default:
                group_table = &group_tab_7_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C8_15:
            nbch = 34560;
            q_val = 84;
            ldpc_type = LDPC_TYPE_B;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_8_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_8_15N_16QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_64QAM:
                group_table = &group_tab_8_15N_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_8_15N_256QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              default:
                group_table = &group_tab_8_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C9_15:
            nbch = 38880;
            q_val = 72;
            ldpc_type = LDPC_TYPE_B;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_9_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_9_15N_16QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_64QAM:
                group_table = &group_tab_9_15N_64QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_256QAM:
                group_table = &group_tab_9_15N_256QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              default:
                group_table = &group_tab_9_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C10_15:
            nbch = 43200;
            q_val = 60;
            ldpc_type = LDPC_TYPE_B;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_10_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_10_15N_16QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_64QAM:
                group_table = &group_tab_10_15N_64QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_256QAM:
                group_table = &group_tab_10_15N_256QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              default:
                group_table = &group_tab_10_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C11_15:
            nbch = 47520;
            q_val = 48;
            ldpc_type = LDPC_TYPE_B;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_11_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_11_15N_16QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_64QAM:
                group_table = &group_tab_11_15N_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_11_15N_256QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              default:
                group_table = &group_tab_11_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C12_15:
            nbch = 51840;
            q_val = 36;
            ldpc_type = LDPC_TYPE_B;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_12_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_12_15N_16QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_64QAM:
                group_table = &group_tab_12_15N_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_12_15N_256QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              default:
                group_table = &group_tab_12_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C13_15:
            nbch = 56160;
            q_val = 24;
            ldpc_type = LDPC_TYPE_B;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_13_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_13_15N_16QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_64QAM:
                group_table = &group_tab_13_15N_64QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_256QAM:
                group_table = &group_tab_13_15N_256QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              default:
                group_table = &group_tab_13_15N_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          default:
            nbch = 0;
            break;
        }
      }
      else if (framesize == FECFRAME_SHORT) {
        frame_size = FRAME_SIZE_SHORT;
        group_size = frame_size / 360;
        switch (rate) {
          case C2_15:
            nbch = 2160;
            ldpc_type = LDPC_TYPE_A;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_2_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_2_15S_16QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_64QAM:
                group_table = &group_tab_2_15S_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_2_15S_256QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              default:
                group_table = &group_tab_2_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C3_15:
            nbch = 3240;
            ldpc_type = LDPC_TYPE_A;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_3_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_3_15S_16QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_64QAM:
                group_table = &group_tab_3_15S_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_3_15S_256QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              default:
                group_table = &group_tab_3_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C4_15:
            nbch = 4320;
            ldpc_type = LDPC_TYPE_A;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_4_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_4_15S_16QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_64QAM:
                group_table = &group_tab_4_15S_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_4_15S_256QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              default:
                group_table = &group_tab_4_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C5_15:
            nbch = 5400;
            ldpc_type = LDPC_TYPE_A;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_5_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_5_15S_16QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_64QAM:
                group_table = &group_tab_5_15S_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_5_15S_256QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              default:
                group_table = &group_tab_5_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C6_15:
            nbch = 6480;
            q_val = 27;
            ldpc_type = LDPC_TYPE_B;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_6_15S_QPSK[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_16QAM:
                group_table = &group_tab_6_15S_16QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_64QAM:
                group_table = &group_tab_6_15S_64QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_256QAM:
                group_table = &group_tab_6_15S_256QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              default:
                group_table = &group_tab_6_15S_QPSK[0];
                block_type = BLOCK_TYPE_B;
                break;
            }
            break;
          case C7_15:
            nbch = 7560;
            q_val = 24;
            ldpc_type = LDPC_TYPE_B;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_7_15S_QPSK[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_16QAM:
                group_table = &group_tab_7_15S_16QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_64QAM:
                group_table = &group_tab_7_15S_64QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_256QAM:
                group_table = &group_tab_7_15S_256QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              default:
                group_table = &group_tab_7_15S_QPSK[0];
                block_type = BLOCK_TYPE_B;
                break;
            }
            break;
          case C8_15:
            nbch = 8640;
            q_val = 21;
            ldpc_type = LDPC_TYPE_B;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_8_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_8_15S_16QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_64QAM:
                group_table = &group_tab_8_15S_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_8_15S_256QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              default:
                group_table = &group_tab_8_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C9_15:
            nbch = 9720;
            q_val = 18;
            ldpc_type = LDPC_TYPE_B;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_9_15S_QPSK[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_16QAM:
                group_table = &group_tab_9_15S_16QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_64QAM:
                group_table = &group_tab_9_15S_64QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_256QAM:
                group_table = &group_tab_9_15S_256QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              default:
                group_table = &group_tab_9_15S_QPSK[0];
                block_type = BLOCK_TYPE_B;
                break;
            }
            break;
          case C10_15:
            nbch = 10800;
            q_val = 15;
            ldpc_type = LDPC_TYPE_B;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_10_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_10_15S_16QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_64QAM:
                group_table = &group_tab_10_15S_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_10_15S_256QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              default:
                group_table = &group_tab_10_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C11_15:
            nbch = 11880;
            q_val = 12;
            ldpc_type = LDPC_TYPE_B;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_11_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_11_15S_16QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_64QAM:
                group_table = &group_tab_11_15S_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_11_15S_256QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              default:
                group_table = &group_tab_11_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C12_15:
            nbch = 12960;
            q_val = 9;
            ldpc_type = LDPC_TYPE_B;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_12_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_12_15S_16QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_64QAM:
                group_table = &group_tab_12_15S_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_12_15S_256QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              default:
                group_table = &group_tab_12_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          case C13_15:
            nbch = 14040;
            q_val = 6;
            ldpc_type = LDPC_TYPE_B;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_13_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_16QAM:
                group_table = &group_tab_13_15S_16QAM[0];
                block_type = BLOCK_TYPE_B;
                break;
              case MOD_64QAM:
                group_table = &group_tab_13_15S_64QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              case MOD_256QAM:
                group_table = &group_tab_13_15S_256QAM[0];
                block_type = BLOCK_TYPE_A;
                break;
              default:
                group_table = &group_tab_13_15S_QPSK[0];
                block_type = BLOCK_TYPE_A;
                break;
            }
            break;
          default:
            nbch = 0;
            break;
        }
      }
      switch (constellation) {
        case MOD_QPSK:
          mod = 2;
          if (framesize == FECFRAME_NORMAL) {
            nr2 = 0;
          }
          else if (framesize == FECFRAME_SHORT) {
            nr2 = 180;
          }
          set_output_multiple(frame_size / mod);
          packed_items = frame_size / mod;
          break;
        case MOD_16QAM:
          mod = 4;
          if (framesize == FECFRAME_NORMAL) {
            nr2 = 0;
          }
          else if (framesize == FECFRAME_SHORT) {
            nr2 = 90;
          }
          set_output_multiple(frame_size / mod);
          packed_items = frame_size / mod;
          break;
        case MOD_64QAM:
          mod = 6;
          if (framesize == FECFRAME_NORMAL) {
            nr2 = 0;
          }
          else if (framesize == FECFRAME_SHORT) {
            nr2 = 180;
          }
          set_output_multiple(frame_size / mod);
          packed_items = frame_size / mod;
          break;
        case MOD_256QAM:
          mod = 8;
          if (framesize == FECFRAME_NORMAL) {
            nr2 = 180;
          }
          else if (framesize == FECFRAME_SHORT) {
            nr2 = 225;
          }
          set_output_multiple(frame_size / mod);
          packed_items = frame_size / mod;
          break;
        default:
          mod = 1;
          if (framesize == FECFRAME_NORMAL) {
            nr2 = 0;
          }
          else if (framesize == FECFRAME_SHORT) {
            nr2 = 180;
          }
          set_output_multiple(frame_size / mod);
          packed_items = frame_size / mod;
          break;
      }
    }

    /*
     * Our virtual destructor.
     */
    interleaver_bb_impl::~interleaver_bb_impl()
    {
    }

    void
    interleaver_bb_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items * mod;
    }

    int
    interleaver_bb_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      int consumed = 0;
      int produced = 0;
      int rows, rows2, index, block, indexb;
      unsigned int pack;

      switch (signal_constellation) {
        case MOD_QPSK:
          for (int i = 0; i < noutput_items; i += packed_items) {
            rows = (frame_size / 2) - nr2;
            const unsigned char *c1, *c2;
            c1 = &tempv[0];
            c2 = &tempv[rows];
            if (block_type == BLOCK_TYPE_A) {
              if (ldpc_type == LDPC_TYPE_A) {
                index = 0;
                for (int j = 0; j < group_size; j++) {
                  block = group_table[j];
                  indexb = block * 360;
                  for (int k = 0; k < 360; k++) {
                    tempv[index++] = in[indexb++];;
                  }
                }
                in += frame_size;
              }
              else {
                for (int k = 0; k < nbch; k++) {
                  tempu[k] = *in++;
                }
                for (int t = 0; t < q_val; t++) {
                  for (int s = 0; s < 360; s++) {
                    tempu[nbch + (360 * t) + s] = in[(q_val * s) + t];
                  }
                }
                in = in + (q_val * 360);
                index = 0;
                for (int j = 0; j < group_size; j++) {
                  block = group_table[j];
                  indexb = block * 360;
                  for (int k = 0; k < 360; k++) {
                    tempv[index++] = tempu[indexb++];
                  }
                }
              }
              index = 0;
              for (int j = 0; j < rows; j++) {
                tempu[index++] = c1[j];
                tempu[index++] = c2[j];
              }
              if (nr2) {
                rows2 = frame_size - (nr2 * mod);
                c1 = &tempv[rows2];
                c2 = &tempv[rows2 + nr2];
                for (int j = 0; j < nr2; j++) {
                  tempu[index++] = c1[j];
                  tempu[index++] = c2[j];
                }
              }
              index = 0;
              for (int j = 0; j < rows + nr2; j++) {
                out[produced] = tempu[index++] << 1;
                out[produced++] |= tempu[index++];
                consumed += mod;
              }
            }
            else {
              /* block type B */
            }
          }
          break;
        case MOD_16QAM:
          for (int i = 0; i < noutput_items; i += packed_items) {
            rows = (frame_size / mod) - nr2;
            const unsigned char *c1, *c2, *c3, *c4;
            c1 = &tempv[0];
            c2 = &tempv[rows];
            c3 = &tempv[rows * 2];
            c4 = &tempv[rows * 3];
            if (block_type == BLOCK_TYPE_A) {
              if (ldpc_type == LDPC_TYPE_A) {
                index = 0;
                for (int j = 0; j < group_size; j++) {
                  block = group_table[j];
                  indexb = block * 360;
                  for (int k = 0; k < 360; k++) {
                    tempv[index++] = in[indexb++];;
                  }
                }
                in += frame_size;
              }
              else {
                for (int k = 0; k < nbch; k++) {
                  tempu[k] = *in++;
                }
                for (int t = 0; t < q_val; t++) {
                  for (int s = 0; s < 360; s++) {
                    tempu[nbch + (360 * t) + s] = in[(q_val * s) + t];
                  }
                }
                in = in + (q_val * 360);
                index = 0;
                for (int j = 0; j < group_size; j++) {
                  block = group_table[j];
                  indexb = block * 360;
                  for (int k = 0; k < 360; k++) {
                    tempv[index++] = tempu[indexb++];
                  }
                }
              }
              index = 0;
              for (int j = 0; j < rows; j++) {
                tempu[index++] = c1[j];
                tempu[index++] = c2[j];
                tempu[index++] = c3[j];
                tempu[index++] = c4[j];
              }
              if (nr2) {
                rows2 = frame_size - (nr2 * mod);
                c1 = &tempv[rows2];
                c2 = &tempv[rows2 + nr2];
                c3 = &tempv[rows2 + (nr2 * 2)];
                c4 = &tempv[rows2 + (nr2 * 3)];
                for (int j = 0; j < nr2; j++) {
                  tempu[index++] = c1[j];
                  tempu[index++] = c2[j];
                  tempu[index++] = c3[j];
                  tempu[index++] = c4[j];
                }
              }
              index = 0;
              for (int j = 0; j < rows + nr2; j++) {
                pack = 0;
                for (int e = 3; e >= 0 ; e--) {
                  pack |= tempu[index++] << e;
                }
                out[produced++] = pack & 0xf;
                consumed += mod;
              }
            }
            else {
              /* block type B */
            }
          }
          break;
        case MOD_64QAM:
          for (int i = 0; i < noutput_items; i += packed_items) {
            rows = (frame_size / mod) - nr2;
            const unsigned char *c1, *c2, *c3, *c4, *c5, *c6;
            c1 = &tempv[0];
            c2 = &tempv[rows];
            c3 = &tempv[rows * 2];
            c4 = &tempv[rows * 3];
            c5 = &tempv[rows * 4];
            c6 = &tempv[rows * 5];
            if (block_type == BLOCK_TYPE_A) {
              if (ldpc_type == LDPC_TYPE_A) {
                index = 0;
                for (int j = 0; j < group_size; j++) {
                  block = group_table[j];
                  indexb = block * 360;
                  for (int k = 0; k < 360; k++) {
                    tempv[index++] = in[indexb++];;
                  }
                }
                in += frame_size;
              }
              else {
                for (int k = 0; k < nbch; k++) {
                  tempu[k] = *in++;
                }
                for (int t = 0; t < q_val; t++) {
                  for (int s = 0; s < 360; s++) {
                    tempu[nbch + (360 * t) + s] = in[(q_val * s) + t];
                  }
                }
                in = in + (q_val * 360);
                index = 0;
                for (int j = 0; j < group_size; j++) {
                  block = group_table[j];
                  indexb = block * 360;
                  for (int k = 0; k < 360; k++) {
                    tempv[index++] = tempu[indexb++];
                  }
                }
              }
              index = 0;
              for (int j = 0; j < rows; j++) {
                tempu[index++] = c1[j];
                tempu[index++] = c2[j];
                tempu[index++] = c3[j];
                tempu[index++] = c4[j];
                tempu[index++] = c5[j];
                tempu[index++] = c6[j];
              }
              if (nr2) {
                rows2 = frame_size - (nr2 * mod);
                c1 = &tempv[rows2];
                c2 = &tempv[rows2 + nr2];
                c3 = &tempv[rows2 + (nr2 * 2)];
                c4 = &tempv[rows2 + (nr2 * 3)];
                c5 = &tempv[rows2 + (nr2 * 4)];
                c6 = &tempv[rows2 + (nr2 * 5)];
                for (int j = 0; j < nr2; j++) {
                  tempu[index++] = c1[j];
                  tempu[index++] = c2[j];
                  tempu[index++] = c3[j];
                  tempu[index++] = c4[j];
                  tempu[index++] = c5[j];
                  tempu[index++] = c6[j];
                }
              }
              index = 0;
              for (int j = 0; j < rows + nr2; j++) {
                pack = 0;
                for (int e = 5; e >= 0 ; e--) {
                  pack |= tempu[index++] << e;
                }
                out[produced++] = pack & 0x3f;
                consumed += mod;
              }
            }
            else {
              /* block type B */
            }
          }
          break;
        case MOD_256QAM:
          for (int i = 0; i < noutput_items; i += packed_items) {
            rows = (frame_size / mod) - nr2;
            const unsigned char *c1, *c2, *c3, *c4, *c5, *c6, *c7, *c8;
            c1 = &tempv[0];
            c2 = &tempv[rows];
            c3 = &tempv[rows * 2];
            c4 = &tempv[rows * 3];
            c5 = &tempv[rows * 4];
            c6 = &tempv[rows * 5];
            c7 = &tempv[rows * 6];
            c8 = &tempv[rows * 7];
            if (block_type == BLOCK_TYPE_A) {
              if (ldpc_type == LDPC_TYPE_A) {
                index = 0;
                for (int j = 0; j < group_size; j++) {
                  block = group_table[j];
                  indexb = block * 360;
                  for (int k = 0; k < 360; k++) {
                    tempv[index++] = in[indexb++];;
                  }
                }
                in += frame_size;
              }
              else {
                for (int k = 0; k < nbch; k++) {
                  tempu[k] = *in++;
                }
                for (int t = 0; t < q_val; t++) {
                  for (int s = 0; s < 360; s++) {
                    tempu[nbch + (360 * t) + s] = in[(q_val * s) + t];
                  }
                }
                in = in + (q_val * 360);
                index = 0;
                for (int j = 0; j < group_size; j++) {
                  block = group_table[j];
                  indexb = block * 360;
                  for (int k = 0; k < 360; k++) {
                    tempv[index++] = tempu[indexb++];
                  }
                }
              }
              index = 0;
              for (int j = 0; j < rows; j++) {
                tempu[index++] = c1[j];
                tempu[index++] = c2[j];
                tempu[index++] = c3[j];
                tempu[index++] = c4[j];
                tempu[index++] = c5[j];
                tempu[index++] = c6[j];
                tempu[index++] = c7[j];
                tempu[index++] = c8[j];
              }
              if (nr2) {
                rows2 = frame_size - (nr2 * mod);
                c1 = &tempv[rows2];
                c2 = &tempv[rows2 + nr2];
                c3 = &tempv[rows2 + (nr2 * 2)];
                c4 = &tempv[rows2 + (nr2 * 3)];
                c5 = &tempv[rows2 + (nr2 * 4)];
                c6 = &tempv[rows2 + (nr2 * 5)];
                c7 = &tempv[rows2 + (nr2 * 6)];
                c8 = &tempv[rows2 + (nr2 * 7)];
                for (int j = 0; j < nr2; j++) {
                  tempu[index++] = c1[j];
                  tempu[index++] = c2[j];
                  tempu[index++] = c3[j];
                  tempu[index++] = c4[j];
                  tempu[index++] = c5[j];
                  tempu[index++] = c6[j];
                  tempu[index++] = c7[j];
                  tempu[index++] = c8[j];
                }
              }
              index = 0;
              for (int j = 0; j < rows + nr2; j++) {
                pack = 0;
                for (int e = 7; e >= 0 ; e--) {
                  pack |= tempu[index++] << e;
                }
                out[produced++] = pack & 0xff;
                consumed += mod;
              }
            }
            else {
              /* block type B */
            }
          }
          break;
      }

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (consumed);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    const int interleaver_bb_impl::group_tab_2_15N_QPSK[180] = {
      70, 149, 136, 153, 104, 110, 134, 61, 129, 126, 58, 150, 177, 168, 78, 71, 120, 60,
      155, 175, 9, 161, 103, 123, 91, 173, 57, 106, 143, 151, 89, 86, 35, 77, 133, 31,
      7, 23, 51, 5, 121, 83, 64, 176, 119, 98, 49, 130, 128, 79, 162, 32, 172, 87,
      131, 45, 114, 93, 96, 39, 68, 105, 85, 109, 13, 33, 145, 18, 12, 54, 111, 14,
      156, 8, 16, 73, 2, 84, 47, 42, 101, 63, 88, 25, 52, 170, 24, 69, 142, 178,
      20, 65, 97, 66, 80, 11, 59, 19, 115, 154, 26, 147, 28, 50, 160, 102, 55, 139,
      125, 116, 138, 167, 53, 169, 165, 99, 159, 148, 179, 0, 146, 90, 6, 100, 74, 117,
      48, 75, 135, 41, 137, 76, 92, 164, 113, 152, 72, 36, 3, 163, 15, 46, 21, 44,
      108, 34, 56, 140, 127, 158, 94, 67, 122, 1, 27, 171, 30, 157, 112, 81, 118, 43,
      29, 124, 22, 62, 37, 40, 4, 107, 166, 82, 95, 10, 144, 141, 132, 174, 38, 17
    };

    const int interleaver_bb_impl::group_tab_3_15N_QPSK[180] = {
      75, 170, 132, 174, 7, 111, 30, 4, 49, 133, 50, 160, 92, 106, 27, 126, 116, 178,
      41, 166, 88, 84, 80, 153, 103, 51, 58, 107, 167, 39, 108, 24, 145, 96, 74, 65,
      8, 40, 76, 140, 44, 68, 125, 119, 82, 53, 152, 102, 38, 28, 86, 162, 171, 61,
      93, 147, 117, 32, 150, 26, 59, 3, 148, 173, 141, 130, 154, 97, 33, 172, 115, 118,
      127, 6, 16, 0, 143, 9, 100, 67, 98, 110, 2, 169, 47, 83, 164, 155, 123, 159,
      42, 105, 12, 158, 81, 20, 66, 57, 121, 25, 1, 90, 175, 35, 60, 79, 87, 135,
      10, 139, 156, 177, 77, 89, 73, 113, 52, 109, 134, 36, 176, 54, 69, 146, 31, 15,
      71, 18, 95, 124, 85, 14, 78, 129, 161, 19, 72, 13, 122, 21, 63, 137, 120, 144,
      91, 157, 48, 34, 46, 22, 29, 104, 45, 56, 151, 62, 43, 94, 163, 99, 64, 138,
      101, 23, 11, 17, 136, 128, 114, 112, 165, 5, 142, 179, 37, 70, 131, 55, 168, 149
    };

    const int interleaver_bb_impl::group_tab_4_15N_QPSK[180] = {
      141, 86, 22, 20, 176, 21, 37, 82, 6, 122, 130, 40, 62, 44, 24, 117, 8, 145,
      36, 79, 172, 149, 127, 163, 9, 160, 73, 100, 16, 153, 124, 110, 49, 154, 152, 4,
      168, 54, 177, 158, 113, 57, 2, 102, 161, 147, 18, 103, 1, 41, 104, 144, 39, 105,
      131, 77, 69, 108, 159, 61, 45, 156, 0, 83, 157, 119, 112, 118, 92, 109, 75, 67,
      142, 96, 51, 139, 31, 166, 179, 89, 167, 23, 34, 60, 93, 165, 128, 90, 19, 33,
      70, 173, 174, 129, 55, 98, 88, 97, 146, 123, 84, 111, 132, 71, 140, 136, 10, 115,
      63, 46, 42, 50, 138, 81, 59, 53, 15, 52, 72, 164, 150, 29, 17, 91, 101, 14,
      38, 35, 66, 64, 7, 125, 151, 56, 126, 171, 68, 121, 28, 65, 106, 78, 47, 143,
      12, 169, 120, 27, 74, 48, 133, 43, 116, 137, 94, 3, 25, 134, 13, 107, 162, 32,
      99, 85, 175, 80, 170, 5, 135, 178, 11, 26, 76, 95, 87, 155, 58, 30, 148, 114
    };

    const int interleaver_bb_impl::group_tab_5_15N_QPSK[180] = {
      39, 47, 96, 176, 33, 75, 165, 38, 27, 58, 90, 76, 17, 46, 10, 91, 133, 69,
      171, 32, 117, 78, 13, 146, 101, 36, 0, 138, 25, 77, 122, 49, 14, 125, 140, 93,
      130, 2, 104, 102, 128, 4, 111, 151, 84, 167, 35, 127, 156, 55, 82, 85, 66, 114,
      8, 147, 115, 113, 5, 31, 100, 106, 48, 52, 67, 107, 18, 126, 112, 50, 9, 143,
      28, 160, 71, 79, 43, 98, 86, 94, 64, 3, 166, 105, 103, 118, 63, 51, 139, 172,
      141, 175, 56, 74, 95, 29, 45, 129, 120, 168, 92, 150, 7, 162, 153, 137, 108, 159,
      157, 173, 23, 89, 132, 57, 37, 70, 134, 40, 21, 149, 80, 1, 121, 59, 110, 142,
      152, 15, 154, 145, 12, 170, 54, 155, 99, 22, 123, 72, 177, 131, 116, 44, 158, 73,
      11, 65, 164, 119, 174, 34, 83, 53, 24, 42, 60, 26, 161, 68, 178, 41, 148, 109,
      87, 144, 135, 20, 62, 81, 169, 124, 6, 19, 30, 163, 61, 179, 136, 97, 16, 88
    };

    const int interleaver_bb_impl::group_tab_6_15N_QPSK[180] = {
      0, 14, 19, 21, 2, 11, 22, 9, 8, 7, 16, 3, 26, 24, 27, 80, 100, 121,
      107, 31, 36, 42, 46, 49, 75, 93, 127, 95, 119, 73, 61, 63, 117, 89, 99, 129,
      52, 111, 124, 48, 122, 82, 106, 91, 92, 71, 103, 102, 81, 113, 101, 97, 33, 115,
      59, 112, 90, 51, 126, 85, 123, 40, 83, 53, 69, 70, 132, 134, 136, 138, 140, 142,
      144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178,
      4, 5, 10, 12, 20, 6, 18, 13, 17, 15, 1, 29, 28, 23, 25, 67, 116, 66,
      104, 44, 50, 47, 84, 76, 65, 130, 56, 128, 77, 39, 94, 87, 120, 62, 88, 74,
      35, 110, 131, 98, 60, 37, 45, 78, 125, 41, 34, 118, 38, 72, 108, 58, 43, 109,
      57, 105, 68, 86, 79, 96, 32, 114, 64, 55, 30, 54, 133, 135, 137, 139, 141, 143,
      145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179
    };

    const int interleaver_bb_impl::group_tab_7_15N_QPSK[180] = {
      152, 172, 113, 167, 100, 163, 159, 144, 114, 47, 161, 125, 99, 89, 179, 123, 149, 177,
      1, 132, 37, 26, 16, 57, 166, 81, 133, 112, 33, 151, 117, 83, 52, 178, 85, 124,
      143, 28, 59, 130, 31, 157, 170, 44, 61, 102, 155, 111, 153, 55, 54, 176, 17, 68,
      169, 20, 104, 38, 147, 7, 174, 6, 90, 15, 56, 120, 13, 34, 48, 122, 110, 154,
      76, 64, 75, 84, 162, 77, 103, 156, 128, 150, 87, 27, 42, 3, 23, 96, 171, 145,
      91, 24, 78, 5, 69, 175, 8, 29, 106, 137, 131, 43, 93, 160, 108, 164, 12, 140,
      71, 63, 141, 109, 129, 82, 80, 173, 105, 9, 66, 65, 92, 32, 41, 72, 74, 4,
      36, 94, 67, 158, 10, 88, 142, 45, 126, 2, 86, 118, 73, 79, 121, 148, 95, 70,
      51, 53, 21, 115, 135, 25, 168, 11, 136, 18, 138, 134, 119, 146, 0, 97, 22, 165,
      40, 19, 60, 46, 14, 49, 139, 58, 101, 39, 116, 127, 30, 98, 50, 107, 35, 62
    };

    const int interleaver_bb_impl::group_tab_8_15N_QPSK[180] = {
      0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34,
      36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70,
      72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106,
      108, 110, 112, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 136, 138, 140, 142,
      144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178,
      1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35,
      37, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71,
      73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95, 97, 99, 101, 103, 105, 107,
      109, 111, 113, 115, 117, 119, 121, 123, 125, 127, 129, 131, 133, 135, 137, 139, 141, 143,
      145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179
    };

    const int interleaver_bb_impl::group_tab_9_15N_QPSK[180] = {
      0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34,
      36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70,
      72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106,
      108, 110, 112, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 136, 138, 140, 142,
      144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178,
      1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35,
      37, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71,
      73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95, 97, 99, 101, 103, 105, 107,
      109, 111, 113, 115, 117, 119, 121, 123, 125, 127, 129, 131, 133, 135, 137, 139, 141, 143,
      145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179
    };

    const int interleaver_bb_impl::group_tab_10_15N_QPSK[180] = {
      0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34,
      36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70,
      72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106,
      108, 110, 112, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 136, 138, 140, 142,
      144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178,
      1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35,
      37, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71,
      73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95, 97, 99, 101, 103, 105, 107,
      109, 111, 113, 115, 117, 119, 121, 123, 125, 127, 129, 131, 133, 135, 137, 139, 141, 143,
      145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179
    };

    const int interleaver_bb_impl::group_tab_11_15N_QPSK[180] = {
      0, 14, 19, 21, 2, 11, 22, 9, 8, 7, 16, 3, 26, 24, 27, 80, 100, 121,
      107, 31, 36, 42, 46, 49, 75, 93, 127, 95, 119, 73, 61, 63, 117, 89, 99, 129,
      52, 111, 124, 48, 122, 82, 106, 91, 92, 71, 103, 102, 81, 113, 101, 97, 33, 115,
      59, 112, 90, 51, 126, 85, 123, 40, 83, 53, 69, 70, 132, 134, 136, 138, 140, 142,
      144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178,
      4, 5, 10, 12, 20, 6, 18, 13, 17, 15, 1, 29, 28, 23, 25, 67, 116, 66,
      104, 44, 50, 47, 84, 76, 65, 130, 56, 128, 77, 39, 94, 87, 120, 62, 88, 74,
      35, 110, 131, 98, 60, 37, 45, 78, 125, 41, 34, 118, 38, 72, 108, 58, 43, 109,
      57, 105, 68, 86, 79, 96, 32, 114, 64, 55, 30, 54, 133, 135, 137, 139, 141, 143,
      145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179
    };

    const int interleaver_bb_impl::group_tab_12_15N_QPSK[180] = {
      0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34,
      36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70,
      72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106,
      108, 110, 112, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 136, 138, 140, 142,
      144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178,
      1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35,
      37, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71,
      73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95, 97, 99, 101, 103, 105, 107,
      109, 111, 113, 115, 117, 119, 121, 123, 125, 127, 129, 131, 133, 135, 137, 139, 141, 143,
      145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179
    };

    const int interleaver_bb_impl::group_tab_13_15N_QPSK[180] = {
      0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34,
      36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70,
      72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106,
      108, 110, 112, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 136, 138, 140, 142,
      144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178,
      1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35,
      37, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71,
      73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95, 97, 99, 101, 103, 105, 107,
      109, 111, 113, 115, 117, 119, 121, 123, 125, 127, 129, 131, 133, 135, 137, 139, 141, 143,
      145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179
    };

    const int interleaver_bb_impl::group_tab_2_15S_QPSK[45] = {
      0, 2, 4, 6, 8, 10, 12, 14, 16,
      18, 20, 22, 24, 26, 28, 30, 32, 34,
      36, 38, 40, 42, 1, 3, 5, 7, 9,
      11, 13, 15, 17, 19, 21, 23, 25, 27,
      29, 31, 33, 35, 37, 39, 41, 43, 44
    };

    const int interleaver_bb_impl::group_tab_3_15S_QPSK[45] = {
      15, 22, 34, 19, 7, 17, 28, 43, 30,
      32, 14, 1, 11, 0, 3, 9, 10, 38,
      24, 4, 23, 18, 27, 39, 29, 33, 8,
      2, 40, 21, 20, 36, 44, 12, 37, 13,
      35, 6, 31, 26, 16, 25, 42, 5, 41
    };

    const int interleaver_bb_impl::group_tab_4_15S_QPSK[45] = {
      0, 2, 4, 6, 8, 10, 12, 14, 16,
      18, 20, 22, 24, 26, 28, 30, 32, 34,
      36, 38, 40, 42, 1, 3, 5, 7, 9,
      11, 13, 15, 17, 19, 21, 23, 25, 27,
      29, 31, 33, 35, 37, 39, 41, 43, 44
    };

    const int interleaver_bb_impl::group_tab_5_15S_QPSK[45] = {
      35, 7, 29, 11, 14, 32, 38, 28, 20,
      17, 25, 39, 19, 4, 1, 12, 10, 30,
      0, 44, 43, 2, 21, 5, 13, 34, 37,
      23, 15, 36, 18, 42, 16, 33, 31, 27,
      22, 3, 6, 40, 24, 41, 9, 26, 8
    };

    const int interleaver_bb_impl::group_tab_6_15S_QPSK[45] = {
      7, 4, 0, 5, 27, 30, 25, 13, 31,
      9, 34, 10, 17, 11, 8, 12, 15, 16,
      18, 19, 20, 21, 22, 23, 1, 35, 24,
      29, 33, 6, 26, 14, 32, 28, 2, 3,
      36, 37, 38, 39, 40, 41, 42, 43, 44
    };

    const int interleaver_bb_impl::group_tab_7_15S_QPSK[45] = {
      3, 7, 1, 4, 18, 21, 22, 6, 9,
      5, 17, 14, 13, 15, 10, 20, 8, 19,
      16, 12, 0, 11, 2, 23, 24, 25, 26,
      27, 28, 29, 30, 31, 32, 33, 34, 35,
      36, 37, 38, 39, 40, 41, 42, 43, 44
    };

    const int interleaver_bb_impl::group_tab_8_15S_QPSK[45] = {
      0, 2, 4, 6, 8, 10, 12, 14, 16,
      18, 20, 22, 24, 26, 28, 30, 32, 34,
      36, 38, 40, 42, 1, 3, 5, 7, 9,
      11, 13, 15, 17, 19, 21, 23, 25, 27,
      29, 31, 33, 35, 37, 39, 41, 43, 44
    };

    const int interleaver_bb_impl::group_tab_9_15S_QPSK[45] = {
      0, 1, 2, 3, 4, 5, 6, 7, 8,
      9, 10, 11, 12, 13, 14, 15, 16, 17,
      18, 19, 20, 21, 22, 23, 24, 25, 26,
      27, 28, 29, 30, 31, 32, 33, 34, 35,
      36, 37, 38, 39, 40, 41, 42, 43, 44
    };

    const int interleaver_bb_impl::group_tab_10_15S_QPSK[45] = {
      1, 4, 5, 6, 24, 21, 18, 7, 17,
      12, 8, 20, 23, 29, 28, 30, 32, 34,
      36, 38, 40, 42, 0, 2, 3, 14, 22,
      13, 10, 25, 9, 27, 19, 16, 15, 26,
      11, 31, 33, 35, 37, 39, 41, 43, 44
    };

    const int interleaver_bb_impl::group_tab_11_15S_QPSK[45] = {
      0, 2, 4, 6, 8, 10, 12, 14, 16,
      18, 20, 22, 24, 26, 28, 30, 32, 34,
      36, 38, 40, 42, 1, 3, 5, 7, 9,
      11, 13, 15, 17, 19, 21, 23, 25, 27,
      29, 31, 33, 35, 37, 39, 41, 43, 44
    };

    const int interleaver_bb_impl::group_tab_12_15S_QPSK[45] = {
      0, 2, 4, 6, 8, 10, 12, 14, 16,
      18, 20, 22, 24, 26, 28, 30, 32, 34,
      36, 38, 40, 42, 1, 3, 5, 7, 9,
      11, 13, 15, 17, 19, 21, 23, 25, 27,
      29, 31, 33, 35, 37, 39, 41, 43, 44
    };

    const int interleaver_bb_impl::group_tab_13_15S_QPSK[45] = {
      26, 10, 12, 38, 28, 15, 0, 44, 34,
      24, 14, 8, 40, 30, 20, 13, 42, 32,
      22, 11, 9, 36, 25, 7, 5, 37, 27,
      4, 16, 43, 33, 23, 2, 18, 39, 29,
      19, 6, 41, 31, 21, 3, 17, 35, 1
    };

    const int interleaver_bb_impl::group_tab_2_15N_16QAM[180] = {
      5, 58, 29, 154, 125, 34, 0, 169, 80, 59, 13, 42, 77, 167, 32, 87, 24, 92,
      124, 143, 114, 120, 166, 138, 64, 136, 149, 57, 18, 101, 119, 35, 33, 113, 75, 108,
      104, 3, 27, 39, 172, 159, 129, 62, 146, 142, 19, 147, 111, 70, 74, 79, 10, 132,
      1, 161, 155, 90, 15, 133, 47, 112, 84, 28, 160, 117, 150, 49, 7, 81, 44, 63,
      118, 4, 158, 148, 82, 69, 36, 162, 86, 71, 22, 26, 61, 40, 126, 170, 177, 23,
      91, 68, 56, 110, 21, 93, 107, 85, 20, 128, 109, 66, 83, 12, 179, 141, 97, 78,
      157, 72, 130, 99, 165, 45, 11, 152, 168, 14, 16, 2, 137, 140, 121, 173, 50, 55,
      94, 144, 73, 51, 98, 174, 178, 17, 100, 9, 122, 54, 38, 156, 131, 127, 164, 102,
      116, 176, 30, 37, 139, 95, 43, 135, 53, 89, 106, 171, 76, 175, 153, 96, 151, 115,
      52, 6, 123, 134, 31, 103, 163, 65, 105, 48, 25, 8, 60, 67, 88, 46, 41, 145
    };

    const int interleaver_bb_impl::group_tab_3_15N_16QAM[180] = {
      52, 92, 175, 26, 45, 81, 117, 74, 119, 147, 120, 135, 144, 87, 3, 51, 20, 170,
      143, 125, 15, 39, 5, 174, 79, 16, 176, 44, 19, 69, 11, 111, 121, 37, 160, 88,
      50, 76, 129, 138, 157, 86, 113, 164, 142, 98, 9, 93, 166, 78, 73, 167, 168, 40,
      131, 27, 89, 156, 177, 171, 116, 152, 0, 127, 36, 8, 153, 59, 75, 13, 105, 55,
      122, 132, 172, 2, 58, 126, 162, 30, 77, 158, 17, 96, 100, 42, 63, 134, 154, 6,
      90, 128, 83, 60, 146, 124, 178, 99, 123, 108, 133, 159, 151, 145, 61, 53, 68, 31,
      41, 94, 35, 21, 49, 82, 80, 4, 155, 7, 57, 95, 62, 56, 65, 140, 163, 148,
      23, 161, 169, 47, 67, 139, 72, 43, 110, 46, 150, 109, 115, 32, 14, 179, 85, 165,
      112, 25, 64, 173, 10, 102, 114, 71, 66, 84, 24, 141, 29, 104, 107, 54, 12, 91,
      1, 118, 136, 18, 101, 149, 130, 103, 106, 38, 70, 48, 28, 137, 97, 34, 22, 33
    };

    const int interleaver_bb_impl::group_tab_4_15N_16QAM[180] = {
      165, 8, 136, 2, 58, 30, 127, 64, 38, 164, 123, 45, 78, 17, 47, 105, 159, 134,
      124, 147, 148, 109, 67, 98, 157, 57, 156, 170, 46, 12, 172, 29, 9, 3, 144, 97,
      83, 151, 26, 52, 10, 39, 50, 104, 92, 163, 72, 125, 36, 14, 55, 48, 1, 149,
      33, 110, 6, 130, 140, 89, 77, 22, 171, 139, 112, 113, 152, 16, 7, 85, 11, 28,
      153, 73, 62, 44, 135, 116, 4, 61, 117, 53, 111, 178, 94, 81, 68, 114, 173, 75,
      101, 88, 65, 99, 126, 141, 43, 15, 18, 90, 35, 24, 142, 25, 120, 19, 154, 0,
      174, 93, 167, 150, 107, 86, 129, 175, 87, 21, 66, 106, 82, 179, 118, 41, 95, 145,
      37, 23, 168, 166, 49, 103, 108, 56, 91, 69, 128, 121, 96, 133, 100, 161, 143, 119,
      102, 59, 20, 40, 70, 79, 80, 51, 13, 177, 131, 132, 176, 155, 31, 63, 5, 162,
      76, 42, 160, 115, 71, 158, 54, 137, 146, 32, 169, 122, 138, 84, 74, 60, 34, 27
    };

    const int interleaver_bb_impl::group_tab_5_15N_16QAM[180] = {
      129, 65, 160, 140, 32, 50, 162, 86, 177, 57, 157, 9, 134, 104, 24, 7, 122, 46,
      17, 77, 31, 92, 163, 148, 133, 99, 18, 0, 167, 101, 110, 135, 124, 71, 107, 5,
      123, 69, 108, 141, 179, 96, 113, 83, 176, 52, 117, 81, 125, 59, 15, 137, 170, 63,
      112, 88, 34, 61, 106, 3, 42, 100, 152, 87, 171, 72, 161, 4, 178, 64, 150, 10,
      128, 49, 26, 75, 41, 102, 28, 2, 168, 93, 156, 12, 38, 45, 151, 142, 44, 66,
      25, 139, 173, 51, 29, 147, 175, 90, 164, 80, 131, 58, 114, 145, 121, 70, 115, 146,
      120, 55, 158, 8, 39, 97, 159, 138, 33, 47, 116, 79, 174, 74, 21, 6, 130, 54,
      109, 76, 35, 98, 155, 144, 36, 94, 23, 78, 165, 56, 154, 89, 132, 67, 119, 143,
      40, 53, 20, 136, 172, 91, 27, 13, 127, 73, 105, 85, 30, 103, 19, 84, 37, 48,
      153, 11, 166, 60, 111, 14, 169, 95, 118, 1, 126, 68, 22, 149, 43, 62, 16, 82
    };

    const int interleaver_bb_impl::group_tab_6_15N_16QAM[180] = {
      55, 146, 83, 52, 62, 176, 160, 68, 53, 56, 81, 97, 79, 113, 163, 61, 58, 69,
      133, 108, 66, 71, 86, 144, 57, 67, 116, 59, 70, 156, 172, 65, 149, 155, 82, 138,
      136, 141, 111, 96, 170, 90, 140, 64, 159, 15, 14, 37, 54, 44, 63, 43, 18, 47,
      7, 25, 34, 29, 30, 26, 39, 16, 41, 45, 36, 0, 23, 32, 28, 27, 38, 48,
      33, 22, 49, 51, 60, 46, 21, 4, 3, 20, 13, 50, 35, 24, 40, 17, 42, 6,
      112, 93, 127, 101, 94, 115, 105, 31, 19, 177, 74, 10, 145, 162, 102, 120, 126, 95,
      73, 152, 129, 174, 125, 72, 128, 78, 171, 8, 142, 178, 154, 85, 107, 75, 12, 9,
      151, 77, 117, 109, 80, 106, 134, 98, 1, 122, 173, 161, 150, 110, 175, 166, 131, 119,
      103, 139, 148, 157, 114, 147, 87, 158, 121, 164, 104, 89, 179, 123, 118, 99, 88, 11,
      92, 165, 84, 168, 124, 169, 2, 130, 167, 153, 137, 143, 91, 100, 5, 76, 132, 135
    };

    const int interleaver_bb_impl::group_tab_7_15N_16QAM[180] = {
      174, 148, 56, 168, 38, 7, 110, 9, 42, 153, 160, 15, 46, 21, 121, 88, 114, 85,
      13, 83, 74, 81, 70, 27, 119, 118, 144, 31, 80, 109, 73, 141, 93, 45, 16, 77,
      108, 57, 36, 78, 124, 79, 169, 143, 6, 58, 75, 67, 5, 104, 125, 140, 172, 8,
      39, 17, 29, 159, 86, 87, 41, 99, 89, 47, 128, 43, 161, 154, 101, 163, 116, 94,
      120, 71, 158, 145, 37, 112, 68, 95, 1, 113, 64, 72, 90, 92, 35, 167, 44, 149,
      66, 28, 82, 178, 176, 152, 23, 115, 130, 98, 123, 102, 24, 129, 150, 34, 136, 171,
      54, 107, 2, 3, 60, 69, 10, 117, 91, 157, 33, 105, 155, 62, 162, 40, 127, 14,
      165, 26, 52, 19, 48, 137, 4, 22, 122, 173, 18, 11, 111, 106, 76, 53, 61, 147,
      97, 175, 32, 59, 166, 179, 135, 177, 103, 100, 139, 50, 146, 134, 133, 96, 49, 126,
      151, 84, 156, 30, 138, 164, 132, 12, 0, 20, 63, 170, 142, 65, 55, 25, 51, 131
    };

    const int interleaver_bb_impl::group_tab_8_15N_16QAM[180] = {
      71, 81, 170, 101, 143, 77, 128, 112, 155, 41, 40, 54, 57, 28, 179, 114, 97, 13,
      18, 151, 91, 88, 79, 92, 137, 27, 122, 107, 135, 82, 125, 103, 74, 36, 9, 93,
      0, 86, 63, 158, 148, 25, 167, 116, 70, 43, 102, 106, 149, 24, 169, 113, 127, 34,
      165, 100, 136, 75, 134, 156, 96, 84, 178, 150, 140, 20, 126, 73, 68, 130, 121, 48,
      53, 22, 129, 99, 11, 33, 124, 157, 161, 29, 123, 160, 55, 26, 168, 98, 67, 15,
      7, 94, 144, 1, 61, 65, 146, 42, 172, 115, 59, 76, 4, 162, 39, 85, 12, 72,
      58, 44, 132, 47, 141, 35, 176, 104, 139, 80, 6, 95, 87, 90, 173, 163, 69, 32,
      8, 154, 145, 23, 177, 111, 60, 38, 171, 62, 46, 21, 5, 153, 49, 78, 2, 109,
      147, 89, 166, 152, 138, 31, 14, 131, 50, 37, 16, 117, 66, 19, 10, 159, 142, 105,
      3, 164, 51, 83, 174, 108, 52, 17, 64, 119, 45, 133, 175, 110, 56, 30, 120, 118
    };

    const int interleaver_bb_impl::group_tab_9_15N_16QAM[180] = {
      23, 89, 10, 142, 19, 41, 1, 146, 68, 87, 9, 51, 114, 92, 121, 69, 107, 97,
      166, 162, 55, 174, 126, 149, 110, 128, 172, 28, 111, 78, 82, 120, 71, 52, 5, 141,
      29, 30, 132, 148, 72, 85, 17, 160, 156, 154, 131, 164, 65, 76, 125, 50, 16, 130,
      129, 143, 133, 98, 0, 42, 63, 83, 173, 49, 74, 43, 8, 147, 61, 36, 167, 119,
      27, 86, 102, 48, 115, 99, 38, 163, 73, 101, 4, 153, 118, 90, 124, 151, 66, 93,
      123, 157, 24, 44, 168, 80, 15, 39, 178, 45, 21, 37, 11, 136, 113, 77, 122, 158,
      64, 81, 6, 60, 54, 35, 13, 57, 171, 100, 117, 46, 62, 33, 175, 137, 59, 103,
      127, 70, 108, 88, 179, 40, 112, 104, 170, 140, 67, 32, 105, 159, 26, 96, 169, 135,
      109, 47, 177, 56, 116, 79, 106, 150, 25, 94, 134, 152, 22, 84, 176, 139, 20, 34,
      165, 138, 7, 91, 12, 145, 58, 95, 2, 144, 53, 75, 14, 155, 18, 31, 3, 161
    };

    const int interleaver_bb_impl::group_tab_10_15N_16QAM[180] = {
      68, 71, 54, 19, 25, 21, 102, 32, 105, 29, 16, 79, 53, 82, 107, 91, 67, 94,
      85, 48, 83, 58, 42, 57, 28, 76, 31, 26, 96, 65, 119, 114, 109, 9, 125, 81,
      43, 103, 93, 70, 46, 89, 112, 61, 45, 66, 38, 77, 115, 56, 87, 113, 100, 75,
      72, 60, 47, 92, 36, 98, 4, 59, 6, 44, 20, 86, 3, 73, 95, 104, 8, 34,
      0, 84, 111, 35, 30, 64, 55, 80, 40, 97, 101, 2, 69, 63, 74, 62, 118, 110,
      159, 18, 50, 33, 7, 175, 51, 131, 106, 134, 88, 140, 117, 132, 147, 153, 116, 161,
      10, 39, 126, 136, 90, 37, 174, 41, 158, 5, 120, 12, 52, 99, 146, 144, 78, 155,
      128, 165, 141, 179, 150, 157, 171, 143, 108, 170, 22, 49, 11, 27, 160, 178, 133, 142,
      121, 168, 173, 123, 13, 15, 154, 127, 139, 151, 163, 172, 138, 176, 145, 129, 162, 152,
      177, 137, 149, 167, 1, 14, 169, 124, 148, 164, 130, 17, 156, 122, 23, 166, 135, 24
    };

    const int interleaver_bb_impl::group_tab_11_15N_16QAM[180] = {
      21, 11, 12, 9, 0, 6, 24, 25, 85, 103, 118, 122, 71, 101, 41, 93, 55, 73,
      100, 40, 106, 119, 45, 80, 128, 68, 129, 61, 124, 36, 126, 117, 114, 132, 136, 140,
      144, 148, 152, 156, 160, 164, 168, 172, 176, 20, 18, 10, 13, 16, 8, 26, 27, 54,
      111, 52, 44, 87, 113, 115, 58, 116, 49, 77, 95, 86, 30, 78, 81, 56, 125, 53,
      89, 94, 50, 123, 65, 83, 133, 137, 141, 145, 149, 153, 157, 161, 165, 169, 173, 177,
      2, 17, 1, 4, 7, 15, 29, 82, 32, 102, 76, 121, 92, 130, 127, 62, 107, 38,
      46, 43, 110, 75, 104, 70, 91, 69, 96, 120, 42, 34, 79, 35, 105, 134, 138, 142,
      146, 150, 154, 158, 162, 166, 170, 174, 178, 19, 5, 3, 14, 22, 28, 23, 109, 51,
      108, 131, 33, 84, 88, 64, 63, 59, 57, 97, 98, 48, 31, 99, 37, 72, 39, 74,
      66, 60, 67, 47, 112, 90, 135, 139, 143, 147, 151, 155, 159, 163, 167, 171, 175, 179
    };

    const int interleaver_bb_impl::group_tab_12_15N_16QAM[180] = {
      120, 32, 38, 113, 71, 31, 65, 109, 36, 106, 134, 66, 29, 86, 136, 108, 83, 70,
      79, 81, 105, 48, 30, 125, 107, 44, 99, 75, 64, 78, 51, 95, 88, 49, 60, 54,
      122, 140, 137, 89, 74, 129, 82, 164, 59, 3, 67, 92, 98, 42, 77, 28, 121, 87,
      18, 21, 93, 72, 2, 142, 112, 9, 50, 8, 90, 139, 14, 97, 63, 85, 104, 124,
      52, 20, 118, 34, 5, 94, 41, 68, 80, 110, 12, 133, 131, 53, 116, 123, 96, 61,
      111, 33, 173, 165, 175, 166, 169, 174, 159, 148, 158, 155, 145, 178, 126, 100, 154, 156,
      179, 157, 46, 149, 171, 37, 153, 163, 152, 146, 177, 103, 160, 147, 76, 172, 144, 150,
      132, 176, 168, 167, 162, 170, 138, 151, 161, 40, 26, 130, 119, 114, 117, 115, 84, 57,
      62, 13, 47, 24, 0, 7, 10, 69, 19, 127, 17, 16, 27, 91, 4, 73, 35, 102,
      15, 55, 23, 25, 11, 56, 45, 58, 128, 43, 135, 1, 143, 141, 6, 22, 101, 39
    };

    const int interleaver_bb_impl::group_tab_13_15N_16QAM[180] = {
      0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68,
      72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140,
      144, 148, 152, 156, 160, 164, 168, 172, 176, 1, 5, 9, 13, 17, 21, 25, 29, 33,
      37, 41, 45, 49, 53, 57, 61, 65, 69, 73, 77, 81, 85, 89, 93, 97, 101, 105,
      109, 113, 117, 121, 125, 129, 133, 137, 141, 145, 149, 153, 157, 161, 165, 169, 173, 177,
      2, 6, 10, 14, 18, 22, 26, 30, 34, 38, 42, 46, 50, 54, 58, 62, 66, 70,
      74, 78, 82, 86, 90, 94, 98, 102, 106, 110, 114, 118, 122, 126, 130, 134, 138, 142,
      146, 150, 154, 158, 162, 166, 170, 174, 178, 3, 7, 11, 15, 19, 23, 27, 31, 35,
      39, 43, 47, 51, 55, 59, 63, 67, 71, 75, 79, 83, 87, 91, 95, 99, 103, 107,
      111, 115, 119, 123, 127, 131, 135, 139, 143, 147, 151, 155, 159, 163, 167, 171, 175, 179
    };

    const int interleaver_bb_impl::group_tab_2_15S_16QAM[45] = {
      5, 33, 18, 8, 29, 10, 21, 14, 30,
      26, 11, 23, 27, 4, 7, 6, 24, 44,
      38, 31, 34, 43, 13, 0, 15, 42, 17,
      2, 20, 12, 40, 39, 35, 32, 1, 3,
      41, 37, 9, 25, 19, 22, 16, 28, 36
    };

    const int interleaver_bb_impl::group_tab_3_15S_16QAM[45] = {
      18, 16, 5, 29, 26, 43, 23, 6, 1,
      24, 7, 19, 37, 2, 27, 3, 10, 15,
      36, 39, 22, 12, 35, 33, 4, 17, 30,
      31, 21, 9, 11, 41, 0, 32, 20, 40,
      25, 8, 34, 38, 28, 14, 44, 13, 42
    };

    const int interleaver_bb_impl::group_tab_4_15S_16QAM[45] = {
      34, 3, 19, 35, 25, 2, 17, 36, 26,
      38, 0, 40, 27, 10, 7, 43, 21, 28,
      15, 6, 1, 37, 18, 30, 32, 33, 29,
      22, 12, 13, 5, 23, 44, 14, 4, 31,
      20, 39, 42, 11, 9, 16, 41, 8, 24
    };

    const int interleaver_bb_impl::group_tab_5_15S_16QAM[45] = {
      3, 33, 39, 2, 38, 29, 0, 10, 25,
      17, 7, 21, 44, 37, 8, 34, 20, 1,
      4, 31, 11, 42, 22, 13, 12, 28, 26,
      43, 30, 14, 16, 23, 24, 15, 5, 18,
      9, 36, 6, 19, 32, 40, 41, 35, 27
    };

    const int interleaver_bb_impl::group_tab_6_15S_16QAM[45] = {
      12, 13, 15, 30, 27, 25, 11, 34, 9,
      4, 31, 22, 6, 32, 7, 21, 17, 3,
      1, 26, 10, 33, 19, 2, 18, 5, 28,
      35, 8, 16, 29, 23, 14, 0, 20, 24,
      36, 37, 38, 39, 40, 41, 42, 43, 44
    };

    const int interleaver_bb_impl::group_tab_7_15S_16QAM[45] = {
      19, 3, 32, 38, 16, 17, 29, 33, 14,
      10, 6, 2, 20, 15, 40, 39, 12, 22,
      23, 34, 31, 13, 44, 43, 36, 24, 37,
      42, 0, 9, 4, 21, 5, 35, 26, 41,
      7, 28, 11, 25, 8, 18, 1, 30, 27
    };

    const int interleaver_bb_impl::group_tab_8_15S_16QAM[45] = {
      36, 5, 22, 26, 1, 13, 3, 33, 9,
      6, 23, 20, 35, 10, 17, 41, 30, 15,
      21, 42, 29, 11, 37, 4, 2, 38, 44,
      0, 18, 19, 8, 31, 28, 43, 14, 34,
      32, 25, 40, 12, 16, 24, 39, 27, 7
    };

    const int interleaver_bb_impl::group_tab_9_15S_16QAM[45] = {
      4, 6, 19, 2, 5, 30, 20, 11, 22,
      12, 15, 0, 36, 37, 38, 39, 26, 14,
      34, 35, 16, 13, 18, 42, 7, 10, 25,
      43, 40, 17, 41, 24, 33, 31, 23, 32,
      21, 3, 27, 28, 8, 9, 29, 1, 44
    };

    const int interleaver_bb_impl::group_tab_10_15S_16QAM[45] = {
      27, 11, 20, 1, 7, 5, 29, 35, 9,
      10, 34, 18, 25, 28, 6, 13, 17, 0,
      23, 16, 41, 15, 19, 44, 24, 37, 4,
      31, 8, 32, 14, 42, 12, 2, 40, 30,
      36, 39, 43, 21, 3, 22, 26, 33, 38
    };

    const int interleaver_bb_impl::group_tab_11_15S_16QAM[45] = {
      2, 4, 41, 8, 13, 7, 0, 24, 3,
      22, 5, 32, 10, 9, 36, 37, 29, 11,
      25, 16, 20, 21, 35, 34, 15, 1, 6,
      14, 27, 30, 33, 12, 17, 28, 23, 40,
      26, 31, 38, 39, 18, 19, 42, 43, 44
    };

    const int interleaver_bb_impl::group_tab_12_15S_16QAM[45] = {
      3, 6, 7, 27, 2, 23, 10, 30, 22,
      28, 24, 20, 37, 21, 4, 14, 11, 42,
      16, 9, 15, 26, 33, 40, 5, 8, 44,
      34, 18, 0, 32, 29, 19, 41, 38, 17,
      25, 43, 35, 36, 13, 39, 12, 1, 31
    };

    const int interleaver_bb_impl::group_tab_13_15S_16QAM[45] = {
      12, 7, 20, 43, 29, 13, 32, 30, 25,
      0, 17, 18, 9, 1, 41, 42, 6, 33,
      28, 14, 16, 11, 39, 40, 15, 4, 23,
      5, 2, 24, 22, 38, 10, 8, 19, 34,
      26, 36, 37, 27, 21, 31, 3, 35, 44
    };

    const int interleaver_bb_impl::group_tab_2_15N_64QAM[180] = {
      57, 149, 83, 142, 29, 20, 30, 52, 5, 100, 156, 22, 130, 167, 121, 126, 137, 158,
      132, 82, 138, 128, 89, 88, 162, 32, 107, 3, 97, 166, 125, 129, 1, 6, 68, 148,
      40, 87, 0, 80, 49, 24, 78, 101, 43, 112, 75, 172, 23, 154, 12, 146, 19, 135,
      48, 170, 123, 147, 95, 91, 13, 35, 127, 61, 60, 139, 44, 59, 55, 109, 157, 177,
      153, 165, 66, 152, 77, 98, 131, 11, 81, 62, 175, 141, 171, 51, 155, 76, 150, 174,
      58, 143, 37, 63, 31, 41, 140, 118, 94, 27, 10, 70, 56, 93, 176, 124, 151, 106,
      46, 163, 179, 4, 18, 144, 178, 161, 145, 71, 114, 7, 105, 133, 84, 86, 17, 21,
      28, 54, 74, 65, 110, 122, 169, 64, 111, 119, 42, 85, 73, 8, 116, 79, 120, 69,
      53, 115, 67, 104, 16, 173, 92, 15, 159, 134, 99, 96, 117, 38, 9, 26, 164, 47,
      103, 113, 136, 168, 102, 14, 45, 72, 25, 50, 34, 36, 90, 160, 2, 33, 39, 108
    };

    const int interleaver_bb_impl::group_tab_3_15N_64QAM[180] = {
      74, 72, 104, 62, 122, 35, 130, 0, 95, 150, 139, 151, 133, 109, 31, 59, 18, 148,
      9, 105, 57, 132, 102, 100, 115, 101, 7, 21, 141, 30, 8, 1, 93, 92, 163, 108,
      52, 159, 24, 89, 117, 88, 178, 113, 98, 179, 144, 156, 54, 164, 12, 63, 39, 22,
      25, 137, 13, 41, 44, 80, 87, 111, 145, 23, 85, 166, 83, 55, 154, 20, 84, 58,
      26, 126, 170, 103, 11, 33, 172, 155, 116, 169, 142, 70, 161, 47, 3, 162, 77, 19,
      28, 97, 124, 6, 168, 107, 60, 76, 143, 121, 42, 157, 65, 43, 173, 56, 171, 90,
      131, 119, 94, 5, 68, 138, 149, 73, 67, 53, 61, 4, 86, 99, 75, 36, 15, 48,
      177, 167, 174, 51, 176, 81, 120, 158, 123, 34, 49, 128, 10, 134, 147, 96, 160, 50,
      146, 16, 38, 78, 91, 152, 46, 127, 27, 175, 135, 79, 125, 82, 2, 129, 153, 14,
      40, 32, 114, 106, 17, 110, 140, 71, 136, 112, 45, 64, 29, 69, 118, 66, 37, 165
    };

    const int interleaver_bb_impl::group_tab_4_15N_64QAM[180] = {
      141, 80, 47, 89, 44, 7, 46, 11, 175, 173, 99, 2, 155, 52, 86, 128, 174, 33,
      170, 31, 35, 162, 64, 95, 92, 4, 16, 49, 137, 104, 29, 9, 60, 167, 50, 23,
      43, 176, 121, 71, 132, 103, 144, 39, 12, 90, 114, 131, 106, 76, 118, 66, 24, 58,
      122, 150, 57, 149, 93, 53, 14, 73, 165, 82, 126, 97, 59, 133, 154, 153, 72, 36,
      5, 96, 120, 134, 101, 61, 115, 0, 28, 42, 18, 145, 156, 85, 146, 6, 161, 10,
      22, 138, 127, 151, 87, 54, 20, 139, 140, 152, 13, 91, 111, 25, 123, 77, 78, 69,
      3, 177, 41, 81, 19, 107, 45, 148, 70, 160, 51, 21, 116, 48, 157, 17, 125, 142,
      83, 110, 37, 98, 179, 129, 168, 172, 1, 40, 166, 159, 147, 56, 100, 63, 26, 169,
      135, 15, 75, 84, 163, 79, 143, 113, 94, 74, 102, 30, 38, 178, 68, 108, 136, 105,
      158, 117, 34, 109, 67, 62, 32, 119, 124, 171, 8, 55, 65, 130, 88, 112, 27, 164
    };

    const int interleaver_bb_impl::group_tab_5_15N_64QAM[180] = {
      166, 54, 6, 27, 141, 134, 58, 46, 55, 91, 56, 100, 172, 80, 18, 152, 12, 108,
      170, 29, 144, 147, 106, 165, 17, 127, 57, 88, 35, 72, 5, 63, 118, 1, 85, 77,
      61, 62, 84, 159, 92, 102, 98, 177, 132, 139, 59, 149, 11, 8, 154, 129, 33, 15,
      143, 4, 95, 101, 53, 42, 40, 9, 111, 130, 123, 82, 81, 114, 119, 175, 157, 41,
      38, 128, 161, 52, 142, 7, 26, 145, 2, 68, 28, 126, 121, 70, 16, 65, 83, 125,
      50, 79, 37, 74, 164, 168, 160, 122, 60, 32, 24, 138, 75, 69, 0, 36, 97, 117,
      14, 109, 173, 120, 112, 87, 176, 124, 151, 67, 13, 94, 105, 133, 64, 76, 153, 31,
      136, 140, 150, 39, 96, 66, 3, 115, 20, 99, 171, 49, 25, 45, 22, 30, 156, 158,
      163, 135, 21, 146, 90, 169, 78, 93, 178, 116, 19, 155, 110, 73, 104, 167, 44, 113,
      162, 89, 47, 43, 86, 48, 107, 71, 137, 51, 174, 103, 131, 179, 148, 10, 23, 34
    };

    const int interleaver_bb_impl::group_tab_6_15N_64QAM[180] = {
      29, 17, 38, 37, 27, 43, 31, 35, 16, 46, 44, 9, 23, 1, 34, 45, 14, 18,
      156, 19, 22, 40, 50, 24, 56, 49, 26, 42, 69, 47, 59, 61, 66, 52, 64, 65,
      67, 54, 170, 68, 132, 51, 70, 41, 21, 5, 160, 7, 13, 55, 62, 53, 63, 58,
      3, 167, 71, 57, 151, 60, 36, 25, 74, 39, 32, 72, 85, 86, 107, 113, 48, 88,
      2, 129, 137, 20, 73, 166, 75, 77, 142, 174, 15, 149, 28, 145, 92, 169, 30, 133,
      163, 119, 82, 176, 152, 134, 139, 148, 164, 99, 173, 104, 83, 106, 112, 135, 153, 0,
      128, 144, 98, 171, 94, 97, 143, 110, 118, 127, 84, 79, 108, 126, 131, 93, 111, 91,
      4, 125, 162, 157, 158, 109, 140, 123, 154, 150, 80, 11, 12, 146, 96, 81, 165, 8,
      89, 138, 105, 141, 103, 6, 100, 161, 172, 78, 101, 115, 179, 147, 116, 136, 122, 87,
      33, 130, 124, 175, 120, 90, 102, 10, 114, 159, 76, 177, 178, 121, 168, 95, 117, 155
    };

    const int interleaver_bb_impl::group_tab_7_15N_64QAM[180] = {
      103, 36, 155, 175, 52, 130, 16, 178, 141, 86, 49, 129, 73, 84, 142, 177, 110, 8,
      96, 77, 139, 167, 109, 2, 17, 37, 146, 169, 54, 134, 101, 78, 135, 70, 153, 6,
      29, 41, 143, 63, 47, 124, 90, 31, 152, 98, 59, 133, 15, 79, 164, 67, 50, 128,
      23, 34, 154, 69, 45, 9, 27, 35, 156, 170, 113, 127, 102, 82, 149, 176, 46, 13,
      22, 30, 163, 60, 114, 11, 92, 44, 157, 74, 48, 132, 24, 87, 140, 66, 118, 123,
      104, 89, 136, 64, 107, 14, 99, 43, 115, 71, 117, 12, 26, 38, 147, 62, 57, 131,
      94, 33, 151, 172, 116, 10, 25, 75, 144, 179, 51, 120, 20, 80, 160, 174, 106, 1,
      21, 88, 137, 61, 105, 5, 18, 32, 158, 72, 56, 125, 28, 42, 161, 168, 53, 7,
      100, 40, 145, 171, 55, 3, 95, 83, 162, 173, 119, 126, 91, 39, 150, 165, 112, 122,
      93, 76, 138, 166, 108, 121, 97, 81, 148, 65, 111, 4, 19, 85, 159, 68, 58, 0
    };

    const int interleaver_bb_impl::group_tab_8_15N_64QAM[180] = {
      86, 71, 51, 48, 89, 94, 46, 81, 67, 49, 80, 37, 55, 61, 36, 57, 52, 92,
      60, 82, 76, 72, 44, 42, 91, 62, 50, 90, 40, 78, 53, 58, 47, 85, 70, 4,
      69, 43, 54, 84, 93, 38, 8, 64, 6, 18, 77, 95, 66, 59, 83, 73, 17, 87,
      3, 75, 65, 88, 79, 14, 151, 117, 32, 22, 123, 30, 33, 162, 144, 9, 121, 108,
      139, 142, 24, 34, 20, 157, 159, 138, 143, 29, 140, 163, 150, 175, 114, 31, 12, 35,
      145, 28, 27, 26, 16, 98, 102, 103, 133, 161, 21, 25, 107, 153, 45, 156, 23, 125,
      141, 56, 166, 5, 1, 170, 119, 68, 134, 41, 74, 179, 2, 129, 169, 101, 99, 109,
      127, 168, 176, 11, 0, 122, 110, 113, 146, 132, 165, 19, 13, 39, 7, 164, 106, 172,
      154, 149, 10, 173, 131, 167, 63, 147, 155, 100, 171, 158, 160, 15, 178, 148, 152, 104,
      124, 177, 97, 130, 118, 137, 111, 126, 120, 105, 115, 136, 112, 96, 135, 116, 174, 128
    };

    const int interleaver_bb_impl::group_tab_9_15N_64QAM[180] = {
      175, 60, 133, 11, 5, 4, 70, 97, 131, 80, 42, 136, 50, 104, 32, 75, 176, 87,
      109, 61, 39, 107, 0, 172, 23, 90, 54, 160, 48, 173, 27, 100, 129, 14, 7, 142,
      20, 103, 38, 126, 157, 144, 21, 64, 44, 79, 105, 146, 49, 93, 1, 84, 81, 145,
      18, 15, 106, 91, 12, 169, 63, 71, 125, 37, 120, 138, 17, 113, 31, 130, 140, 8,
      25, 74, 134, 115, 9, 171, 46, 68, 33, 116, 2, 179, 52, 92, 36, 78, 164, 177,
      24, 72, 122, 118, 162, 121, 16, 73, 45, 53, 77, 110, 30, 66, 29, 76, 158, 148,
      111, 94, 43, 83, 139, 10, 56, 98, 114, 117, 152, 174, 47, 62, 128, 85, 155, 178,
      26, 96, 41, 82, 150, 143, 58, 69, 127, 86, 13, 141, 35, 101, 149, 108, 3, 154,
      51, 95, 132, 135, 163, 137, 28, 102, 123, 112, 151, 167, 59, 19, 156, 119, 153, 168,
      55, 65, 34, 6, 159, 170, 57, 67, 40, 89, 147, 165, 22, 99, 124, 88, 161, 166
    };

    const int interleaver_bb_impl::group_tab_10_15N_64QAM[180] = {
      16, 163, 92, 56, 111, 141, 65, 118, 78, 55, 5, 148, 19, 153, 75, 128, 32, 178,
      22, 156, 99, 124, 4, 168, 20, 115, 87, 122, 9, 166, 27, 155, 94, 134, 38, 137,
      67, 161, 90, 127, 43, 171, 64, 162, 98, 133, 34, 138, 73, 154, 100, 58, 103, 169,
      23, 117, 88, 50, 13, 175, 68, 39, 102, 54, 37, 149, 29, 150, 104, 59, 3, 139,
      69, 110, 77, 131, 42, 142, 25, 158, 80, 47, 35, 143, 72, 151, 84, 57, 8, 176,
      61, 46, 41, 51, 10, 173, 63, 107, 125, 48, 11, 177, 24, 30, 91, 76, 109, 140,
      74, 114, 82, 120, 1, 79, 66, 119, 93, 159, 36, 174, 26, 112, 101, 123, 44, 145,
      60, 157, 97, 45, 33, 167, 70, 152, 85, 126, 40, 135, 62, 108, 95, 49, 31, 147,
      71, 113, 89, 132, 6, 144, 18, 105, 83, 130, 2, 172, 17, 164, 81, 52, 7, 179,
      28, 160, 136, 121, 14, 146, 15, 106, 86, 129, 12, 170, 21, 116, 96, 53, 0, 165
    };

    const int interleaver_bb_impl::group_tab_11_15N_64QAM[180] = {
      12, 15, 2, 16, 27, 50, 35, 74, 38, 70, 108, 32, 112, 54, 30, 122, 72, 116,
      36, 90, 49, 85, 132, 138, 144, 150, 156, 162, 168, 174, 0, 14, 9, 5, 23, 66,
      68, 52, 96, 117, 84, 128, 100, 63, 60, 127, 81, 99, 53, 55, 103, 95, 133, 139,
      145, 151, 157, 163, 169, 175, 10, 22, 13, 11, 28, 104, 37, 57, 115, 46, 65, 129,
      107, 75, 119, 110, 31, 43, 97, 78, 125, 58, 134, 140, 146, 152, 158, 164, 170, 176,
      4, 19, 6, 8, 24, 44, 101, 94, 118, 130, 69, 71, 83, 34, 86, 124, 48, 106,
      89, 40, 102, 91, 135, 141, 147, 153, 159, 165, 171, 177, 3, 20, 7, 17, 25, 87,
      41, 120, 47, 80, 59, 62, 88, 45, 56, 131, 61, 126, 113, 92, 51, 98, 136, 142,
      148, 154, 160, 166, 172, 178, 21, 18, 1, 26, 29, 39, 73, 121, 105, 77, 42, 114,
      93, 82, 111, 109, 67, 79, 123, 64, 76, 33, 137, 143, 149, 155, 161, 167, 173, 179
    };

    const int interleaver_bb_impl::group_tab_12_15N_64QAM[180] = {
      83, 93, 94, 47, 55, 40, 38, 77, 110, 124, 87, 61, 102, 76, 33, 35, 92, 59,
      74, 11, 138, 72, 67, 37, 10, 95, 139, 131, 44, 57, 97, 53, 142, 0, 136, 9,
      143, 86, 100, 21, 15, 75, 62, 19, 65, 129, 101, 79, 22, 68, 73, 23, 18, 81,
      98, 112, 8, 128, 103, 25, 43, 126, 54, 90, 28, 109, 46, 91, 41, 82, 113, 134,
      52, 105, 78, 27, 135, 96, 56, 140, 64, 66, 89, 34, 120, 108, 63, 45, 69, 121,
      88, 39, 29, 133, 106, 117, 127, 32, 42, 58, 71, 118, 51, 84, 85, 80, 104, 132,
      111, 30, 26, 48, 50, 31, 141, 116, 123, 114, 70, 107, 178, 145, 173, 36, 144, 130,
      176, 171, 175, 125, 99, 162, 159, 20, 164, 115, 169, 172, 165, 161, 151, 119, 122, 152,
      157, 4, 137, 148, 153, 170, 154, 166, 13, 150, 16, 167, 174, 163, 49, 6, 168, 147,
      146, 1, 149, 158, 179, 12, 5, 160, 177, 60, 24, 156, 7, 155, 17, 3, 2, 14
    };

    const int interleaver_bb_impl::group_tab_13_15N_64QAM[180] = {
      146, 91, 63, 144, 46, 12, 58, 137, 25, 79, 70, 33, 134, 148, 66, 38, 163, 118,
      139, 130, 72, 92, 160, 23, 133, 153, 128, 86, 152, 106, 53, 93, 61, 5, 158, 172,
      121, 135, 44, 149, 168, 0, 124, 143, 27, 30, 151, 114, 113, 43, 138, 89, 159, 17,
      120, 136, 102, 81, 170, 176, 142, 104, 21, 78, 155, 8, 52, 95, 62, 40, 174, 6,
      131, 48, 18, 1, 179, 34, 123, 77, 26, 84, 157, 85, 56, 147, 67, 76, 162, 10,
      51, 103, 140, 87, 175, 115, 4, 101, 69, 80, 169, 75, 49, 97, 154, 83, 14, 2,
      132, 96, 16, 37, 166, 109, 54, 42, 28, 32, 171, 119, 55, 94, 65, 20, 165, 3,
      47, 90, 117, 88, 177, 11, 59, 68, 73, 41, 150, 111, 127, 100, 110, 31, 167, 13,
      122, 145, 71, 22, 173, 116, 126, 141, 29, 39, 178, 57, 125, 36, 19, 7, 156, 107,
      9, 98, 74, 45, 161, 112, 50, 99, 24, 35, 164, 64, 129, 15, 60, 82, 108, 105
    };

    const int interleaver_bb_impl::group_tab_2_15S_64QAM[45] = {
      7, 11, 4, 38, 19, 25, 2, 43, 15,
      26, 18, 14, 9, 29, 44, 32, 0, 5,
      35, 10, 1, 12, 6, 36, 21, 33, 37,
      34, 3, 31, 20, 16, 40, 23, 41, 22,
      30, 39, 13, 24, 17, 42, 28, 8, 27
    };

    const int interleaver_bb_impl::group_tab_3_15S_64QAM[45] = {
      19, 34, 22, 6, 29, 25, 23, 36, 7,
      8, 24, 16, 27, 43, 11, 35, 5, 28,
      13, 4, 3, 17, 15, 38, 20, 0, 26,
      12, 1, 39, 31, 41, 44, 30, 9, 21,
      42, 18, 14, 32, 10, 2, 37, 33, 40
    };

    const int interleaver_bb_impl::group_tab_4_15S_64QAM[45] = {
      41, 34, 32, 37, 5, 8, 13, 15, 30,
      31, 22, 25, 42, 20, 23, 17, 1, 40,
      44, 12, 6, 43, 7, 29, 33, 16, 11,
      0, 35, 4, 14, 28, 21, 3, 24, 19,
      18, 36, 10, 38, 26, 2, 39, 27, 9
    };

    const int interleaver_bb_impl::group_tab_5_15S_64QAM[45] = {
      25, 44, 8, 39, 37, 2, 11, 7, 0,
      12, 4, 31, 33, 38, 43, 21, 26, 13,
      28, 29, 1, 27, 18, 17, 34, 3, 42,
      10, 19, 20, 32, 36, 40, 9, 41, 5,
      35, 30, 22, 15, 16, 6, 24, 23, 14
    };

    const int interleaver_bb_impl::group_tab_6_15S_64QAM[45] = {
      31, 12, 39, 32, 30, 24, 28, 15, 38,
      23, 27, 41, 0, 6, 17, 37, 42, 20,
      11, 4, 40, 2, 3, 26, 10, 7, 13,
      25, 1, 18, 8, 5, 14, 36, 35, 33,
      22, 9, 44, 16, 34, 19, 21, 29, 43
    };

    const int interleaver_bb_impl::group_tab_7_15S_64QAM[45] = {
      2, 14, 10, 0, 37, 42, 38, 40, 24,
      29, 28, 35, 18, 16, 20, 27, 41, 30,
      15, 19, 9, 43, 25, 3, 6, 7, 31,
      32, 26, 36, 17, 1, 13, 5, 39, 33,
      4, 8, 23, 22, 11, 34, 44, 12, 21
    };

    const int interleaver_bb_impl::group_tab_8_15S_64QAM[45] = {
      36, 6, 2, 20, 43, 17, 33, 22, 23,
      25, 13, 0, 10, 7, 21, 1, 19, 26,
      8, 14, 31, 35, 16, 5, 29, 40, 11,
      9, 4, 34, 15, 42, 32, 28, 18, 37,
      30, 39, 24, 41, 3, 38, 27, 12, 44
    };

    const int interleaver_bb_impl::group_tab_9_15S_64QAM[45] = {
      21, 5, 43, 38, 40, 1, 3, 17, 11,
      37, 10, 41, 9, 15, 25, 44, 14, 27,
      7, 18, 20, 35, 16, 0, 6, 19, 8,
      22, 29, 28, 34, 31, 33, 30, 32, 42,
      13, 4, 24, 26, 36, 2, 23, 12, 39
    };

    const int interleaver_bb_impl::group_tab_10_15S_64QAM[45] = {
      14, 22, 18, 11, 28, 26, 2, 38, 10,
      0, 5, 12, 24, 17, 29, 16, 39, 13,
      23, 8, 25, 43, 34, 33, 27, 15, 7,
      1, 9, 35, 40, 32, 30, 20, 36, 31,
      21, 41, 44, 3, 42, 6, 19, 37, 4
    };

    const int interleaver_bb_impl::group_tab_11_15S_64QAM[45] = {
      31, 20, 21, 25, 4, 16, 9, 3, 17,
      24, 5, 10, 12, 28, 6, 19, 8, 15,
      13, 11, 29, 22, 27, 14, 23, 34, 26,
      18, 42, 2, 37, 44, 39, 33, 35, 41,
      0, 36, 7, 40, 38, 1, 30, 32, 43
    };

    const int interleaver_bb_impl::group_tab_12_15S_64QAM[45] = {
      17, 11, 14, 7, 31, 10, 2, 26, 0,
      32, 29, 22, 33, 12, 20, 28, 27, 39,
      37, 15, 4, 5, 8, 13, 38, 18, 23,
      34, 24, 6, 1, 9, 16, 44, 21, 3,
      36, 30, 40, 35, 43, 42, 25, 19, 41
    };

    const int interleaver_bb_impl::group_tab_13_15S_64QAM[45] = {
      9, 7, 15, 10, 11, 12, 13, 6, 21,
      17, 14, 20, 26, 8, 25, 32, 34, 23,
      2, 4, 31, 18, 5, 27, 29, 3, 38,
      36, 39, 43, 41, 42, 40, 44, 1, 28,
      33, 22, 16, 19, 24, 0, 30, 35, 37
    };

    const int interleaver_bb_impl::group_tab_2_15N_256QAM[180] = {
      112, 78, 104, 6, 59, 80, 49, 120, 114, 27, 113, 3, 109, 44, 69, 164, 91, 137,
      39, 31, 21, 127, 151, 8, 47, 176, 117, 68, 122, 148, 79, 73, 7, 166, 51, 50,
      116, 66, 152, 61, 29, 107, 22, 154, 118, 94, 24, 35, 55, 38, 88, 54, 2, 15,
      19, 67, 101, 74, 169, 138, 41, 162, 175, 136, 62, 161, 121, 163, 115, 135, 123, 25,
      140, 156, 58, 33, 119, 111, 146, 129, 150, 147, 97, 18, 60, 4, 81, 168, 43, 105,
      36, 65, 13, 5, 108, 145, 23, 70, 20, 173, 159, 100, 128, 172, 170, 1, 37, 83,
      102, 103, 157, 139, 179, 32, 144, 92, 131, 75, 155, 14, 9, 149, 63, 11, 134, 53,
      99, 17, 57, 90, 30, 98, 64, 40, 87, 158, 77, 93, 124, 46, 171, 141, 133, 85,
      177, 132, 26, 160, 42, 34, 82, 96, 48, 10, 142, 125, 178, 153, 72, 45, 89, 52,
      28, 126, 143, 167, 76, 86, 130, 110, 174, 16, 165, 56, 84, 95, 0, 106, 12, 71
    };

    const int interleaver_bb_impl::group_tab_3_15N_256QAM[180] = {
      136, 28, 85, 38, 40, 89, 133, 117, 3, 58, 154, 77, 14, 179, 96, 101, 26, 169,
      37, 83, 162, 165, 24, 66, 109, 126, 10, 155, 70, 157, 105, 175, 67, 158, 32, 42,
      147, 140, 30, 7, 92, 59, 119, 56, 0, 5, 90, 174, 13, 47, 76, 88, 86, 108,
      27, 18, 12, 8, 61, 145, 75, 125, 112, 69, 120, 137, 116, 20, 178, 98, 176, 29,
      68, 168, 124, 21, 35, 150, 131, 159, 163, 84, 23, 123, 65, 103, 93, 99, 102, 31,
      64, 74, 46, 94, 80, 129, 142, 128, 148, 111, 134, 173, 60, 118, 2, 170, 135, 1,
      115, 143, 95, 177, 73, 43, 11, 114, 91, 78, 107, 172, 25, 36, 164, 149, 153, 110,
      44, 146, 82, 127, 45, 33, 50, 41, 52, 156, 34, 4, 79, 141, 138, 122, 53, 160,
      81, 16, 100, 130, 71, 121, 132, 9, 22, 113, 6, 152, 15, 171, 17, 57, 49, 151,
      161, 63, 55, 139, 166, 97, 19, 51, 72, 167, 106, 48, 144, 87, 104, 62, 54, 39
    };

    const int interleaver_bb_impl::group_tab_4_15N_256QAM[180] = {
      13, 121, 137, 29, 27, 1, 70, 116, 35, 132, 109, 51, 55, 58, 11, 67, 136, 25,
      145, 7, 75, 107, 45, 21, 127, 52, 90, 22, 100, 123, 69, 112, 155, 92, 151, 59,
      5, 179, 44, 87, 56, 139, 65, 170, 46, 0, 124, 78, 166, 8, 61, 97, 120, 103,
      4, 19, 64, 79, 28, 134, 93, 86, 60, 135, 126, 53, 63, 14, 122, 17, 150, 76,
      42, 39, 23, 153, 95, 66, 50, 141, 176, 34, 161, 26, 106, 10, 43, 85, 131, 2,
      147, 148, 144, 54, 115, 146, 101, 172, 114, 119, 3, 96, 133, 99, 167, 164, 9, 142,
      68, 149, 94, 83, 16, 175, 73, 38, 143, 159, 130, 84, 169, 18, 138, 102, 72, 47,
      32, 160, 82, 81, 168, 30, 12, 173, 156, 158, 125, 98, 62, 178, 48, 163, 117, 110,
      91, 37, 80, 105, 31, 174, 111, 49, 113, 108, 74, 157, 128, 24, 118, 40, 88, 177,
      154, 6, 162, 129, 77, 36, 165, 20, 89, 140, 15, 33, 104, 152, 71, 171, 57, 41
    };

    const int interleaver_bb_impl::group_tab_5_15N_256QAM[180] = {
      39, 45, 128, 84, 143, 148, 2, 75, 43, 50, 130, 87, 137, 151, 7, 71, 55, 51,
      133, 90, 140, 149, 6, 177, 37, 124, 99, 83, 23, 159, 0, 176, 41, 121, 96, 89,
      30, 161, 18, 172, 60, 49, 134, 104, 139, 166, 14, 179, 62, 48, 129, 105, 146, 160,
      16, 174, 33, 54, 132, 112, 145, 150, 9, 77, 34, 117, 92, 82, 136, 165, 4, 67,
      36, 44, 101, 81, 141, 156, 3, 175, 58, 47, 91, 102, 32, 158, 13, 178, 63, 118,
      100, 85, 26, 167, 1, 173, 38, 116, 131, 107, 138, 162, 8, 72, 42, 115, 98, 108,
      24, 152, 17, 171, 64, 123, 94, 110, 28, 147, 19, 169, 61, 46, 97, 106, 144, 164,
      5, 70, 59, 53, 127, 88, 31, 153, 10, 73, 66, 119, 126, 111, 29, 155, 15, 170,
      57, 120, 125, 80, 142, 168, 11, 68, 56, 52, 95, 103, 27, 154, 21, 78, 40, 122,
      93, 86, 25, 163, 20, 79, 35, 114, 135, 109, 22, 157, 12, 69, 65, 74, 76, 113
    };

    const int interleaver_bb_impl::group_tab_6_15N_256QAM[180] = {
      99, 100, 15, 107, 54, 76, 153, 174, 61, 0, 36, 71, 62, 137, 108, 114, 65, 98,
      151, 19, 112, 109, 152, 117, 35, 93, 43, 90, 154, 73, 150, 165, 23, 16, 91, 5,
      169, 175, 120, 149, 26, 59, 49, 56, 156, 136, 110, 80, 58, 55, 40, 103, 159, 83,
      127, 111, 155, 167, 11, 52, 116, 142, 133, 1, 2, 96, 77, 86, 122, 6, 131, 29,
      51, 21, 17, 45, 126, 12, 3, 168, 41, 30, 37, 64, 164, 78, 8, 118, 113, 39,
      48, 140, 14, 60, 82, 134, 25, 33, 50, 84, 28, 105, 123, 145, 7, 27, 34, 92,
      115, 147, 74, 10, 68, 102, 67, 63, 101, 18, 66, 129, 24, 4, 119, 87, 42, 170,
      143, 121, 38, 57, 95, 148, 89, 81, 158, 171, 32, 22, 69, 53, 130, 104, 161, 75,
      141, 9, 47, 79, 162, 146, 124, 157, 70, 106, 31, 132, 166, 128, 138, 125, 44, 13,
      85, 88, 135, 144, 173, 163, 20, 46, 97, 94, 139, 172, 72, 160, 176, 177, 178, 179
    };

    const int interleaver_bb_impl::group_tab_7_15N_256QAM[180] = {
      24, 157, 0, 43, 126, 172, 135, 65, 32, 18, 114, 42, 162, 67, 104, 61, 23, 11,
      4, 96, 163, 75, 109, 58, 79, 154, 3, 95, 168, 73, 103, 60, 84, 148, 113, 40,
      164, 173, 143, 49, 29, 156, 7, 89, 132, 179, 138, 53, 85, 12, 117, 36, 122, 66,
      107, 64, 28, 147, 2, 90, 131, 70, 144, 55, 26, 15, 112, 35, 128, 176, 106, 59,
      80, 19, 6, 92, 129, 174, 99, 62, 82, 13, 121, 41, 127, 71, 139, 63, 25, 151,
      9, 39, 159, 69, 142, 52, 77, 21, 119, 38, 167, 178, 101, 56, 87, 155, 5, 91,
      166, 169, 146, 50, 81, 20, 111, 88, 165, 177, 108, 47, 27, 149, 115, 33, 161, 72,
      102, 57, 86, 16, 110, 97, 123, 68, 100, 48, 31, 14, 8, 93, 130, 170, 133, 44,
      78, 150, 118, 94, 158, 76, 134, 46, 83, 152, 1, 37, 160, 171, 136, 54, 22, 17,
      116, 34, 125, 175, 105, 45, 30, 153, 10, 98, 124, 74, 137, 51, 120, 141, 140, 145
    };

    const int interleaver_bb_impl::group_tab_8_15N_256QAM[180] = {
      85, 3, 148, 161, 96, 99, 154, 13, 78, 160, 61, 36, 21, 141, 121, 115, 82, 1,
      59, 72, 43, 135, 168, 139, 46, 10, 56, 67, 108, 134, 111, 105, 66, 89, 137, 130,
      104, 143, 113, 11, 84, 157, 32, 73, 90, 38, 117, 146, 53, 2, 60, 93, 91, 71,
      114, 19, 47, 4, 26, 75, 109, 41, 50, 153, 54, 163, 31, 24, 106, 42, 170, 62,
      80, 164, 65, 128, 12, 142, 167, 155, 88, 8, 22, 131, 158, 33, 178, 145, 70, 9,
      51, 69, 102, 140, 173, 147, 83, 165, 30, 126, 100, 138, 171, 103, 45, 159, 27, 74,
      97, 122, 120, 16, 52, 162, 132, 124, 94, 133, 172, 149, 86, 77, 25, 68, 177, 64,
      174, 15, 0, 125, 63, 35, 34, 40, 179, 20, 44, 7, 55, 28, 101, 150, 110, 18,
      119, 5, 29, 76, 107, 136, 112, 144, 48, 81, 57, 49, 92, 95, 118, 17, 156, 166,
      23, 129, 79, 37, 175, 152, 87, 6, 58, 127, 98, 123, 39, 14, 116, 169, 176, 151
    };

    const int interleaver_bb_impl::group_tab_9_15N_256QAM[180] = {
      58, 70, 23, 32, 26, 63, 55, 48, 35, 41, 53, 20, 38, 51, 61, 65, 44, 29,
      7, 2, 113, 68, 96, 104, 106, 89, 27, 0, 119, 21, 4, 49, 46, 100, 13, 36,
      57, 98, 102, 9, 42, 39, 33, 62, 22, 95, 101, 15, 91, 25, 93, 132, 69, 87,
      47, 59, 67, 124, 17, 11, 31, 43, 40, 37, 85, 50, 97, 140, 45, 92, 56, 30,
      34, 60, 107, 24, 52, 94, 64, 5, 71, 90, 66, 103, 88, 86, 84, 19, 169, 159,
      147, 126, 28, 130, 14, 162, 144, 166, 108, 153, 115, 135, 120, 122, 112, 139, 151, 156,
      16, 172, 164, 123, 99, 54, 136, 81, 105, 128, 116, 150, 155, 76, 18, 142, 170, 175,
      83, 146, 78, 109, 73, 131, 127, 82, 167, 77, 110, 79, 137, 152, 3, 173, 148, 72,
      158, 117, 1, 6, 12, 8, 161, 74, 143, 133, 168, 171, 134, 163, 138, 121, 141, 160,
      111, 10, 149, 80, 75, 165, 157, 174, 129, 145, 114, 125, 154, 118, 176, 177, 178, 179
    };

    const int interleaver_bb_impl::group_tab_10_15N_256QAM[180] = {
      45, 31, 67, 35, 159, 157, 177, 2, 44, 23, 73, 148, 163, 118, 176, 4, 14, 97,
      142, 37, 143, 149, 179, 3, 12, 32, 140, 42, 167, 166, 41, 126, 13, 30, 144, 57,
      113, 147, 173, 6, 52, 24, 39, 64, 80, 112, 104, 174, 11, 151, 71, 109, 162, 79,
      171, 127, 46, 92, 38, 132, 81, 120, 100, 1, 53, 88, 76, 60, 103, 139, 99, 125,
      48, 93, 135, 161, 77, 110, 107, 121, 18, 95, 69, 63, 83, 111, 170, 7, 16, 98,
      141, 61, 86, 116, 172, 130, 49, 25, 40, 65, 87, 108, 101, 5, 21, 89, 75, 43,
      82, 146, 105, 128, 17, 29, 106, 34, 160, 155, 175, 124, 15, 28, 134, 62, 119, 145,
      72, 10, 58, 91, 74, 36, 68, 150, 8, 9, 54, 26, 137, 56, 165, 115, 114, 0,
      47, 27, 22, 20, 168, 154, 102, 123, 50, 94, 66, 33, 85, 59, 164, 131, 51, 90,
      70, 138, 84, 117, 178, 122, 19, 96, 156, 55, 78, 158, 169, 129, 133, 152, 136, 153
    };

    const int interleaver_bb_impl::group_tab_11_15N_256QAM[180] = {
      27, 68, 35, 117, 138, 83, 127, 10, 60, 73, 47, 115, 155, 81, 170, 9, 65, 66,
      52, 112, 150, 77, 171, 161, 22, 20, 39, 106, 147, 90, 126, 165, 23, 16, 45, 113,
      154, 86, 173, 158, 24, 71, 40, 107, 136, 94, 128, 163, 31, 72, 33, 101, 134, 80,
      175, 7, 61, 19, 49, 111, 135, 92, 130, 6, 62, 74, 43, 116, 133, 89, 129, 8,
      28, 15, 34, 105, 146, 84, 174, 4, 32, 75, 44, 118, 132, 96, 169, 159, 58, 18,
      42, 100, 141, 87, 131, 157, 63, 11, 48, 108, 151, 79, 177, 168, 26, 17, 36, 102,
      137, 95, 122, 1, 25, 21, 50, 120, 153, 97, 121, 0, 55, 14, 46, 114, 152, 91,
      178, 3, 30, 13, 37, 103, 145, 82, 125, 166, 57, 76, 51, 99, 144, 85, 123, 162,
      56, 12, 53, 119, 139, 78, 179, 5, 64, 70, 54, 110, 148, 93, 172, 164, 29, 69,
      38, 109, 143, 88, 124, 160, 59, 67, 41, 104, 149, 98, 176, 2, 167, 156, 140, 142
    };

    const int interleaver_bb_impl::group_tab_12_15N_256QAM[180] = {
      51, 122, 91, 111, 95, 100, 119, 130, 78, 57, 65, 26, 61, 126, 105, 143, 70, 132,
      39, 102, 115, 116, 6, 14, 3, 21, 71, 134, 2, 0, 140, 106, 7, 118, 23, 35,
      20, 17, 50, 48, 112, 13, 66, 5, 75, 42, 129, 107, 30, 45, 137, 114, 37, 87,
      53, 85, 101, 141, 120, 99, 88, 117, 64, 28, 135, 138, 108, 113, 58, 97, 38, 124,
      86, 33, 74, 32, 29, 128, 67, 104, 80, 127, 56, 34, 89, 94, 49, 55, 93, 136,
      68, 62, 54, 40, 81, 103, 121, 76, 44, 84, 96, 123, 154, 98, 82, 142, 46, 169,
      131, 72, 47, 69, 125, 31, 83, 36, 59, 90, 79, 52, 133, 60, 92, 139, 110, 27,
      73, 43, 77, 109, 63, 41, 168, 147, 161, 165, 175, 162, 164, 158, 157, 160, 150, 171,
      167, 145, 151, 153, 9, 155, 170, 146, 166, 149, 15, 159, 11, 176, 152, 156, 144, 148,
      172, 178, 24, 22, 179, 4, 163, 174, 173, 19, 10, 177, 12, 16, 1, 8, 18, 25
    };

    const int interleaver_bb_impl::group_tab_13_15N_256QAM[180] = {
      59, 85, 108, 128, 49, 91, 163, 3, 58, 16, 106, 126, 74, 141, 167, 35, 57, 82,
      30, 123, 68, 95, 160, 42, 62, 21, 102, 131, 52, 142, 157, 10, 55, 79, 24, 130,
      73, 92, 179, 2, 61, 11, 104, 122, 45, 140, 159, 43, 148, 19, 23, 111, 76, 135,
      169, 39, 63, 77, 25, 117, 75, 94, 155, 5, 145, 14, 26, 127, 46, 138, 158, 38,
      64, 86, 105, 118, 50, 137, 175, 7, 144, 84, 22, 113, 54, 98, 172, 9, 146, 17,
      27, 114, 51, 139, 156, 37, 147, 78, 103, 115, 66, 97, 168, 34, 60, 83, 107, 121,
      48, 93, 174, 33, 65, 87, 99, 124, 71, 136, 154, 0, 150, 20, 101, 112, 70, 96,
      170, 1, 149, 80, 28, 125, 53, 90, 173, 6, 153, 13, 29, 116, 72, 88, 165, 8,
      143, 12, 31, 119, 47, 89, 164, 40, 151, 81, 109, 110, 44, 134, 162, 36, 152, 15,
      100, 129, 67, 133, 166, 41, 56, 18, 32, 120, 69, 132, 161, 4, 177, 176, 178, 171
    };

    const int interleaver_bb_impl::group_tab_2_15S_256QAM[45] = {
      31, 3, 38, 9, 34, 6, 4, 18, 15,
      1, 21, 19, 42, 20, 12, 13, 30, 26,
      14, 2, 10, 35, 28, 44, 23, 11, 22,
      16, 29, 40, 27, 37, 25, 41, 5, 43,
      39, 36, 7, 24, 32, 17, 33, 8, 0
    };

    const int interleaver_bb_impl::group_tab_3_15S_256QAM[45] = {
      5, 22, 23, 26, 29, 27, 16, 1, 4,
      25, 41, 21, 12, 2, 6, 8, 7, 19,
      44, 42, 39, 40, 43, 35, 10, 28, 13,
      15, 37, 32, 3, 24, 36, 38, 11, 18,
      33, 30, 14, 9, 34, 20, 0, 17, 31
    };

    const int interleaver_bb_impl::group_tab_4_15S_256QAM[45] = {
      38, 20, 0, 34, 33, 41, 14, 30, 44,
      7, 37, 8, 4, 9, 43, 15, 19, 32,
      23, 5, 22, 26, 10, 12, 3, 31, 36,
      21, 24, 11, 16, 18, 17, 29, 35, 42,
      13, 40, 1, 28, 2, 25, 6, 39, 27
    };

    const int interleaver_bb_impl::group_tab_5_15S_256QAM[45] = {
      4, 23, 3, 6, 18, 5, 0, 2, 7,
      26, 21, 27, 39, 42, 38, 31, 1, 34,
      20, 37, 40, 24, 43, 25, 33, 9, 22,
      36, 30, 35, 11, 10, 17, 32, 13, 12,
      41, 15, 14, 19, 16, 8, 44, 29, 28
    };

    const int interleaver_bb_impl::group_tab_6_15S_256QAM[45] = {
      17, 13, 25, 24, 14, 21, 1, 37, 2,
      3, 11, 22, 18, 5, 10, 23, 12, 4,
      26, 16, 38, 36, 33, 39, 0, 6, 7,
      31, 32, 34, 27, 35, 15, 9, 30, 28,
      19, 8, 20, 29, 40, 41, 42, 43, 44
    };

    const int interleaver_bb_impl::group_tab_7_15S_256QAM[45] = {
      13, 16, 4, 12, 44, 15, 8, 14, 0,
      3, 30, 20, 35, 21, 10, 6, 19, 17,
      26, 39, 7, 24, 9, 27, 5, 37, 23,
      32, 40, 31, 38, 42, 34, 25, 36, 2,
      22, 43, 33, 28, 1, 18, 11, 41, 29
    };

    const int interleaver_bb_impl::group_tab_8_15S_256QAM[45] = {
      41, 2, 12, 6, 33, 1, 13, 11, 26,
      10, 39, 43, 36, 23, 42, 7, 44, 20,
      8, 38, 18, 22, 24, 40, 4, 28, 29,
      19, 14, 5, 9, 0, 30, 25, 35, 37,
      27, 32, 31, 34, 21, 3, 15, 17, 16
    };

    const int interleaver_bb_impl::group_tab_9_15S_256QAM[45] = {
      5, 7, 9, 22, 10, 12, 3, 43, 6,
      4, 24, 13, 14, 11, 15, 18, 19, 17,
      16, 41, 25, 26, 20, 23, 21, 33, 31,
      28, 39, 36, 30, 37, 27, 32, 34, 35,
      29, 2, 42, 0, 1, 8, 40, 38, 44
    };

    const int interleaver_bb_impl::group_tab_10_15S_256QAM[45] = {
      28, 20, 18, 38, 39, 2, 3, 30, 19,
      4, 14, 36, 7, 0, 25, 17, 10, 6,
      33, 15, 8, 26, 42, 24, 11, 21, 23,
      5, 40, 41, 29, 32, 37, 44, 43, 31,
      35, 34, 22, 1, 16, 27, 9, 13, 12
    };

    const int interleaver_bb_impl::group_tab_11_15S_256QAM[45] = {
      8, 13, 0, 11, 9, 4, 36, 37, 16,
      3, 10, 14, 24, 20, 33, 34, 25, 2,
      21, 31, 12, 19, 7, 5, 27, 23, 26,
      1, 18, 22, 35, 6, 32, 30, 28, 15,
      29, 17, 39, 38, 40, 41, 42, 43, 44
    };

    const int interleaver_bb_impl::group_tab_12_15S_256QAM[45] = {
      28, 21, 10, 15, 8, 22, 26, 2, 14,
      1, 27, 3, 39, 20, 34, 25, 12, 6,
      7, 40, 30, 29, 38, 16, 43, 33, 4,
      35, 9, 32, 5, 36, 0, 41, 37, 18,
      17, 13, 24, 42, 31, 23, 19, 11, 44
    };

    const int interleaver_bb_impl::group_tab_13_15S_256QAM[45] = {
      9, 13, 10, 7, 11, 6, 1, 14, 12,
      8, 21, 15, 4, 36, 25, 30, 24, 28,
      29, 20, 27, 5, 18, 17, 22, 33, 0,
      16, 23, 31, 42, 3, 40, 39, 41, 43,
      37, 44, 26, 2, 19, 38, 32, 35, 34
    };

  } /* namespace atsc3 */
} /* namespace gr */
