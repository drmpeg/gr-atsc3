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
                break;
              case MOD_16QAM:
                group_table = &group_tab_2_15N_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_2_15N_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_2_15N_256QAM[0];
                break;
              default:
                group_table = &group_tab_2_15N_QPSK[0];
                break;
            }
            break;
          case C3_15:
            nbch = 12960;
            ldpc_type = LDPC_TYPE_A;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_3_15N_QPSK[0];
                break;
              case MOD_16QAM:
                group_table = &group_tab_3_15N_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_3_15N_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_3_15N_256QAM[0];
                break;
              default:
                group_table = &group_tab_3_15N_QPSK[0];
                break;
            }
            break;
          case C4_15:
            nbch = 17280;
            ldpc_type = LDPC_TYPE_A;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_4_15N_QPSK[0];
                break;
              case MOD_16QAM:
                group_table = &group_tab_4_15N_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_4_15N_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_4_15N_256QAM[0];
                break;
              default:
                group_table = &group_tab_4_15N_QPSK[0];
                break;
            }
            break;
          case C5_15:
            nbch = 21600;
            ldpc_type = LDPC_TYPE_A;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_5_15N_QPSK[0];
                break;
              case MOD_16QAM:
                group_table = &group_tab_5_15N_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_5_15N_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_5_15N_256QAM[0];
                break;
              default:
                group_table = &group_tab_5_15N_QPSK[0];
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
                break;
              case MOD_16QAM:
                group_table = &group_tab_6_15N_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_6_15N_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_6_15N_256QAM[0];
                break;
              default:
                group_table = &group_tab_6_15N_QPSK[0];
                break;
            }
            break;
          case C7_15:
            nbch = 30240;
            ldpc_type = LDPC_TYPE_A;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_7_15N_QPSK[0];
                break;
              case MOD_16QAM:
                group_table = &group_tab_7_15N_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_7_15N_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_7_15N_256QAM[0];
                break;
              default:
                group_table = &group_tab_7_15N_QPSK[0];
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
                break;
              case MOD_16QAM:
                group_table = &group_tab_8_15N_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_8_15N_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_8_15N_256QAM[0];
                break;
              default:
                group_table = &group_tab_8_15N_QPSK[0];
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
                break;
              case MOD_16QAM:
                group_table = &group_tab_9_15N_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_9_15N_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_9_15N_256QAM[0];
                break;
              default:
                group_table = &group_tab_9_15N_QPSK[0];
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
                break;
              case MOD_16QAM:
                group_table = &group_tab_10_15N_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_10_15N_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_10_15N_256QAM[0];
                break;
              default:
                group_table = &group_tab_10_15N_QPSK[0];
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
                break;
              case MOD_16QAM:
                group_table = &group_tab_11_15N_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_11_15N_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_11_15N_256QAM[0];
                break;
              default:
                group_table = &group_tab_11_15N_QPSK[0];
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
                break;
              case MOD_16QAM:
                group_table = &group_tab_12_15N_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_12_15N_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_12_15N_256QAM[0];
                break;
              default:
                group_table = &group_tab_12_15N_QPSK[0];
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
                break;
              case MOD_16QAM:
                group_table = &group_tab_13_15N_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_13_15N_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_13_15N_256QAM[0];
                break;
              default:
                group_table = &group_tab_13_15N_QPSK[0];
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
                break;
              case MOD_16QAM:
                group_table = &group_tab_2_15S_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_2_15S_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_2_15S_256QAM[0];
                break;
              default:
                group_table = &group_tab_2_15S_QPSK[0];
                break;
            }
            break;
          case C3_15:
            nbch = 3240;
            ldpc_type = LDPC_TYPE_A;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_3_15S_QPSK[0];
                break;
              case MOD_16QAM:
                group_table = &group_tab_3_15S_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_3_15S_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_3_15S_256QAM[0];
                break;
              default:
                group_table = &group_tab_3_15S_QPSK[0];
                break;
            }
            break;
          case C4_15:
            nbch = 4320;
            ldpc_type = LDPC_TYPE_A;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_4_15S_QPSK[0];
                break;
              case MOD_16QAM:
                group_table = &group_tab_4_15S_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_4_15S_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_4_15S_256QAM[0];
                break;
              default:
                group_table = &group_tab_4_15S_QPSK[0];
                break;
            }
            break;
          case C5_15:
            nbch = 5400;
            ldpc_type = LDPC_TYPE_A;
            switch (constellation) {
              case MOD_QPSK:
                group_table = &group_tab_5_15S_QPSK[0];
                break;
              case MOD_16QAM:
                group_table = &group_tab_5_15S_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_5_15S_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_5_15S_256QAM[0];
                break;
              default:
                group_table = &group_tab_5_15S_QPSK[0];
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
                break;
              case MOD_16QAM:
                group_table = &group_tab_6_15S_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_6_15S_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_6_15S_256QAM[0];
                break;
              default:
                group_table = &group_tab_6_15S_QPSK[0];
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
                break;
              case MOD_16QAM:
                group_table = &group_tab_7_15S_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_7_15S_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_7_15S_256QAM[0];
                break;
              default:
                group_table = &group_tab_7_15S_QPSK[0];
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
                break;
              case MOD_16QAM:
                group_table = &group_tab_8_15S_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_8_15S_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_8_15S_256QAM[0];
                break;
              default:
                group_table = &group_tab_8_15S_QPSK[0];
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
                break;
              case MOD_16QAM:
                group_table = &group_tab_9_15S_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_9_15S_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_9_15S_256QAM[0];
                break;
              default:
                group_table = &group_tab_9_15S_QPSK[0];
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
                break;
              case MOD_16QAM:
                group_table = &group_tab_10_15S_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_10_15S_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_10_15S_256QAM[0];
                break;
              default:
                group_table = &group_tab_10_15S_QPSK[0];
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
                break;
              case MOD_16QAM:
                group_table = &group_tab_11_15S_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_11_15S_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_11_15S_256QAM[0];
                break;
              default:
                group_table = &group_tab_11_15S_QPSK[0];
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
                break;
              case MOD_16QAM:
                group_table = &group_tab_12_15S_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_12_15S_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_12_15S_256QAM[0];
                break;
              default:
                group_table = &group_tab_12_15S_QPSK[0];
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
                break;
              case MOD_16QAM:
                group_table = &group_tab_13_15S_16QAM[0];
                break;
              case MOD_64QAM:
                group_table = &group_tab_13_15S_64QAM[0];
                break;
              case MOD_256QAM:
                group_table = &group_tab_13_15S_256QAM[0];
                break;
              default:
                group_table = &group_tab_13_15S_QPSK[0];
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
          set_output_multiple(frame_size / mod);
          packed_items = frame_size / mod;
          break;
        case MOD_16QAM:
          mod = 4;
          set_output_multiple(frame_size / mod);
          packed_items = frame_size / mod;
          break;
        case MOD_64QAM:
          mod = 6;
          set_output_multiple(frame_size / mod);
          packed_items = frame_size / mod;
          break;
        case MOD_256QAM:
          mod = 8;
          set_output_multiple(frame_size / mod);
          packed_items = frame_size / mod;
          break;
        default:
          mod = 1;
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
      int rows, offset, index, block, indexv;
      unsigned int pack;
      const int *mux;

      switch (signal_constellation) {
        case MOD_QPSK:
          for (int i = 0; i < noutput_items; i += packed_items) {
            rows = frame_size / 2;
            const unsigned char *c1, *c2;
            c1 = &tempv[0];
            c2 = &tempv[rows];
            if (ldpc_type == LDPC_TYPE_B) {
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
                indexv = block * 360;
                for (int k = 0; k < 360; k++) {
                  tempv[indexv++] = tempu[index++];
                }
              }
              index = 0;
              for (int j = 0; j < rows; j++) {
                tempu[index++] = c1[j];
                tempu[index++] = c2[j];
              }
              index = 0;
              for (int j = 0; j < rows; j++) {
                out[produced] = tempu[index++] << 1;
                out[produced++] |= tempu[index++];
                consumed += 2;
              }
            }
            else {
              for (int j = 0; j < rows; j++) {
                out[produced] = in[consumed++] << 1;
                out[produced++] |= in[consumed++];
              }
            }
          }
          break;
        case MOD_16QAM:
          for (int i = 0; i < noutput_items; i += packed_items) {
            rows = frame_size / (mod * 2);
            const unsigned char *c1, *c2, *c3, *c4, *c5, *c6, *c7, *c8;
            c1 = &tempv[0];
            c2 = &tempv[rows];
            c3 = &tempv[rows * 2];
            c4 = &tempv[rows * 3];
            c5 = &tempv[rows * 4];
            c6 = &tempv[rows * 5];
            c7 = &tempv[rows * 6];
            c8 = &tempv[rows * 7];
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
            index = 0;
            for (int d = 0; d < frame_size / (mod * 2); d++) {
              pack = 0;
              for (int e = 0; e < (mod * 2); e++) {
                offset = mux[e];
                pack |= tempu[index++] << (((mod * 2) - 1) - offset);
              }
              out[produced++] = pack >> 4;
              out[produced++] = pack & 0xf;
              consumed += (mod * 2);
            }
          }
          break;
        case MOD_64QAM:
          for (int i = 0; i < noutput_items; i += packed_items) {
            rows = frame_size / (mod * 2);
            const unsigned char *c1, *c2, *c3, *c4, *c5, *c6, *c7, *c8, *c9, *c10, *c11, *c12;
            c1 = &tempv[0];
            c2 = &tempv[rows];
            c3 = &tempv[rows * 2];
            c4 = &tempv[rows * 3];
            c5 = &tempv[rows * 4];
            c6 = &tempv[rows * 5];
            c7 = &tempv[rows * 6];
            c8 = &tempv[rows * 7];
            c9 = &tempv[rows * 8];
            c10 = &tempv[rows * 9];
            c11 = &tempv[rows * 10];
            c12 = &tempv[rows * 11];
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
            for (int j = 0; j < rows; j++) {
              tempu[index++] = c1[j];
              tempu[index++] = c2[j];
              tempu[index++] = c3[j];
              tempu[index++] = c4[j];
              tempu[index++] = c5[j];
              tempu[index++] = c6[j];
              tempu[index++] = c7[j];
              tempu[index++] = c8[j];
              tempu[index++] = c9[j];
              tempu[index++] = c10[j];
              tempu[index++] = c11[j];
              tempu[index++] = c12[j];
            }
            index = 0;
            for (int d = 0; d < frame_size / (mod * 2); d++) {
              pack = 0;
              for (int e = 0; e < (mod * 2); e++) {
                offset = mux[e];
                pack |= tempu[index++] << (((mod * 2) - 1) - offset);
              }
              out[produced++] = pack >> 6;
              out[produced++] = pack & 0x3f;
              consumed += (mod * 2);
            }
          }
          break;
        case MOD_256QAM:
          if (frame_size == FRAME_SIZE_NORMAL) {
            for (int i = 0; i < noutput_items; i += packed_items) {
              rows = frame_size / (mod * 2);
              const unsigned char *c1, *c2, *c3, *c4, *c5, *c6, *c7, *c8;
              const unsigned char *c9, *c10, *c11, *c12, *c13, *c14, *c15, *c16;
              c1 = &tempv[0];
              c2 = &tempv[rows];
              c3 = &tempv[rows * 2];
              c4 = &tempv[rows * 3];
              c5 = &tempv[rows * 4];
              c6 = &tempv[rows * 5];
              c7 = &tempv[rows * 6];
              c8 = &tempv[rows * 7];
              c9 = &tempv[rows * 8];
              c10 = &tempv[rows * 9];
              c11 = &tempv[rows * 10];
              c12 = &tempv[rows * 11];
              c13 = &tempv[rows * 12];
              c14 = &tempv[rows * 13];
              c15 = &tempv[rows * 14];
              c16 = &tempv[rows * 15];
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
              for (int j = 0; j < rows; j++) {
                tempu[index++] = c1[j];
                tempu[index++] = c2[j];
                tempu[index++] = c3[j];
                tempu[index++] = c4[j];
                tempu[index++] = c5[j];
                tempu[index++] = c6[j];
                tempu[index++] = c7[j];
                tempu[index++] = c8[j];
                tempu[index++] = c9[j];
                tempu[index++] = c10[j];
                tempu[index++] = c11[j];
                tempu[index++] = c12[j];
                tempu[index++] = c13[j];
                tempu[index++] = c14[j];
                tempu[index++] = c15[j];
                tempu[index++] = c16[j];
              }
              index = 0;
              for (int d = 0; d < frame_size / (mod * 2); d++) {
                pack = 0;
                for (int e = 0; e < (mod * 2); e++) {
                  offset = mux[e];
                  pack |= tempu[index++] << (((mod * 2) - 1) - offset);
                }
                out[produced++] = pack >> 8;
                out[produced++] = pack & 0xff;
                consumed += (mod * 2);
              }
            }
          }
          else {
            for (int i = 0; i < noutput_items; i += packed_items) {
              rows = frame_size / mod;
              const unsigned char *c1, *c2, *c3, *c4, *c5, *c6, *c7, *c8;
              c1 = &tempv[0];
              c2 = &tempv[rows];
              c3 = &tempv[rows * 2];
              c4 = &tempv[rows * 3];
              c5 = &tempv[rows * 4];
              c6 = &tempv[rows * 5];
              c7 = &tempv[rows * 6];
              c8 = &tempv[rows * 7];
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
              index = 0;
              for (int d = 0; d < frame_size / mod; d++) {
                pack = 0;
                for (int e = 0; e < mod; e++) {
                  offset = mux[e];
                  pack |= tempu[index++] << ((mod - 1) - offset);
                }
                out[produced++] = pack & 0xff;
                consumed += mod;
              }
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
      70, 149, 136, 153, 104, 110, 134, 61, 129, 126, 58, 150, 177, 168, 78, 71, 120, 60, 155, 175, 9, 161, 103,
      123, 91, 173, 57, 106, 143, 151, 89, 86, 35, 77, 133, 31, 7, 23, 51, 5, 121, 83, 64, 176, 119, 98,
      49, 130, 128, 79, 162, 32, 172, 87, 131, 45, 114, 93, 96, 39, 68, 105, 85, 109, 13, 33, 145, 18, 12,
      54, 111, 14, 156, 8, 16, 73, 2, 84, 47, 42, 101, 63, 88, 25, 52, 170, 24, 69, 142, 178, 20, 65,
      97, 66, 80, 11, 59, 19, 115, 154, 26, 147, 28, 50, 160, 102, 55, 139, 125, 116, 138, 167, 53, 169, 165,
      99, 159, 148, 179, 0, 146, 90, 6, 100, 74, 117, 48, 75, 135, 41, 137, 76, 92, 164, 113, 152, 72, 36,
      3, 163, 15, 46, 21, 44, 108, 34, 56, 140, 127, 158, 94, 67, 122, 1, 27, 171, 30, 157, 112, 81, 118,
      43, 29, 124, 22, 62, 37, 40, 4, 107, 166, 82, 95, 10, 144, 141, 132, 174, 38, 17
    };

    const int interleaver_bb_impl::group_tab_3_15N_QPSK[180] = {
      75, 170, 132, 174, 7, 111, 30, 4, 49, 133, 50, 160, 92, 106, 27, 126, 116, 178, 41, 166, 88, 84, 80,
      153, 103, 51, 58, 107, 167, 39, 108, 24, 145, 96, 74, 65, 8, 40, 76, 140, 44, 68, 125, 119, 82, 53,
      152, 102, 38, 28, 86, 162, 171, 61, 93, 147, 117, 32, 150, 26, 59, 3, 148, 173, 141, 130, 154, 97, 33,
      172, 115, 118, 127, 6, 16, 0, 143, 9, 100, 67, 98, 110, 2, 169, 47, 83, 164, 155, 123, 159, 42, 105,
      12, 158, 81, 20, 66, 57, 121, 25, 1, 90, 175, 35, 60, 79, 87, 135, 10, 139, 156, 177, 77, 89, 73,
      113, 52, 109, 134, 36, 176, 54, 69, 146, 31, 15, 71, 18, 95, 124, 85, 14, 78, 129, 161, 19, 72, 13,
      122, 21, 63, 137, 120, 144, 91, 157, 48, 34, 46, 22, 29, 104, 45, 56, 151, 62, 43, 94, 163, 99, 64,
      138, 101, 23, 11, 17, 136, 128, 114, 112, 165, 5, 142, 179, 37, 70, 131, 55, 168, 149
    };

    const int interleaver_bb_impl::group_tab_4_15N_QPSK[180] = {
      141, 86, 22, 20, 176, 21, 37, 82, 6, 122, 130, 40, 62, 44, 24, 117, 8, 145, 36, 79, 172, 149, 127,
      163, 9, 160, 73, 100, 16, 153, 124, 110, 49, 154, 152, 4, 168, 54, 177, 158, 113, 57, 2, 102, 161, 147,
      18, 103, 1, 41, 104, 144, 39, 105, 131, 77, 69, 108, 159, 61, 45, 156, 0, 83, 157, 119, 112, 118, 92,
      109, 75, 67, 142, 96, 51, 139, 31, 166, 179, 89, 167, 23, 34, 60, 93, 165, 128, 90, 19, 33, 70, 173,
      174, 129, 55, 98, 88, 97, 146, 123, 84, 111, 132, 71, 140, 136, 10, 115, 63, 46, 42, 50, 138, 81, 59,
      53, 15, 52, 72, 164, 150, 29, 17, 91, 101, 14, 38, 35, 66, 64, 7, 125, 151, 56, 126, 171, 68, 121,
      28, 65, 106, 78, 47, 143, 12, 169, 120, 27, 74, 48, 133, 43, 116, 137, 94, 3, 25, 134, 13, 107, 162,
      32, 99, 85, 175, 80, 170, 5, 135, 178, 11, 26, 76, 95, 87, 155, 58, 30, 148, 114
    };

    const int interleaver_bb_impl::group_tab_5_15N_QPSK[180] = {
      39, 47, 96, 176, 33, 75, 165, 38, 27, 58, 90, 76, 17, 46, 10, 91, 133, 69, 171, 32, 117, 78, 13,
      146, 101, 36, 0, 138, 25, 77, 122, 49, 14, 125, 140, 93, 130, 2, 104, 102, 128, 4, 111, 151, 84, 167,
      35, 127, 156, 55, 82, 85, 66, 114, 8, 147, 115, 113, 5, 31, 100, 106, 48, 52, 67, 107, 18, 126, 112,
      50, 9, 143, 28, 160, 71, 79, 43, 98, 86, 94, 64, 3, 166, 105, 103, 118, 63, 51, 139, 172, 141, 175,
      56, 74, 95, 29, 45, 129, 120, 168, 92, 150, 7, 162, 153, 137, 108, 159, 157, 173, 23, 89, 132, 57, 37,
      70, 134, 40, 21, 149, 80, 1, 121, 59, 110, 142, 152, 15, 154, 145, 12, 170, 54, 155, 99, 22, 123, 72,
      177, 131, 116, 44, 158, 73, 11, 65, 164, 119, 174, 34, 83, 53, 24, 42, 60, 26, 161, 68, 178, 41, 148,
      109, 87, 144, 135, 20, 62, 81, 169, 124, 6, 19, 30, 163, 61, 179, 136, 97, 16, 88
    };

    const int interleaver_bb_impl::group_tab_6_15N_QPSK[180] = {
      0, 14, 19, 21, 2, 11, 22, 9, 8, 7, 16, 3, 26, 24, 27, 80, 100, 121, 107, 31, 36, 42, 46,
      49, 75, 93, 127, 95, 119, 73, 61, 63, 117, 89, 99, 129, 52, 111, 124, 48, 122, 82, 106, 91, 92, 71,
      103, 102, 81, 113, 101, 97, 33, 115, 59, 112, 90, 51, 126, 85, 123, 40, 83, 53, 69, 70, 132, 134, 136,
      138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 4, 5,
      10, 12, 20, 6, 18, 13, 17, 15, 1, 29, 28, 23, 25, 67, 116, 66, 104, 44, 50, 47, 84, 76, 65,
      130, 56, 128, 77, 39, 94, 87, 120, 62, 88, 74, 35, 110, 131, 98, 60, 37, 45, 78, 125, 41, 34, 118,
      38, 72, 108, 58, 43, 109, 57, 105, 68, 86, 79, 96, 32, 114, 64, 55, 30, 54, 133, 135, 137, 139, 141,
      143, 145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179
    };

    const int interleaver_bb_impl::group_tab_7_15N_QPSK[180] = {
      152, 172, 113, 167, 100, 163, 159, 144, 114, 47, 161, 125, 99, 89, 179, 123, 149, 177, 1, 132, 37, 26, 16,
      57, 166, 81, 133, 112, 33, 151, 117, 83, 52, 178, 85, 124, 143, 28, 59, 130, 31, 157, 170, 44, 61, 102,
      155, 111, 153, 55, 54, 176, 17, 68, 169, 20, 104, 38, 147, 7, 174, 6, 90, 15, 56, 120, 13, 34, 48,
      122, 110, 154, 76, 64, 75, 84, 162, 77, 103, 156, 128, 150, 87, 27, 42, 3, 23, 96, 171, 145, 91, 24,
      78, 5, 69, 175, 8, 29, 106, 137, 131, 43, 93, 160, 108, 164, 12, 140, 71, 63, 141, 109, 129, 82, 80,
      173, 105, 9, 66, 65, 92, 32, 41, 72, 74, 4, 36, 94, 67, 158, 10, 88, 142, 45, 126, 2, 86, 118,
      73, 79, 121, 148, 95, 70, 51, 53, 21, 115, 135, 25, 168, 11, 136, 18, 138, 134, 119, 146, 0, 97, 22,
      165, 40, 19, 60, 46, 14, 49, 139, 58, 101, 39, 116, 127, 30, 98, 50, 107, 35, 62
    };

    const int interleaver_bb_impl::group_tab_8_15N_QPSK[180] = {
      0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44,
      46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90,
      92, 94, 96, 98, 100, 102, 104, 106, 108, 110, 112, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 136,
      138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 1, 3,
      5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 47, 49,
      51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71, 73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95,
      97, 99, 101, 103, 105, 107, 109, 111, 113, 115, 117, 119, 121, 123, 125, 127, 129, 131, 133, 135, 137, 139, 141,
      143, 145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179
    };

    const int interleaver_bb_impl::group_tab_9_15N_QPSK[180] = {
      0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44,
      46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90,
      92, 94, 96, 98, 100, 102, 104, 106, 108, 110, 112, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 136,
      138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 1, 3,
      5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 47, 49,
      51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71, 73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95,
      97, 99, 101, 103, 105, 107, 109, 111, 113, 115, 117, 119, 121, 123, 125, 127, 129, 131, 133, 135, 137, 139, 141,
      143, 145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179
    };

    const int interleaver_bb_impl::group_tab_10_15N_QPSK[180] = {
      0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44,
      46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90,
      92, 94, 96, 98, 100, 102, 104, 106, 108, 110, 112, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 136,
      138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 1, 3,
      5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 47, 49,
      51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71, 73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95,
      97, 99, 101, 103, 105, 107, 109, 111, 113, 115, 117, 119, 121, 123, 125, 127, 129, 131, 133, 135, 137, 139, 141,
      143, 145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179
    };

    const int interleaver_bb_impl::group_tab_11_15N_QPSK[180] = {
      0, 14, 19, 21, 2, 11, 22, 9, 8, 7, 16, 3, 26, 24, 27, 80, 100, 121, 107, 31, 36, 42, 46,
      49, 75, 93, 127, 95, 119, 73, 61, 63, 117, 89, 99, 129, 52, 111, 124, 48, 122, 82, 106, 91, 92, 71,
      103, 102, 81, 113, 101, 97, 33, 115, 59, 112, 90, 51, 126, 85, 123, 40, 83, 53, 69, 70, 132, 134, 136,
      138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 4, 5,
      10, 12, 20, 6, 18, 13, 17, 15, 1, 29, 28, 23, 25, 67, 116, 66, 104, 44, 50, 47, 84, 76, 65,
      130, 56, 128, 77, 39, 94, 87, 120, 62, 88, 74, 35, 110, 131, 98, 60, 37, 45, 78, 125, 41, 34, 118,
      38, 72, 108, 58, 43, 109, 57, 105, 68, 86, 79, 96, 32, 114, 64, 55, 30, 54, 133, 135, 137, 139, 141,
      143, 145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179
    };

    const int interleaver_bb_impl::group_tab_12_15N_QPSK[180] = {
      0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44,
      46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90,
      92, 94, 96, 98, 100, 102, 104, 106, 108, 110, 112, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 136,
      138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 1, 3,
      5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 47, 49,
      51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71, 73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95,
      97, 99, 101, 103, 105, 107, 109, 111, 113, 115, 117, 119, 121, 123, 125, 127, 129, 131, 133, 135, 137, 139, 141,
      143, 145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179
    };

    const int interleaver_bb_impl::group_tab_13_15N_QPSK[180] = {
      0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44,
      46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90,
      92, 94, 96, 98, 100, 102, 104, 106, 108, 110, 112, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 136,
      138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 1, 3,
      5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 47, 49,
      51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71, 73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95,
      97, 99, 101, 103, 105, 107, 109, 111, 113, 115, 117, 119, 121, 123, 125, 127, 129, 131, 133, 135, 137, 139, 141,
      143, 145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179
    };

    const int interleaver_bb_impl::group_tab_2_15S_QPSK[45] = {};
    const int interleaver_bb_impl::group_tab_3_15S_QPSK[45] = {};
    const int interleaver_bb_impl::group_tab_4_15S_QPSK[45] = {};
    const int interleaver_bb_impl::group_tab_5_15S_QPSK[45] = {};
    const int interleaver_bb_impl::group_tab_6_15S_QPSK[45] = {};
    const int interleaver_bb_impl::group_tab_7_15S_QPSK[45] = {};
    const int interleaver_bb_impl::group_tab_8_15S_QPSK[45] = {};
    const int interleaver_bb_impl::group_tab_9_15S_QPSK[45] = {};
    const int interleaver_bb_impl::group_tab_10_15S_QPSK[45] = {};
    const int interleaver_bb_impl::group_tab_11_15S_QPSK[45] = {};
    const int interleaver_bb_impl::group_tab_12_15S_QPSK[45] = {};
    const int interleaver_bb_impl::group_tab_13_15S_QPSK[45] = {};

    const int interleaver_bb_impl::group_tab_2_15N_16QAM[180] = {};
    const int interleaver_bb_impl::group_tab_3_15N_16QAM[180] = {};
    const int interleaver_bb_impl::group_tab_4_15N_16QAM[180] = {};
    const int interleaver_bb_impl::group_tab_5_15N_16QAM[180] = {};
    const int interleaver_bb_impl::group_tab_6_15N_16QAM[180] = {};
    const int interleaver_bb_impl::group_tab_7_15N_16QAM[180] = {};
    const int interleaver_bb_impl::group_tab_8_15N_16QAM[180] = {};
    const int interleaver_bb_impl::group_tab_9_15N_16QAM[180] = {};
    const int interleaver_bb_impl::group_tab_10_15N_16QAM[180] = {};
    const int interleaver_bb_impl::group_tab_11_15N_16QAM[180] = {};
    const int interleaver_bb_impl::group_tab_12_15N_16QAM[180] = {};
    const int interleaver_bb_impl::group_tab_13_15N_16QAM[180] = {};

    const int interleaver_bb_impl::group_tab_2_15S_16QAM[45] = {};
    const int interleaver_bb_impl::group_tab_3_15S_16QAM[45] = {};
    const int interleaver_bb_impl::group_tab_4_15S_16QAM[45] = {};
    const int interleaver_bb_impl::group_tab_5_15S_16QAM[45] = {};
    const int interleaver_bb_impl::group_tab_6_15S_16QAM[45] = {};
    const int interleaver_bb_impl::group_tab_7_15S_16QAM[45] = {};
    const int interleaver_bb_impl::group_tab_8_15S_16QAM[45] = {};
    const int interleaver_bb_impl::group_tab_9_15S_16QAM[45] = {};
    const int interleaver_bb_impl::group_tab_10_15S_16QAM[45] = {};
    const int interleaver_bb_impl::group_tab_11_15S_16QAM[45] = {};
    const int interleaver_bb_impl::group_tab_12_15S_16QAM[45] = {};
    const int interleaver_bb_impl::group_tab_13_15S_16QAM[45] = {};

    const int interleaver_bb_impl::group_tab_2_15N_64QAM[180] = {};
    const int interleaver_bb_impl::group_tab_3_15N_64QAM[180] = {};
    const int interleaver_bb_impl::group_tab_4_15N_64QAM[180] = {};
    const int interleaver_bb_impl::group_tab_5_15N_64QAM[180] = {};
    const int interleaver_bb_impl::group_tab_6_15N_64QAM[180] = {};
    const int interleaver_bb_impl::group_tab_7_15N_64QAM[180] = {};
    const int interleaver_bb_impl::group_tab_8_15N_64QAM[180] = {};
    const int interleaver_bb_impl::group_tab_9_15N_64QAM[180] = {};
    const int interleaver_bb_impl::group_tab_10_15N_64QAM[180] = {};
    const int interleaver_bb_impl::group_tab_11_15N_64QAM[180] = {};
    const int interleaver_bb_impl::group_tab_12_15N_64QAM[180] = {};
    const int interleaver_bb_impl::group_tab_13_15N_64QAM[180] = {};

    const int interleaver_bb_impl::group_tab_2_15S_64QAM[45] = {};
    const int interleaver_bb_impl::group_tab_3_15S_64QAM[45] = {};
    const int interleaver_bb_impl::group_tab_4_15S_64QAM[45] = {};
    const int interleaver_bb_impl::group_tab_5_15S_64QAM[45] = {};
    const int interleaver_bb_impl::group_tab_6_15S_64QAM[45] = {};
    const int interleaver_bb_impl::group_tab_7_15S_64QAM[45] = {};
    const int interleaver_bb_impl::group_tab_8_15S_64QAM[45] = {};
    const int interleaver_bb_impl::group_tab_9_15S_64QAM[45] = {};
    const int interleaver_bb_impl::group_tab_10_15S_64QAM[45] = {};
    const int interleaver_bb_impl::group_tab_11_15S_64QAM[45] = {};
    const int interleaver_bb_impl::group_tab_12_15S_64QAM[45] = {};
    const int interleaver_bb_impl::group_tab_13_15S_64QAM[45] = {};

    const int interleaver_bb_impl::group_tab_2_15N_256QAM[180] = {};
    const int interleaver_bb_impl::group_tab_3_15N_256QAM[180] = {};
    const int interleaver_bb_impl::group_tab_4_15N_256QAM[180] = {};
    const int interleaver_bb_impl::group_tab_5_15N_256QAM[180] = {};
    const int interleaver_bb_impl::group_tab_6_15N_256QAM[180] = {};
    const int interleaver_bb_impl::group_tab_7_15N_256QAM[180] = {};
    const int interleaver_bb_impl::group_tab_8_15N_256QAM[180] = {};
    const int interleaver_bb_impl::group_tab_9_15N_256QAM[180] = {};
    const int interleaver_bb_impl::group_tab_10_15N_256QAM[180] = {};
    const int interleaver_bb_impl::group_tab_11_15N_256QAM[180] = {};
    const int interleaver_bb_impl::group_tab_12_15N_256QAM[180] = {};
    const int interleaver_bb_impl::group_tab_13_15N_256QAM[180] = {};

    const int interleaver_bb_impl::group_tab_2_15S_256QAM[45] = {};
    const int interleaver_bb_impl::group_tab_3_15S_256QAM[45] = {};
    const int interleaver_bb_impl::group_tab_4_15S_256QAM[45] = {};
    const int interleaver_bb_impl::group_tab_5_15S_256QAM[45] = {};
    const int interleaver_bb_impl::group_tab_6_15S_256QAM[45] = {};
    const int interleaver_bb_impl::group_tab_7_15S_256QAM[45] = {};
    const int interleaver_bb_impl::group_tab_8_15S_256QAM[45] = {};
    const int interleaver_bb_impl::group_tab_9_15S_256QAM[45] = {};
    const int interleaver_bb_impl::group_tab_10_15S_256QAM[45] = {};
    const int interleaver_bb_impl::group_tab_11_15S_256QAM[45] = {};
    const int interleaver_bb_impl::group_tab_12_15S_256QAM[45] = {};
    const int interleaver_bb_impl::group_tab_13_15S_256QAM[45] = {};


  } /* namespace atsc3 */
} /* namespace gr */
