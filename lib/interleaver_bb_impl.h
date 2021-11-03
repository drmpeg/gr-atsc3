/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_INTERLEAVER_BB_IMPL_H
#define INCLUDED_ATSC3_INTERLEAVER_BB_IMPL_H

#include <atsc3/interleaver_bb.h>
#include "atsc3_defines.h"

namespace gr {
  namespace atsc3 {

    class interleaver_bb_impl : public interleaver_bb
    {
     private:
      int frame_size;
      int group_size;
      int signal_constellation;
      int code_rate;
      int ldpc_type;
      int block_type;
      int nbch;
      int q_val;
      int mod;
      int packed_items;
      int nr2;
      int npart2;
      unsigned char tempu[FRAME_SIZE_NORMAL];
      unsigned char tempv[FRAME_SIZE_NORMAL];

      const int *group_table;

      const static int group_tab_2_15N_QPSK[180];
      const static int group_tab_3_15N_QPSK[180];
      const static int group_tab_4_15N_QPSK[180];
      const static int group_tab_5_15N_QPSK[180];
      const static int group_tab_6_15N_QPSK[180];
      const static int group_tab_7_15N_QPSK[180];
      const static int group_tab_8_15N_QPSK[180];
      const static int group_tab_9_15N_QPSK[180];
      const static int group_tab_10_15N_QPSK[180];
      const static int group_tab_11_15N_QPSK[180];
      const static int group_tab_12_15N_QPSK[180];
      const static int group_tab_13_15N_QPSK[180];

      const static int group_tab_2_15S_QPSK[45];
      const static int group_tab_3_15S_QPSK[45];
      const static int group_tab_4_15S_QPSK[45];
      const static int group_tab_5_15S_QPSK[45];
      const static int group_tab_6_15S_QPSK[45];
      const static int group_tab_7_15S_QPSK[45];
      const static int group_tab_8_15S_QPSK[45];
      const static int group_tab_9_15S_QPSK[45];
      const static int group_tab_10_15S_QPSK[45];
      const static int group_tab_11_15S_QPSK[45];
      const static int group_tab_12_15S_QPSK[45];
      const static int group_tab_13_15S_QPSK[45];

      const static int group_tab_2_15N_16QAM[180];
      const static int group_tab_3_15N_16QAM[180];
      const static int group_tab_4_15N_16QAM[180];
      const static int group_tab_5_15N_16QAM[180];
      const static int group_tab_6_15N_16QAM[180];
      const static int group_tab_7_15N_16QAM[180];
      const static int group_tab_8_15N_16QAM[180];
      const static int group_tab_9_15N_16QAM[180];
      const static int group_tab_10_15N_16QAM[180];
      const static int group_tab_11_15N_16QAM[180];
      const static int group_tab_12_15N_16QAM[180];
      const static int group_tab_13_15N_16QAM[180];

      const static int group_tab_2_15S_16QAM[45];
      const static int group_tab_3_15S_16QAM[45];
      const static int group_tab_4_15S_16QAM[45];
      const static int group_tab_5_15S_16QAM[45];
      const static int group_tab_6_15S_16QAM[45];
      const static int group_tab_7_15S_16QAM[45];
      const static int group_tab_8_15S_16QAM[45];
      const static int group_tab_9_15S_16QAM[45];
      const static int group_tab_10_15S_16QAM[45];
      const static int group_tab_11_15S_16QAM[45];
      const static int group_tab_12_15S_16QAM[45];
      const static int group_tab_13_15S_16QAM[45];

      const static int group_tab_2_15N_64QAM[180];
      const static int group_tab_3_15N_64QAM[180];
      const static int group_tab_4_15N_64QAM[180];
      const static int group_tab_5_15N_64QAM[180];
      const static int group_tab_6_15N_64QAM[180];
      const static int group_tab_7_15N_64QAM[180];
      const static int group_tab_8_15N_64QAM[180];
      const static int group_tab_9_15N_64QAM[180];
      const static int group_tab_10_15N_64QAM[180];
      const static int group_tab_11_15N_64QAM[180];
      const static int group_tab_12_15N_64QAM[180];
      const static int group_tab_13_15N_64QAM[180];

      const static int group_tab_2_15S_64QAM[45];
      const static int group_tab_3_15S_64QAM[45];
      const static int group_tab_4_15S_64QAM[45];
      const static int group_tab_5_15S_64QAM[45];
      const static int group_tab_6_15S_64QAM[45];
      const static int group_tab_7_15S_64QAM[45];
      const static int group_tab_8_15S_64QAM[45];
      const static int group_tab_9_15S_64QAM[45];
      const static int group_tab_10_15S_64QAM[45];
      const static int group_tab_11_15S_64QAM[45];
      const static int group_tab_12_15S_64QAM[45];
      const static int group_tab_13_15S_64QAM[45];

      const static int group_tab_2_15N_256QAM[180];
      const static int group_tab_3_15N_256QAM[180];
      const static int group_tab_4_15N_256QAM[180];
      const static int group_tab_5_15N_256QAM[180];
      const static int group_tab_6_15N_256QAM[180];
      const static int group_tab_7_15N_256QAM[180];
      const static int group_tab_8_15N_256QAM[180];
      const static int group_tab_9_15N_256QAM[180];
      const static int group_tab_10_15N_256QAM[180];
      const static int group_tab_11_15N_256QAM[180];
      const static int group_tab_12_15N_256QAM[180];
      const static int group_tab_13_15N_256QAM[180];

      const static int group_tab_2_15S_256QAM[45];
      const static int group_tab_3_15S_256QAM[45];
      const static int group_tab_4_15S_256QAM[45];
      const static int group_tab_5_15S_256QAM[45];
      const static int group_tab_6_15S_256QAM[45];
      const static int group_tab_7_15S_256QAM[45];
      const static int group_tab_8_15S_256QAM[45];
      const static int group_tab_9_15S_256QAM[45];
      const static int group_tab_10_15S_256QAM[45];
      const static int group_tab_11_15S_256QAM[45];
      const static int group_tab_12_15S_256QAM[45];
      const static int group_tab_13_15S_256QAM[45];

     public:
      interleaver_bb_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation);
      ~interleaver_bb_impl();

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_INTERLEAVER_BB_IMPL_H */
