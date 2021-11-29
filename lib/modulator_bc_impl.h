/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_MODULATOR_BC_IMPL_H
#define INCLUDED_ATSC3_MODULATOR_BC_IMPL_H

#include <atsc3/modulator_bc.h>
#include "atsc3_defines.h"

namespace gr {
  namespace atsc3 {

    class modulator_bc_impl : public modulator_bc
    {
     private:
      int signal_constellation;
      int cell_size;
      gr_complex m_qpsk[4];
      gr_complex m_16qam[16];
      gr_complex m_64qam[64];
      gr_complex m_256qam[256];
      const float *m_1024qam;
      const float *m_4096qam;

      const static gr_complex mod_table_16QAM[12][4];
      const static gr_complex mod_table_64QAM[12][16];
      const static gr_complex mod_table_256QAM[12][64];
      const static float mod_table_1024QAM[12][16];
      const static float mod_table_4096QAM[12][32];
      const static int map_table_1024QAM[32];
      const static int map_table_4096QAM[64];

     public:
      modulator_bc_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation);
      ~modulator_bc_impl();

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_MODULATOR_BC_IMPL_H */
