/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef INCLUDED_ATSC3_CONFIG_H
#define INCLUDED_ATSC3_CONFIG_H

namespace gr {
  namespace atsc3 {

    enum atsc3_code_rate_t {
      C2_15 = 0,
      C3_15,
      C4_15,
      C5_15,
      C6_15,
      C7_15,
      C8_15,
      C9_15,
      C10_15,
      C11_15,
      C12_15,
      C13_15,
    };

    enum atsc3_framesize_t {
      FECFRAME_SHORT = 0,
      FECFRAME_NORMAL,
    };

    enum atsc3_constellation_t {
      MOD_QPSK = 0,
      MOD_16QAM,
      MOD_64QAM,
      MOD_256QAM,
      MOD_1024QAM,
      MOD_4096QAM,
    };

    enum atsc3_guardinterval_t {
      GI_1_192 = 0,
      GI_2_384,
      GI_3_512,
      GI_4_768,
      GI_5_1024,
      GI_6_1536,
      GI_7_2048,
      GI_8_2432,
      GI_9_3072,
      GI_10_3648,
      GI_11_4096,
      GI_12_4864,
    };

  } // namespace atsc3
} // namespace gr

typedef gr::atsc3::atsc3_code_rate_t atsc3_code_rate_t;
typedef gr::atsc3::atsc3_framesize_t atsc3_framesize_t;
typedef gr::atsc3::atsc3_constellation_t atsc3_constellation_t;
typedef gr::atsc3::atsc3_guardinterval_t atsc3_guardinterval_t;

#endif /* INCLUDED_ATSC3_CONFIG_H */
