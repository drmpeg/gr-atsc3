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
      GI_RESERVED = 0,
      GI_1_192,
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

    enum atsc3_mimo_scattered_pilot_encoding_t {
      MSPE_WALSH_HADAMARD_PILOTS = 0,
      MSPE_NULL_PILOTS,
    };

    enum atsc3_time_info_flag_t {
      TIF_NOT_INCLUDED = 0,
      TIF_MS_PRECISION,
      TIF_US_PRECISION,
      TIF_NS_PRECISION,
    };

    enum atsc3_papr_t {
      PAPR_OFF = 0,
      PAPR_TR,
      PAPR_ACE,
      PAPR_BOTH,
    };

    enum atsc3_frame_length_mode_t {
      FLM_TIME_ALIGNED = 0,
      FLM_SYMBOL_ALIGNED,
    };

    enum atsc3_additional_parity_mode_t {
      APM_K0 = 0,
      APM_K1,
      APM_K2,
    };

    enum atsc3_miso_t {
      MISO_OFF = 0,
      MISO_64,
      MISO_256,
    };

    enum atsc3_fftsize_t {
      FFTSIZE_8K = 0,
      FFTSIZE_16K,
      FFTSIZE_32K,
    };

    enum atsc3_pilotpattern_t {
      PILOT_SP3_2 = 0,
      PILOT_SP3_4,
      PILOT_SP4_2,
      PILOT_SP4_4,
      PILOT_SP6_2,
      PILOT_SP6_4,
      PILOT_SP8_2,
      PILOT_SP8_4,
      PILOT_SP12_2,
      PILOT_SP12_4,
      PILOT_SP16_2,
      PILOT_SP16_4,
      PILOT_SP24_2,
      PILOT_SP24_4,
      PILOT_SP32_2,
      PILOT_SP32_4,
    };

    enum atsc3_reduced_carriers_t {
      CRED_0 = 0,
      CRED_1,
      CRED_2,
      CRED_3,
      CRED_4,
    };

    enum atsc3_l1_fec_mode_t {
      L1_FEC_MODE_1 = 0,
      L1_FEC_MODE_2,
      L1_FEC_MODE_3,
      L1_FEC_MODE_4,
      L1_FEC_MODE_5,
      L1_FEC_MODE_6,
      L1_FEC_MODE_7,
    };

    enum atsc3_fec_type_t {
      FEC_TYPE_BCH_16K = 0,
      FEC_TYPE_BCH_64K,
      FEC_TYPE_CRC_16K,
      FEC_TYPE_CRC_64K,
      FEC_TYPE_ONLY_16K,
      FEC_TYPE_ONLY_64K,
    };

    enum atsc3_first_sbs_t {
      SBS_OFF = 0,
      SBS_ON,
    };

  } // namespace atsc3
} // namespace gr

typedef gr::atsc3::atsc3_code_rate_t atsc3_code_rate_t;
typedef gr::atsc3::atsc3_framesize_t atsc3_framesize_t;
typedef gr::atsc3::atsc3_constellation_t atsc3_constellation_t;
typedef gr::atsc3::atsc3_guardinterval_t atsc3_guardinterval_t;
typedef gr::atsc3::atsc3_mimo_scattered_pilot_encoding_t atsc3_mimo_scattered_pilot_encoding_t;
typedef gr::atsc3::atsc3_time_info_flag_t atsc3_time_info_flag_t;
typedef gr::atsc3::atsc3_papr_t atsc3_papr_t;
typedef gr::atsc3::atsc3_frame_length_mode_t atsc3_frame_length_mode_t;
typedef gr::atsc3::atsc3_additional_parity_mode_t atsc3_additional_parity_mode_t;
typedef gr::atsc3::atsc3_miso_t atsc3_miso_t;
typedef gr::atsc3::atsc3_fftsize_t atsc3_fftsize_t;
typedef gr::atsc3::atsc3_pilotpattern_t atsc3_pilotpattern_t;
typedef gr::atsc3::atsc3_reduced_carriers_t atsc3_reduced_carriers_t;
typedef gr::atsc3::atsc3_l1_fec_mode_t atsc3_l1_fec_mode_t;
typedef gr::atsc3::atsc3_fec_type_t atsc3_fec_type_t;
typedef gr::atsc3::atsc3_first_sbs_t atsc3_first_sbs_t;

#endif /* INCLUDED_ATSC3_CONFIG_H */
