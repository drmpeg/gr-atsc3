/* -*- c++ -*- */
/*
 * Copyright 2021-2023 Ron Economos.
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

    enum atsc3_miso_tx_t {
      MISO_TX_1_OF_2 = 0,
      MISO_TX_2_OF_2,
      MISO_TX_1_OF_3,
      MISO_TX_2_OF_3,
      MISO_TX_3_OF_3,
      MISO_TX_1_OF_4,
      MISO_TX_2_OF_4,
      MISO_TX_3_OF_4,
      MISO_TX_4_OF_4,
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

    enum atsc3_scattered_pilot_boost_t {
      SPB_0 = 0,
      SPB_1,
      SPB_2,
      SPB_3,
      SPB_4,
    };

    enum atsc3_l1_select_t {
      L1_BASIC = 0,
      L1_DETAIL,
    };

    enum atsc3_frequency_interleaver_t {
      FREQ_PREAMBLE_ONLY = 0,
      FREQ_ALL_SYMBOLS,
    };

    enum atsc3_showlevels_t {
      SHOWLEVELS_OFF = 0,
      SHOWLEVELS_ON,
    };

    enum atsc3_min_time_to_next_t {
      MTTN_50 = 0,
      MTTN_100,
      MTTN_150,
      MTTN_200,
      MTTN_250,
      MTTN_300,
      MTTN_350,
      MTTN_400,
      MTTN_500,
      MTTN_600,
      MTTN_700,
      MTTN_800,
      MTTN_900,
      MTTN_1000,
      MTTN_1100,
      MTTN_1200,
      MTTN_1300,
      MTTN_1500,
      MTTN_1700,
      MTTN_1900,
      MTTN_2100,
      MTTN_2300,
      MTTN_2500,
      MTTN_2700,
      MTTN_2900,
      MTTN_3300,
      MTTN_3700,
      MTTN_4100,
      MTTN_4500,
      MTTN_4900,
      MTTN_5300,
    };

    enum atsc3_bootstrap_mode_t {
      BOOTSTRAP_VALIDATION = 0,
      BOOTSTRAP_INTERPOLATION,
    };

    enum atsc3_time_interleaver_mode_t {
      TI_MODE_OFF = 0,
      TI_MODE_CONVOLUTIONAL,
      TI_MODE_HYBRID,
      TI_MODE_RESERVED,
    };

    enum atsc3_time_interleaver_depth_t {
      TI_DEPTH_512 = 0,
      TI_DEPTH_724,
      TI_DEPTH_887,
      TI_DEPTH_1024,
      TI_DEPTH_1254,
      TI_DEPTH_1448,
    };

    enum atsc3_plp_fec_mode_t {
      PLP_FEC_NONE = 0,
      PLP_FEC_CRC32,
      PLP_FEC_BCH,
    };

    enum atsc3_ldm_injection_level_t {
      LDM_LEVEL_00DB = 0,
      LDM_LEVEL_05DB,
      LDM_LEVEL_10DB,
      LDM_LEVEL_15DB,
      LDM_LEVEL_20DB,
      LDM_LEVEL_25DB,
      LDM_LEVEL_30DB,
      LDM_LEVEL_35DB,
      LDM_LEVEL_40DB,
      LDM_LEVEL_45DB,
      LDM_LEVEL_50DB,
      LDM_LEVEL_60DB,
      LDM_LEVEL_70DB,
      LDM_LEVEL_80DB,
      LDM_LEVEL_90DB,
      LDM_LEVEL_100DB,
      LDM_LEVEL_110DB,
      LDM_LEVEL_120DB,
      LDM_LEVEL_130DB,
      LDM_LEVEL_140DB,
      LDM_LEVEL_150DB,
      LDM_LEVEL_160DB,
      LDM_LEVEL_170DB,
      LDM_LEVEL_180DB,
      LDM_LEVEL_190DB,
      LDM_LEVEL_200DB,
      LDM_LEVEL_210DB,
      LDM_LEVEL_220DB,
      LDM_LEVEL_230DB,
      LDM_LEVEL_240DB,
      LDM_LEVEL_250DB,
    };

    enum atsc3_pilotgenerator_mode_t {
      PILOTGENERATOR_FREQ = 0,
      PILOTGENERATOR_TIME,
    };

    enum atsc3_cyclicprefixer_mode_t {
      CYCLICPREFIXER_SINGLE_SUBFRAME = 0,
      CYCLICPREFIXER_DUAL_SUBFRAME_PLP0,
      CYCLICPREFIXER_DUAL_SUBFRAME_PLP1,
    };

    enum atsc3_lls_insertion_mode_t {
      LLS_OFF = 0,
      LLS_ON,
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
typedef gr::atsc3::atsc3_miso_tx_t atsc3_miso_tx_t;
typedef gr::atsc3::atsc3_fftsize_t atsc3_fftsize_t;
typedef gr::atsc3::atsc3_pilotpattern_t atsc3_pilotpattern_t;
typedef gr::atsc3::atsc3_reduced_carriers_t atsc3_reduced_carriers_t;
typedef gr::atsc3::atsc3_l1_fec_mode_t atsc3_l1_fec_mode_t;
typedef gr::atsc3::atsc3_fec_type_t atsc3_fec_type_t;
typedef gr::atsc3::atsc3_first_sbs_t atsc3_first_sbs_t;
typedef gr::atsc3::atsc3_scattered_pilot_boost_t atsc3_scattered_pilot_boost_t;
typedef gr::atsc3::atsc3_l1_select_t atsc3_l1_select_t;
typedef gr::atsc3::atsc3_frequency_interleaver_t atsc3_frequency_interleaver_t;
typedef gr::atsc3::atsc3_showlevels_t atsc3_showlevels_t;
typedef gr::atsc3::atsc3_min_time_to_next_t atsc3_min_time_to_next_t;
typedef gr::atsc3::atsc3_bootstrap_mode_t atsc3_bootstrap_mode_t;
typedef gr::atsc3::atsc3_time_interleaver_mode_t atsc3_time_interleaver_mode_t;
typedef gr::atsc3::atsc3_time_interleaver_depth_t atsc3_time_interleaver_depth_t;
typedef gr::atsc3::atsc3_plp_fec_mode_t atsc3_plp_fec_mode_t;
typedef gr::atsc3::atsc3_ldm_injection_level_t atsc3_ldm_injection_level_t;
typedef gr::atsc3::atsc3_pilotgenerator_mode_t atsc3_pilotgenerator_mode_t;
typedef gr::atsc3::atsc3_cyclicprefixer_mode_t atsc3_cyclicprefixer_mode_t;
typedef gr::atsc3::atsc3_lls_insertion_mode_t atsc3_lls_insertion_mode_t;

#endif /* INCLUDED_ATSC3_CONFIG_H */
