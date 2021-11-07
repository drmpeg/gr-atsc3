/*
 * Copyright 2021 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(atsc3_config.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(214f946c6ac6efe0137fe08cc68a182f)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <atsc3/atsc3_config.h>
// pydoc.h is automatically generated in the build directory
#include <atsc3_config_pydoc.h>

void bind_atsc3_config(py::module& m)
{


    py::enum_<::gr::atsc3::atsc3_code_rate_t>(m,"atsc3_code_rate_t")
        .value("C2_15", ::gr::atsc3::C2_15) // 0
        .value("C3_15", ::gr::atsc3::C3_15) // 1
        .value("C4_15", ::gr::atsc3::C4_15) // 2
        .value("C5_15", ::gr::atsc3::C5_15) // 3
        .value("C6_15", ::gr::atsc3::C6_15) // 4
        .value("C7_15", ::gr::atsc3::C7_15) // 5
        .value("C8_15", ::gr::atsc3::C8_15) // 6
        .value("C9_15", ::gr::atsc3::C9_15) // 7
        .value("C10_15", ::gr::atsc3::C10_15) // 8
        .value("C11_15", ::gr::atsc3::C11_15) // 9
        .value("C12_15", ::gr::atsc3::C12_15) // 10
        .value("C13_15", ::gr::atsc3::C13_15) // 11
        .export_values()
    ;

    py::implicitly_convertible<int, ::gr::atsc3::atsc3_code_rate_t>();
    py::enum_<::gr::atsc3::atsc3_framesize_t>(m,"atsc3_framesize_t")
        .value("FECFRAME_SHORT", ::gr::atsc3::FECFRAME_SHORT) // 0
        .value("FECFRAME_NORMAL", ::gr::atsc3::FECFRAME_NORMAL) // 1
        .export_values()
    ;

    py::implicitly_convertible<int, ::gr::atsc3::atsc3_framesize_t>();
    py::enum_<::gr::atsc3::atsc3_constellation_t>(m,"atsc3_constellation_t")
        .value("MOD_QPSK", ::gr::atsc3::MOD_QPSK) // 0
        .value("MOD_16QAM", ::gr::atsc3::MOD_16QAM) // 1
        .value("MOD_64QAM", ::gr::atsc3::MOD_64QAM) // 2
        .value("MOD_256QAM", ::gr::atsc3::MOD_256QAM) // 3
        .value("MOD_1024QAM", ::gr::atsc3::MOD_1024QAM) // 4
        .value("MOD_4096QAM", ::gr::atsc3::MOD_4096QAM) // 5
        .export_values()
    ;

    py::implicitly_convertible<int, ::gr::atsc3::atsc3_constellation_t>();
    py::enum_<::gr::atsc3::atsc3_guardinterval_t>(m,"atsc3_guardinterval_t")
        .value("GI_RESERVED", ::gr::atsc3::GI_RESERVED) // 0
        .value("GI_1_192", ::gr::atsc3::GI_1_192) // 1
        .value("GI_2_384", ::gr::atsc3::GI_2_384) // 2
        .value("GI_3_512", ::gr::atsc3::GI_3_512) // 3
        .value("GI_4_768", ::gr::atsc3::GI_4_768) // 4
        .value("GI_5_1024", ::gr::atsc3::GI_5_1024) // 5
        .value("GI_6_1536", ::gr::atsc3::GI_6_1536) // 6
        .value("GI_7_2048", ::gr::atsc3::GI_7_2048) // 7
        .value("GI_8_2432", ::gr::atsc3::GI_8_2432) // 8
        .value("GI_9_3072", ::gr::atsc3::GI_9_3072) // 9
        .value("GI_10_3648", ::gr::atsc3::GI_10_3648) // 10
        .value("GI_11_4096", ::gr::atsc3::GI_11_4096) // 11
        .value("GI_12_4864", ::gr::atsc3::GI_12_4864) // 12
        .export_values()
    ;

    py::implicitly_convertible<int, ::gr::atsc3::atsc3_guardinterval_t>();
    py::enum_<::gr::atsc3::atsc3_mimo_scattered_pilot_encoding_t>(m,"atsc3_mimo_scattered_pilot_encoding_t")
        .value("MSPE_WALSH_HADAMARD_PILOTS", ::gr::atsc3::MSPE_WALSH_HADAMARD_PILOTS) // 0
        .value("MSPE_NULL_PILOTS", ::gr::atsc3::MSPE_NULL_PILOTS) // 1
        .export_values()
    ;

    py::implicitly_convertible<int, ::gr::atsc3::atsc3_mimo_scattered_pilot_encoding_t>();
    py::enum_<::gr::atsc3::atsc3_time_info_flag_t>(m,"atsc3_time_info_flag_t")
        .value("TIF_NOT_INCLUDED", ::gr::atsc3::TIF_NOT_INCLUDED) // 0
        .value("TIF_MS_PRECISION", ::gr::atsc3::TIF_MS_PRECISION) // 1
        .value("TIF_US_PRECISION", ::gr::atsc3::TIF_US_PRECISION) // 2
        .value("TIF_NS_PRECISION", ::gr::atsc3::TIF_NS_PRECISION) // 3
        .export_values()
    ;

    py::implicitly_convertible<int, ::gr::atsc3::atsc3_time_info_flag_t>();
    py::enum_<::gr::atsc3::atsc3_papr_t>(m,"atsc3_papr_t")
        .value("PAPR_OFF", ::gr::atsc3::PAPR_OFF) // 0
        .value("PAPR_TR", ::gr::atsc3::PAPR_TR) // 1
        .value("PAPR_ACE", ::gr::atsc3::PAPR_ACE) // 2
        .value("PAPR_BOTH", ::gr::atsc3::PAPR_BOTH) // 3
        .export_values()
    ;

    py::implicitly_convertible<int, ::gr::atsc3::atsc3_papr_t>();
    py::enum_<::gr::atsc3::atsc3_time_length_mode_t>(m,"atsc3_time_length_mode_t")
        .value("TLM_TIME_ALIGNED", ::gr::atsc3::TLM_TIME_ALIGNED) // 0
        .value("TLM_SYMBOL_ALIGNED", ::gr::atsc3::TLM_SYMBOL_ALIGNED) // 1
        .export_values()
    ;

    py::implicitly_convertible<int, ::gr::atsc3::atsc3_time_length_mode_t>();
    py::enum_<::gr::atsc3::atsc3_additional_parity_mode_t>(m,"atsc3_additional_parity_mode_t")
        .value("APM_K0", ::gr::atsc3::APM_K0) // 0
        .value("APM_K1", ::gr::atsc3::APM_K1) // 1
        .value("APM_K2", ::gr::atsc3::APM_K2) // 2
        .export_values()
    ;

    py::implicitly_convertible<int, ::gr::atsc3::atsc3_additional_parity_mode_t>();
    py::enum_<::gr::atsc3::atsc3_miso_t>(m,"atsc3_miso_t")
        .value("MISO_OFF", ::gr::atsc3::MISO_OFF) // 0
        .value("MISO_64", ::gr::atsc3::MISO_64) // 1
        .value("MISO_256", ::gr::atsc3::MISO_256) // 2
        .export_values()
    ;

    py::implicitly_convertible<int, ::gr::atsc3::atsc3_miso_t>();
    py::enum_<::gr::atsc3::atsc3_fftsize_t>(m,"atsc3_fftsize_t")
        .value("FFTSIZE_8K", ::gr::atsc3::FFTSIZE_8K) // 0
        .value("FFTSIZE_16K", ::gr::atsc3::FFTSIZE_16K) // 1
        .value("FFTSIZE_32K", ::gr::atsc3::FFTSIZE_32K) // 2
        .export_values()
    ;

    py::implicitly_convertible<int, ::gr::atsc3::atsc3_fftsize_t>();
    py::enum_<::gr::atsc3::atsc3_pilotpattern_t>(m,"atsc3_pilotpattern_t")
        .value("PILOT_SP3_2", ::gr::atsc3::PILOT_SP3_2) // 0
        .value("PILOT_SP3_4", ::gr::atsc3::PILOT_SP3_4) // 1
        .value("PILOT_SP4_2", ::gr::atsc3::PILOT_SP4_2) // 2
        .value("PILOT_SP4_4", ::gr::atsc3::PILOT_SP4_4) // 3
        .value("PILOT_SP6_2", ::gr::atsc3::PILOT_SP6_2) // 4
        .value("PILOT_SP6_4", ::gr::atsc3::PILOT_SP6_4) // 5
        .value("PILOT_SP8_2", ::gr::atsc3::PILOT_SP8_2) // 6
        .value("PILOT_SP8_4", ::gr::atsc3::PILOT_SP8_4) // 7
        .value("PILOT_SP12_2", ::gr::atsc3::PILOT_SP12_2) // 8
        .value("PILOT_SP12_4", ::gr::atsc3::PILOT_SP12_4) // 9
        .value("PILOT_SP16_2", ::gr::atsc3::PILOT_SP16_2) // 10
        .value("PILOT_SP16_4", ::gr::atsc3::PILOT_SP16_4) // 11
        .value("PILOT_SP24_2", ::gr::atsc3::PILOT_SP24_2) // 12
        .value("PILOT_SP24_4", ::gr::atsc3::PILOT_SP24_4) // 13
        .value("PILOT_SP32_2", ::gr::atsc3::PILOT_SP32_2) // 14
        .value("PILOT_SP32_4", ::gr::atsc3::PILOT_SP32_4) // 15
        .export_values()
    ;

    py::implicitly_convertible<int, ::gr::atsc3::atsc3_pilotpattern_t>();
    py::enum_<::gr::atsc3::atsc3_reduced_carriers_t>(m,"atsc3_reduced_carriers_t")
        .value("CRED_0", ::gr::atsc3::CRED_0) // 0
        .value("CRED_1", ::gr::atsc3::CRED_1) // 1
        .value("CRED_2", ::gr::atsc3::CRED_2) // 2
        .value("CRED_3", ::gr::atsc3::CRED_3) // 3
        .value("CRED_4", ::gr::atsc3::CRED_4) // 4
        .export_values()
    ;

    py::implicitly_convertible<int, ::gr::atsc3::atsc3_reduced_carriers_t>();
    py::enum_<::gr::atsc3::atsc3_l1_fec_mode_t>(m,"atsc3_l1_fec_mode_t")
        .value("L1_FEC_MODE_1", ::gr::atsc3::L1_FEC_MODE_1) // 0
        .value("L1_FEC_MODE_2", ::gr::atsc3::L1_FEC_MODE_2) // 1
        .value("L1_FEC_MODE_3", ::gr::atsc3::L1_FEC_MODE_3) // 2
        .value("L1_FEC_MODE_4", ::gr::atsc3::L1_FEC_MODE_4) // 3
        .value("L1_FEC_MODE_5", ::gr::atsc3::L1_FEC_MODE_5) // 4
        .value("L1_FEC_MODE_6", ::gr::atsc3::L1_FEC_MODE_6) // 5
        .value("L1_FEC_MODE_7", ::gr::atsc3::L1_FEC_MODE_7) // 6
        .export_values()
    ;

    py::implicitly_convertible<int, ::gr::atsc3::atsc3_l1_fec_mode_t>();
    py::enum_<::gr::atsc3::atsc3_fec_type_t>(m,"atsc3_fec_type_t")
        .value("FEC_TYPE_BCH_16K", ::gr::atsc3::FEC_TYPE_BCH_16K) // 0
        .value("FEC_TYPE_BCH_64K", ::gr::atsc3::FEC_TYPE_BCH_64K) // 1
        .value("FEC_TYPE_CRC_16K", ::gr::atsc3::FEC_TYPE_CRC_16K) // 2
        .value("FEC_TYPE_CRC_64K", ::gr::atsc3::FEC_TYPE_CRC_64K) // 3
        .value("FEC_TYPE_ONLY_16K", ::gr::atsc3::FEC_TYPE_ONLY_16K) // 4
        .value("FEC_TYPE_ONLY_64K", ::gr::atsc3::FEC_TYPE_ONLY_64K) // 5
        .export_values()
    ;

    py::implicitly_convertible<int, ::gr::atsc3::atsc3_fec_type_t>();



}








