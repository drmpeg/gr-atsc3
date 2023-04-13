/* -*- c++ -*- */
/*
 * Copyright 2022,2023 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_SUBFRAMEMAPPER_CC_H
#define INCLUDED_ATSC3_SUBFRAMEMAPPER_CC_H

#include <atsc3/api.h>
#include <atsc3/atsc3_config.h>
#include <gnuradio/block.h>

namespace gr {
  namespace atsc3 {

    /*!
     * \brief <+description of block+>
     * \ingroup atsc3
     *
     */
    class ATSC3_API subframemapper_cc : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<subframemapper_cc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of atsc3::subframemapper_cc.
       *
       * To avoid accidental use of raw pointers, atsc3::subframemapper_cc's
       * constructor is in a private implementation
       * class. atsc3::subframemapper_cc::make is the public interface for
       * creating new instances.
       */
      static sptr make(atsc3_framesize_t framesizeplp0, atsc3_code_rate_t rateplp0, atsc3_plp_fec_mode_t fecmodeplp0, atsc3_constellation_t constellationplp0, atsc3_fftsize_t fftsizeplp0, int numpayloadsymsplp0, int numpreamblesyms, atsc3_guardinterval_t guardintervalplp0, atsc3_pilotpattern_t pilotpatternplp0, atsc3_scattered_pilot_boost_t pilotboostplp0, atsc3_first_sbs_t firstsbs, atsc3_frequency_interleaver_t fimodeplp0, atsc3_time_interleaver_mode_t timodeplp0, atsc3_time_interleaver_depth_t tidepthplp0, int tiblocksplp0, int tifecblocksmaxplp0, int tifecblocksplp0, atsc3_lls_insertion_mode_t llsmodeplp0, atsc3_reduced_carriers_t credplp0, atsc3_miso_t misomodeplp0, atsc3_framesize_t framesizeplp1, atsc3_code_rate_t rateplp1, atsc3_plp_fec_mode_t fecmodeplp1, atsc3_constellation_t constellationplp1, atsc3_fftsize_t fftsizeplp1, int numpayloadsymsplp1, atsc3_guardinterval_t guardintervalplp1, atsc3_pilotpattern_t pilotpatternplp1, atsc3_scattered_pilot_boost_t pilotboostplp1, atsc3_frequency_interleaver_t fimodeplp1, atsc3_time_interleaver_mode_t timodeplp1, atsc3_time_interleaver_depth_t tidepthplp1, int tiblocksplp1, int tifecblocksmaxplp1, int tifecblocksplp1, atsc3_lls_insertion_mode_t llsmodeplp1, atsc3_reduced_carriers_t credplp1, atsc3_miso_t misomodeplp1, atsc3_frame_length_mode_t flmode, int flen, atsc3_papr_t paprmode, atsc3_l1_fec_mode_t l1bmode, atsc3_l1_fec_mode_t l1dmode);
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_SUBFRAMEMAPPER_CC_H */
