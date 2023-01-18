/* -*- c++ -*- */
/*
 * Copyright 2022,2023 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_LDMFRAMEMAPPER_CC_H
#define INCLUDED_ATSC3_LDMFRAMEMAPPER_CC_H

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
    class ATSC3_API ldmframemapper_cc : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<ldmframemapper_cc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of atsc3::ldmframemapper_cc.
       *
       * To avoid accidental use of raw pointers, atsc3::ldmframemapper_cc's
       * constructor is in a private implementation
       * class. atsc3::ldmframemapper_cc::make is the public interface for
       * creating new instances.
       */
      static sptr make(atsc3_framesize_t framesize_core, atsc3_code_rate_t rate_core, atsc3_plp_fec_mode_t fecmode_core, atsc3_constellation_t constellation_core, atsc3_framesize_t framesize_enh, atsc3_code_rate_t rate_enh, atsc3_plp_fec_mode_t fecmode_enh, atsc3_constellation_t constellation_enh, atsc3_ldm_injection_level_t level, atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_scattered_pilot_boost_t pilotboost, atsc3_first_sbs_t firstsbs, atsc3_frequency_interleaver_t fimode, atsc3_time_interleaver_mode_t timode, atsc3_time_interleaver_depth_t tidepth, int tiblocks, int tifecblocksmax, int tifecblocks, atsc3_reduced_carriers_t cred, atsc3_frame_length_mode_t flmode, int flen, atsc3_papr_t paprmode, atsc3_l1_fec_mode_t l1bmode, atsc3_l1_fec_mode_t l1dmode);
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_LDMFRAMEMAPPER_CC_H */
