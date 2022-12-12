/* -*- c++ -*- */
/*
 * Copyright 2022 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_TDMFRAMEMAPPER_CC_H
#define INCLUDED_ATSC3_TDMFRAMEMAPPER_CC_H

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
    class ATSC3_API tdmframemapper_cc : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<tdmframemapper_cc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of atsc3::tdmframemapper_cc.
       *
       * To avoid accidental use of raw pointers, atsc3::tdmframemapper_cc's
       * constructor is in a private implementation
       * class. atsc3::tdmframemapper_cc::make is the public interface for
       * creating new instances.
       */
      static sptr make(atsc3_framesize_t framesize1st, atsc3_code_rate_t rate1st, atsc3_plp_fec_mode_t fecmode1st, atsc3_constellation_t constellation1st, atsc3_time_interleaver_mode_t timode1st, atsc3_framesize_t framesize2nd, atsc3_code_rate_t rate2nd, atsc3_plp_fec_mode_t fecmode2nd, atsc3_constellation_t constellation2nd, atsc3_time_interleaver_mode_t timode2nd, atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_scattered_pilot_boost_t pilotboost, atsc3_first_sbs_t firstsbs, atsc3_frequency_interleaver_t fimode, atsc3_reduced_carriers_t cred, atsc3_papr_t paprmode, atsc3_l1_fec_mode_t l1bmode, atsc3_l1_fec_mode_t l1dmode);
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_TDMFRAMEMAPPER_CC_H */
