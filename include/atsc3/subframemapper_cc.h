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
      static sptr make(atsc3_framesize_t framesizesub0, atsc3_code_rate_t ratesub0, atsc3_plp_fec_mode_t fecmodesub0, atsc3_constellation_t constellationsub0, atsc3_fftsize_t fftsizesub0, int numpayloadsymssub0, int numpreamblesyms, atsc3_guardinterval_t guardintervalsub0, atsc3_pilotpattern_t pilotpatternsub0, atsc3_scattered_pilot_boost_t pilotboostsub0, atsc3_first_sbs_t firstsbs, atsc3_frequency_interleaver_t fimodesub0, atsc3_time_interleaver_mode_t timodesub0, atsc3_time_interleaver_depth_t tidepthsub0, int tiblockssub0, int tifecblocksmaxsub0, int tifecblockssub0, int plpsizesub0, atsc3_lls_insertion_mode_t llsmodesub0, atsc3_reduced_carriers_t credsub0, atsc3_miso_t misomodesub0, atsc3_framesize_t framesizesub1, atsc3_code_rate_t ratesub1, atsc3_plp_fec_mode_t fecmodesub1, atsc3_constellation_t constellationsub1, atsc3_fftsize_t fftsizesub1, int numpayloadsymssub1, atsc3_guardinterval_t guardintervalsub1, atsc3_pilotpattern_t pilotpatternsub1, atsc3_scattered_pilot_boost_t pilotboostsub1, atsc3_frequency_interleaver_t fimodesub1, atsc3_time_interleaver_mode_t timodesub1, atsc3_time_interleaver_depth_t tidepthsub1, int tiblockssub1, int tifecblocksmaxsub1, int tifecblockssub1, int plpsizesub1, atsc3_lls_insertion_mode_t llsmodesub1, atsc3_reduced_carriers_t credsub1, atsc3_miso_t misomodesub1, atsc3_frame_length_mode_t flmode, int flen, atsc3_papr_t paprmode, atsc3_l1_fec_mode_t l1bmode, atsc3_l1_fec_mode_t l1dmode);
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_SUBFRAMEMAPPER_CC_H */
