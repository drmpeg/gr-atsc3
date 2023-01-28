/* -*- c++ -*- */
/*
 * Copyright 2021-2023 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_PILOTGENERATOR_CC_H
#define INCLUDED_ATSC3_PILOTGENERATOR_CC_H

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
    class ATSC3_API pilotgenerator_cc : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<pilotgenerator_cc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of atsc3::pilotgenerator_cc.
       *
       * To avoid accidental use of raw pointers, atsc3::pilotgenerator_cc's
       * constructor is in a private implementation
       * class. atsc3::pilotgenerator_cc::make is the public interface for
       * creating new instances.
       */
      static sptr make(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_scattered_pilot_boost_t pilotboost, atsc3_first_sbs_t firstsbs, atsc3_reduced_carriers_t cred, atsc3_miso_t misomode, atsc3_miso_tx_t misotxid, atsc3_papr_t paprmode, atsc3_pilotgenerator_mode_t outputmode, unsigned int fftlength, unsigned int vlength);
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_PILOTGENERATOR_CC_H */
