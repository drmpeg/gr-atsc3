/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_PAPRTR_CC_H
#define INCLUDED_ATSC3_PAPRTR_CC_H

#include <atsc3/api.h>
#include <atsc3/atsc3_config.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace atsc3 {

    /*!
     * \brief <+description of block+>
     * \ingroup atsc3
     *
     */
    class ATSC3_API paprtr_cc : virtual public gr::sync_block
    {
     public:
      typedef std::shared_ptr<paprtr_cc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of atsc3::paprtr_cc.
       *
       * To avoid accidental use of raw pointers, atsc3::paprtr_cc's
       * constructor is in a private implementation
       * class. atsc3::paprtr_cc::make is the public interface for
       * creating new instances.
       */
      static sptr make(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_pilotpattern_t pilotpattern, atsc3_first_sbs_t firstsbs, atsc3_reduced_carriers_t cred, atsc3_papr_t paprmode, float vclip, int iterations, unsigned int vlength);
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_PAPRTR_CC_H */
