/* -*- c++ -*- */
/*
 * Copyright 2022,2023 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_SUBBOOTSTRAP_CC_H
#define INCLUDED_ATSC3_SUBBOOTSTRAP_CC_H

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
    class ATSC3_API subbootstrap_cc : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<subbootstrap_cc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of atsc3::subbootstrap_cc.
       *
       * To avoid accidental use of raw pointers, atsc3::subbootstrap_cc's
       * constructor is in a private implementation
       * class. atsc3::subbootstrap_cc::make is the public interface for
       * creating new instances.
       */
      static sptr make(atsc3_fftsize_t fftsizesub0, int numpayloadsymssub0, int numpreamblesyms, atsc3_guardinterval_t guardintervalsub0, atsc3_pilotpattern_t pilotpatternsub0, atsc3_fftsize_t fftsizesub1, int numpayloadsymssub1, atsc3_guardinterval_t guardintervalsub1, atsc3_pilotpattern_t pilotpatternsub1, atsc3_min_time_to_next_t frameinterval, atsc3_frame_length_mode_t flmode, int flen, atsc3_l1_fec_mode_t l1bmode, atsc3_bootstrap_mode_t outputmode, atsc3_showlevels_t showlevels, float vclip);
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_SUBBOOTSTRAP_CC_H */
