/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_BBSCRAMBLER_BB_H
#define INCLUDED_ATSC3_BBSCRAMBLER_BB_H

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
    class ATSC3_API bbscrambler_bb : virtual public gr::sync_block
    {
     public:
      typedef std::shared_ptr<bbscrambler_bb> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of atsc3::bbscrambler_bb.
       *
       * To avoid accidental use of raw pointers, atsc3::bbscrambler_bb's
       * constructor is in a private implementation
       * class. atsc3::bbscrambler_bb::make is the public interface for
       * creating new instances.
       */
      static sptr make(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_plp_fec_mode_t fecmode);
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_BBSCRAMBLER_BB_H */
