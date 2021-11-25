/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_BCH_BB_H
#define INCLUDED_ATSC3_BCH_BB_H

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
    class ATSC3_API bch_bb : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<bch_bb> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of atsc3::bch_bb.
       *
       * To avoid accidental use of raw pointers, atsc3::bch_bb's
       * constructor is in a private implementation
       * class. atsc3::bch_bb::make is the public interface for
       * creating new instances.
       */
      static sptr make(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_plp_fec_mode_t fecmode);
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_BCH_BB_H */
