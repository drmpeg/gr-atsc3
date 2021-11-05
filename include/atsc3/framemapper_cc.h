/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_FRAMEMAPPER_CC_H
#define INCLUDED_ATSC3_FRAMEMAPPER_CC_H

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
    class ATSC3_API framemapper_cc : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<framemapper_cc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of atsc3::framemapper_cc.
       *
       * To avoid accidental use of raw pointers, atsc3::framemapper_cc's
       * constructor is in a private implementation
       * class. atsc3::framemapper_cc::make is the public interface for
       * creating new instances.
       */
      static sptr make(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation, atsc3_guardinterval_t guardinterval);
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_FRAMEMAPPER_CC_H */
