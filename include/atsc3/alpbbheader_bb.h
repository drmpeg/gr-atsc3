/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_ALPBBHEADER_BB_H
#define INCLUDED_ATSC3_ALPBBHEADER_BB_H

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
    class ATSC3_API alpbbheader_bb : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<alpbbheader_bb> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of atsc3::alpbbheader_bb.
       *
       * To avoid accidental use of raw pointers, atsc3::alpbbheader_bb's
       * constructor is in a private implementation
       * class. atsc3::alpbbheader_bb::make is the public interface for
       * creating new instances.
       */
      static sptr make(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_lls_insertion_mode_t llsmode, atsc3_lls_service_count_t llscount);
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_ALPBBHEADER_BB_H */
