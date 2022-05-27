/* -*- c++ -*- */
/*
 * Copyright 2022 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_LDMCOMBINER_CC_H
#define INCLUDED_ATSC3_LDMCOMBINER_CC_H

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
    class ATSC3_API ldmcombiner_cc : virtual public gr::sync_block
    {
     public:
      typedef std::shared_ptr<ldmcombiner_cc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of atsc3::ldmcombiner_cc.
       *
       * To avoid accidental use of raw pointers, atsc3::ldmcombiner_cc's
       * constructor is in a private implementation
       * class. atsc3::ldmcombiner_cc::make is the public interface for
       * creating new instances.
       */
      static sptr make(atsc3_ldm_injection_level_t level);
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_LDMCOMBINER_CC_H */
