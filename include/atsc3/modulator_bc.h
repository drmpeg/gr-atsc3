/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_MODULATOR_BC_H
#define INCLUDED_ATSC3_MODULATOR_BC_H

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
    class ATSC3_API modulator_bc : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<modulator_bc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of atsc3::modulator_bc.
       *
       * To avoid accidental use of raw pointers, atsc3::modulator_bc's
       * constructor is in a private implementation
       * class. atsc3::modulator_bc::make is the public interface for
       * creating new instances.
       */
      static sptr make(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation);
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_MODULATOR_BC_H */
