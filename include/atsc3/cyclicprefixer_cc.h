/* -*- c++ -*- */
/*
 * Copyright 2022,2023 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_CYCLICPREFIXER_CC_H
#define INCLUDED_ATSC3_CYCLICPREFIXER_CC_H

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
    class ATSC3_API cyclicprefixer_cc : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<cyclicprefixer_cc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of atsc3::cyclicprefixer_cc.
       *
       * To avoid accidental use of raw pointers, atsc3::cyclicprefixer_cc's
       * constructor is in a private implementation
       * class. atsc3::cyclicprefixer_cc::make is the public interface for
       * creating new instances.
       */
      static sptr make(atsc3_cyclicprefixer_mode_t submode, atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_fftsize_t altfftsize, int altnumpayloadsyms, int altnumpreamblesyms, atsc3_guardinterval_t altguardinterval, atsc3_frame_length_mode_t flmode, int flen, unsigned int vlength);
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_CYCLICPREFIXER_CC_H */
