/* -*- c++ -*- */
/*
 * Copyright 2022 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_LDMCOMBINER_CC_IMPL_H
#define INCLUDED_ATSC3_LDMCOMBINER_CC_IMPL_H

#include <atsc3/ldmcombiner_cc.h>
#include "atsc3_defines.h"

namespace gr {
  namespace atsc3 {

    class ldmcombiner_cc_impl : public ldmcombiner_cc
    {
     private:
      float alpha_vector[5400];
      float beta_vector[5400];

     public:
      ldmcombiner_cc_impl(atsc3_ldm_injection_level_t level);
      ~ldmcombiner_cc_impl();

      int work(
              int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );
    };

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_LDMCOMBINER_CC_IMPL_H */
