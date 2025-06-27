/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ATSC3_PARAMS_H
#define INCLUDED_ATSC3_PARAMS_H

#include "atsc3_defines.h"
#include <atsc3/atsc3_config.h>

namespace gr {
  namespace atsc3 {

    struct fec_params_t {
      int kbch;
      int nbch;
      int ldpc_type;
      int q_val;
      int q1_val;
      int q2_val;
      int m1_val;
      int m2_val;
      int rate_index;
    };

    fec_params_t fec_params(atsc3_framesize_t framesize, atsc3_code_rate_t rate);

  } // namespace atsc3
} // namespace gr

#endif /* INCLUDED_ATSC3_PARAMS_H */
