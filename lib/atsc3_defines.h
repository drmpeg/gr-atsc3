/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef INCLUDED_ATSC3_DEFINES_H
#define INCLUDED_ATSC3_DEFINES_H

#define TRUE 1
#define FALSE 0

#define FRAME_SIZE_NORMAL 64800
#define FRAME_SIZE_SHORT 16200

#define MAX_BCH_PARITY_BITS 192
#define POLYNOMIAL 0xd31c /* G(x) = 1+X+X^3+X^6+X^7+X^11+X^12+X^13+X^16 */

// LDPC type
#define LDPC_TYPE_A 0
#define LDPC_TYPE_B 1

// Block interleaver type
#define BLOCK_TYPE_A 0
#define BLOCK_TYPE_B 1

#define MAX_L1DETAIL_MSG_SIZE 6312

#define LDPC_TABLE_3_15S_ENTRIES 129
#define LDPC_ENCODE_TABLE_LENGTH (LDPC_TABLE_3_15S_ENTRIES * 360)

#define BOOTSTRAP_SAMPLES 13824
#define SAMPLES_PER_MILLISECOND_6MHZ 6912

#endif /* INCLUDED_ATSC3_DEFINES_H */
