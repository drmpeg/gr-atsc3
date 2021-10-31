/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef INCLUDED_ATSC3_DEFINES_H
#define INCLUDED_ATSC3_DEFINES_H

#define FRAME_SIZE_NORMAL 64800
#define FRAME_SIZE_SHORT 16200

#define MAX_BCH_PARITY_BITS 192
#define POLYNOMIAL 0xd31c /* G(x) = 1+X+X^3+X^6+X^7+X^11+X^12+X^13+X^16 */

// LDPC type
#define LDPC_TYPE_A 0
#define LDPC_TYPE_B 1

#define LDPC_ENCODE_TABLE_LENGTH (FRAME_SIZE_NORMAL * 10)

#endif /* INCLUDED_ATSC3_DEFINES_H */
