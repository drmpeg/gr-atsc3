/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "bch_bb_impl.h"

namespace gr {
  namespace atsc3 {

    using input_type = unsigned char;
    using output_type = unsigned char;
    bch_bb::sptr
    bch_bb::make(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_plp_fec_mode_t fecmode)
    {
      return gnuradio::make_block_sptr<bch_bb_impl>(
        framesize, rate, fecmode);
    }


    /*
     * The private constructor
     */
    bch_bb_impl::bch_bb_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_plp_fec_mode_t fecmode)
      : gr::block("bch_bb",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      switch (fecmode) {
        case PLP_FEC_NONE:
          num_fec_bits = 0;
          break;
        case PLP_FEC_CRC32:
          num_fec_bits = 32;
          break;
        case PLP_FEC_BCH:
          if (framesize == FECFRAME_NORMAL) {
            num_fec_bits = 192;
          }
          else if (framesize == FECFRAME_SHORT) {
            num_fec_bits = 168;
          }
          break;
        default:
          num_fec_bits = 0;
          break;
      }
      if (framesize == FECFRAME_NORMAL) {
        num_parity_bits = 192;
        switch (rate) {
          case C2_15:
            nbch = 8640;
            break;
          case C3_15:
            nbch = 12960;
            break;
          case C4_15:
            nbch = 17280;
            break;
          case C5_15:
            nbch = 21600;
            break;
          case C6_15:
            nbch = 25920;
            break;
          case C7_15:
            nbch = 30240;
            break;
          case C8_15:
            nbch = 34560;
            break;
          case C9_15:
            nbch = 38880;
            break;
          case C10_15:
            nbch = 43200;
            break;
          case C11_15:
            nbch = 47520;
            break;
          case C12_15:
            nbch = 51840;
            break;
          case C13_15:
            nbch = 56160;
            break;
          default:
            nbch = 0;
            break;
        }
      }
      else if (framesize == FECFRAME_SHORT) {
        num_parity_bits = 168;
        switch (rate) {
          case C2_15:
            nbch = 2160;
            break;
          case C3_15:
            nbch = 3240;
            break;
          case C4_15:
            nbch = 4320;
            break;
          case C5_15:
            nbch = 5400;
            break;
          case C6_15:
            nbch = 6480;
            break;
          case C7_15:
            nbch = 7560;
            break;
          case C8_15:
            nbch = 8640;
            break;
          case C9_15:
            nbch = 9720;
            break;
          case C10_15:
            nbch = 10800;
            break;
          case C11_15:
            nbch = 11880;
            break;
          case C12_15:
            nbch = 12960;
            break;
          case C13_15:
            nbch = 14040;
            break;
          default:
            nbch = 0;
            break;
        }
      }
      kbch = nbch - num_fec_bits;
      frame_size = framesize;
      plp_fec_mode = fecmode;
      bch_poly_build_tables();
      crc32_init();
      set_output_multiple(nbch);
    }

    /*
     * Our virtual destructor.
     */
    bch_bb_impl::~bch_bb_impl()
    {
    }

    void
    bch_bb_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = (noutput_items / nbch) * kbch;
    }

    /*
     * Polynomial calculation routines
     * multiply polynomials
     */

    int
    bch_bb_impl::poly_mult(const int* ina, int lena, const int* inb, int lenb, int* out)
    {
      memset(out, 0, sizeof(int) * (lena + lenb));

      for (int i = 0; i < lena; i++) {
        for (int j = 0; j < lenb; j++) {
          if (ina[i] * inb[j] > 0) {
            out[i + j]++; // count number of terms for this pwr of x
          }
        }
      }
      int max = 0;
      for (int i = 0; i < lena + lenb; i++) {
        out[i] = out[i] & 1; // If even ignore the term
        if (out[i]) {
          max = i;
        }
      }
      // return the size of array to house the result.
      return max + 1;
    }

    // precalculate the crc from:
    // http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html - cf. CRC-32 Lookup

    void
    bch_bb_impl::calculate_crc_table(void)
    {
      for (int divident = 0; divident < 256; divident++) {
        std::bitset<MAX_BCH_PARITY_BITS> curByte(divident);
        curByte <<= num_parity_bits - 8;

        for (unsigned char bit = 0; bit < 8; bit++) {
          if ((curByte[num_parity_bits - 1]) != 0) {
            curByte <<= 1;
            curByte ^= polynome;
          }
          else {
            curByte <<= 1;
          }
        }
        crc_table[divident] = curByte;
      }
    }

    void
    bch_bb_impl::bch_poly_build_tables(void)
    {
      // Normal polynomials
      const int polyn01[] = { 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };
      const int polyn02[] = { 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1 };
      const int polyn03[] = { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1 };
      const int polyn04[] = { 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1 };
      const int polyn05[] = { 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1 };
      const int polyn06[] = { 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1 };
      const int polyn07[] = { 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1 };
      const int polyn08[] = { 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1 };
      const int polyn09[] = { 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1 };
      const int polyn10[] = { 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1 };
      const int polyn11[] = { 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1 };
      const int polyn12[] = { 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1 };

      // Short polynomials
      const int polys01[] = { 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 };
      const int polys02[] = { 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1 };
      const int polys03[] = { 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1 };
      const int polys04[] = { 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1 };
      const int polys05[] = { 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1 };
      const int polys06[] = { 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1 };
      const int polys07[] = { 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1 };
      const int polys08[] = { 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1 };
      const int polys09[] = { 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1 };
      const int polys10[] = { 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1 };
      const int polys11[] = { 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1 };
      const int polys12[] = { 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1 };

      int len;
      int polyout[2][200];

      if (frame_size == FECFRAME_NORMAL) {
        len = poly_mult(polyn01, 17, polyn02, 17, polyout[0]);
        len = poly_mult(polyn03, 17, polyout[0], len, polyout[1]);
        len = poly_mult(polyn04, 17, polyout[1], len, polyout[0]);
        len = poly_mult(polyn05, 17, polyout[0], len, polyout[1]);
        len = poly_mult(polyn06, 17, polyout[1], len, polyout[0]);
        len = poly_mult(polyn07, 17, polyout[0], len, polyout[1]);
        len = poly_mult(polyn08, 17, polyout[1], len, polyout[0]);
        len = poly_mult(polyn09, 17, polyout[0], len, polyout[1]);
        len = poly_mult(polyn10, 17, polyout[1], len, polyout[0]);
        len = poly_mult(polyn11, 17, polyout[0], len, polyout[1]);
        len = poly_mult(polyn12, 17, polyout[1], len, polyout[0]);

        for (unsigned int i = 0; i < num_parity_bits; i++) {
          polynome[i] = polyout[0][i];
        }
      }
      else if (frame_size == FECFRAME_SHORT) {
        len = poly_mult(polys01, 15, polys02, 15, polyout[0]);
        len = poly_mult(polys03, 15, polyout[0], len, polyout[1]);
        len = poly_mult(polys04, 15, polyout[1], len, polyout[0]);
        len = poly_mult(polys05, 15, polyout[0], len, polyout[1]);
        len = poly_mult(polys06, 15, polyout[1], len, polyout[0]);
        len = poly_mult(polys07, 15, polyout[0], len, polyout[1]);
        len = poly_mult(polys08, 15, polyout[1], len, polyout[0]);
        len = poly_mult(polys09, 15, polyout[0], len, polyout[1]);
        len = poly_mult(polys10, 15, polyout[1], len, polyout[0]);
        len = poly_mult(polys11, 15, polyout[0], len, polyout[1]);
        len = poly_mult(polys12, 15, polyout[1], len, polyout[0]);

        for (unsigned int i = 0; i < num_parity_bits; i++) {
          polynome[i] = polyout[0][i];
        }
      }
      calculate_crc_table();
    }

    void
    bch_bb_impl::crc32_init(void)
    {
      unsigned int i, j, k;

      for (i = 0; i < 256; i++) {
        k = 0;
        for (j = (i << 24) | 0x800000; j != 0x80000000; j <<= 1) {
          k = (k << 1) ^ (((k ^ j) & 0x80000000) ? 0x00210801 : 0);
        }
        crc32_table[i] = k;
      }
    }

    int
    bch_bb_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      unsigned char b, temp, msb;
      unsigned int crc32;

      // We can use a 192 bits long bitset, all higher bits not used by the bch will just be
      // ignored
      std::bitset<MAX_BCH_PARITY_BITS> parity_bits;
      int consumed = 0;

      for (int i = 0; i < noutput_items; i += nbch) {
        memcpy(out, in, sizeof(unsigned char) * kbch);
        out += kbch;
        switch (plp_fec_mode) {
          case PLP_FEC_NONE:
            in += kbch;
            consumed += kbch;
            break;
          case PLP_FEC_CRC32:
            crc32 = 0xffffffff;
            for (int j = 0; j < kbch / 8; j++) {
              b = 0;
              for (int e = 0; e < 8; e++) {
                temp = *in++;
                consumed++;
                b |= temp << (7 - e);
              }
              crc32 = (crc32 << 8) ^ crc32_table[((crc32 >> 24) ^ b) & 0xff];
            }
            for (int n = 0; n < num_fec_bits; n++) {
              *out++ = (char)(crc32 >> ((num_fec_bits - 1) - n)) & 0x1;
            }
            break;
          case PLP_FEC_BCH:
            for (int j = 0; j < kbch / 8; j++) {
              b = 0;

              // calculate the crc using the lookup table, cf.
              // http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
              for (int e = 0; e < 8; e++) {
                temp = *in++;
                consumed++;

                b |= temp << (7 - e);
              }

              msb = 0;
              for (int n = 1; n <= 8; n++) {
                temp = parity_bits[num_parity_bits - n];
                msb |= temp << (8 - n);
              }
              /* XOR-in next input byte into MSB of crc and get this MSB, that's our new
               * intermediate divident */
              unsigned char pos = (msb ^ b);
              /* Shift out the MSB used for division per lookuptable and XOR with the
               * remainder */
              parity_bits = (parity_bits << 8) ^ crc_table[pos];
            }

            // Now add the parity bits to the output
            for (unsigned int n = 0; n < num_parity_bits; n++) {
              *out++ = (char)parity_bits[num_parity_bits - 1];
              parity_bits <<= 1;
            }
            break;
          default:
            break;
        }
      }
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (consumed);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace atsc3 */
} /* namespace gr */
