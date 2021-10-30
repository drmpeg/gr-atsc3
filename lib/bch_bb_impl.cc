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
    bch_bb::make(atsc3_framesize_t framesize, atsc3_code_rate_t rate)
    {
      return gnuradio::make_block_sptr<bch_bb_impl>(
        framesize, rate);
    }


    /*
     * The private constructor
     */
    bch_bb_impl::bch_bb_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate)
      : gr::block("bch_bb",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      if (framesize == FECFRAME_NORMAL) {
        num_parity_bits = 192;
        switch (rate) {
          case C2_15:
            kbch = 8448;
            nbch = 8640;
            break;
          case C3_15:
            kbch = 12768;
            nbch = 12960;
            break;
          case C4_15:
            kbch = 17088;
            nbch = 17280;
            break;
          case C5_15:
            kbch = 21408;
            nbch = 21600;
            break;
          case C6_15:
            kbch = 25728;
            nbch = 25920;
            break;
          case C7_15:
            kbch = 30048;
            nbch = 30240;
            break;
          case C8_15:
            kbch = 34368;
            nbch = 34560;
            break;
          case C9_15:
            kbch = 38688;
            nbch = 38880;
            break;
          case C10_15:
            kbch = 43008;
            nbch = 43200;
            break;
          case C11_15:
            kbch = 47328;
            nbch = 47520;
            break;
          case C12_15:
            kbch = 51648;
            nbch = 51840;
            break;
          case C13_15:
            kbch = 55968;
            nbch = 56160;
            break;
          default:
            kbch = 0;
            nbch = 0;
            break;
        }
      }
      else if (framesize == FECFRAME_SHORT) {
        num_parity_bits = 168;
        switch (rate) {
          case C2_15:
            kbch = 1992;
            nbch = 2160;
            break;
          case C3_15:
            kbch = 3072;
            nbch = 3240;
            break;
          case C4_15:
            kbch = 4152;
            nbch = 4320;
            break;
          case C5_15:
            kbch = 5232;
            nbch = 5400;
            break;
          case C6_15:
            kbch = 6312;
            nbch = 6480;
            break;
          case C7_15:
            kbch = 7392;
            nbch = 7560;
            break;
          case C8_15:
            kbch = 8472;
            nbch = 8640;
            break;
          case C9_15:
            kbch = 9552;
            nbch = 9720;
            break;
          case C10_15:
            kbch = 10632;
            nbch = 10800;
            break;
          case C11_15:
            kbch = 11712;
            nbch = 11880;
            break;
          case C12_15:
            kbch = 12792;
            nbch = 12960;
            break;
          case C13_15:
            kbch = 13872;
            nbch = 14040;
            break;
          default:
            kbch = 0;
            nbch = 0;
            break;
        }
      }
      frame_size = framesize;
      bch_poly_build_tables();
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

    int
    bch_bb_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      unsigned char b, temp, msb;

      // We can use a 192 bits long bitset, all higher bits not used by the bch will just be
      // ignored
      std::bitset<MAX_BCH_PARITY_BITS> parity_bits;
      int consumed = 0;

      for (int i = 0; i < noutput_items; i += nbch) {
        memcpy(out, in, sizeof(unsigned char) * kbch);
        out += kbch;
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
      }
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (consumed);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace atsc3 */
} /* namespace gr */
