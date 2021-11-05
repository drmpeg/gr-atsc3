/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "framemapper_cc_impl.h"

namespace gr {
  namespace atsc3 {

    using input_type = gr_complex;
    using output_type = gr_complex;
    framemapper_cc::sptr
    framemapper_cc::make(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation, atsc3_guardinterval_t guardinterval)
    {
      return gnuradio::make_block_sptr<framemapper_cc_impl>(
        framesize, rate, constellation, guardinterval);
    }


    /*
     * The private constructor
     */
    framemapper_cc_impl::framemapper_cc_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation, atsc3_guardinterval_t guardinterval)
      : gr::block("framemapper_cc",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      L1_Basic *l1basicinit = &L1_Signalling[0].l1basic_data;

      init_fm_randomizer();
      num_parity_bits = 168;
      bch_poly_build_tables();
      l1basicinit->version = 0;
      l1basicinit->mimo_scattered_pilot_encoding = MSPE_WALSH_HADAMARD_PILOTS;
      l1basicinit->lls_flag = FALSE;
      l1basicinit->time_info_flag = TIF_NOT_INCLUDED;
      l1basicinit->return_channel_flag = FALSE;
      l1basicinit->papr_reduction = PAPR_OFF;
      l1basicinit->frame_length_mode = TLM_SYMBOL_ALIGNED;
      l1basicinit->frame_length = 0;
      l1basicinit->excess_samples_per_symbol = 0;
      l1basicinit->time_offset = 0;
      l1basicinit->additional_samples = 0;
      l1basicinit->num_subframes = 0;
      l1basicinit->preamble_num_symbols = 1;
      l1basicinit->preamble_reduced_carriers = 0;
      l1basicinit->L1_Detail_content_tag = 0;
      l1basicinit->L1_Detail_size_bytes = 25;
      l1basicinit->L1_Detail_fec_type = DFT_MODE_1;
      l1basicinit->L1_Detail_additional_parity_mode = APM_K0;
      l1basicinit->L1_Detail_total_cells = 2787;
      l1basicinit->first_sub_mimo = FALSE;
      l1basicinit->first_sub_miso = MISO_OFF;
      l1basicinit->first_sub_fft_size = FFTSIZE_8K;
      l1basicinit->first_sub_reduced_carriers = CRED_0;
      l1basicinit->first_sub_guard_interval = GI_5_1024;
      l1basicinit->first_sub_num_ofdm_symbols = 71;
      l1basicinit->first_sub_scattered_pilot_pattern = PILOT_SP3_4;
      l1basicinit->first_sub_scattered_pilot_boost = 4;
      l1basicinit->first_sub_sbs_first = FALSE;
      l1basicinit->first_sub_sbs_last = TRUE;
      l1basicinit->reserved = 0xffffffffffff;
      l1basicinit->crc = 0;
      add_l1basic(&l1basic_cache[0]);
    }

    /*
     * Our virtual destructor.
     */
    framemapper_cc_impl::~framemapper_cc_impl()
    {
    }

    void
    framemapper_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

#define CRC_POLY 0x00210801

    int
    framemapper_cc_impl::add_crc32_bits(unsigned char* in, int length)
    {
      int crc = 0xffffffff;
      int b;
      int i = 0;

      for (int n = 0; n < length; n++) {
        b = in[i++] ^ ((crc >> 31) & 0x01);
        crc <<= 1;
        if (b) {
          crc ^= CRC_POLY;
        }
      }

      for (int n = 31; n >= 0; n--) {
        in[i++] = (crc & (1 << n)) ? 1 : 0;
      }
      return 32;
    }

    void
    framemapper_cc_impl::init_fm_randomizer(void)
    {
      int sr = 0x18f;
      int b;

      for (int i = 0; i < FRAME_SIZE_SHORT; i++) {
        fm_randomize[i] = ((sr & 0x4) << 5) | ((sr & 0x8 ) << 3) | ((sr & 0x10) << 1) | \
                          ((sr & 0x20) >> 1) | ((sr & 0x200) >> 6) | ((sr & 0x1000) >> 10) | \
                          ((sr & 0x2000) >> 12) | ((sr & 0x8000) >> 15);
        b = sr & 1;
        sr >>= 1;
        if (b) {
          sr ^= POLYNOMIAL;
        }
      }
    }

    int
    framemapper_cc_impl::poly_mult(const int* ina, int lena, const int* inb, int lenb, int* out)
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
    framemapper_cc_impl::calculate_crc_table(void)
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
    framemapper_cc_impl::bch_poly_build_tables(void)
    {
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

      for (int i = 0; i < num_parity_bits; i++) {
        polynome[i] = polyout[0][i];
      }
      calculate_crc_table();
    }

    void
    framemapper_cc_impl::add_l1basic(gr_complex *out)
    {
      int temp, index, offset_bits = 0;
      long long templong;
      std::bitset<MAX_BCH_PARITY_BITS> parity_bits;
      unsigned char b, tempbch, msb;
      unsigned char *l1basic = l1_interleave;
      L1_Basic *l1basicinit = &L1_Signalling[0].l1basic_data;

      temp = l1basicinit->version;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      l1basic[offset_bits++] = l1basicinit->mimo_scattered_pilot_encoding;
      l1basic[offset_bits++] = l1basicinit->lls_flag;
      temp = l1basicinit->time_info_flag;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      l1basic[offset_bits++] = l1basicinit->return_channel_flag;
      temp = l1basicinit->papr_reduction;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      l1basic[offset_bits++] = l1basicinit->frame_length_mode;
      if (l1basicinit->frame_length_mode == FALSE) {
        temp = l1basicinit->frame_length;
        for (int n = 9; n >= 0; n--) {
          l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
        }
        temp = l1basicinit->excess_samples_per_symbol;
        for (int n = 12; n >= 0; n--) {
          l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
        }
      }
      else {
        temp = l1basicinit->time_offset;
        for (int n = 15; n >= 0; n--) {
          l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
        }
        temp = l1basicinit->additional_samples;
        for (int n = 6; n >= 0; n--) {
          l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
        }
      }
      temp = l1basicinit->num_subframes;
      for (int n = 7; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->preamble_num_symbols;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->preamble_reduced_carriers;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->L1_Detail_content_tag;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->L1_Detail_size_bytes;
      for (int n = 12; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->L1_Detail_fec_type;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->L1_Detail_additional_parity_mode;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->L1_Detail_total_cells;
      for (int n = 18; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      l1basic[offset_bits++] = l1basicinit->first_sub_mimo;
      temp = l1basicinit->first_sub_miso;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->first_sub_fft_size;
      for (int n = 1; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->first_sub_reduced_carriers;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->first_sub_guard_interval;
      for (int n = 3; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->first_sub_num_ofdm_symbols;
      for (int n = 10; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->first_sub_scattered_pilot_pattern;
      for (int n = 4; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      temp = l1basicinit->first_sub_scattered_pilot_boost;
      for (int n = 2; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      l1basic[offset_bits++] = l1basicinit->first_sub_sbs_first;
      l1basic[offset_bits++] = l1basicinit->first_sub_sbs_last;
      templong = l1basicinit->reserved;
      for (int n = 47; n >= 0; n--) {
        l1basic[offset_bits++] = templong & (1 << n) ? 1 : 0;
      }
      offset_bits += add_crc32_bits(l1basic, offset_bits);

      for (int i = 0; i < offset_bits; i += 8) {
        temp = 0;
        index = 0;
        for (int j = 7; j >= 0; j--) {
          temp |= l1basic[i + index++] << j;
        }
        temp ^= fm_randomize[i / 8];
        index = 0;
        for (int n = 7; n >= 0; n--) {
          l1basic[i + index++] = temp & (1 << n) ? 1 : 0;
        }
        b = temp;
        msb = 0;
        for (int n = 1; n <= 8; n++) {
          tempbch = parity_bits[num_parity_bits - n];
          msb |= tempbch << (8 - n);
        }
        /* XOR-in next input byte into MSB of crc and get this MSB, that's our new
         * intermediate divident */
        unsigned char pos = (msb ^ b);
        /* Shift out the MSB used for division per lookuptable and XOR with the
         * remainder */
        parity_bits = (parity_bits << 8) ^ crc_table[pos];
      }
      for (int n = 0; n < num_parity_bits; n++) {
        l1basic[offset_bits++] = (char)parity_bits[num_parity_bits - 1];
        parity_bits <<= 1;
      }

      for (int i = 0; i < offset_bits; i += 8) {
        temp = 0;
        index = 0;
        for (int j = 7; j >= 0; j--) {
          temp |= l1basic[i + index] << j;
          index++;
        }
        printf("%02x", temp);
      }
      printf("\n");
    }

    void
    framemapper_cc_impl::add_l1detail(gr_complex *out)
    {
    }

    int
    framemapper_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace atsc3 */
} /* namespace gr */
