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
    framemapper_cc::make(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation, atsc3_fftsize_t fftsize, atsc3_guardinterval_t guardinterval, atsc3_l1_fec_mode_t l1bmode, atsc3_l1_fec_mode_t l1dmode)
    {
      return gnuradio::make_block_sptr<framemapper_cc_impl>(
        framesize, rate, constellation, fftsize, guardinterval, l1bmode, l1dmode);
    }


    /*
     * The private constructor
     */
    framemapper_cc_impl::framemapper_cc_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation, atsc3_fftsize_t fftsize, atsc3_guardinterval_t guardinterval, atsc3_l1_fec_mode_t l1bmode, atsc3_l1_fec_mode_t l1dmode)
      : gr::block("framemapper_cc",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      L1_Basic *l1basicinit = &L1_Signalling[0].l1basic_data;
      double normalization;
      int rateindex, i, j;

      fft_size = fftsize;
      l1b_mode = l1bmode;
      l1d_mode = l1dmode;
      init_fm_randomizer();
      num_parity_bits = 168;
      bch_poly_build_tables();
      q1_val = 3;
      q2_val = 33;
      m1_val = 1080;
      m2_val = 11880;
      ldpc_bf_type_a(ldpc_tab_3_15S);
      normalization = std::sqrt(2.0);
      m_qpsk[0] = gr_complex( 1.0 / normalization,  1.0 / normalization);
      m_qpsk[1] = gr_complex(-1.0 / normalization,  1.0 / normalization);
      m_qpsk[2] = gr_complex( 1.0 / normalization, -1.0 / normalization);
      m_qpsk[3] = gr_complex(-1.0 / normalization, -1.0 / normalization);
      m_16qam[0] = mod_table_16QAM[0];
      m_16qam[1] = mod_table_16QAM[1];
      m_16qam[2] = mod_table_16QAM[2];
      m_16qam[3] = mod_table_16QAM[3];
      m_16qam[4] = -std::conj(mod_table_16QAM[0]);
      m_16qam[5] = -std::conj(mod_table_16QAM[1]);
      m_16qam[6] = -std::conj(mod_table_16QAM[2]);
      m_16qam[7] = -std::conj(mod_table_16QAM[3]);
      m_16qam[8] = std::conj(mod_table_16QAM[0]);
      m_16qam[9] = std::conj(mod_table_16QAM[1]);
      m_16qam[10] = std::conj(mod_table_16QAM[2]);
      m_16qam[11] = std::conj(mod_table_16QAM[3]);
      m_16qam[12] = -mod_table_16QAM[0];
      m_16qam[13] = -mod_table_16QAM[1];
      m_16qam[14] = -mod_table_16QAM[2];
      m_16qam[15] = -mod_table_16QAM[3];
      for (i = 0, j = 0; i < 16; i++, j++) {
        m_64qam[i] = mod_table_64QAM[j];
      }
      for (i = 16, j = 0; i < 32; i++, j++) {
        m_64qam[i] = -std::conj(mod_table_64QAM[j]);
      }
      for (i = 32, j = 0; i < 48; i++, j++) {
        m_64qam[i] = std::conj(mod_table_64QAM[j]);
      }
      for (i = 48, j = 0; i < 64; i++, j++) {
        m_64qam[i] = -mod_table_64QAM[j];
      }
      if (l1bmode == L1_FEC_MODE_6) {
        rateindex = 0;
        for (i = 0, j = 0; i < 64; i++, j++) {
          m_l1b_256qam[i] = mod_table_256QAM[rateindex][j];
        }
        for (i = 64, j = 0; i < 128; i++, j++) {
          m_l1b_256qam[i] = -std::conj(mod_table_256QAM[rateindex][j]);
        }
        for (i = 128, j = 0; i < 192; i++, j++) {
          m_l1b_256qam[i] = std::conj(mod_table_256QAM[rateindex][j]);
        }
        for (i = 192, j = 0; i < 256; i++, j++) {
          m_l1b_256qam[i] = -mod_table_256QAM[rateindex][j];
        }
      }
      else {
        rateindex = 1;
        for (i = 0, j = 0; i < 64; i++, j++) {
          m_l1b_256qam[i] = mod_table_256QAM[rateindex][j];
        }
        for (i = 64, j = 0; i < 128; i++, j++) {
          m_l1b_256qam[i] = -std::conj(mod_table_256QAM[rateindex][j]);
        }
        for (i = 128, j = 0; i < 192; i++, j++) {
          m_l1b_256qam[i] = std::conj(mod_table_256QAM[rateindex][j]);
        }
        for (i = 192, j = 0; i < 256; i++, j++) {
          m_l1b_256qam[i] = -mod_table_256QAM[rateindex][j];
        }
      }
      if (l1dmode == L1_FEC_MODE_6) {
        rateindex = 0;
        for (i = 0, j = 0; i < 64; i++, j++) {
          m_l1d_256qam[i] = mod_table_256QAM[rateindex][j];
        }
        for (i = 64, j = 0; i < 128; i++, j++) {
          m_l1d_256qam[i] = -std::conj(mod_table_256QAM[rateindex][j]);
        }
        for (i = 128, j = 0; i < 192; i++, j++) {
          m_l1d_256qam[i] = std::conj(mod_table_256QAM[rateindex][j]);
        }
        for (i = 192, j = 0; i < 256; i++, j++) {
          m_l1d_256qam[i] = -mod_table_256QAM[rateindex][j];
        }
      }
      else {
        rateindex = 1;
        for (i = 0, j = 0; i < 64; i++, j++) {
          m_l1d_256qam[i] = mod_table_256QAM[rateindex][j];
        }
        for (i = 64, j = 0; i < 128; i++, j++) {
          m_l1d_256qam[i] = -std::conj(mod_table_256QAM[rateindex][j]);
        }
        for (i = 128, j = 0; i < 192; i++, j++) {
          m_l1d_256qam[i] = std::conj(mod_table_256QAM[rateindex][j]);
        }
        for (i = 192, j = 0; i < 256; i++, j++) {
          m_l1d_256qam[i] = -mod_table_256QAM[rateindex][j];
        }
      }
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
      l1basicinit->preamble_num_symbols = 0;
      l1basicinit->preamble_reduced_carriers = 0;
      l1basicinit->L1_Detail_content_tag = 0;
      l1basicinit->L1_Detail_size_bytes = 25;
      l1basicinit->L1_Detail_fec_type = l1dmode;
      l1basicinit->L1_Detail_additional_parity_mode = APM_K0;
      l1basicinit->L1_Detail_total_cells = 204;
      l1basicinit->first_sub_mimo = FALSE;
      l1basicinit->first_sub_miso = MISO_OFF;
      l1basicinit->first_sub_fft_size = FFTSIZE_8K;
      l1basicinit->first_sub_reduced_carriers = CRED_0;
      l1basicinit->first_sub_guard_interval = GI_5_1024;
      l1basicinit->first_sub_num_ofdm_symbols = 71;
      l1basicinit->first_sub_scattered_pilot_pattern = PILOT_SP3_4;
      l1basicinit->first_sub_scattered_pilot_boost = 4;
      l1basicinit->first_sub_sbs_first = TRUE;
      l1basicinit->first_sub_sbs_last = TRUE;
      l1basicinit->reserved = 0xffffffffffff;
      l1basicinit->crc = 0;
      set_output_multiple(934 * 2);
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
      ninput_items_required[0] = 8100;
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
      int npad, padbits, count, nrepeat;
      int block, indexb, nouter, symbols;
      int npunctemp, npunc, nfectemp, nfec;
      int B, mod, rows, pack;
      long long templong;
      std::bitset<MAX_BCH_PARITY_BITS> parity_bits;
      unsigned char b, tempbch, msb;
      unsigned char *l1basic = l1_basic;
      unsigned char *l1temp = l1_temp;
      L1_Basic *l1basicinit = &L1_Signalling[0].l1basic_data;
      const unsigned char* d;
      int plen = FRAME_SIZE_SHORT - NBCH_3_15;
      const int q1 = q1_val;
      const int q2 = q2_val;
      const int m1 = m1_val;
      const unsigned char *c1, *c2, *c3, *c4, *c5, *c6, *c7, *c8;

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

      /* scrambling and BCH encoding */
      for (int i = 0; i < offset_bits; i += 8) {
        temp = index = 0;
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

      /* zero padding */
      nouter = offset_bits + num_parity_bits;
      npad = (NBCH_3_15 - nouter) / 360;
      memset(&l1temp[0], 0x55, sizeof(unsigned char) * FRAME_SIZE_SHORT);
      for (int i = 0; i < npad; i++) {
        memset(&l1temp[shortening_table[0][i] * 360], 0, sizeof(unsigned char) * 360);
      }
      padbits = (NBCH_3_15 - nouter) - (360 * npad);
      memset(&l1temp[shortening_table[0][npad] * 360], 0, sizeof(unsigned char) * padbits);
      index = 0;
      for (int i = npad + 1; i < 9; i++) {
        memcpy(&l1temp[shortening_table[0][i] * 360], &l1basic[index], sizeof(unsigned char) * offset_bits);
        index += 360;
      }
      index = count = 0;
      for (int n = 0; n < NBCH_3_15; n++) {
        if (l1temp[index] == 0x55) {
          l1temp[index++] = (char)parity_bits[num_parity_bits - 1];
          parity_bits <<= 1;
          count++;
          if (count == num_parity_bits) {
            break;
          }
        }
        else {
          index++;
        }
      }

      /* LDPC encoding */
      memcpy(&l1basic[0], &l1temp[0], sizeof(unsigned char) * NBCH_3_15);
      // First zero all the parity bits
      memset(buffer, 0, sizeof(unsigned char)*plen);
      // now do the parity checking
      d = &l1basic[0];
      for (int j = 0; j < ldpc_encode_1st.table_length; j++) {
        buffer[ldpc_encode_1st.p[j]] ^= d[ldpc_encode_1st.d[j]];
      }
      for(int j = 1; j < m1; j++) {
        buffer[j] ^= buffer[j-1];
      }
      for (int t = 0; t < q1; t++) {
        for (int s = 0; s < 360; s++) {
          l1basic[NBCH_3_15 + (360 * t) + s] = buffer[(q1 * s) + t];
        }
      }
      for (int j = 0; j < ldpc_encode_2nd.table_length; j++) {
        buffer[ldpc_encode_2nd.p[j]] ^= d[ldpc_encode_2nd.d[j]];
      }
      for (int t = 0; t < q2; t++) {
        for (int s = 0; s < 360; s++) {
          l1basic[NBCH_3_15 + m1 + (360 * t) + s] = buffer[(m1 + q2 * s) + t];
        }
      }

      /* group-wise interleaver */
      memcpy(&l1temp[0], &l1basic[0], sizeof(unsigned char) * NBCH_3_15);
      index = NBCH_3_15;
      for (int j = 0; j < 36; j++) {
        block = group_table_basic[j];
        indexb = block * 360;
        for (int k = 0; k < 360; k++) {
          l1temp[index++] = l1basic[indexb++];;
        }
      }

      /* repetition and parity puncturing */
      switch (l1b_mode) {
        case L1_FEC_MODE_1:
          B = 9360;
          mod = 2;
          break;
        case L1_FEC_MODE_2:
          B = 11460;
          mod = 2;
          break;
        case L1_FEC_MODE_3:
          B = 12360;
          mod = 2;
          break;
        case L1_FEC_MODE_4:
          mod = 4;
          B = 12292;
          break;
        case L1_FEC_MODE_5:
          B = 12350;
          mod = 6;
          break;
        case L1_FEC_MODE_6:
          mod = 8;
          B = 12432;
          break;
        case L1_FEC_MODE_7:
          mod = 8;
          B = 12766;
          break;
        default:
          mod = 2;
          B = 9360;
          break;
      }
      if (l1b_mode == L1_FEC_MODE_1) {
        nrepeat = 2 * (0 * nouter) + 3672;
      }
      else {
        nrepeat = 0;
      }
      npunctemp = floor(0 * (NBCH_3_15 - nouter)) + B;
      nfectemp = nouter + 12960 - npunctemp;
      nfec = ceil(nfectemp / mod) * mod;
      npunc = npunctemp - (nfec - nfectemp);
      symbols = nfec + nrepeat;
      memcpy(&l1basic[0], &l1temp[0], sizeof(unsigned char) * NBCH_3_15);
      memcpy(&l1basic[NBCH_3_15], &l1temp[NBCH_3_15], sizeof(unsigned char) * nrepeat);
      memcpy(&l1basic[NBCH_3_15 + nrepeat], &l1temp[NBCH_3_15], sizeof(unsigned char) * (FRAME_SIZE_SHORT - NBCH_3_15 - npunc));

      /* zero removal */
      for (int i = 0; i < npad; i++) {
        memset(&l1basic[shortening_table[0][i] * 360], 0x55, sizeof(unsigned char) * 360);
      }
      memset(&l1basic[shortening_table[0][npad] * 360], 0x55, sizeof(unsigned char) * padbits);
      index = count = 0;
      for (int i = 0; i < NBCH_3_15; i++) {
        if (l1basic[index] != 0x55) {
          l1temp[count++] = l1basic[index++];
        }
        else {
          index++;
        }
      }
      memcpy(&l1temp[count], &l1basic[NBCH_3_15], sizeof(unsigned char) * (symbols - count));

      /* block interleaver, bit demuxing and constellation mapping */
      rows = symbols / mod;
      switch (l1b_mode) {
        case L1_FEC_MODE_1:
        case L1_FEC_MODE_2:
        case L1_FEC_MODE_3:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1basic[index++] = c1[j];
            l1basic[index++] = c2[j];
          }
          index = 0;
          for (int j = 0; j < rows; j++) {
            temp = l1basic[index++] << 1;
            temp |= l1basic[index++];
            *out++ = m_qpsk[temp];
          }
          break;
        case L1_FEC_MODE_4:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          c3 = &l1temp[rows * 2];
          c4 = &l1temp[rows * 3];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1basic[index++] = c1[j];
            l1basic[index++] = c2[j];
            l1basic[index++] = c3[j];
            l1basic[index++] = c4[j];
          }
          index = count = 0;
          for (int j = 0; j < rows; j++) {
            pack = 0;
            for (int e = 3; e >= 0 ; e--) {
              pack |= l1basic[index++] << e;
            }
            temp = pack << count;
            pack = temp & 0xf;
            temp >>= 4;
            pack |= temp;
            count = (count + 1) & 0x3;
            *out++ = m_16qam[pack & 0xf];
          }
          break;
        case L1_FEC_MODE_5:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          c3 = &l1temp[rows * 2];
          c4 = &l1temp[rows * 3];
          c5 = &l1temp[rows * 4];
          c6 = &l1temp[rows * 5];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1basic[index++] = c1[j];
            l1basic[index++] = c2[j];
            l1basic[index++] = c3[j];
            l1basic[index++] = c4[j];
            l1basic[index++] = c5[j];
            l1basic[index++] = c6[j];
          }
          index = count = 0;
          for (int j = 0; j < rows; j++) {
            pack = 0;
            for (int e = 5; e >= 0 ; e--) {
              pack |= l1basic[index++] << e;
            }
            temp = pack << count;
            pack = temp & 0x3f;
            temp >>= 6;
            pack |= temp;
            count = (count + 1);
            if (count == 6) {  /* faster than modulo */
              count = 0;
            }
            *out++ = m_64qam[pack & 0x3f];
          }
          break;
        case L1_FEC_MODE_6:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          c3 = &l1temp[rows * 2];
          c4 = &l1temp[rows * 3];
          c5 = &l1temp[rows * 4];
          c6 = &l1temp[rows * 5];
          c7 = &l1temp[rows * 6];
          c8 = &l1temp[rows * 7];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1basic[index++] = c1[j];
            l1basic[index++] = c2[j];
            l1basic[index++] = c3[j];
            l1basic[index++] = c4[j];
            l1basic[index++] = c5[j];
            l1basic[index++] = c6[j];
            l1basic[index++] = c7[j];
            l1basic[index++] = c8[j];
          }
          index = count = 0;
          for (int j = 0; j < rows; j++) {
            pack = 0;
            for (int e = 7; e >= 0 ; e--) {
              pack |= l1basic[index++] << e;
            }
            temp = pack << count;
            pack = temp & 0xff;
            temp >>= 8;
            pack |= temp;
            count = (count + 1) & 0x7;
            *out++ = m_l1b_256qam[pack & 0xff];
          }
          break;
        case L1_FEC_MODE_7:
          c1 = &l1temp[0];
          c2 = &l1temp[rows];
          c3 = &l1temp[rows * 2];
          c4 = &l1temp[rows * 3];
          c5 = &l1temp[rows * 4];
          c6 = &l1temp[rows * 5];
          c7 = &l1temp[rows * 6];
          c8 = &l1temp[rows * 7];
          index = 0;
          for (int j = 0; j < rows; j++) {
            l1basic[index++] = c1[j];
            l1basic[index++] = c2[j];
            l1basic[index++] = c3[j];
            l1basic[index++] = c4[j];
            l1basic[index++] = c5[j];
            l1basic[index++] = c6[j];
            l1basic[index++] = c7[j];
            l1basic[index++] = c8[j];
          }
          index = count = 0;
          for (int j = 0; j < rows; j++) {
            pack = 0;
            for (int e = 7; e >= 0 ; e--) {
              pack |= l1basic[index++] << e;
            }
            temp = pack << count;
            pack = temp & 0xff;
            temp >>= 8;
            pack |= temp;
            count = (count + 1) & 0x7;
            *out++ = m_l1b_256qam[pack & 0xff];
          }
          break;
        default:
          c1 = &l1temp[0];
          c2 = &l1temp[symbols / mod];
          index = 0;
          for (int j = 0; j < symbols / mod; j++) {
            l1basic[index++] = c1[j];
            l1basic[index++] = c2[j];
          }
          index = 0;
          for (int j = 0; j < rows; j++) {
            temp = l1basic[index++] << 1;
            temp |= l1basic[index++];
            *out++ = m_qpsk[temp];
          }
          break;
      }
    }

    void
    framemapper_cc_impl::add_l1detail(gr_complex *out)
    {
#if 0
#if 1
      for (int i = 0; i < symbols; i += 8) {
        temp = index = 0;
        for (int j = 7; j >= 0; j--) {
          temp |= l1temp[i + index] << j;
          index++;
        }
        if ((i % 128) == 0) {
          if (i != 0) {
            printf("\n");
          }
        }
        printf("%02X", temp);
      }
      printf("\n");
#else
      index = 0;
      for (int i = 0; i < FRAME_SIZE_SHORT - NBCH_3_15 + nrepeat - npunc + count; i++) {
        if ((i % 64) == 0) {
          if (i != 0) {
            printf("\n");
          }
        }
        printf("%d", l1temp[index++]);
      }
#endif
#endif
    }

    int
    framemapper_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);

      for (int i = 0; i < noutput_items; i += 7640) {
        add_l1basic(out);
      }

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    const int framemapper_cc_impl::shortening_table[8][18] = {
      {4, 1, 5, 2, 8, 6, 0, 7, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {7, 8, 5, 4, 1, 2, 6, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {6, 1, 7, 8, 0, 2, 4, 3, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 12, 15, 13, 2, 5, 7, 9, 8, 6, 16, 10, 14, 1, 17, 11, 4, 3},
      {0, 15, 5, 16, 17, 1, 6, 13, 11, 4, 7, 12, 8, 14, 2, 3, 9, 10},
      {2, 4, 5, 17, 9, 7, 1, 6, 15, 8, 10, 14, 16, 0, 11, 13, 12, 3},
      {0, 15, 5, 16, 17, 1, 6, 13, 11, 4, 7, 12, 8, 14, 2, 3, 9, 10},
      {15, 7, 8, 11, 5, 10, 16, 4, 12, 3, 0, 6, 9, 1, 14, 17, 2, 13}
    };

    const uint16_t framemapper_cc_impl::ldpc_tab_3_15S[12][12] = {
      {11, 8, 372, 841, 4522, 5253, 7430, 8542, 9822, 10550, 11896, 11988},
      {11, 80, 255, 667, 1511, 3549, 5239, 5422, 5497, 7157, 7854, 11267},
      {11, 257, 406, 792, 2916, 3072, 3214, 3638, 4090, 8175, 8892, 9003},
      {11, 80, 150, 346, 1883, 6838, 7818, 9482, 10366, 10514, 11468, 12341},
      {11, 32, 100, 978, 3493, 6751, 7787, 8496, 10170, 10318, 10451, 12561},
      {11, 504, 803, 856, 2048, 6775, 7631, 8110, 8221, 8371, 9443, 10990},
      {11, 152, 283, 696, 1164, 4514, 4649, 7260, 7370, 11925, 11986, 12092},
      {11, 127, 1034, 1044, 1842, 3184, 3397, 5931, 7577, 11898, 12339, 12689},
      {11, 107, 513, 979, 3934, 4374, 4658, 7286, 7809, 8830, 10804, 10893},
      {10, 2045, 2499, 7197, 8887, 9420, 9922, 10132, 10540, 10816, 11876, 0},
      {10, 2932, 6241, 7136, 7835, 8541, 9403, 9817, 11679, 12377, 12810, 0},
      {10, 2211, 2288, 3937, 4310, 5952, 6597, 9692, 10445, 11064, 11272, 0}
    };

    const int framemapper_cc_impl::group_table_basic[36] = {
      20, 23, 25, 32, 38, 41, 18, 9, 10, 11, 31, 24,
      14, 15, 26, 40, 33, 19, 28, 34, 16, 39, 27, 30,
      21, 44, 43, 35, 42, 36, 12, 13, 29, 22, 37, 17
    };

    const gr_complex framemapper_cc_impl::mod_table_16QAM[4] = {
      gr_complex(0.2535, 0.4923), gr_complex(0.4923, 0.2535), gr_complex(0.4927, 1.2044), gr_complex(1.2044, 0.4927)
    };

    const gr_complex framemapper_cc_impl::mod_table_64QAM[16] = {
      gr_complex(0.1305, 0.3311), gr_complex(0.1633, 0.3162), gr_complex(0.1622, 0.7113), gr_complex(0.3905, 0.6163),
      gr_complex(0.3311, 0.1305), gr_complex(0.3162, 0.1633), gr_complex(0.7113, 0.1622), gr_complex(0.6163, 0.3905),
      gr_complex(0.2909, 1.4626), gr_complex(0.8285, 1.2399), gr_complex(0.2062, 1.0367), gr_complex(0.5872, 0.8789),
      gr_complex(1.4626, 0.2909), gr_complex(1.2399, 0.8285), gr_complex(1.0367, 0.2062), gr_complex(0.8789, 0.5872)
    };

    const gr_complex framemapper_cc_impl::mod_table_256QAM[2][64] = {
      {gr_complex(0.0899, 0.1337), gr_complex(0.0910, 0.1377), gr_complex(0.0873, 0.3862), gr_complex(0.0883, 0.3873),
       gr_complex(0.1115, 0.1442), gr_complex(0.1135, 0.1472), gr_complex(0.2067, 0.3591), gr_complex(0.1975, 0.3621),
       gr_complex(0.1048, 0.7533), gr_complex(0.1770, 0.7412), gr_complex(0.1022, 0.5904), gr_complex(0.1191, 0.5890),
       gr_complex(0.4264, 0.6230), gr_complex(0.3650, 0.6689), gr_complex(0.3254, 0.5153), gr_complex(0.2959, 0.5302),
       gr_complex(0.3256, 0.0768), gr_complex(0.3266, 0.0870), gr_complex(0.4721, 0.0994), gr_complex(0.4721, 0.1206),
       gr_complex(0.2927, 0.1267), gr_complex(0.2947, 0.1296), gr_complex(0.3823, 0.2592), gr_complex(0.3944, 0.2521),
       gr_complex(0.7755, 0.1118), gr_complex(0.7513, 0.2154), gr_complex(0.6591, 0.1033), gr_complex(0.6446, 0.1737),
       gr_complex(0.5906, 0.4930), gr_complex(0.6538, 0.4155), gr_complex(0.4981, 0.3921), gr_complex(0.5373, 0.3586),
       gr_complex(0.1630, 1.6621), gr_complex(0.4720, 1.5898), gr_complex(0.1268, 1.3488), gr_complex(0.3752, 1.2961),
       gr_complex(1.0398, 1.2991), gr_complex(0.7733, 1.4772), gr_complex(0.8380, 1.0552), gr_complex(0.6242, 1.2081),
       gr_complex(0.1103, 0.9397), gr_complex(0.2415, 0.9155), gr_complex(0.1118, 1.1163), gr_complex(0.3079, 1.0866),
       gr_complex(0.5647, 0.7638), gr_complex(0.4385, 0.8433), gr_complex(0.6846, 0.8841), gr_complex(0.5165, 1.0034),
       gr_complex(1.6489, 0.1630), gr_complex(1.5848, 0.4983), gr_complex(1.3437, 0.1389), gr_complex(1.2850, 0.4025),
       gr_complex(1.2728, 1.0661), gr_complex(1.4509, 0.7925), gr_complex(1.0249, 0.8794), gr_complex(1.1758, 0.6545),
       gr_complex(0.9629, 0.1113), gr_complex(0.9226, 0.2849), gr_complex(1.1062, 0.1118), gr_complex(1.0674, 0.3393),
       gr_complex(0.7234, 0.6223), gr_complex(0.8211, 0.4860), gr_complex(0.8457, 0.7260), gr_complex(0.9640, 0.5518)},
      {gr_complex(1.2412, 1.0688), gr_complex(1.2668, 0.8034), gr_complex(0.9860, 1.1758), gr_complex(1.0365, 0.9065),
       gr_complex(1.2111, 0.5135), gr_complex(1.4187, 0.6066), gr_complex(1.0103, 0.4879), gr_complex(1.0380, 0.6906),
       gr_complex(0.6963, 1.3442), gr_complex(0.7089, 1.1122), gr_complex(0.1256, 1.4745), gr_complex(0.8331, 0.9455),
       gr_complex(0.6615, 0.6012), gr_complex(0.6894, 0.7594), gr_complex(0.8373, 0.5633), gr_complex(0.8552, 0.7410),
       gr_complex(1.2666, 0.1027), gr_complex(1.4915, 0.1198), gr_complex(1.0766, 0.0945), gr_complex(0.9007, 0.0848),
       gr_complex(1.2454, 0.3064), gr_complex(1.4646, 0.3600), gr_complex(1.0570, 0.2995), gr_complex(0.9140, 0.2530),
       gr_complex(0.5461, 0.0679), gr_complex(0.5681, 0.1947), gr_complex(0.6874, 0.0537), gr_complex(0.7375, 0.1492),
       gr_complex(0.6290, 0.4553), gr_complex(0.6007, 0.3177), gr_complex(0.7885, 0.4231), gr_complex(0.7627, 0.2849),
       gr_complex(0.0816, 1.1632), gr_complex(0.0830, 0.9813), gr_complex(0.2528, 1.2315), gr_complex(0.2502, 1.0100),
       gr_complex(0.0732, 0.6827), gr_complex(0.0811, 0.8293), gr_complex(0.2159, 0.6673), gr_complex(0.2359, 0.8283),
       gr_complex(0.4302, 1.4458), gr_complex(0.5852, 0.9680), gr_complex(0.4528, 1.2074), gr_complex(0.4167, 1.0099),
       gr_complex(0.5035, 0.6307), gr_complex(0.5359, 0.7954), gr_complex(0.3580, 0.6532), gr_complex(0.3841, 0.8207),
       gr_complex(0.0576, 0.0745), gr_complex(0.0581, 0.2241), gr_complex(0.1720, 0.0742), gr_complex(0.1753, 0.2222),
       gr_complex(0.0652, 0.5269), gr_complex(0.0611, 0.3767), gr_complex(0.1972, 0.5178), gr_complex(0.1836, 0.3695),
       gr_complex(0.4145, 0.0709), gr_complex(0.4266, 0.2100), gr_complex(0.2912, 0.0730), gr_complex(0.2982, 0.2177),
       gr_complex(0.4766, 0.4821), gr_complex(0.4497, 0.3448), gr_complex(0.3334, 0.5025), gr_complex(0.3125, 0.3601)}
    };

  } /* namespace atsc3 */
} /* namespace gr */
