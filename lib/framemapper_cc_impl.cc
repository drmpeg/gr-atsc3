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
    framemapper_cc_impl::add_l1basic(gr_complex *out)
    {
      int temp, offset_bits = 0;
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
      temp = l1basicinit->reserved;
      for (int n = 47; n >= 0; n--) {
        l1basic[offset_bits++] = temp & (1 << n) ? 1 : 0;
      }
      offset_bits += add_crc32_bits(l1basic, offset_bits);
      printf("\n");
      for (int i = 0; i < 25 * 8; i += 8) {
        temp = 0;
        int index = 0;
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
