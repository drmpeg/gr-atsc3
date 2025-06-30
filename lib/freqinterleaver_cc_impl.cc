/* -*- c++ -*- */
/*
 * Copyright 2021-2023 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "freqinterleaver_cc_impl.h"
#include "params.h"

namespace gr {
  namespace atsc3 {

    using input_type = gr_complex;
    using output_type = gr_complex;
    freqinterleaver_cc::sptr
    freqinterleaver_cc::make(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_first_sbs_t firstsbs, atsc3_frequency_interleaver_t mode, atsc3_reduced_carriers_t cred, atsc3_papr_t paprmode)
    {
      return gnuradio::make_block_sptr<freqinterleaver_cc_impl>(
        fftsize, numpayloadsyms, numpreamblesyms, guardinterval, pilotpattern, firstsbs, mode, cred, paprmode);
    }


    /*
     * The private constructor
     */
    freqinterleaver_cc_impl::freqinterleaver_cc_impl(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_first_sbs_t firstsbs, atsc3_frequency_interleaver_t mode, atsc3_reduced_carriers_t cred, atsc3_papr_t paprmode)
      : gr::sync_block("freqinterleaver_cc",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      int total_preamble_cells, totalcells;
      int first_preamble_cells;
      int preamble_cells;
      int data_cells;
      int sbs_cells;
      int papr_cells;
      int maxstates;

      fft_size = fftsize;
      interleaver_mode = mode;
      symbols = numpreamblesyms + numpayloadsyms;
      struct ofdm_params_t p = ofdm_params(fftsize, guardinterval, pilotpattern, (atsc3_scattered_pilot_boost_t) 0, cred);
      maxstates = p.fftsamples;
      papr_cells = p.papr_cells;
      first_preamble_cells = p.first_preamble_cells;
      preamble_cells = p.preamble_cells;
      data_cells = p.data_cells;
      sbs_cells = p.sbs_cells;
      if (paprmode != PAPR_TR) {
        papr_cells = 0;
      }
      frame_symbols[0] = PREAMBLE_SYMBOL;
      for (int n = 1; n < numpreamblesyms; n++) {
        frame_symbols[n] = PREAMBLE_SYMBOL;
      }
      if (firstsbs == SBS_ON) {
        frame_symbols[numpreamblesyms] = SBS_SYMBOL;
        for (int n = 0; n < numpayloadsyms - 2; n++) {
          frame_symbols[n + numpreamblesyms + 1] = DATA_SYMBOL;
        }
      }
      else {
        for (int n = 0; n < numpayloadsyms - 1; n++) {
          frame_symbols[n + numpreamblesyms] = DATA_SYMBOL;
        }
      }
      frame_symbols[numpreamblesyms + numpayloadsyms - 1] = SBS_SYMBOL;

      frame_cells[0] = first_preamble_cells;
      total_preamble_cells = 0;
      for (int n = 1; n < numpreamblesyms; n++) {
        frame_cells[n] = (preamble_cells - papr_cells);
        total_preamble_cells += (preamble_cells - papr_cells);
      }
      if (firstsbs == SBS_ON) {
        frame_cells[numpreamblesyms] = sbs_cells - papr_cells;
        for (int n = 0; n < numpayloadsyms - 2; n++) {
          frame_cells[n + numpreamblesyms + 1] = (data_cells - papr_cells);
        }
      }
      else {
        for (int n = 0; n < numpayloadsyms - 1; n++) {
          frame_cells[n + numpreamblesyms] = (data_cells - papr_cells);
        }
      }
      frame_cells[numpreamblesyms + numpayloadsyms - 1] = (sbs_cells - papr_cells);

      HevenFP.resize(symbols);
      HoddFP.resize(symbols);
      HevenP.resize(symbols);
      HoddP.resize(symbols);
      HevenSBS.resize(symbols);
      HoddSBS.resize(symbols);
      Heven.resize(symbols);
      Hodd.resize(symbols);
      for (std::vector<std::vector<int>>::size_type i = 0; i != Heven.size(); i++) {
        HevenFP[i].resize(maxstates);
        HoddFP[i].resize(maxstates);
        HevenP[i].resize(maxstates);
        HoddP[i].resize(maxstates);
        HevenSBS[i].resize(maxstates);
        HoddSBS[i].resize(maxstates);
        Heven[i].resize(maxstates);
        Hodd[i].resize(maxstates);
      }
      init_address(first_preamble_cells, preamble_cells - papr_cells, sbs_cells - papr_cells, data_cells - papr_cells);

      if (numpreamblesyms == 0) {
        first_preamble_cells = 0;
      }
      if (firstsbs == SBS_ON) {
        totalcells = first_preamble_cells + total_preamble_cells + ((numpayloadsyms - 2) * (data_cells - papr_cells)) + ((sbs_cells - papr_cells) * 2);
      }
      else {
        totalcells = first_preamble_cells + total_preamble_cells + ((numpayloadsyms - 1) * (data_cells - papr_cells)) + (sbs_cells - papr_cells);
      }
      output_cells = totalcells;
      printf("output cells = %d\n", totalcells);
      set_output_multiple(totalcells);
    }

    /*
     * Our virtual destructor.
     */
    freqinterleaver_cc_impl::~freqinterleaver_cc_impl()
    {
    }

    void
    freqinterleaver_cc_impl::init_address(int firstpreamblecells, int preamblecells, int sbscells, int datacells)
    {
      int max_states, xor_size, xor_size2, pn_mask, result;
      int q_evenFP;
      int q_oddFP;
      int q_evenP;
      int q_oddP;
      int q_evenSBS;
      int q_oddSBS;
      int q_even;
      int q_odd;
      int lfsr;
      int lfsr2 = 0;
      int logic8k[4] = {0, 1, 4, 6};
      int logic16k[6] = {0, 1, 4, 5, 9, 11};
      int logic32k[4] = {0, 1, 2, 12};
      int logic8k2[6] = {0, 1, 4, 5, 9, 11};
      int logic16k2[4] = {0, 1, 2, 12};
      int logic32k2[2] = {0, 1};
      int* logic;
      int* logic2;
      const int *bitpermeven, *bitpermodd;
      int pn_degree, even, odd;

      switch (fft_size) {
        case FFTSIZE_8K:
          pn_degree = 12;
          pn_mask = 0xfff;
          max_states = 8192;
          logic = &logic8k[0];
          logic2 = &logic8k2[0];
          xor_size = 4;
          xor_size2 = 6;
          bitpermeven = &bitperm8keven[0];
          bitpermodd = &bitperm8kodd[0];
          break;
        case FFTSIZE_16K:
          pn_degree = 13;
          pn_mask = 0x1fff;
          max_states = 16384;
          logic = &logic16k[0];
          logic2 = &logic16k2[0];
          xor_size = 6;
          xor_size2 = 4;
          bitpermeven = &bitperm16keven[0];
          bitpermodd = &bitperm16kodd[0];
          break;
        case FFTSIZE_32K:
          pn_degree = 14;
          pn_mask = 0x3fff;
          max_states = 32768;
          logic = &logic32k[0];
          logic2 = &logic32k2[0];
          xor_size = 4;
          xor_size2 = 2;
          bitpermeven = &bitperm32k[0];
          bitpermodd = &bitperm32k[0];
          break;
        default:
          pn_degree = 12;
          pn_mask = 0xfff;
          max_states = 8192;
          logic = &logic8k[0];
          logic2 = &logic8k2[0];
          xor_size = 4;
          xor_size2 = 6;
          bitpermeven = &bitperm8keven[0];
          bitpermodd = &bitperm8kodd[0];
          break;
      }

      for (int i = 0; i < symbols; i++) {
        std::vector<int>& HevenFP = this->HevenFP[i];
        std::vector<int>& HoddFP = this->HoddFP[i];
        std::vector<int>& HevenP = this->HevenP[i];
        std::vector<int>& HoddP = this->HoddP[i];
        std::vector<int>& HevenSBS = this->HevenSBS[i];
        std::vector<int>& HoddSBS = this->HoddSBS[i];
        std::vector<int>& Heven = this->Heven[i];
        std::vector<int>& Hodd = this->Hodd[i];
        q_evenFP = 0;
        q_oddFP = 0;
        q_evenP = 0;
        q_oddP = 0;
        q_evenSBS = 0;
        q_oddSBS = 0;
        q_even = 0;
        q_odd = 0;
        if ((i % 2) == 0) {
          if (i == 0) {
            lfsr2 = (pn_mask << 1) | 0x1;
          }
          else {
            result = 0;
            for (int k = 0; k < xor_size2; k++) {
              result ^= (lfsr2 >> logic2[k]) & 1;
            }
            lfsr2 &= (pn_mask << 1) | 0x1;
            lfsr2 >>= 1;
            lfsr2 |= result << (pn_degree);
          }
        }

        for (int j = 0; j < max_states; j++) {
          if (j == 0 || j == 1) {
            lfsr = 0;
          }
          else if (j == 2) {
            lfsr = 1;
          }
          else {
            result = 0;
            for (int k = 0; k < xor_size; k++) {
              result ^= (lfsr >> logic[k]) & 1;
            }
            lfsr &= pn_mask;
            lfsr >>= 1;
            lfsr |= result << (pn_degree - 1);
          }
          even = 0;
          odd = 0;
          for (int n = 0; n < pn_degree; n++) {
            even |= ((lfsr >> n) & 0x1) << bitpermeven[n];
          }
          for (int n = 0; n < pn_degree; n++) {
            odd |= ((lfsr >> n) & 0x1) << bitpermodd[n];
          }
          even = even + ((j % 2) * (max_states / 2));
          odd = odd + ((j % 2) * (max_states / 2));
          even ^= lfsr2;
          odd ^= lfsr2;
          if (even < firstpreamblecells) {
            HevenFP[q_evenFP++] = even;
          }
          if (odd < firstpreamblecells) {
            HoddFP[q_oddFP++] = odd;
          }
          if (even < preamblecells) {
            HevenP[q_evenP++] = even;
          }
          if (odd < preamblecells) {
            HoddP[q_oddP++] = odd;
          }
          if (even < sbscells) {
            HevenSBS[q_evenSBS++] = even;
          }
          if (odd < sbscells) {
            HoddSBS[q_oddSBS++] = odd;
          }
          if (even < datacells) {
            Heven[q_even++] = even;
          }
          if (odd < datacells) {
            Hodd[q_odd++] = odd;
          }
        }
        if (fft_size == FFTSIZE_32K) {
          for (int n = 0; n < q_oddFP; n++) {
            int a;
            a = HoddFP[n];
            HevenFP[a] = n;
          }
          for (int n = 0; n < q_oddP; n++) {
            int a;
            a = HoddP[n];
            HevenP[a] = n;
          }
          for (int n = 0; n < q_oddSBS; n++) {
            int a;
            a = HoddSBS[n];
            HevenSBS[a] = n;
          }
          for (int n = 0; n < q_odd; n++) {
            int a;
            a = Hodd[n];
            Heven[a] = n;
          }
        }
      }
    }

    int
    freqinterleaver_cc_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      std::vector<int> H;
      int index;

      for (int i = 0; i < noutput_items; i += output_cells) {
        for (int j = 0; j < symbols; j++) {
          if (frame_symbols[j] == PREAMBLE_SYMBOL) {
            if (j == 0) {
              if ((j % 2) == 0) {
                H = HevenFP[j];
              }
              else {
                H = HoddFP[j];
              }
            }
            else {
              if ((j % 2) == 0) {
                H = HevenP[j];
              }
              else {
                H = HoddP[j];
              }
            }
            for (int n = 0; n < frame_cells[j]; n++) {
              *out++ = in[H[n]];
            }
            in += frame_cells[j];
          }
          else if (interleaver_mode == FREQ_ALL_SYMBOLS) {
            if (frame_symbols[j] == SBS_SYMBOL) {
              if ((j % 2) == 0) {
                H = HevenSBS[j];
              }
              else {
                H = HoddSBS[j];
              }
            }
            else {
              if ((j % 2) == 0) {
                H = Heven[j];
              }
              else {
                H = Hodd[j];
              }
            }
            for (int n = 0; n < frame_cells[j]; n++) {
              *out++ = in[H[n]];
            }
            in += frame_cells[j];
          }
          else {
            index = 0;
            for (int n = 0; n < frame_cells[j]; n++) {
              *out++ = in[index++];
            }
            in += frame_cells[j];
          }
        }
      }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    const int freqinterleaver_cc_impl::bitperm8keven[12] = {7, 1, 4, 2, 9, 6, 8, 10, 0, 3, 11, 5
    };

    const int freqinterleaver_cc_impl::bitperm8kodd[12] = {11, 4, 9, 3, 1, 2, 5, 0, 6, 7, 10, 8
    };

    const int freqinterleaver_cc_impl::bitperm16keven[13] = {9, 7, 6, 10, 12, 5, 1, 11, 0, 2, 3, 4, 8
    };

    const int freqinterleaver_cc_impl::bitperm16kodd[13] = {6, 8, 10, 12, 2, 0, 4, 1, 11, 3, 5, 9, 7
    };

    const int freqinterleaver_cc_impl::bitperm32k[14] = {7, 13, 3, 4, 9, 2, 12, 11, 1, 8, 10, 0, 5, 6
    };

  } /* namespace atsc3 */
} /* namespace gr */
