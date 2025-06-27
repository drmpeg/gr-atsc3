/* -*- c++ -*- */
/*
 * Copyright 2021-2023 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "pilotgenerator_cc_impl.h"
#include "params.h"
#include <volk/volk.h>

namespace gr {
  namespace atsc3 {

    using input_type = gr_complex;
    using output_type = gr_complex;
    pilotgenerator_cc::sptr
    pilotgenerator_cc::make(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_scattered_pilot_boost_t pilotboost, atsc3_first_sbs_t firstsbs, atsc3_reduced_carriers_t cred, atsc3_miso_t misomode, atsc3_miso_tx_t misotxid, atsc3_papr_t paprmode, atsc3_pilotgenerator_mode_t outputmode, unsigned int fftlength, unsigned int vlength)
    {
      return gnuradio::make_block_sptr<pilotgenerator_cc_impl>(
        fftsize, numpayloadsyms, numpreamblesyms, guardinterval, pilotpattern, pilotboost, firstsbs, cred, misomode, misotxid, paprmode, outputmode, fftlength, vlength);
    }


    /*
     * The private constructor
     */
    pilotgenerator_cc_impl::pilotgenerator_cc_impl(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_guardinterval_t guardinterval, atsc3_pilotpattern_t pilotpattern, atsc3_scattered_pilot_boost_t pilotboost, atsc3_first_sbs_t firstsbs, atsc3_reduced_carriers_t cred, atsc3_miso_t misomode, atsc3_miso_tx_t misotxid, atsc3_papr_t paprmode, atsc3_pilotgenerator_mode_t outputmode, unsigned int fftlength, unsigned int vlength)
      : gr::block("pilotgenerator_cc",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type) * vlength)),
        ofdm_fft(fftlength, 1),
        miso_fft(fftlength, 1),
        ofdm_fft_size(fftlength)
    {
      double power, preamble_power, scattered_power;
      double preamble_ifft_power, data_ifft_power;
      double first_preamble_ifft_power;
      int total_preamble_cells, totalcells;
      int first_preamble_cells;
      int preamble_cells;
      int data_cells;
      int sbs_cells;
      int papr_cells;
      gr_complex* dst;
      gr_complex* src;
      float angle;

      fft_size = fftsize;
      pilot_pattern = pilotpattern;
      papr_mode = paprmode;
      miso_mode = misomode;
      output_mode = outputmode;
      cred_coeff = cred;
      symbols = numpreamblesyms + numpayloadsyms;
      preamble_symbols = numpreamblesyms;
      struct ofdm_params_t p = ofdm_params(fftsize, guardinterval, pilotpattern, pilotboost, cred);
      papr_cells = p.papr_cells;
      carriers = p.carriers;
      max_carriers = p.max_carriers;
      preamble_carriers = p.preamble_carriers;
      first_preamble_cells = p.first_preamble_cells;
      preamble_cells = p.preamble_cells;
      preamble_dx = p.preamble_dx;
      preamble_power = p.preamble_power;
      preamble_ifft_power = p.preamble_ifft_power;
      first_preamble_ifft_power = p.first_preamble_ifft_power;
      data_ifft_power = p.data_ifft_power;
      data_cells = p.data_cells;
      sbs_cells = p.sbs_cells;
      switch (pilotpattern) {
        case PILOT_SP3_2:
          dx = 3;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP3_2][pilotboost];
          break;
        case PILOT_SP3_4:
          dx = 3;
          dy = 4;
          scattered_power = scattered_power_table[PILOT_SP3_4][pilotboost];
          break;
        case PILOT_SP4_2:
          dx = 4;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP4_2][pilotboost];
          break;
        case PILOT_SP4_4:
          dx = 4;
          dy = 4;
          scattered_power = scattered_power_table[PILOT_SP4_4][pilotboost];
          break;
        case PILOT_SP6_2:
          dx = 6;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP6_2][pilotboost];
          break;
        case PILOT_SP6_4:
          dx = 6;
          dy = 4;
          scattered_power = scattered_power_table[PILOT_SP6_4][pilotboost];
          break;
        case PILOT_SP8_2:
          dx = 8;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP8_2][pilotboost];
          break;
        case PILOT_SP8_4:
          dx = 8;
          dy = 4;
          scattered_power = scattered_power_table[PILOT_SP8_4][pilotboost];
          break;
        case PILOT_SP12_2:
          dx = 12;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP12_2][pilotboost];
          break;
        case PILOT_SP12_4:
          dx = 12;
          dy = 4;
          scattered_power = scattered_power_table[PILOT_SP12_4][pilotboost];
          break;
        case PILOT_SP16_2:
          dx = 16;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP16_2][pilotboost];
          break;
        case PILOT_SP16_4:
          dx = 16;
          dy = 4;
          scattered_power = scattered_power_table[PILOT_SP16_4][pilotboost];
          break;
        case PILOT_SP24_2:
          dx = 24;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP24_2][pilotboost];
          break;
        case PILOT_SP24_4:
          dx = 24;
          dy = 4;
          scattered_power = scattered_power_table[PILOT_SP24_4][pilotboost];
          break;
        case PILOT_SP32_2:
          dx = 32;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP32_2][pilotboost];
          break;
        case PILOT_SP32_4:
          dx = 32;
          dy = 4;
          scattered_power = scattered_power_table[PILOT_SP32_4][pilotboost];
          break;
        default:
          dx = 3;
          dy = 2;
          scattered_power = scattered_power_table[PILOT_SP3_2][pilotboost];
          break;
      }
      power = pow(10, preamble_power / 20.0);
      pr_bpsk[0] = gr_complex(power, 0.0);
      pr_bpsk[1] = gr_complex(-(power), 0.0);
      power = pow(10, scattered_power / 20.0);
      sp_bpsk[0] = gr_complex(power, 0.0);
      sp_bpsk[1] = gr_complex(-(power), 0.0);
      power = pow(10, 8.52 / 20.0);
      cp_bpsk[0] = gr_complex(power, 0.0);
      cp_bpsk[1] = gr_complex(-(power), 0.0);
      init_prbs();
      if (paprmode != PAPR_TR) {
        papr_cells = 0;
      }
      frame_symbols[0] = PREAMBLE_SYMBOL;
      total_preamble_cells = 0;
      for (int n = 1; n < numpreamblesyms; n++) {
        frame_symbols[n] = PREAMBLE_SYMBOL;
        total_preamble_cells += (preamble_cells - papr_cells);
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
      data_carrier_map.resize(symbols);
      for (std::vector<std::vector<int>>::size_type i = 0; i != data_carrier_map.size(); i++) {
        data_carrier_map[i].resize(max_carriers);
      }
      init_pilots();
      if (numpreamblesyms == 0) {
        first_preamble_cells = 0;
      }
      if (firstsbs == SBS_ON) {
        totalcells = first_preamble_cells + total_preamble_cells + ((numpayloadsyms - 2) * (data_cells - papr_cells)) + ((sbs_cells - papr_cells) * 2);
      }
      else {
        totalcells = first_preamble_cells + total_preamble_cells + ((numpayloadsyms - 1) * (data_cells - papr_cells)) + (sbs_cells - papr_cells);
      }
      input_cells = totalcells;
      printf("input cells = %d\n", input_cells);
      first_preamble_normalization = 1.0 / std::sqrt(first_preamble_ifft_power);
      preamble_normalization = 1.0 / std::sqrt(preamble_ifft_power);
      data_normalization = 1.0 / std::sqrt(data_ifft_power);
      if (misomode == MISO_64) {
        miso_size = 64;
        switch (misotxid) {
          case MISO_TX_1_OF_2:
            miso_coefficients = &miso_coefficients_64_1_2[0];
            break;
          case MISO_TX_2_OF_2:
            miso_coefficients = &miso_coefficients_64_2_2[0];
            break;
          case MISO_TX_1_OF_3:
            miso_coefficients = &miso_coefficients_64_1_3[0];
            break;
          case MISO_TX_2_OF_3:
            miso_coefficients = &miso_coefficients_64_2_3[0];
            break;
          case MISO_TX_3_OF_3:
            miso_coefficients = &miso_coefficients_64_3_3[0];
            break;
          case MISO_TX_1_OF_4:
            miso_coefficients = &miso_coefficients_64_1_4[0];
            break;
          case MISO_TX_2_OF_4:
            miso_coefficients = &miso_coefficients_64_2_4[0];
            break;
          case MISO_TX_3_OF_4:
            miso_coefficients = &miso_coefficients_64_3_4[0];
            break;
          case MISO_TX_4_OF_4:
            miso_coefficients = &miso_coefficients_64_4_4[0];
            break;
        }
      }
      else if (misomode == MISO_256) {
        miso_size = 256;
        switch (misotxid) {
          case MISO_TX_1_OF_2:
            miso_coefficients = &miso_coefficients_256_1_2[0];
            break;
          case MISO_TX_2_OF_2:
            miso_coefficients = &miso_coefficients_256_2_2[0];
            break;
          case MISO_TX_1_OF_3:
            miso_coefficients = &miso_coefficients_256_1_3[0];
            break;
          case MISO_TX_2_OF_3:
            miso_coefficients = &miso_coefficients_256_2_3[0];
            break;
          case MISO_TX_3_OF_3:
            miso_coefficients = &miso_coefficients_256_3_3[0];
            break;
          case MISO_TX_1_OF_4:
            miso_coefficients = &miso_coefficients_256_1_4[0];
            break;
          case MISO_TX_2_OF_4:
            miso_coefficients = &miso_coefficients_256_2_4[0];
            break;
          case MISO_TX_3_OF_4:
            miso_coefficients = &miso_coefficients_256_3_4[0];
            break;
          case MISO_TX_4_OF_4:
            miso_coefficients = &miso_coefficients_256_4_4[0];
            break;
        }
      }
      if (misomode != MISO_OFF) {
        miso_rotation.resize(ofdm_fft_size);
        dst = miso_fft.get_inbuf();
        memcpy(&dst[0], &miso_coefficients[0], sizeof(gr_complex) * miso_size);
        std::fill_n(&dst[miso_size], ofdm_fft_size - miso_size, 0);
        miso_fft.execute();
        src = miso_fft.get_outbuf();
        for (int n = 0; n < ofdm_fft_size; n++) {
          angle = std::arg(src[n]);
          miso_rotation[n] = std::exp(gr_complexd(0.0, angle));
        }
      }
      if (outputmode == PILOTGENERATOR_TIME) {
        insertion_items = symbols;
      }
      else {
        insertion_items = ((carriers * (symbols - 1)) + preamble_carriers);
      }
      set_output_multiple(insertion_items);
    }

    /*
     * Our virtual destructor.
     */
    pilotgenerator_cc_impl::~pilotgenerator_cc_impl()
    {
    }

    void
    pilotgenerator_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = input_cells * (noutput_items / insertion_items);
    }

    void
    pilotgenerator_cc_impl::init_prbs(void)
    {
      int sr = 0x1b;

      for (int i = 0; i < MAX_CARRIERS; i++) {
        int b = ((sr) ^ (sr >> 1) ^ (sr >> 3) ^ (sr >> 4)) & 1;
        prbs[i] = sr & 1;
        sr >>= 1;
        if (b) {
          sr |= 0x1000;
        }
      }
    }

    void
    pilotgenerator_cc_impl::init_pilots()
    {
      for (int symbol = 0; symbol < symbols; ++symbol) {
        int remainder, shift, index, preamblecarriers, trshift;
        std::vector<int>& data_carrier_map = this->data_carrier_map[symbol];
        for (int i = 0; i < carriers; i++) {
          data_carrier_map[i] = DATA_CARRIER;
        }
        trshift = dx * ((symbol - (preamble_symbols - dy)) % dy);
        if (frame_symbols[symbol] == SBS_SYMBOL || frame_symbols[symbol] == PREAMBLE_SYMBOL) {
          trshift = 0;
        }
        switch (fft_size) {
          case FFTSIZE_8K:
            if (frame_symbols[symbol] == PREAMBLE_SYMBOL) {
              index = 0;
              if (symbol == 0) {
                preamblecarriers = preamble_carriers;
                shift = (max_carriers - preamble_carriers) / 2;
              }
              else {
                preamblecarriers = carriers;
                shift = (max_carriers - carriers) / 2;
              }
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_8K[index] == i) {
                  if (continual_pilot_table_8K[index] > shift) {
                    data_carrier_map[i - shift] = CONTINUAL_CARRIER;
                  }
                  index++;
                }
              }
              for (int i = 0; i < preamblecarriers; i++) {
                if ((i % preamble_dx) == 0) {
                  data_carrier_map[i] = PREAMBLE_CARRIER;
                }
              }
              if (symbol != 0 && papr_mode == PAPR_TR) {
                index = 0;
                if (preamble_dx == 3 || preamble_dx == 4 || preamble_dx == 8) {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_alt_table_8K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
                else {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_table_8K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
              }
            }
            else if (frame_symbols[symbol] == SBS_SYMBOL) {
              index = 0;
              shift = (max_carriers - carriers) / 2;
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_8K[index] == i) {
                  if (continual_pilot_table_8K[index] > shift) {
                    data_carrier_map[i - shift] = CONTINUAL_CARRIER;
                  }
                  index++;
                }
              }
              for (int i = 0; i < carriers; i++) {
                if ((i % dx) == 0) {
                  data_carrier_map[i] = SCATTERED_CARRIER;
                }
              }
              if (papr_mode == PAPR_TR) {
                index = 0;
                if (dx == 3 || dx == 4 || dx == 8) {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_alt_table_8K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
                else {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_table_8K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
              }
            }
            else {
              index = 0;
              shift = (max_carriers - carriers) / 2;
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_8K[index] == i) {
                  if (continual_pilot_table_8K[index] > shift) {
                    data_carrier_map[i - shift] = CONTINUAL_CARRIER;
                  }
                  index++;
                }
              }
              for (int i = 0; i < carriers; i++) {
                remainder = i % (dx * dy);
                if (remainder == (dx * ((symbol - preamble_symbols) % dy))) {
                  data_carrier_map[i] = SCATTERED_CARRIER;
                }
              }
              if (papr_mode == PAPR_TR) {
                index = 0;
                for (int i = 0; i < max_carriers; i++) {
                  if ((trpapr_table_8K[index] + trshift) == i) {
                    data_carrier_map[i - shift] = TRPAPR_CARRIER;
                    index++;
                  }
                }
              }
            }
            if ((frame_symbols[symbol] == SBS_SYMBOL) || (frame_symbols[symbol] == DATA_SYMBOL)) {
              data_carrier_map[0] = SCATTERED_CARRIER;
              data_carrier_map[carriers - 1] = SCATTERED_CARRIER;
              switch (pilot_pattern) {
                case PILOT_SP3_2:
                  data_carrier_map[1731] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP3_4:
                  data_carrier_map[1731] = SCATTERED_CARRIER;
                  data_carrier_map[2886] = SCATTERED_CARRIER;
                  data_carrier_map[5733] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP4_2:
                  data_carrier_map[1732] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP4_4:
                  data_carrier_map[1732] = SCATTERED_CARRIER;
                  data_carrier_map[2888] = SCATTERED_CARRIER;
                  data_carrier_map[5724] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP6_2:
                  data_carrier_map[1734] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP6_4:
                  data_carrier_map[1734] = SCATTERED_CARRIER;
                  data_carrier_map[2892] = SCATTERED_CARRIER;
                  data_carrier_map[5730] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP8_2:
                  data_carrier_map[1736] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP8_4:
                  data_carrier_map[1736] = SCATTERED_CARRIER;
                  data_carrier_map[2896] = SCATTERED_CARRIER;
                  data_carrier_map[5720] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP12_2:
                  data_carrier_map[1740] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP12_4:
                  data_carrier_map[1740] = SCATTERED_CARRIER;
                  data_carrier_map[2904] = SCATTERED_CARRIER;
                  data_carrier_map[5748] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP16_2:
                  data_carrier_map[1744] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP16_4:
                  data_carrier_map[1744] = SCATTERED_CARRIER;
                  if ((cred_coeff & 0x1) == 0) {
                    data_carrier_map[2912] = SCATTERED_CARRIER;
                    data_carrier_map[5744] = SCATTERED_CARRIER;
                  }
                  break;
                case PILOT_SP24_2:
                  break;
                case PILOT_SP24_4:
                  break;
                case PILOT_SP32_2:
                  if ((cred_coeff & 0x1) == 0) {
                    data_carrier_map[1696] = SCATTERED_CARRIER;
                  }
                  break;
                case PILOT_SP32_4:
                  switch (cred_coeff) {
                    case CRED_0:
                      data_carrier_map[1696] = SCATTERED_CARRIER;
                      data_carrier_map[2880] = SCATTERED_CARRIER;
                      data_carrier_map[5728] = SCATTERED_CARRIER;
                      break;
                    case CRED_1:
                      break;
                    case CRED_2:
                      data_carrier_map[1696] = SCATTERED_CARRIER;
                      break;
                    case CRED_3:
                      data_carrier_map[1696] = SCATTERED_CARRIER;
                      data_carrier_map[2880] = SCATTERED_CARRIER;
                      break;
                    case CRED_4:
                      data_carrier_map[1696] = SCATTERED_CARRIER;
                      data_carrier_map[2880] = SCATTERED_CARRIER;
                      data_carrier_map[5728] = SCATTERED_CARRIER;
                      break;
                    default:
                      break;
                  }
                  break;
                default:
                  break;
              }
            }
            break;
          case FFTSIZE_16K:
            if (frame_symbols[symbol] == PREAMBLE_SYMBOL) {
              index = 0;
              if (symbol == 0) {
                preamblecarriers = preamble_carriers;
                shift = (max_carriers - preamble_carriers) / 2;
              }
              else {
                preamblecarriers = carriers;
                shift = (max_carriers - carriers) / 2;
              }
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_16K[index] == i) {
                  if (continual_pilot_table_16K[index] > shift) {
                    data_carrier_map[i - shift] = CONTINUAL_CARRIER;
                  }
                  index++;
                }
              }
              for (int i = 0; i < preamblecarriers; i++) {
                if ((i % preamble_dx) == 0) {
                  data_carrier_map[i] = PREAMBLE_CARRIER;
                }
              }
              if (symbol != 0 && papr_mode == PAPR_TR) {
                index = 0;
                if (preamble_dx == 3 || preamble_dx == 4 || preamble_dx == 8) {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_alt_table_16K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
                else {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_table_16K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
              }
            }
            else if (frame_symbols[symbol] == SBS_SYMBOL) {
              index = 0;
              shift = (max_carriers - carriers) / 2;
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_16K[index] == i) {
                  if (continual_pilot_table_16K[index] > shift) {
                    data_carrier_map[i - shift] = CONTINUAL_CARRIER;
                  }
                  index++;
                }
              }
              for (int i = 0; i < carriers; i++) {
                if ((i % dx) == 0) {
                  data_carrier_map[i] = SCATTERED_CARRIER;
                }
              }
              if (papr_mode == PAPR_TR) {
                index = 0;
                if (dx == 3 || dx == 4 || dx == 8) {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_alt_table_16K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
                else {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_table_16K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
              }
            }
            else {
              index = 0;
              shift = (max_carriers - carriers) / 2;
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_16K[index] == i) {
                  if (continual_pilot_table_16K[index] > shift) {
                    data_carrier_map[i - shift] = CONTINUAL_CARRIER;
                  }
                  index++;
                }
              }
              for (int i = 0; i < carriers; i++) {
                remainder = i % (dx * dy);
                if (remainder == (dx * ((symbol - preamble_symbols) % dy))) {
                  data_carrier_map[i] = SCATTERED_CARRIER;
                }
              }
              if (papr_mode == PAPR_TR) {
                index = 0;
                for (int i = 0; i < max_carriers; i++) {
                  if ((trpapr_table_16K[index] + trshift) == i) {
                    data_carrier_map[i - shift] = TRPAPR_CARRIER;
                    index++;
                  }
                }
              }
            }
            if ((frame_symbols[symbol] == SBS_SYMBOL) || (frame_symbols[symbol] == DATA_SYMBOL)) {
              data_carrier_map[0] = SCATTERED_CARRIER;
              data_carrier_map[carriers - 1] = SCATTERED_CARRIER;
              switch (pilot_pattern) {
                case PILOT_SP3_2:
                  data_carrier_map[3471] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP3_4:
                  data_carrier_map[3471] = SCATTERED_CARRIER;
                  data_carrier_map[5778] = SCATTERED_CARRIER;
                  data_carrier_map[11469] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP4_2:
                  data_carrier_map[3460] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP4_4:
                  data_carrier_map[3460] = SCATTERED_CARRIER;
                  data_carrier_map[5768] = SCATTERED_CARRIER;
                  data_carrier_map[11452] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP6_2:
                  data_carrier_map[3462] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP6_4:
                  data_carrier_map[3462] = SCATTERED_CARRIER;
                  data_carrier_map[5772] = SCATTERED_CARRIER;
                  data_carrier_map[11466] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP8_2:
                  data_carrier_map[3464] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP8_4:
                  data_carrier_map[3464] = SCATTERED_CARRIER;
                  data_carrier_map[5776] = SCATTERED_CARRIER;
                  data_carrier_map[11448] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP12_2:
                  data_carrier_map[3468] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP12_4:
                  data_carrier_map[3468] = SCATTERED_CARRIER;
                  data_carrier_map[5784] = SCATTERED_CARRIER;
                  data_carrier_map[11460] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP16_2:
                  data_carrier_map[3472] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP16_4:
                  data_carrier_map[3472] = SCATTERED_CARRIER;
                  data_carrier_map[5792] = SCATTERED_CARRIER;
                  data_carrier_map[11440] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP24_2:
                  data_carrier_map[3480] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP24_4:
                  data_carrier_map[3480] = SCATTERED_CARRIER;
                  data_carrier_map[5808] = SCATTERED_CARRIER;
                  data_carrier_map[11496] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP32_2:
                  data_carrier_map[3488] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP32_4:
                  data_carrier_map[3488] = SCATTERED_CARRIER;
                  if ((cred_coeff & 0x1) == 0) {
                    data_carrier_map[5824] = SCATTERED_CARRIER;
                    data_carrier_map[11488] = SCATTERED_CARRIER;
                  }
                  break;
                default:
                  break;
              }
            }
            break;
          case FFTSIZE_32K:
            if (frame_symbols[symbol] == PREAMBLE_SYMBOL) {
              index = 0;
              if (symbol == 0) {
                preamblecarriers = preamble_carriers;
                shift = (max_carriers - preamble_carriers) / 2;
              }
              else {
                preamblecarriers = carriers;
                shift = (max_carriers - carriers) / 2;
              }
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_32K[index] == i) {
                  if (continual_pilot_table_32K[index] > shift) {
                    data_carrier_map[i - shift] = CONTINUAL_CARRIER;
                  }
                  index++;
                }
              }
              for (int i = 0; i < preamblecarriers; i++) {
                if ((i % preamble_dx) == 0) {
                  data_carrier_map[i] = PREAMBLE_CARRIER;
                }
              }
              if (symbol != 0 && papr_mode == PAPR_TR) {
                index = 0;
                if (preamble_dx == 3 || preamble_dx == 4 || preamble_dx == 8) {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_alt_table_32K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
                else {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_table_32K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
              }
            }
            else if (frame_symbols[symbol] == SBS_SYMBOL) {
              index = 0;
              shift = (max_carriers - carriers) / 2;
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_32K[index] == i) {
                  if (continual_pilot_table_32K[index] > shift) {
                    data_carrier_map[i - shift] = CONTINUAL_CARRIER;
                  }
                  index++;
                }
              }
              for (int i = 0; i < carriers; i++) {
                if ((i % dx) == 0) {
                  data_carrier_map[i] = SCATTERED_CARRIER;
                }
              }
              if (papr_mode == PAPR_TR) {
                index = 0;
                if (dx == 3 || dx == 4 || dx == 8) {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_alt_table_32K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
                else {
                  for (int i = 0; i < max_carriers; i++) {
                    if ((trpapr_table_32K[index] + trshift) == i) {
                      data_carrier_map[i - shift] = TRPAPR_CARRIER;
                      index++;
                    }
                  }
                }
              }
            }
            else {
              index = 0;
              shift = (max_carriers - carriers) / 2;
              for (int i = 0; i < max_carriers; i++) {
                if (continual_pilot_table_32K[index] == i) {
                  if (continual_pilot_table_32K[index] > shift) {
                    data_carrier_map[i - shift] = CONTINUAL_CARRIER;
                  }
                  index++;
                }
              }
              for (int i = 0; i < carriers; i++) {
                remainder = i % (dx * dy);
                if (remainder == (dx * ((symbol - preamble_symbols) % dy))) {
                  data_carrier_map[i] = SCATTERED_CARRIER;
                }
              }
              if (papr_mode == PAPR_TR) {
                index = 0;
                for (int i = 0; i < max_carriers; i++) {
                  if ((trpapr_table_32K[index] + trshift) == i) {
                    data_carrier_map[i - shift] = TRPAPR_CARRIER;
                    index++;
                  }
                }
              }
            }
            if ((frame_symbols[symbol] == SBS_SYMBOL) || (frame_symbols[symbol] == DATA_SYMBOL)) {
              data_carrier_map[0] = SCATTERED_CARRIER;
              data_carrier_map[carriers - 1] = SCATTERED_CARRIER;
              switch (pilot_pattern) {
                case PILOT_SP3_2:
                  data_carrier_map[6939] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP3_4:
                  break;
                case PILOT_SP4_2:
                  break;
                case PILOT_SP4_4:
                  break;
                case PILOT_SP6_2:
                  data_carrier_map[6942] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP6_4:
                  break;
                case PILOT_SP8_2:
                  data_carrier_map[6920] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP8_4:
                  break;
                case PILOT_SP12_2:
                  data_carrier_map[6924] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP12_4:
                  break;
                case PILOT_SP16_2:
                  data_carrier_map[6928] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP16_4:
                  break;
                case PILOT_SP24_2:
                  data_carrier_map[6936] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP24_4:
                  break;
                case PILOT_SP32_2:
                  data_carrier_map[6944] = SCATTERED_CARRIER;
                  break;
                case PILOT_SP32_4:
                  break;
                default:
                  break;
              }
            }
            break;
        }
      }
    }

    const gr_complex zero = gr_complex(0.0, 0.0);

    int
    pilotgenerator_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      int indexin = 0;
      int preamblecarriers;
      int left_nulls, right_nulls;
      float normalization;
      gr_complex* dst;

      for (int i = 0; i < noutput_items; i += insertion_items) {
        for (int j = 0; j < symbols; j++) {
          if (frame_symbols[j] == PREAMBLE_SYMBOL) {
            if (j == 0) {
              preamblecarriers = preamble_carriers;
              normalization = first_preamble_normalization;
            }
            else {
              preamblecarriers = carriers;
              normalization = preamble_normalization;
            }
            left_nulls = output_mode ? ((ofdm_fft_size - preamblecarriers) / 2) + 1 : 0;
            right_nulls = output_mode ? (ofdm_fft_size - preamblecarriers) / 2 : 0;
            for (int n = 0; n < left_nulls; n++) {
              *out++ = zero;
            }
            for (int n = 0; n < preamblecarriers; n++) {
              if (data_carrier_map[j][n] == PREAMBLE_CARRIER) {
                *out++ = pr_bpsk[prbs[n]];
              }
              else if (data_carrier_map[j][n] == CONTINUAL_CARRIER) {
                *out++ = cp_bpsk[prbs[n]];
              }
              else if (data_carrier_map[j][n] == TRPAPR_CARRIER) {
                *out++ = zero;
              }
              else {
                *out++ = in[indexin++];
              }
            }
            for (int n = 0; n < right_nulls; n++) {
              *out++ = zero;
            }
          }
          else {
            normalization = data_normalization;
            left_nulls = output_mode ? ((ofdm_fft_size - carriers) / 2) + 1 : 0;
            right_nulls = output_mode ? (ofdm_fft_size - carriers) / 2 : 0;
            for (int n = 0; n < left_nulls; n++) {
              *out++ = zero;
            }
            for (int n = 0; n < carriers; n++) {
              if (data_carrier_map[j][n] == SCATTERED_CARRIER) {
                *out++ = sp_bpsk[prbs[n]];
              }
              else if (data_carrier_map[j][n] == CONTINUAL_CARRIER) {
                *out++ = cp_bpsk[prbs[n]];
              }
              else if (data_carrier_map[j][n] == TRPAPR_CARRIER) {
                *out++ = zero;
              }
              else {
                *out++ = in[indexin++];
              }
            }
            for (int n = 0; n < right_nulls; n++) {
              *out++ = zero;
            }
          }
          if (miso_mode != MISO_OFF) {
            if (frame_symbols[j] != PREAMBLE_SYMBOL) {
              out -= carriers + right_nulls;
              for (int n = 0; n < carriers; n++) {
                *out++ *= miso_rotation[n];
              }
              out += right_nulls;
            }
          }
          if (output_mode == PILOTGENERATOR_TIME) {
            out -= ofdm_fft_size;
#if 0
            if (equalization_enable == EQUALIZATION_ON) {
              volk_32fc_x2_multiply_32fc(out, out, inverse_sinc, ofdm_fft_size);
            }
#endif
            dst = ofdm_fft.get_inbuf();
            memcpy(&dst[ofdm_fft_size / 2], &out[0], sizeof(gr_complex) * ofdm_fft_size / 2);
            memcpy(&dst[0], &out[ofdm_fft_size / 2], sizeof(gr_complex) * ofdm_fft_size / 2);
            ofdm_fft.execute();
            volk_32f_s32f_multiply_32f(reinterpret_cast<float*>(out), reinterpret_cast<float*>(ofdm_fft.get_outbuf()), normalization, ofdm_fft_size * 2);
            out += ofdm_fft_size;
          }
        }
      }

      if (indexin != input_cells) {
        printf("input_cells = %d, indexin = %d\n", input_cells, indexin);
      }
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (indexin);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    const double pilotgenerator_cc_impl::scattered_power_table[16][5] = {
      {0.00 , 0.00 , 1.40 , 2.20 , 2.90},
      {0.00 , 1.40 , 2.90 , 3.80 , 4.40},
      {0.00 , 0.60 , 2.10 , 3.00 , 3.60},
      {0.00 , 2.10 , 3.60 , 4.40 , 5.10},
      {0.00 , 1.60 , 3.10 , 4.00 , 4.60},
      {0.00 , 3.00 , 4.50 , 5.40 , 6.00},
      {0.00 , 2.20 , 3.80 , 4.60 , 5.30},
      {0.00 , 3.60 , 5.10 , 6.00 , 6.60},
      {0.00 , 3.20 , 4.70 , 5.60 , 6.20},
      {0.00 , 4.50 , 6.00 , 6.90 , 7.50},
      {0.00 , 3.80 , 5.30 , 6.20 , 6.80},
      {0.00 , 5.20 , 6.70 , 7.60 , 8.20},
      {0.00 , 4.70 , 6.20 , 7.10 , 7.70},
      {0.00 , 6.10 , 7.60 , 8.50 , 9.10},
      {0.00 , 5.40 , 6.90 , 7.70 , 8.40},
      {0.00 , 6.70 , 8.20 , 9.10 , 9.70}
    };

    const int pilotgenerator_cc_impl::continual_pilot_table_8K[48] = {
      59, 167, 307, 469, 637, 751, 865, 1031, 1159, 1333, 1447, 1607, 1811, 1943, 2041, 2197,
      2323, 2519, 2605, 2767, 2963, 3029, 3175, 3325, 3467, 3665, 3833, 3901, 4073, 4235, 4325, 4511,
      4627, 4825, 4907, 5051, 5227, 5389, 5531, 5627, 5833, 5905, 6053, 6197, 6353, 6563, 6637, 6809
    };

    const int pilotgenerator_cc_impl::continual_pilot_table_16K[96] = {
      118, 178, 334, 434, 614, 670, 938, 1070, 1274, 1358, 1502, 1618, 1730, 1918, 2062, 2078,
      2318, 2566, 2666, 2750, 2894, 3010, 3214, 3250, 3622, 3686, 3886, 3962, 4082, 4166, 4394, 4558,
      4646, 4718, 5038, 5170, 5210, 5342, 5534, 5614, 5926, 5942, 6058, 6134, 6350, 6410, 6650, 6782,
      6934, 7154, 7330, 7438, 7666, 7742, 7802, 7894, 8146, 8258, 8470, 8494, 8650, 8722, 9022, 9118,
      9254, 9422, 9650, 9670, 9814, 9902, 10102, 10166, 10454, 10598, 10778, 10822, 11062, 11138,
      11254, 11318, 11666, 11758, 11810, 11974, 12106, 12242, 12394, 12502, 12706, 12866, 13126, 13190,
      13274, 13466, 13618, 13666
    };

    const int pilotgenerator_cc_impl::continual_pilot_table_32K[192] = {
      236, 316, 356, 412, 668, 716, 868, 1100, 1228, 1268, 1340, 1396, 1876, 1916, 2140, 2236,
      2548, 2644, 2716, 2860, 3004, 3164, 3236, 3436, 3460, 3700, 3836, 4028, 4124, 4132, 4156, 4316,
      4636, 5012, 5132, 5140, 5332, 5372, 5500, 5524, 5788, 6004, 6020, 6092, 6428, 6452, 6500, 6740,
      7244, 7316, 7372, 7444, 7772, 7844, 7924, 8020, 8164, 8308, 8332, 8348, 8788, 8804, 9116, 9140,
      9292, 9412, 9436, 9604, 10076, 10204, 10340, 10348, 10420, 10660, 10684, 10708, 11068, 11132,
      11228, 11356, 11852, 11860, 11884, 12044, 12116, 12164, 12268, 12316, 12700, 12772, 12820, 12988,
      13300, 13340, 13564, 13780, 13868, 14084, 14308, 14348, 14660, 14828, 14876, 14948, 15332, 15380,
      15484, 15532, 15604, 15764, 15788, 15796, 16292, 16420, 16516, 16580, 16940, 16964, 16988, 17228,
      17300, 17308, 17444, 17572, 18044, 18212, 18236, 18356, 18508, 18532, 18844, 18860, 19300, 19316,
      19340, 19484, 19628, 19724, 19804, 19876, 20204, 20276, 20332, 20404, 20908, 21148, 21196, 21220,
      21556, 21628, 21644, 21860, 22124, 22148, 22276, 22316, 22508, 22516, 22636, 23012, 23332, 23492,
      23516, 23524, 23620, 23812, 23948, 24188, 24212, 24412, 24484, 24644, 24788, 24932, 25004, 25100,
      25412, 25508, 25732, 25772, 26252, 26308, 26380, 26420, 26548, 26780, 26932, 26980, 27236, 27292,
      27332, 27412
    };

    const int pilotgenerator_cc_impl::trpapr_table_8K[72] = {
      250, 386, 407, 550, 591, 717, 763, 787, 797, 839, 950, 1090, 1105, 1199, 1738, 1867,
      1903, 1997, 2114, 2260, 2356, 2427, 2428, 2444, 2452, 2475, 2564, 2649, 2663, 2678, 2740, 2777,
      2819, 2986, 3097, 3134, 3253, 3284, 3323, 3442, 3596, 3694, 3719, 3751, 3763, 3836, 4154, 4257,
      4355, 4580, 4587, 4678, 4805, 5084, 5126, 5161, 5229, 5321, 5445, 5649, 5741, 5746, 5885, 5918,
      6075, 6093, 6319, 6421, 6463, 6511, 6517, 6577
    };

    const int pilotgenerator_cc_impl::trpapr_table_16K[144] = {
      421, 548, 589, 621, 644, 727, 770, 813, 857, 862, 1113, 1187, 1201, 1220, 1393, 1517,
      1821, 1899, 1924, 2003, 2023, 2143, 2146, 2290, 2474, 2482, 2597, 2644, 2749, 2818, 2951, 3014,
      3212, 3237, 3363, 3430, 3515, 3517, 3745, 3758, 4049, 4165, 4354, 4399, 4575, 4763, 4789, 4802,
      4834, 4970, 5260, 5386, 5395, 5402, 5579, 5716, 5734, 5884, 5895, 6073, 6123, 6158, 6212, 6243,
      6521, 6593, 6604, 6607, 6772, 6842, 6908, 6986, 7220, 7331, 7396, 7407, 7588, 7635, 7665, 7893,
      7925, 7949, 8019, 8038, 8167, 8289, 8295, 8338, 8549, 8555, 8660, 8857, 8925, 9007, 9057, 9121,
      9364, 9375, 9423, 9446, 9479, 9502, 9527, 9860, 9919, 9938, 10138, 10189, 10191, 10275, 10333,
      10377, 10988, 11109, 11261, 11266, 11362, 11390, 11534, 11623, 11893, 11989, 12037, 12101, 12119,
      12185, 12254, 12369, 12371, 12380, 12401, 12586, 12597, 12638, 12913, 12974, 13001, 13045, 13052,
      13111, 13143, 13150, 13151, 13300
    };

    const int pilotgenerator_cc_impl::trpapr_table_32K[288] = {
      803, 805, 811, 901, 1001, 1027, 1245, 1258, 1318, 1478, 1507, 1509, 1556, 1577, 1655, 1742,
      1978, 2001, 2056, 2110, 2164, 2227, 2305, 2356, 2408, 2522, 2563, 2780, 2805, 2879, 3010, 3019,
      3128, 3389, 3649, 3730, 3873, 4027, 4066, 4087, 4181, 4246, 4259, 4364, 4406, 4515, 4690, 4773,
      4893, 4916, 4941, 4951, 4965, 5165, 5222, 5416, 5638, 5687, 5729, 5930, 5997, 6005, 6161, 6218,
      6292, 6344, 6370, 6386, 6505, 6974, 7079, 7114, 7275, 7334, 7665, 7765, 7868, 7917, 7966, 8023,
      8055, 8089, 8091, 8191, 8374, 8495, 8651, 8690, 8755, 8821, 9139, 9189, 9274, 9561, 9611, 9692,
      9711, 9782, 9873, 9964, 10011, 10209, 10575, 10601, 10623, 10690, 10967, 11045, 11083, 11084,
      11090, 11128, 11153, 11530, 11737, 11829, 11903, 11907, 11930, 11942, 12356, 12429, 12484, 12547,
      12562, 12605, 12767, 12863, 13019, 13052, 13053, 13167, 13210, 13244, 13259, 13342, 13370, 13384,
      13447, 13694, 13918, 14002, 14077, 14111, 14216, 14243, 14270, 14450, 14451, 14456, 14479, 14653,
      14692, 14827, 14865, 14871, 14908, 15215, 15227, 15284, 15313, 15333, 15537, 15643, 15754, 15789,
      16065, 16209, 16213, 16217, 16259, 16367, 16369, 16646, 16780, 16906, 16946, 17012, 17167, 17192,
      17325, 17414, 17629, 17687, 17746, 17788, 17833, 17885, 17913, 18067, 18089, 18316, 18337, 18370,
      18376, 18440, 18550, 18680, 18910, 18937, 19047, 19052, 19117, 19383, 19396, 19496, 19601, 19778,
      19797, 20038, 20357, 20379, 20455, 20669, 20707, 20708, 20751, 20846, 20853, 20906, 21051, 21079,
      21213, 21267, 21308, 21355, 21523, 21574, 21815, 21893, 21973, 22084, 22172, 22271, 22713, 22905,
      23039, 23195, 23303, 23635, 23732, 23749, 23799, 23885, 23944, 24149, 24311, 24379, 24471, 24553,
      24585, 24611, 24616, 24621, 24761, 24789, 24844, 24847, 24977, 25015, 25160, 25207, 25283, 25351,
      25363, 25394, 25540, 25603, 25647, 25747, 25768, 25915, 25928, 26071, 26092, 26139, 26180, 26209,
      26270, 26273, 26278, 26326, 26341, 26392, 26559, 26642, 26776, 26842
    };

    const int pilotgenerator_cc_impl::trpapr_alt_table_8K[72] = {
      295, 329, 347, 365, 463, 473, 481, 553, 578, 602, 742, 749, 829, 922, 941, 1115,
      1123, 1174, 1363, 1394, 1402, 1615, 1657, 1702, 1898, 1910, 1997, 2399, 2506, 2522, 2687, 2735,
      3043, 3295, 3389, 3454, 3557, 3647, 3719, 3793, 3794, 3874, 3898, 3970, 4054, 4450, 4609, 4666,
      4829, 4855, 4879, 4961, 4969, 5171, 5182, 5242, 5393, 5545, 5567, 5618, 5630, 5734, 5861, 5897,
      5987, 5989, 6002, 6062, 6074, 6205, 6334, 6497
    };

    const int pilotgenerator_cc_impl::trpapr_alt_table_16K[144] = {
      509, 739, 770, 890, 970, 989, 1031, 1033, 1121, 1223, 1231, 1285, 1526, 1559, 1603, 1615,
      1690, 1771, 1903, 1910, 1958, 2033, 2146, 2225, 2302, 2306, 2345, 2447, 2477, 2561, 2578, 2597,
      2635, 2654, 2687, 2891, 2938, 3029, 3271, 3479, 3667, 3713, 3791, 3977, 4067, 4150, 4217, 4387,
      4501, 4541, 4657, 4733, 4742, 4963, 5011, 5149, 5311, 5362, 5491, 5531, 5609, 5722, 5747, 5798,
      5842, 5881, 5959, 5983, 6059, 6166, 6178, 6214, 6230, 6382, 6557, 6625, 6811, 6881, 6994, 7261,
      7535, 7546, 7711, 7897, 7898, 7918, 7997, 8125, 8398, 8483, 8530, 8686, 8731, 8855, 9001, 9026,
      9110, 9206, 9223, 9325, 9466, 9493, 9890, 9893, 10537, 10570, 10691, 10835, 10837, 11098, 11126,
      11146, 11198, 11270, 11393, 11629, 11657, 11795, 11867, 11909, 11983, 12046, 12107, 12119, 12353,
      12482, 12569, 12575, 12662, 12691, 12739, 12787, 12902, 12917, 12985, 13010, 13022, 13073, 13102,
      13141, 13159, 13225, 13255, 13303
    };

    const int pilotgenerator_cc_impl::trpapr_alt_table_32K[288] = {
      793, 884, 899, 914, 1004, 1183, 1198, 1276, 1300, 1339, 1348, 1444, 1487, 1490, 1766, 1870,
      1903, 1909, 1961, 2053, 2092, 2099, 2431, 2572, 2578, 2618, 2719, 2725, 2746, 2777, 2798, 2891,
      2966, 2972, 3023, 3037, 3076, 3257, 3284, 3326, 3389, 3425, 3454, 3523, 3602, 3826, 3838, 3875,
      3955, 4094, 4126, 4261, 4349, 4357, 4451, 4646, 4655, 4913, 5075, 5083, 5306, 5317, 5587, 5821,
      6038, 6053, 6062, 6137, 6268, 6286, 6490, 6517, 6529, 6554, 6593, 6671, 6751, 6827, 6845, 7043,
      7111, 7147, 7196, 7393, 7451, 7475, 7517, 7750, 7769, 7780, 8023, 8081, 8263, 8290, 8425, 8492,
      8939, 8986, 9113, 9271, 9298, 9343, 9455, 9476, 9637, 9821, 9829, 9913, 9953, 9988, 10001, 10007,
      10018, 10082, 10172, 10421, 10553, 10582, 10622, 10678, 10843, 10885, 10901, 11404, 11674, 11959,
      12007, 12199, 12227, 12290, 12301, 12629, 12631, 12658, 12739, 12866, 12977, 13121, 13294, 13843,
      13849, 13852, 13933, 14134, 14317, 14335, 14342, 14407, 14651, 14758, 14815, 14833, 14999, 15046,
      15097, 15158, 15383, 15503, 15727, 15881, 16139, 16238, 16277, 16331, 16444, 16490, 16747, 16870,
      16981, 17641, 17710, 17714, 17845, 18011, 18046, 18086, 18097, 18283, 18334, 18364, 18431, 18497,
      18527, 18604, 18686, 18709, 18731, 18740, 18749, 18772, 18893, 19045, 19075, 19087, 19091, 19099,
      19127, 19169, 19259, 19427, 19433, 19450, 19517, 19526, 19610, 19807, 19843, 19891, 20062, 20159,
      20246, 20420, 20516, 20530, 20686, 20801, 20870, 20974, 21131, 21158, 21565, 21635, 21785, 21820,
      21914, 21926, 22046, 22375, 22406, 22601, 22679, 22699, 22772, 22819, 22847, 22900, 22982, 22987,
      23063, 23254, 23335, 23357, 23561, 23590, 23711, 23753, 23902, 24037, 24085, 24101, 24115, 24167,
      24182, 24361, 24374, 24421, 24427, 24458, 24463, 24706, 24748, 24941, 25079, 25127, 25195, 25285,
      25444, 25492, 25505, 25667, 25682, 25729, 25741, 25765, 25973, 26171, 26180, 26227, 26353, 26381,
      26542, 26603, 26651, 26671, 26759, 26804, 26807, 26827
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_64_1_2[64] = {
      gr_complex(-0.0082, -0.0031), gr_complex( 0.0041, -0.0092), gr_complex(-0.0192, -0.0795),
      gr_complex(-0.0321,  0.0025), gr_complex(-0.0234,  0.0615), gr_complex(-0.0257, -0.0544),
      gr_complex(-0.0599, -0.0362), gr_complex(-0.0472,  0.0203), gr_complex( 0.0925, -0.0460),
      gr_complex( 0.0217, -0.0165), gr_complex(-0.1060,  0.0672), gr_complex( 0.0356,  0.0135),
      gr_complex(-0.0765, -0.0388), gr_complex(-0.1158,  0.1144), gr_complex( 0.0197,  0.1516),
      gr_complex(-0.0359,  0.0376), gr_complex(-0.0932,  0.0818), gr_complex(-0.0454,  0.1093),
      gr_complex( 0.0692, -0.0114), gr_complex(-0.0801, -0.0271), gr_complex(-0.1156, -0.0505),
      gr_complex( 0.2071, -0.0987), gr_complex( 0.0217,  0.1298), gr_complex( 0.0305,  0.1189),
      gr_complex( 0.1325,  0.1816), gr_complex( 0.0220,  0.1673), gr_complex( 0.2034, -0.1647),
      gr_complex( 0.1139,  0.0092), gr_complex(-0.0485,  0.2120), gr_complex( 0.0006,  0.0205),
      gr_complex(-0.0667,  0.0071), gr_complex(-0.2077, -0.2282), gr_complex(-0.1265,  0.0798),
      gr_complex( 0.0276,  0.1123), gr_complex(-0.0043,  0.1419), gr_complex( 0.0147, -0.0002),
      gr_complex( 0.0066, -0.0353), gr_complex(-0.1775,  0.1532), gr_complex(-0.0491,  0.0762),
      gr_complex(-0.1185, -0.1370), gr_complex( 0.1381,  0.1197), gr_complex( 0.0402, -0.1385),
      gr_complex(-0.0414, -0.0215), gr_complex( 0.0425, -0.2408), gr_complex( 0.0778,  0.1461),
      gr_complex(-0.2283,  0.0223), gr_complex( 0.0196,  0.0578), gr_complex(-0.0309, -0.1078),
      gr_complex( 0.0340,  0.0156), gr_complex( 0.0431, -0.0505), gr_complex(-0.0656, -0.1008),
      gr_complex( 0.0719,  0.1280), gr_complex(-0.0367, -0.0387), gr_complex( 0.0549,  0.0875),
      gr_complex(-0.0199, -0.0339), gr_complex( 0.0247, -0.0080), gr_complex(-0.0482, -0.0159),
      gr_complex( 0.0474, -0.0310), gr_complex(-0.0262,  0.0055), gr_complex(-0.0185, -0.0061),
      gr_complex( 0.0547,  0.0236), gr_complex(-0.0084,  0.0174), gr_complex( 0.0030,  0.0153),
      gr_complex(-0.0339, -0.0449)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_64_2_2[64] = {
      gr_complex( 0.0570,  0.0402), gr_complex(-0.0200,  0.0170), gr_complex(-0.0538,  0.0115),
      gr_complex(-0.0197, -0.1022), gr_complex(-0.0244, -0.0409), gr_complex( 0.0925,  0.0289),
      gr_complex(-0.0082,  0.0075), gr_complex( 0.0795,  0.0501), gr_complex( 0.0124, -0.0371),
      gr_complex( 0.0266,  0.0384), gr_complex(-0.0644, -0.0655), gr_complex(-0.0565,  0.0211),
      gr_complex( 0.0711,  0.0100), gr_complex(-0.0186,  0.0228), gr_complex( 0.1920, -0.0084),
      gr_complex( 0.0029, -0.0932), gr_complex( 0.1438,  0.0361), gr_complex(-0.0405,  0.0996),
      gr_complex(-0.1229,  0.1549), gr_complex(-0.0177, -0.1009), gr_complex( 0.0610, -0.0604),
      gr_complex( 0.3258, -0.0067), gr_complex( 0.0486,  0.1430), gr_complex( 0.0057,  0.0169),
      gr_complex(-0.0468,  0.0400), gr_complex(-0.0580,  0.1502), gr_complex( 0.0839,  0.2601),
      gr_complex(-0.0909, -0.0273), gr_complex( 0.0683,  0.0433), gr_complex(-0.1055,  0.0668),
      gr_complex(-0.2471, -0.0933), gr_complex( 0.0682,  0.2381), gr_complex( 0.0195, -0.1981),
      gr_complex(-0.0465,  0.0036), gr_complex(-0.0331,  0.0257), gr_complex( 0.1589,  0.0603),
      gr_complex( 0.0641,  0.1233), gr_complex( 0.0678,  0.1191), gr_complex( 0.0271, -0.2271),
      gr_complex( 0.0877,  0.1282), gr_complex(-0.1054,  0.0719), gr_complex( 0.1926, -0.0134),
      gr_complex( 0.0272, -0.0083), gr_complex(-0.0126,  0.0324), gr_complex( 0.1290, -0.0779),
      gr_complex( 0.1207, -0.0885), gr_complex(-0.0827, -0.1182), gr_complex(-0.0020, -0.0275),
      gr_complex( 0.0280, -0.0564), gr_complex(-0.1056, -0.0500), gr_complex(-0.1652, -0.0210),
      gr_complex(-0.0103,  0.1312), gr_complex(-0.0025,  0.0292), gr_complex( 0.0094, -0.0186),
      gr_complex( 0.0529,  0.0211), gr_complex( 0.0285,  0.0582), gr_complex( 0.0386, -0.0935),
      gr_complex( 0.0283, -0.0598), gr_complex(-0.0550,  0.0053), gr_complex( 0.0113, -0.0073),
      gr_complex(-0.0100, -0.0336), gr_complex(-0.0605,  0.0318), gr_complex(-0.0212,  0.0315),
      gr_complex( 0.0100,  0.0136)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_64_1_3[64] = {
      gr_complex(-0.0202, -0.0157), gr_complex(-0.0397, -0.0295), gr_complex( 0.0051, -0.0108),
      gr_complex( 0.0365, -0.0003), gr_complex(-0.0195, -0.0111), gr_complex(-0.1033,  0.0267),
      gr_complex(-0.0409,  0.0354), gr_complex(-0.0243, -0.0359), gr_complex(-0.0291, -0.0020),
      gr_complex( 0.0027,  0.1260), gr_complex( 0.0495,  0.0905), gr_complex( 0.0206, -0.0591),
      gr_complex(-0.0658,  0.0847), gr_complex( 0.0361,  0.0878), gr_complex( 0.1156, -0.0646),
      gr_complex(-0.1034, -0.0663), gr_complex(-0.1042, -0.0011), gr_complex( 0.0373,  0.0515),
      gr_complex(-0.1169, -0.0665), gr_complex(-0.1598, -0.0456), gr_complex( 0.0191, -0.0271),
      gr_complex( 0.0155, -0.0469), gr_complex(-0.0536,  0.1606), gr_complex(-0.0881,  0.1063),
      gr_complex( 0.0136,  0.1160), gr_complex(-0.0165,  0.0980), gr_complex( 0.3054, -0.0506),
      gr_complex( 0.1229,  0.1970), gr_complex(-0.1988,  0.2747), gr_complex( 0.0473,  0.0365),
      gr_complex(-0.0451,  0.1114), gr_complex(-0.1865, -0.1219), gr_complex(-0.0820, -0.0034),
      gr_complex(-0.1170,  0.0836), gr_complex( 0.0354,  0.0839), gr_complex( 0.2146,  0.0002),
      gr_complex( 0.1105, -0.1351), gr_complex(-0.0920,  0.0875), gr_complex(-0.0300, -0.1244),
      gr_complex(-0.1987, -0.0400), gr_complex( 0.0853,  0.0506), gr_complex(-0.1360, -0.1131),
      gr_complex( 0.0036,  0.0701), gr_complex( 0.1415, -0.1823), gr_complex(-0.0022,  0.1309),
      gr_complex(-0.1185, -0.1542), gr_complex( 0.0331, -0.0297), gr_complex(-0.0268, -0.0252),
      gr_complex( 0.1085,  0.1090), gr_complex(-0.1165, -0.0823), gr_complex( 0.0427, -0.0327),
      gr_complex(-0.0317, -0.0209), gr_complex(-0.0064,  0.0446), gr_complex(-0.0394,  0.0371),
      gr_complex( 0.0534, -0.0665), gr_complex( 0.0524,  0.0609), gr_complex(-0.0057, -0.0144),
      gr_complex( 0.0164,  0.0052), gr_complex(-0.0345, -0.0144), gr_complex( 0.0396,  0.0416),
      gr_complex(-0.0661,  0.0017), gr_complex( 0.0363,  0.0182), gr_complex( 0.0092, -0.0382),
      gr_complex( 0.0467,  0.0337)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_64_2_3[64] = {
      gr_complex(-0.0351, -0.0356), gr_complex( 0.0087, -0.0039), gr_complex(-0.0400, -0.0028),
      gr_complex( 0.0228,  0.0060), gr_complex( 0.0347, -0.0134), gr_complex(-0.0351, -0.0292),
      gr_complex(-0.0091, -0.0239), gr_complex(-0.1048, -0.0459), gr_complex( 0.0814, -0.0157),
      gr_complex( 0.0278,  0.0517), gr_complex(-0.0310,  0.0042), gr_complex( 0.0082, -0.0551),
      gr_complex(-0.1074, -0.1147), gr_complex( 0.0709,  0.0104), gr_complex( 0.0134, -0.0183),
      gr_complex( 0.0391,  0.0550), gr_complex( 0.2130,  0.1695), gr_complex(-0.2063,  0.0635),
      gr_complex(-0.0821,  0.0257), gr_complex(-0.1664, -0.1884), gr_complex( 0.0600,  0.0789),
      gr_complex( 0.1633,  0.1717), gr_complex(-0.1040,  0.1295), gr_complex( 0.1314, -0.0204),
      gr_complex(-0.1336,  0.0564), gr_complex(-0.0383,  0.1788), gr_complex(-0.1792,  0.2524),
      gr_complex(-0.0639,  0.0308), gr_complex(-0.0327,  0.0499), gr_complex(-0.0426, -0.0327),
      gr_complex(-0.1421, -0.0480), gr_complex( 0.0324,  0.1438), gr_complex( 0.0780, -0.0666),
      gr_complex(-0.1150, -0.0442), gr_complex( 0.1399,  0.0277), gr_complex( 0.0795,  0.0892),
      gr_complex(-0.0085, -0.0374), gr_complex( 0.0953,  0.0980), gr_complex( 0.1254, -0.1780),
      gr_complex(-0.1259,  0.0849), gr_complex(-0.1303,  0.0004), gr_complex( 0.1888,  0.0584),
      gr_complex(-0.1031, -0.0692), gr_complex(-0.0871, -0.0654), gr_complex(-0.0039, -0.0491),
      gr_complex(-0.0021,  0.0591), gr_complex(-0.1821, -0.1699), gr_complex(-0.0221,  0.0450),
      gr_complex(-0.1060,  0.1698), gr_complex( 0.0634,  0.0286), gr_complex( 0.0531,  0.0341),
      gr_complex( 0.0299,  0.0984), gr_complex( 0.0960, -0.0295), gr_complex( 0.1152, -0.0318),
      gr_complex(-0.0462, -0.0994), gr_complex(-0.0231, -0.0581), gr_complex(-0.0681,  0.0278),
      gr_complex(-0.0226,  0.0018), gr_complex(-0.0124, -0.0168), gr_complex(-0.0306,  0.0504),
      gr_complex(-0.0153,  0.0185), gr_complex( 0.0665, -0.0023), gr_complex(-0.0147, -0.0080),
      gr_complex(-0.0025, -0.0243)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_64_3_3[64] = {
      gr_complex( 0.0436,  0.0054), gr_complex( 0.0171, -0.0086), gr_complex(-0.0079, -0.0112),
      gr_complex( 0.0037, -0.0093), gr_complex( 0.0549, -0.0019), gr_complex(-0.0391,  0.0219),
      gr_complex(-0.0306, -0.0150), gr_complex( 0.0398,  0.0042), gr_complex(-0.0360,  0.0737),
      gr_complex( 0.0432, -0.0191), gr_complex(-0.0297, -0.0303), gr_complex(-0.0421, -0.0442),
      gr_complex(-0.0423,  0.0888), gr_complex( 0.0681, -0.0220), gr_complex(-0.1486, -0.0084),
      gr_complex( 0.1086, -0.0260), gr_complex(-0.0102,  0.0763), gr_complex(-0.0520, -0.0592),
      gr_complex( 0.0022, -0.0342), gr_complex(-0.0331,  0.0601), gr_complex(-0.0826,  0.0136),
      gr_complex( 0.1155,  0.2692), gr_complex( 0.0776, -0.2126), gr_complex( 0.0330, -0.0684),
      gr_complex( 0.0150, -0.0269), gr_complex(-0.1286,  0.0254), gr_complex(-0.0529, -0.3026),
      gr_complex( 0.0871,  0.1886), gr_complex(-0.0810,  0.0120), gr_complex( 0.0934, -0.0558),
      gr_complex( 0.1699, -0.0469), gr_complex( 0.1644, -0.1603), gr_complex(-0.1104,  0.0125),
      gr_complex( 0.0423, -0.0163), gr_complex( 0.0720,  0.0480), gr_complex( 0.0027, -0.0066),
      gr_complex(-0.0916,  0.0016), gr_complex(-0.0783,  0.2550), gr_complex(-0.1807,  0.1094),
      gr_complex( 0.0347,  0.1010), gr_complex(-0.0434,  0.1560), gr_complex( 0.0427,  0.1170),
      gr_complex(-0.0744, -0.0089), gr_complex(-0.1619, -0.0234), gr_complex( 0.1864,  0.1523),
      gr_complex( 0.1173, -0.0573), gr_complex(-0.0136,  0.0547), gr_complex(-0.0126, -0.0084),
      gr_complex(-0.0269,  0.0097), gr_complex( 0.1539,  0.0925), gr_complex(-0.0388,  0.0407),
      gr_complex(-0.0974,  0.1576), gr_complex(-0.0565,  0.0087), gr_complex(-0.1553, -0.1074),
      gr_complex( 0.0834,  0.0164), gr_complex( 0.1502, -0.0071), gr_complex( 0.1019,  0.1022),
      gr_complex( 0.0256,  0.0919), gr_complex(-0.0529,  0.0828), gr_complex(-0.0240,  0.0590),
      gr_complex(-0.0182, -0.0669), gr_complex(-0.0049, -0.0340), gr_complex( 0.0299, -0.0039),
      gr_complex(-0.0036, -0.0072)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_64_1_4[64] = {
      gr_complex( 0.0240, -0.0409), gr_complex( 0.0012, -0.0723), gr_complex(-0.0192, -0.0187),
      gr_complex( 0.0487,  0.0256), gr_complex( 0.0508, -0.0429), gr_complex(-0.0400, -0.0598),
      gr_complex(-0.0299,  0.0292), gr_complex( 0.0171, -0.0085), gr_complex(-0.0452, -0.0488),
      gr_complex(-0.1012,  0.0784), gr_complex( 0.0182,  0.0737), gr_complex( 0.0460, -0.0497),
      gr_complex(-0.1181,  0.0534), gr_complex(-0.0447,  0.1291), gr_complex( 0.1023, -0.0167),
      gr_complex(-0.0501, -0.1357), gr_complex(-0.0843, -0.1035), gr_complex(-0.0382, -0.0540),
      gr_complex(-0.0657, -0.1025), gr_complex(-0.0362, -0.0847), gr_complex( 0.0219, -0.0437),
      gr_complex( 0.0146, -0.0333), gr_complex(-0.1189,  0.1498), gr_complex(-0.1678,  0.0733),
      gr_complex(-0.0205,  0.0349), gr_complex(-0.0220,  0.1345), gr_complex( 0.2133, -0.0547),
      gr_complex( 0.1213,  0.1973), gr_complex(-0.1696,  0.3086), gr_complex(-0.0591, -0.0141),
      gr_complex( 0.0167,  0.0481), gr_complex(-0.1727, -0.1232), gr_complex(-0.0513,  0.0157),
      gr_complex(-0.1042,  0.1508), gr_complex( 0.0128,  0.0261), gr_complex( 0.1502,  0.0075),
      gr_complex( 0.1558, -0.0742), gr_complex(-0.1810,  0.0746), gr_complex( 0.0573, -0.0604),
      gr_complex(-0.1213, -0.1572), gr_complex(-0.0473,  0.2651), gr_complex(-0.1701, -0.1300),
      gr_complex(-0.0475,  0.0891), gr_complex( 0.0860, -0.1701), gr_complex( 0.0287, -0.0118),
      gr_complex(-0.2152, -0.1151), gr_complex( 0.0911, -0.0229), gr_complex( 0.0227,  0.0191),
      gr_complex( 0.0164,  0.0224), gr_complex(-0.0265, -0.0111), gr_complex(-0.0469, -0.0784),
      gr_complex(-0.0250,  0.1309), gr_complex(-0.0179, -0.0898), gr_complex( 0.0119,  0.0266),
      gr_complex( 0.0177, -0.0294), gr_complex( 0.0367, -0.0083), gr_complex(-0.0132, -0.0501),
      gr_complex( 0.0458, -0.0252), gr_complex(-0.0354,  0.0197), gr_complex( 0.0074,  0.0287),
      gr_complex( 0.0374,  0.0192), gr_complex( 0.0393, -0.0109), gr_complex(-0.0036, -0.0001),
      gr_complex(-0.0159, -0.0341)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_64_2_4[64] = {
      gr_complex(-0.0263, -0.0479), gr_complex(-0.0301, -0.0107), gr_complex( 0.0333, -0.0063),
      gr_complex( 0.0204,  0.0289), gr_complex(-0.0105,  0.0583), gr_complex(-0.0649, -0.1346),
      gr_complex(-0.0243, -0.0626), gr_complex( 0.0599, -0.0728), gr_complex( 0.0119,  0.0809),
      gr_complex( 0.0566,  0.0771), gr_complex(-0.0317, -0.1009), gr_complex( 0.0592,  0.0811),
      gr_complex(-0.1045, -0.1392), gr_complex(-0.1084,  0.0149), gr_complex( 0.0758, -0.0852),
      gr_complex( 0.0799, -0.0393), gr_complex( 0.2092,  0.1991), gr_complex(-0.1139, -0.0389),
      gr_complex(-0.0529, -0.0304), gr_complex(-0.0902, -0.1026), gr_complex( 0.0859,  0.0258),
      gr_complex( 0.1311,  0.1771), gr_complex( 0.0212,  0.0289), gr_complex( 0.1347, -0.0075),
      gr_complex(-0.0492,  0.1194), gr_complex(-0.0598,  0.0809), gr_complex(-0.0676,  0.2751),
      gr_complex(-0.0033,  0.0489), gr_complex(-0.1294,  0.1162), gr_complex(-0.1280,  0.0001),
      gr_complex(-0.0536, -0.1314), gr_complex( 0.0334,  0.1431), gr_complex( 0.0520,  0.0127),
      gr_complex(-0.0859, -0.0728), gr_complex(-0.0147,  0.0550), gr_complex( 0.1340,  0.2318),
      gr_complex(-0.0182, -0.0708), gr_complex( 0.0938,  0.1883), gr_complex( 0.0505, -0.0878),
      gr_complex(-0.0316,  0.0609), gr_complex(-0.1373,  0.0291), gr_complex( 0.2823,  0.0664),
      gr_complex( 0.0269, -0.0262), gr_complex(-0.0174, -0.0057), gr_complex( 0.0651, -0.0408),
      gr_complex( 0.1702,  0.0152), gr_complex(-0.1067, -0.1807), gr_complex(-0.0661, -0.0909),
      gr_complex(-0.1148,  0.1257), gr_complex(-0.0864, -0.0229), gr_complex(-0.0557, -0.0173),
      gr_complex(-0.0489,  0.1720), gr_complex( 0.0189,  0.0422), gr_complex( 0.1237,  0.0095),
      gr_complex( 0.0096,  0.0209), gr_complex(-0.0192, -0.0478), gr_complex( 0.0511, -0.0060),
      gr_complex( 0.0123,  0.0400), gr_complex(-0.0456, -0.0497), gr_complex(-0.0084,  0.0188),
      gr_complex(-0.0052,  0.0608), gr_complex( 0.0208, -0.0086), gr_complex( 0.0235, -0.0157),
      gr_complex(-0.0051,  0.0114)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_64_3_4[64] = {
      gr_complex(-0.0233, -0.0047), gr_complex( 0.0235,  0.0189), gr_complex( 0.0180,  0.0330),
      gr_complex(-0.0042, -0.0688), gr_complex( 0.0140, -0.0162), gr_complex(-0.0297,  0.0627),
      gr_complex(-0.0144, -0.0526), gr_complex(-0.0466,  0.0299), gr_complex( 0.0060,  0.0085),
      gr_complex( 0.0173,  0.1123), gr_complex(-0.0095, -0.0079), gr_complex( 0.0554,  0.0071),
      gr_complex( 0.0341, -0.0753), gr_complex(-0.0615,  0.0015), gr_complex(-0.1283,  0.0457),
      gr_complex( 0.0546, -0.1677), gr_complex(-0.0932,  0.2335), gr_complex(-0.1123,  0.0290),
      gr_complex( 0.1421,  0.0877), gr_complex( 0.0467, -0.1229), gr_complex( 0.0059,  0.0987),
      gr_complex( 0.0374, -0.0089), gr_complex(-0.0469, -0.1116), gr_complex(-0.0315,  0.0372),
      gr_complex(-0.0329,  0.0249), gr_complex(-0.1220,  0.1634), gr_complex( 0.0901, -0.0367),
      gr_complex( 0.1698,  0.1548), gr_complex(-0.0707, -0.0577), gr_complex(-0.1228,  0.0759),
      gr_complex( 0.2568,  0.0969), gr_complex(-0.0160,  0.0148), gr_complex( 0.1090, -0.0340),
      gr_complex(-0.0061,  0.2155), gr_complex( 0.0134,  0.0477), gr_complex( 0.0510,  0.0546),
      gr_complex(-0.0063,  0.0183), gr_complex(-0.0733,  0.1909), gr_complex(-0.0475,  0.1187),
      gr_complex(-0.0415,  0.1335), gr_complex( 0.0628,  0.1182), gr_complex(-0.0947,  0.0036),
      gr_complex( 0.0648, -0.2780), gr_complex(-0.0856,  0.0200), gr_complex( 0.0757,  0.0776),
      gr_complex( 0.0913, -0.1074), gr_complex(-0.0469, -0.0884), gr_complex( 0.0390, -0.2093),
      gr_complex( 0.0610, -0.1203), gr_complex( 0.0021,  0.1465), gr_complex(-0.0649,  0.0788),
      gr_complex(-0.2232,  0.0983), gr_complex( 0.0063, -0.0601), gr_complex(-0.0743, -0.1217),
      gr_complex(-0.0850,  0.0442), gr_complex( 0.0500, -0.0122), gr_complex( 0.0083,  0.0355),
      gr_complex( 0.0793,  0.0643), gr_complex( 0.0799,  0.0421), gr_complex(-0.0598,  0.0669),
      gr_complex(-0.0535, -0.0376), gr_complex(-0.0449, -0.0622), gr_complex( 0.0532, -0.0191),
      gr_complex( 0.0824, -0.0414)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_64_4_4[64] = {
      gr_complex( 0.0050,  0.0373), gr_complex( 0.0239,  0.0232), gr_complex( 0.0302, -0.0206),
      gr_complex(-0.0148, -0.0121), gr_complex(-0.0007, -0.0009), gr_complex(-0.0177, -0.0167),
      gr_complex(-0.0166,  0.0221), gr_complex(-0.0181, -0.0276), gr_complex( 0.0012,  0.0362),
      gr_complex( 0.0114,  0.0245), gr_complex(-0.0201,  0.0881), gr_complex( 0.0430,  0.0391),
      gr_complex( 0.0106, -0.0458), gr_complex(-0.0337, -0.0743), gr_complex(-0.0617,  0.0517),
      gr_complex( 0.0011,  0.0323), gr_complex(-0.0128,  0.0034), gr_complex(-0.0525, -0.0809),
      gr_complex(-0.0457,  0.1235), gr_complex( 0.1259,  0.0362), gr_complex(-0.0608, -0.0147),
      gr_complex(-0.0241, -0.2151), gr_complex(-0.2348,  0.0577), gr_complex( 0.0898,  0.0509),
      gr_complex( 0.0000, -0.0149), gr_complex(-0.1193,  0.0479), gr_complex(-0.1443,  0.0473),
      gr_complex( 0.0574, -0.1223), gr_complex( 0.0818,  0.0683), gr_complex( 0.1011, -0.1585),
      gr_complex(-0.0604, -0.0786), gr_complex( 0.0911, -0.2836), gr_complex(-0.1751, -0.0192),
      gr_complex( 0.0056,  0.1524), gr_complex( 0.0249, -0.0000), gr_complex(-0.1729,  0.0337),
      gr_complex(-0.0397, -0.0840), gr_complex(-0.0398, -0.3157), gr_complex( 0.2074, -0.0389),
      gr_complex(-0.0265,  0.2476), gr_complex( 0.2462, -0.1978), gr_complex( 0.1360,  0.1081),
      gr_complex( 0.0269,  0.0323), gr_complex( 0.0348, -0.1474), gr_complex(-0.0682, -0.0088),
      gr_complex( 0.0387, -0.0759), gr_complex(-0.0351,  0.1037), gr_complex(-0.1447, -0.0196),
      gr_complex(-0.1227, -0.0640), gr_complex( 0.0076, -0.1253), gr_complex(-0.0569, -0.0363),
      gr_complex(-0.0810, -0.0190), gr_complex(-0.0890, -0.1005), gr_complex(-0.0343,  0.0293),
      gr_complex(-0.0496,  0.0054), gr_complex(-0.0468, -0.0352), gr_complex( 0.0064, -0.0181),
      gr_complex(-0.0094,  0.0168), gr_complex( 0.0138,  0.0256), gr_complex(-0.0331,  0.1121),
      gr_complex( 0.0025,  0.0350), gr_complex( 0.0306,  0.0165), gr_complex( 0.0001,  0.0608),
      gr_complex(-0.0023, -0.0119)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_256_1_2[256] = {
      gr_complex( 0.0045,  0.0013), gr_complex(-0.0108, -0.0006), gr_complex( 0.0279, -0.0059),
      gr_complex( 0.0071,  0.0211), gr_complex( 0.0110,  0.0023), gr_complex( 0.0030,  0.0203),
      gr_complex( 0.0056, -0.0249), gr_complex( 0.0092, -0.0039), gr_complex(-0.0148, -0.0216),
      gr_complex( 0.0227,  0.0006), gr_complex(-0.0354,  0.0028), gr_complex( 0.0068, -0.0218),
      gr_complex(-0.0016,  0.0243), gr_complex( 0.0215,  0.0011), gr_complex( 0.0128,  0.0190),
      gr_complex(-0.0170, -0.0274), gr_complex( 0.0130,  0.0133), gr_complex(-0.0226, -0.0192),
      gr_complex( 0.0310,  0.0146), gr_complex(-0.0464,  0.0068), gr_complex(-0.0146, -0.0082),
      gr_complex(-0.0006, -0.0058), gr_complex( 0.0176, -0.0200), gr_complex( 0.0024,  0.0210),
      gr_complex(-0.0225, -0.0259), gr_complex(-0.0081,  0.0521), gr_complex(-0.0164, -0.0158),
      gr_complex( 0.0399,  0.0106), gr_complex(-0.0185,  0.0221), gr_complex(-0.0036, -0.0069),
      gr_complex( 0.0136, -0.0067), gr_complex( 0.0455, -0.0925), gr_complex( 0.0238,  0.0229),
      gr_complex(-0.0113, -0.0309), gr_complex( 0.0486,  0.0256), gr_complex(-0.0266, -0.0014),
      gr_complex( 0.0609, -0.0451), gr_complex(-0.0094,  0.0148), gr_complex(-0.0168, -0.0255),
      gr_complex(-0.0434,  0.0156), gr_complex(-0.0064, -0.0473), gr_complex( 0.0249,  0.0037),
      gr_complex(-0.0279,  0.0322), gr_complex( 0.0254,  0.0327), gr_complex(-0.0553,  0.0010),
      gr_complex( 0.0388, -0.0333), gr_complex(-0.0175,  0.0221), gr_complex(-0.0027,  0.0341),
      gr_complex(-0.0651,  0.0130), gr_complex(-0.0144, -0.0090), gr_complex( 0.0138,  0.0292),
      gr_complex( 0.0051, -0.0164), gr_complex( 0.0316, -0.0102), gr_complex( 0.0151, -0.0249),
      gr_complex(-0.0074,  0.0144), gr_complex( 0.0317, -0.0189), gr_complex( 0.0506,  0.0009),
      gr_complex( 0.0413,  0.0186), gr_complex(-0.0218, -0.0459), gr_complex(-0.0635, -0.0790),
      gr_complex( 0.0002, -0.0665), gr_complex(-0.0110,  0.0071), gr_complex(-0.0320,  0.0072),
      gr_complex(-0.0526, -0.0819), gr_complex( 0.0762, -0.0814), gr_complex( 0.0174,  0.0191),
      gr_complex(-0.0698, -0.0311), gr_complex(-0.0830, -0.0056), gr_complex(-0.0097, -0.0280),
      gr_complex(-0.0109, -0.0085), gr_complex(-0.0552,  0.0284), gr_complex( 0.0061,  0.0184),
      gr_complex(-0.0163,  0.0371), gr_complex( 0.0668,  0.0585), gr_complex(-0.0539, -0.0087),
      gr_complex(-0.0005,  0.0255), gr_complex(-0.0018,  0.0540), gr_complex( 0.0266,  0.1584),
      gr_complex( 0.0391, -0.0546), gr_complex(-0.0285, -0.1144), gr_complex(-0.0200, -0.0217),
      gr_complex(-0.1922,  0.0431), gr_complex(-0.0887, -0.0415), gr_complex(-0.0105, -0.0153),
      gr_complex(-0.0595, -0.0274), gr_complex( 0.0361, -0.0148), gr_complex(-0.0054,  0.0302),
      gr_complex( 0.0166, -0.0056), gr_complex(-0.0272, -0.0169), gr_complex(-0.0845,  0.0007),
      gr_complex(-0.0234,  0.0606), gr_complex(-0.0715, -0.0344), gr_complex( 0.0904,  0.0345),
      gr_complex(-0.0377, -0.0186), gr_complex(-0.0195, -0.0167), gr_complex( 0.0785,  0.0161),
      gr_complex(-0.0205,  0.0282), gr_complex(-0.0373, -0.0270), gr_complex(-0.0798,  0.1285),
      gr_complex(-0.0070,  0.1068), gr_complex( 0.1014,  0.0033), gr_complex(-0.0322, -0.0503),
      gr_complex( 0.0449,  0.1164), gr_complex(-0.0837,  0.0089), gr_complex( 0.0767, -0.0548),
      gr_complex( 0.0749,  0.0176), gr_complex(-0.0111, -0.0612), gr_complex( 0.0415,  0.0154),
      gr_complex(-0.0571,  0.0488), gr_complex( 0.0505,  0.1136), gr_complex( 0.0158,  0.0007),
      gr_complex(-0.0648,  0.0715), gr_complex( 0.0750,  0.0274), gr_complex(-0.1055, -0.0100),
      gr_complex(-0.0026,  0.0536), gr_complex( 0.0586,  0.0059), gr_complex( 0.0467,  0.0333),
      gr_complex( 0.0458, -0.0547), gr_complex(-0.0488,  0.0044), gr_complex( 0.0052, -0.1456),
      gr_complex(-0.0093, -0.0251), gr_complex(-0.0297, -0.1343), gr_complex(-0.1614,  0.0594),
      gr_complex( 0.0151, -0.0335), gr_complex(-0.0481, -0.0023), gr_complex( 0.1284, -0.0284),
      gr_complex( 0.0053,  0.0053), gr_complex( 0.0109,  0.0756), gr_complex( 0.0179,  0.0187),
      gr_complex(-0.0849,  0.0130), gr_complex( 0.0319, -0.0024), gr_complex( 0.0061, -0.0114),
      gr_complex(-0.0701,  0.0393), gr_complex( 0.0817,  0.0911), gr_complex(-0.0037,  0.0605),
      gr_complex(-0.1555,  0.1231), gr_complex( 0.0104, -0.0311), gr_complex( 0.0484, -0.0483),
      gr_complex(-0.0707, -0.0378), gr_complex(-0.0296, -0.0045), gr_complex( 0.0104, -0.0154),
      gr_complex(-0.0072, -0.0372), gr_complex(-0.0587,  0.0054), gr_complex(-0.0199,  0.2233),
      gr_complex( 0.0575, -0.0194), gr_complex(-0.0081, -0.0201), gr_complex(-0.0102,  0.0222),
      gr_complex( 0.0369,  0.0553), gr_complex(-0.0254,  0.0855), gr_complex(-0.0859,  0.0052),
      gr_complex( 0.0024, -0.1180), gr_complex(-0.0433, -0.0601), gr_complex( 0.0338, -0.0535),
      gr_complex( 0.0082,  0.0065), gr_complex( 0.0114,  0.0212), gr_complex( 0.0669,  0.0914),
      gr_complex(-0.0076, -0.0434), gr_complex(-0.1692, -0.0465), gr_complex( 0.0061, -0.0173),
      gr_complex(-0.0372,  0.0321), gr_complex(-0.0847,  0.0183), gr_complex(-0.0148,  0.0309),
      gr_complex(-0.1216, -0.0398), gr_complex( 0.0472, -0.0588), gr_complex(-0.0243, -0.0721),
      gr_complex(-0.0627,  0.0034), gr_complex(-0.0449, -0.0218), gr_complex( 0.0586, -0.0129),
      gr_complex(-0.0841, -0.1519), gr_complex( 0.1192,  0.0433), gr_complex( 0.0221, -0.0551),
      gr_complex( 0.0011, -0.0331), gr_complex( 0.0077,  0.0482), gr_complex( 0.0634,  0.0303),
      gr_complex(-0.0655, -0.0616), gr_complex(-0.0098, -0.0318), gr_complex( 0.0003, -0.0170),
      gr_complex(-0.0281,  0.0317), gr_complex(-0.0389,  0.0493), gr_complex( 0.0888,  0.0021),
      gr_complex( 0.0174, -0.0422), gr_complex( 0.0724,  0.0211), gr_complex( 0.0177,  0.0137),
      gr_complex( 0.0176,  0.0210), gr_complex( 0.0103,  0.0242), gr_complex(-0.0493, -0.0425),
      gr_complex( 0.0317, -0.0938), gr_complex( 0.0321,  0.0157), gr_complex(-0.0674, -0.0269),
      gr_complex( 0.0594,  0.0010), gr_complex( 0.0275,  0.0796), gr_complex(-0.0422,  0.0024),
      gr_complex( 0.0128, -0.0425), gr_complex( 0.0095,  0.0124), gr_complex(-0.0093, -0.0187),
      gr_complex( 0.0284, -0.0222), gr_complex( 0.0003, -0.0051), gr_complex( 0.0309,  0.0301),
      gr_complex(-0.0119,  0.0321), gr_complex(-0.0168, -0.0029), gr_complex(-0.0005,  0.0212),
      gr_complex(-0.0300,  0.0279), gr_complex( 0.0030, -0.0286), gr_complex( 0.0259, -0.0401),
      gr_complex(-0.0012,  0.0127), gr_complex(-0.0093, -0.0073), gr_complex(-0.0100, -0.0059),
      gr_complex(-0.0290,  0.0482), gr_complex(-0.0415,  0.0294), gr_complex(-0.0072, -0.0452),
      gr_complex(-0.0037, -0.0259), gr_complex(-0.0238,  0.0300), gr_complex( 0.0270, -0.0398),
      gr_complex( 0.0298,  0.0227), gr_complex(-0.0142,  0.0130), gr_complex(-0.0046, -0.0189),
      gr_complex( 0.0017, -0.0298), gr_complex( 0.0026, -0.0097), gr_complex( 0.0260, -0.0108),
      gr_complex( 0.0105,  0.0164), gr_complex( 0.0120, -0.0159), gr_complex( 0.0331, -0.0039),
      gr_complex(-0.0298, -0.0136), gr_complex( 0.0280,  0.0142), gr_complex( 0.0106,  0.0348),
      gr_complex(-0.0280,  0.0100), gr_complex(-0.0124, -0.0233), gr_complex( 0.0088,  0.0097),
      gr_complex(-0.0110, -0.0159), gr_complex( 0.0194, -0.0227), gr_complex( 0.0193,  0.0144),
      gr_complex( 0.0087,  0.0212), gr_complex(-0.0011, -0.0032), gr_complex( 0.0008, -0.0128),
      gr_complex(-0.0143,  0.0075), gr_complex( 0.0067,  0.0023), gr_complex( 0.0048, -0.0023),
      gr_complex( 0.0019,  0.0119), gr_complex( 0.0062, -0.0109), gr_complex( 0.0046, -0.0070),
      gr_complex( 0.0029,  0.0001), gr_complex(-0.0022,  0.0163), gr_complex(-0.0295,  0.0057),
      gr_complex(-0.0101, -0.0004), gr_complex( 0.0114,  0.0042), gr_complex(-0.0068, -0.0129),
      gr_complex( 0.0088, -0.0014), gr_complex( 0.0313,  0.0246), gr_complex( 0.0066,  0.0117),
      gr_complex(-0.0040,  0.0040), gr_complex(-0.0100,  0.0104), gr_complex(-0.0125, -0.0134),
      gr_complex(-0.0138, -0.0121), gr_complex( 0.0010,  0.0004), gr_complex(-0.0000,  0.0090),
      gr_complex(-0.0002, -0.0013)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_256_2_2[256] = {
      gr_complex(-0.0073, -0.0104), gr_complex( 0.0166, -0.0106), gr_complex( 0.0135,  0.0047),
      gr_complex( 0.0036, -0.0299), gr_complex(-0.0134, -0.0009), gr_complex(-0.0218,  0.0179),
      gr_complex( 0.0187, -0.0170), gr_complex( 0.0149, -0.0003), gr_complex(-0.0432,  0.0004),
      gr_complex( 0.0045, -0.0103), gr_complex(-0.0252,  0.0062), gr_complex( 0.0069, -0.0264),
      gr_complex( 0.0301, -0.0145), gr_complex(-0.0007, -0.0090), gr_complex(-0.0085, -0.0159),
      gr_complex( 0.0549, -0.0085), gr_complex(-0.0089, -0.0140), gr_complex( 0.0317,  0.0104),
      gr_complex( 0.0254,  0.0035), gr_complex( 0.0057,  0.0023), gr_complex(-0.0407,  0.0112),
      gr_complex( 0.0213,  0.0073), gr_complex(-0.0034,  0.0153), gr_complex(-0.0092, -0.0131),
      gr_complex( 0.0175, -0.0359), gr_complex( 0.0068, -0.0005), gr_complex( 0.0384, -0.0283),
      gr_complex( 0.0472, -0.0463), gr_complex( 0.0095, -0.0010), gr_complex( 0.0449,  0.0161),
      gr_complex( 0.1051,  0.0253), gr_complex(-0.0274,  0.0124), gr_complex( 0.0293, -0.0257),
      gr_complex(-0.0134,  0.0156), gr_complex( 0.0053,  0.0121), gr_complex( 0.0281, -0.0126),
      gr_complex(-0.0056,  0.0137), gr_complex(-0.0692, -0.0589), gr_complex( 0.1044, -0.0731),
      gr_complex(-0.0079, -0.0032), gr_complex( 0.0135,  0.0164), gr_complex( 0.0163,  0.0300),
      gr_complex( 0.0539, -0.0584), gr_complex( 0.0336,  0.0218), gr_complex( 0.0577,  0.0849),
      gr_complex( 0.0275,  0.0792), gr_complex( 0.0076,  0.0362), gr_complex(-0.0318,  0.0028),
      gr_complex( 0.0068, -0.0110), gr_complex(-0.0429,  0.0708), gr_complex( 0.0552,  0.0007),
      gr_complex(-0.0155, -0.0319), gr_complex(-0.0311, -0.0200), gr_complex( 0.1112,  0.0758),
      gr_complex( 0.0359,  0.0300), gr_complex(-0.0004,  0.0000), gr_complex( 0.1122,  0.0658),
      gr_complex(-0.0340,  0.0650), gr_complex(-0.0472,  0.0367), gr_complex( 0.0490,  0.0665),
      gr_complex( 0.0144, -0.0037), gr_complex( 0.0780, -0.0095), gr_complex( 0.0143,  0.0539),
      gr_complex(-0.0611, -0.0106), gr_complex(-0.0369,  0.0393), gr_complex( 0.0945, -0.0140),
      gr_complex( 0.0182, -0.0327), gr_complex(-0.0032,  0.0410), gr_complex(-0.1006,  0.0134),
      gr_complex( 0.0526, -0.1420), gr_complex( 0.0129,  0.0012), gr_complex(-0.0032, -0.0237),
      gr_complex(-0.1229,  0.0849), gr_complex( 0.0452, -0.0062), gr_complex(-0.0511,  0.0225),
      gr_complex(-0.0449, -0.0377), gr_complex(-0.0078,  0.0208), gr_complex(-0.0057, -0.0098),
      gr_complex(-0.0526,  0.0390), gr_complex( 0.0603, -0.0091), gr_complex(-0.0054,  0.0590),
      gr_complex( 0.0583, -0.0276), gr_complex(-0.0347, -0.0286), gr_complex(-0.0600,  0.0194),
      gr_complex(-0.0010,  0.0072), gr_complex(-0.0057, -0.0407), gr_complex( 0.0432, -0.0783),
      gr_complex( 0.0234,  0.0464), gr_complex( 0.0314,  0.0385), gr_complex( 0.0422,  0.0630),
      gr_complex( 0.0459, -0.0138), gr_complex( 0.0434,  0.1201), gr_complex( 0.0294, -0.0702),
      gr_complex(-0.0677,  0.0032), gr_complex( 0.0379,  0.0265), gr_complex( 0.0332, -0.0218),
      gr_complex(-0.0077, -0.0185), gr_complex( 0.1006,  0.0250), gr_complex( 0.1501,  0.0007),
      gr_complex(-0.0057, -0.0668), gr_complex(-0.0199, -0.0404), gr_complex( 0.0267,  0.0521),
      gr_complex(-0.0138,  0.0048), gr_complex( 0.0307, -0.0880), gr_complex( 0.0652,  0.0565),
      gr_complex(-0.0290, -0.1216), gr_complex(-0.0985,  0.0166), gr_complex(-0.0584,  0.0611),
      gr_complex( 0.0113,  0.1151), gr_complex( 0.0194,  0.0388), gr_complex(-0.0285,  0.0831),
      gr_complex(-0.0233,  0.0246), gr_complex( 0.0403, -0.0920), gr_complex(-0.0303,  0.0736),
      gr_complex( 0.0705, -0.0641), gr_complex( 0.0103, -0.0475), gr_complex(-0.1161, -0.0656),
      gr_complex( 0.0182, -0.0046), gr_complex(-0.0357, -0.0162), gr_complex(-0.0784, -0.0139),
      gr_complex( 0.0477, -0.0928), gr_complex(-0.0445,  0.0424), gr_complex( 0.1160, -0.0114),
      gr_complex(-0.0406,  0.0022), gr_complex(-0.0588,  0.0083), gr_complex( 0.1201,  0.0709),
      gr_complex( 0.0719, -0.0576), gr_complex( 0.0269, -0.0645), gr_complex(-0.0094, -0.0132),
      gr_complex(-0.0260,  0.0080), gr_complex( 0.0712,  0.0184), gr_complex(-0.0098, -0.0490),
      gr_complex( 0.0169, -0.0200), gr_complex( 0.0203,  0.0482), gr_complex(-0.0081,  0.1039),
      gr_complex(-0.0098,  0.0478), gr_complex( 0.0522, -0.1121), gr_complex( 0.0408,  0.0629),
      gr_complex(-0.0345,  0.0585), gr_complex(-0.0329,  0.0637), gr_complex( 0.0160, -0.0203),
      gr_complex(-0.0493, -0.0943), gr_complex( 0.0236, -0.0585), gr_complex(-0.0631,  0.0471),
      gr_complex( 0.0101, -0.0456), gr_complex(-0.0138,  0.0179), gr_complex(-0.0227, -0.0014),
      gr_complex( 0.0072,  0.0436), gr_complex( 0.0784, -0.0961), gr_complex(-0.0035, -0.0955),
      gr_complex( 0.0078, -0.0019), gr_complex( 0.0166, -0.0644), gr_complex( 0.0060, -0.0507),
      gr_complex( 0.0093, -0.0139), gr_complex( 0.0398,  0.0278), gr_complex( 0.0575, -0.0057),
      gr_complex(-0.0053, -0.0052), gr_complex( 0.0534, -0.0198), gr_complex(-0.0734,  0.0097),
      gr_complex(-0.1609,  0.0359), gr_complex(-0.0013, -0.1555), gr_complex(-0.0824,  0.0782),
      gr_complex( 0.0067,  0.0088), gr_complex(-0.0857,  0.0026), gr_complex(-0.0223,  0.0247),
      gr_complex( 0.0467,  0.0719), gr_complex(-0.0371, -0.0235), gr_complex( 0.1241, -0.0040),
      gr_complex(-0.0691, -0.0179), gr_complex(-0.0694, -0.0322), gr_complex(-0.0237, -0.0702),
      gr_complex(-0.0382, -0.0010), gr_complex( 0.0987,  0.0983), gr_complex( 0.0142,  0.0170),
      gr_complex( 0.0795,  0.0375), gr_complex(-0.0703, -0.0213), gr_complex( 0.0191, -0.0326),
      gr_complex(-0.0402, -0.0195), gr_complex(-0.0722, -0.0347), gr_complex( 0.0038,  0.0505),
      gr_complex(-0.0703,  0.0704), gr_complex(-0.0400,  0.0791), gr_complex(-0.0438, -0.1108),
      gr_complex( 0.0007,  0.0204), gr_complex(-0.0389, -0.0243), gr_complex( 0.0136, -0.0136),
      gr_complex( 0.1022,  0.0397), gr_complex( 0.0214,  0.1137), gr_complex(-0.0226,  0.0733),
      gr_complex(-0.0638, -0.0740), gr_complex( 0.0503,  0.0227), gr_complex( 0.0187, -0.0026),
      gr_complex( 0.0239, -0.0151), gr_complex( 0.0727,  0.0336), gr_complex( 0.0205,  0.0658),
      gr_complex( 0.0344,  0.0325), gr_complex(-0.0090, -0.0018), gr_complex( 0.0009, -0.0165),
      gr_complex(-0.0239, -0.0247), gr_complex( 0.0152,  0.0166), gr_complex(-0.0063, -0.0585),
      gr_complex(-0.0400,  0.0258), gr_complex( 0.0200, -0.0259), gr_complex(-0.0428,  0.0083),
      gr_complex( 0.0151, -0.0258), gr_complex( 0.0086, -0.0107), gr_complex( 0.0368,  0.0410),
      gr_complex( 0.0339,  0.0077), gr_complex(-0.0005,  0.0073), gr_complex( 0.0138, -0.0317),
      gr_complex(-0.0218,  0.0017), gr_complex(-0.0089,  0.0010), gr_complex( 0.0143,  0.0550),
      gr_complex( 0.0341,  0.0227), gr_complex(-0.0066, -0.0270), gr_complex( 0.0312, -0.0149),
      gr_complex( 0.0246, -0.0301), gr_complex( 0.0039,  0.0004), gr_complex( 0.0133, -0.0151),
      gr_complex(-0.0198,  0.0153), gr_complex(-0.0092, -0.0096), gr_complex(-0.0429, -0.0106),
      gr_complex( 0.0097, -0.0737), gr_complex( 0.0046, -0.0130), gr_complex( 0.0129, -0.0058),
      gr_complex( 0.0170, -0.0132), gr_complex(-0.0002,  0.0411), gr_complex(-0.0181,  0.0117),
      gr_complex(-0.0175,  0.0156), gr_complex( 0.0171, -0.0348), gr_complex(-0.0171,  0.0149),
      gr_complex(-0.0010, -0.0102), gr_complex(-0.0067,  0.0091), gr_complex( 0.0035,  0.0365),
      gr_complex(-0.0154, -0.0095), gr_complex(-0.0002,  0.0072), gr_complex( 0.0148, -0.0097),
      gr_complex(-0.0171,  0.0066), gr_complex( 0.0117, -0.0174), gr_complex(-0.0061, -0.0079),
      gr_complex(-0.0015, -0.0015), gr_complex(-0.0070, -0.0044), gr_complex(-0.0049,  0.0038),
      gr_complex(-0.0119, -0.0071), gr_complex(-0.0049, -0.0088), gr_complex( 0.0226, -0.0010),
      gr_complex(-0.0028,  0.0009), gr_complex( 0.0186, -0.0140), gr_complex( 0.0041,  0.0084),
      gr_complex(-0.0156,  0.0193), gr_complex(-0.0136, -0.0069), gr_complex(-0.0173, -0.0058),
      gr_complex( 0.0017,  0.0120), gr_complex(-0.0063,  0.0088), gr_complex( 0.0119,  0.0029),
      gr_complex( 0.0166,  0.0096)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_256_1_3[256] = {
      gr_complex( 0.0093,  0.0035), gr_complex(-0.0144, -0.0004), gr_complex( 0.0299, -0.0033),
      gr_complex( 0.0160,  0.0102), gr_complex( 0.0199,  0.0220), gr_complex(-0.0067,  0.0215),
      gr_complex( 0.0100, -0.0293), gr_complex( 0.0134, -0.0010), gr_complex(-0.0009, -0.0266),
      gr_complex( 0.0204, -0.0031), gr_complex(-0.0355, -0.0003), gr_complex( 0.0197, -0.0093),
      gr_complex( 0.0086,  0.0150), gr_complex( 0.0244, -0.0167), gr_complex( 0.0133,  0.0135),
      gr_complex( 0.0090, -0.0159), gr_complex( 0.0168,  0.0208), gr_complex(-0.0210, -0.0375),
      gr_complex( 0.0229,  0.0021), gr_complex(-0.0420,  0.0028), gr_complex(-0.0246,  0.0031),
      gr_complex(-0.0080, -0.0268), gr_complex( 0.0371, -0.0356), gr_complex(-0.0136,  0.0227),
      gr_complex(-0.0170, -0.0142), gr_complex(-0.0045,  0.0480), gr_complex(-0.0071, -0.0366),
      gr_complex( 0.0288,  0.0179), gr_complex(-0.0264,  0.0225), gr_complex( 0.0237, -0.0035),
      gr_complex( 0.0027, -0.0245), gr_complex( 0.0570, -0.0796), gr_complex(-0.0026,  0.0246),
      gr_complex( 0.0161, -0.0355), gr_complex( 0.0537,  0.0316), gr_complex(-0.0233, -0.0263),
      gr_complex( 0.0668, -0.0349), gr_complex(-0.0130,  0.0105), gr_complex( 0.0028, -0.0168),
      gr_complex(-0.0471, -0.0035), gr_complex(-0.0180, -0.0594), gr_complex( 0.0146, -0.0098),
      gr_complex(-0.0399,  0.0276), gr_complex( 0.0364,  0.0317), gr_complex(-0.0817, -0.0368),
      gr_complex( 0.0265, -0.0181), gr_complex(-0.0291,  0.0104), gr_complex(-0.0115,  0.0348),
      gr_complex(-0.0526,  0.0168), gr_complex(-0.0236, -0.0148), gr_complex( 0.0075,  0.0208),
      gr_complex(-0.0096, -0.0143), gr_complex( 0.0194, -0.0003), gr_complex(-0.0019, -0.0297),
      gr_complex(-0.0099,  0.0034), gr_complex( 0.0373,  0.0004), gr_complex( 0.0504,  0.0041),
      gr_complex( 0.0396,  0.0078), gr_complex(-0.0280, -0.0447), gr_complex(-0.0684, -0.0761),
      gr_complex( 0.0013, -0.0687), gr_complex(-0.0266,  0.0135), gr_complex(-0.0276,  0.0057),
      gr_complex(-0.0436, -0.0795), gr_complex( 0.0698, -0.0933), gr_complex( 0.0147,  0.0168),
      gr_complex(-0.0625, -0.0461), gr_complex(-0.0810, -0.0069), gr_complex(-0.0145, -0.0330),
      gr_complex(-0.0062, -0.0128), gr_complex(-0.0499,  0.0237), gr_complex(-0.0014,  0.0290),
      gr_complex(-0.0104,  0.0420), gr_complex( 0.0406,  0.0562), gr_complex(-0.0764, -0.0041),
      gr_complex( 0.0070,  0.0200), gr_complex( 0.0039,  0.0623), gr_complex( 0.0325,  0.1543),
      gr_complex( 0.0254, -0.0333), gr_complex(-0.0331, -0.1260), gr_complex(-0.0274, -0.0175),
      gr_complex(-0.1802,  0.0386), gr_complex(-0.0826, -0.0474), gr_complex(-0.0258, -0.0101),
      gr_complex(-0.0292, -0.0202), gr_complex( 0.0240, -0.0226), gr_complex( 0.0220,  0.0285),
      gr_complex( 0.0227, -0.0061), gr_complex(-0.0075,  0.0059), gr_complex(-0.0633, -0.0120),
      gr_complex(-0.0072,  0.0634), gr_complex(-0.0747, -0.0503), gr_complex( 0.1021,  0.0365),
      gr_complex(-0.0387, -0.0253), gr_complex(-0.0190, -0.0086), gr_complex( 0.0925, -0.0026),
      gr_complex(-0.0192,  0.0061), gr_complex(-0.0179, -0.0295), gr_complex(-0.0634,  0.1162),
      gr_complex(-0.0086,  0.1066), gr_complex( 0.1164, -0.0125), gr_complex(-0.0330, -0.0678),
      gr_complex( 0.0361,  0.1303), gr_complex(-0.0722,  0.0055), gr_complex( 0.0765, -0.0633),
      gr_complex( 0.0801,  0.0101), gr_complex(-0.0155, -0.0604), gr_complex( 0.0460, -0.0040),
      gr_complex(-0.0541,  0.0487), gr_complex( 0.0762,  0.1252), gr_complex( 0.0262, -0.0261),
      gr_complex(-0.0705,  0.0612), gr_complex( 0.0666,  0.0218), gr_complex(-0.0930, -0.0127),
      gr_complex(-0.0211,  0.0416), gr_complex( 0.0489,  0.0103), gr_complex( 0.0547,  0.0219),
      gr_complex( 0.0406, -0.0510), gr_complex(-0.0610, -0.0032), gr_complex( 0.0174, -0.1520),
      gr_complex(-0.0048, -0.0278), gr_complex(-0.0312, -0.1294), gr_complex(-0.1835,  0.0412),
      gr_complex( 0.0143, -0.0269), gr_complex(-0.0468, -0.0009), gr_complex( 0.1080, -0.0278),
      gr_complex(-0.0166,  0.0021), gr_complex( 0.0001,  0.0837), gr_complex( 0.0418,  0.0247),
      gr_complex(-0.0937,  0.0054), gr_complex( 0.0022, -0.0310), gr_complex( 0.0079, -0.0270),
      gr_complex(-0.0614,  0.0397), gr_complex( 0.0787,  0.0843), gr_complex(-0.0033,  0.0474),
      gr_complex(-0.1693,  0.1235), gr_complex( 0.0122, -0.0348), gr_complex( 0.0387, -0.0324),
      gr_complex(-0.0448, -0.0390), gr_complex(-0.0475, -0.0173), gr_complex(-0.0027, -0.0088),
      gr_complex(-0.0096, -0.0209), gr_complex(-0.0452, -0.0022), gr_complex(-0.0217,  0.2017),
      gr_complex( 0.0453, -0.0255), gr_complex( 0.0117, -0.0240), gr_complex(-0.0084,  0.0352),
      gr_complex( 0.0314,  0.0585), gr_complex(-0.0147,  0.0617), gr_complex(-0.0818,  0.0112),
      gr_complex( 0.0153, -0.1124), gr_complex(-0.0542, -0.0647), gr_complex( 0.0387, -0.0697),
      gr_complex( 0.0013, -0.0080), gr_complex( 0.0163,  0.0258), gr_complex( 0.0604,  0.0681),
      gr_complex(-0.0043, -0.0310), gr_complex(-0.1639, -0.0362), gr_complex( 0.0066, -0.0128),
      gr_complex(-0.0273,  0.0416), gr_complex(-0.0722,  0.0292), gr_complex(-0.0326,  0.0257),
      gr_complex(-0.1075, -0.0301), gr_complex( 0.0540, -0.0550), gr_complex(-0.0210, -0.0760),
      gr_complex(-0.0843,  0.0032), gr_complex(-0.0484, -0.0353), gr_complex( 0.0611, -0.0004),
      gr_complex(-0.0873, -0.1528), gr_complex( 0.1130,  0.0324), gr_complex( 0.0239, -0.0506),
      gr_complex(-0.0057, -0.0426), gr_complex( 0.0094,  0.0433), gr_complex( 0.0779,  0.0424),
      gr_complex(-0.0453, -0.0686), gr_complex(-0.0141, -0.0384), gr_complex( 0.0009, -0.0171),
      gr_complex(-0.0260,  0.0515), gr_complex(-0.0537,  0.0259), gr_complex( 0.1000, -0.0170),
      gr_complex( 0.0177, -0.0343), gr_complex( 0.0729,  0.0267), gr_complex( 0.0347, -0.0025),
      gr_complex( 0.0127,  0.0253), gr_complex( 0.0169,  0.0058), gr_complex(-0.0396, -0.0369),
      gr_complex( 0.0277, -0.0957), gr_complex( 0.0200,  0.0116), gr_complex(-0.0572, -0.0277),
      gr_complex( 0.0446, -0.0165), gr_complex( 0.0166,  0.0693), gr_complex(-0.0497,  0.0294),
      gr_complex( 0.0200, -0.0486), gr_complex( 0.0217,  0.0014), gr_complex(-0.0288, -0.0162),
      gr_complex( 0.0267, -0.0192), gr_complex( 0.0130, -0.0113), gr_complex( 0.0261,  0.0148),
      gr_complex(-0.0077,  0.0362), gr_complex(-0.0126, -0.0088), gr_complex(-0.0037,  0.0234),
      gr_complex(-0.0291,  0.0419), gr_complex(-0.0118, -0.0150), gr_complex( 0.0271, -0.0399),
      gr_complex(-0.0121,  0.0032), gr_complex(-0.0013, -0.0016), gr_complex( 0.0084, -0.0097),
      gr_complex(-0.0451,  0.0443), gr_complex(-0.0418,  0.0570), gr_complex(-0.0074, -0.0479),
      gr_complex(-0.0110, -0.0353), gr_complex(-0.0296,  0.0323), gr_complex( 0.0311, -0.0309),
      gr_complex( 0.0226,  0.0182), gr_complex(-0.0110,  0.0048), gr_complex(-0.0075, -0.0243),
      gr_complex(-0.0034, -0.0296), gr_complex( 0.0066, -0.0163), gr_complex( 0.0301, -0.0276),
      gr_complex(-0.0012,  0.0204), gr_complex( 0.0144, -0.0319), gr_complex( 0.0366, -0.0166),
      gr_complex(-0.0130, -0.0112), gr_complex( 0.0272,  0.0151), gr_complex( 0.0137,  0.0274),
      gr_complex(-0.0393,  0.0249), gr_complex(-0.0048, -0.0319), gr_complex( 0.0107,  0.0053),
      gr_complex(-0.0056, -0.0216), gr_complex( 0.0226, -0.0259), gr_complex( 0.0116,  0.0031),
      gr_complex( 0.0016,  0.0153), gr_complex( 0.0231, -0.0165), gr_complex( 0.0008, -0.0113),
      gr_complex(-0.0108,  0.0108), gr_complex( 0.0191,  0.0025), gr_complex(-0.0050,  0.0091),
      gr_complex(-0.0096,  0.0202), gr_complex( 0.0193, -0.0312), gr_complex( 0.0156, -0.0061),
      gr_complex(-0.0002, -0.0026), gr_complex( 0.0108,  0.0230), gr_complex(-0.0212,  0.0071),
      gr_complex(-0.0185, -0.0032), gr_complex( 0.0123, -0.0045), gr_complex(-0.0010, -0.0055),
      gr_complex( 0.0232, -0.0064), gr_complex( 0.0365,  0.0228), gr_complex( 0.0001,  0.0158),
      gr_complex( 0.0126,  0.0173), gr_complex(-0.0084,  0.0106), gr_complex(-0.0184, -0.0067),
      gr_complex(-0.0067, -0.0112), gr_complex( 0.0002,  0.0045), gr_complex(-0.0115,  0.0071),
      gr_complex( 0.0020, -0.0006)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_256_2_3[256] = {
      gr_complex(-0.0061, -0.0171), gr_complex( 0.0228, -0.0183), gr_complex( 0.0092, -0.0013),
      gr_complex(-0.0034, -0.0253), gr_complex(-0.0078,  0.0053), gr_complex(-0.0151,  0.0060),
      gr_complex( 0.0271, -0.0174), gr_complex( 0.0351,  0.0050), gr_complex(-0.0410,  0.0009),
      gr_complex( 0.0096,  0.0047), gr_complex(-0.0122,  0.0075), gr_complex( 0.0137, -0.0335),
      gr_complex( 0.0226,  0.0110), gr_complex(-0.0041, -0.0116), gr_complex(-0.0114, -0.0232),
      gr_complex( 0.0509,  0.0034), gr_complex( 0.0064, -0.0086), gr_complex( 0.0288,  0.0066),
      gr_complex( 0.0295,  0.0044), gr_complex( 0.0101,  0.0048), gr_complex(-0.0181,  0.0293),
      gr_complex( 0.0283,  0.0201), gr_complex( 0.0050,  0.0085), gr_complex(-0.0110, -0.0237),
      gr_complex( 0.0366, -0.0282), gr_complex( 0.0072,  0.0156), gr_complex( 0.0320, -0.0332),
      gr_complex( 0.0196, -0.0537), gr_complex( 0.0318, -0.0046), gr_complex( 0.0559,  0.0270),
      gr_complex( 0.1062,  0.0272), gr_complex(-0.0388,  0.0178), gr_complex( 0.0426, -0.0314),
      gr_complex( 0.0089,  0.0377), gr_complex( 0.0225,  0.0198), gr_complex( 0.0222,  0.0202),
      gr_complex( 0.0023,  0.0194), gr_complex(-0.0601, -0.0445), gr_complex( 0.0963, -0.0611),
      gr_complex(-0.0236,  0.0244), gr_complex(-0.0036,  0.0153), gr_complex( 0.0120,  0.0385),
      gr_complex( 0.0571, -0.0499), gr_complex( 0.0250,  0.0444), gr_complex( 0.0264,  0.1035),
      gr_complex( 0.0474,  0.0883), gr_complex(-0.0033,  0.0433), gr_complex(-0.0500,  0.0338),
      gr_complex(-0.0017,  0.0269), gr_complex(-0.0472,  0.0554), gr_complex( 0.0348, -0.0027),
      gr_complex(-0.0051, -0.0271), gr_complex(-0.0375, -0.0075), gr_complex( 0.1114,  0.0689),
      gr_complex( 0.0183,  0.0028), gr_complex( 0.0039, -0.0187), gr_complex( 0.0969,  0.0775),
      gr_complex(-0.0375,  0.0498), gr_complex(-0.0777,  0.0331), gr_complex( 0.0574,  0.0589),
      gr_complex( 0.0109, -0.0109), gr_complex( 0.0641, -0.0317), gr_complex(-0.0058,  0.0511),
      gr_complex(-0.0697, -0.0182), gr_complex(-0.0316,  0.0295), gr_complex( 0.0957, -0.0299),
      gr_complex( 0.0137, -0.0455), gr_complex(-0.0044,  0.0211), gr_complex(-0.0894,  0.0093),
      gr_complex( 0.0554, -0.1531), gr_complex( 0.0222, -0.0039), gr_complex(-0.0145, -0.0322),
      gr_complex(-0.1159,  0.0877), gr_complex( 0.0507,  0.0012), gr_complex(-0.0471,  0.0277),
      gr_complex(-0.0506, -0.0361), gr_complex( 0.0072,  0.0263), gr_complex(-0.0025, -0.0177),
      gr_complex(-0.0447,  0.0313), gr_complex( 0.0686,  0.0117), gr_complex( 0.0058,  0.0581),
      gr_complex( 0.0462, -0.0329), gr_complex(-0.0295,  0.0021), gr_complex(-0.0257,  0.0303),
      gr_complex( 0.0089, -0.0029), gr_complex(-0.0159, -0.0158), gr_complex( 0.0659, -0.0864),
      gr_complex( 0.0261,  0.0398), gr_complex( 0.0223,  0.0414), gr_complex( 0.0315,  0.0625),
      gr_complex( 0.0268, -0.0225), gr_complex( 0.0560,  0.1229), gr_complex( 0.0180, -0.0793),
      gr_complex(-0.0775, -0.0089), gr_complex( 0.0208,  0.0134), gr_complex( 0.0352, -0.0306),
      gr_complex(-0.0008, -0.0056), gr_complex( 0.0968,  0.0162), gr_complex( 0.1337, -0.0187),
      gr_complex(-0.0097, -0.0872), gr_complex(-0.0026, -0.0177), gr_complex( 0.0288,  0.0528),
      gr_complex(-0.0186,  0.0019), gr_complex( 0.0252, -0.0790), gr_complex( 0.0650,  0.0691),
      gr_complex(-0.0285, -0.1187), gr_complex(-0.1006,  0.0184), gr_complex(-0.0675,  0.0707),
      gr_complex( 0.0096,  0.0992), gr_complex( 0.0079,  0.0400), gr_complex(-0.0326,  0.0988),
      gr_complex(-0.0202,  0.0068), gr_complex( 0.0375, -0.0783), gr_complex(-0.0111,  0.0627),
      gr_complex( 0.0643, -0.0670), gr_complex( 0.0061, -0.0695), gr_complex(-0.1313, -0.0666),
      gr_complex( 0.0041,  0.0003), gr_complex(-0.0214, -0.0243), gr_complex(-0.0860, -0.0068),
      gr_complex( 0.0457, -0.0899), gr_complex(-0.0288,  0.0457), gr_complex( 0.1070, -0.0230),
      gr_complex(-0.0502, -0.0085), gr_complex(-0.0755, -0.0086), gr_complex( 0.1177,  0.0525),
      gr_complex( 0.0503, -0.0514), gr_complex( 0.0223, -0.0592), gr_complex( 0.0022, -0.0192),
      gr_complex(-0.0135, -0.0044), gr_complex( 0.0899,  0.0276), gr_complex(-0.0168, -0.0333),
      gr_complex( 0.0201, -0.0270), gr_complex( 0.0118,  0.0374), gr_complex(-0.0057,  0.1278),
      gr_complex(-0.0035,  0.0442), gr_complex( 0.0435, -0.0966), gr_complex( 0.0481,  0.0619),
      gr_complex(-0.0270,  0.0647), gr_complex(-0.0510,  0.0717), gr_complex( 0.0150, -0.0242),
      gr_complex(-0.0541, -0.0933), gr_complex( 0.0074, -0.0543), gr_complex(-0.0476,  0.0423),
      gr_complex( 0.0045, -0.0336), gr_complex(-0.0098,  0.0224), gr_complex(-0.0402, -0.0055),
      gr_complex( 0.0088,  0.0368), gr_complex( 0.0933, -0.0768), gr_complex(-0.0030, -0.0857),
      gr_complex( 0.0169,  0.0099), gr_complex(-0.0096, -0.0692), gr_complex( 0.0046, -0.0480),
      gr_complex(-0.0241, -0.0164), gr_complex( 0.0245,  0.0362), gr_complex( 0.0285,  0.0048),
      gr_complex(-0.0206,  0.0028), gr_complex( 0.0289, -0.0211), gr_complex(-0.0816,  0.0201),
      gr_complex(-0.1449,  0.0270), gr_complex(-0.0024, -0.1677), gr_complex(-0.0629,  0.0872),
      gr_complex( 0.0209, -0.0268), gr_complex(-0.0689, -0.0030), gr_complex(-0.0246,  0.0289),
      gr_complex( 0.0570,  0.0683), gr_complex(-0.0236,  0.0091), gr_complex( 0.1468, -0.0052),
      gr_complex(-0.0256, -0.0040), gr_complex(-0.0394, -0.0396), gr_complex(-0.0086, -0.0762),
      gr_complex(-0.0559, -0.0037), gr_complex( 0.0971,  0.0881), gr_complex(-0.0250,  0.0422),
      gr_complex( 0.0617,  0.0549), gr_complex(-0.0982,  0.0005), gr_complex( 0.0371, -0.0353),
      gr_complex(-0.0202, -0.0135), gr_complex(-0.0738, -0.0271), gr_complex(-0.0150,  0.0348),
      gr_complex(-0.0879,  0.0849), gr_complex(-0.0230,  0.0642), gr_complex(-0.0425, -0.1001),
      gr_complex( 0.0341,  0.0200), gr_complex(-0.0240, -0.0101), gr_complex( 0.0172,  0.0061),
      gr_complex( 0.1050,  0.0404), gr_complex( 0.0082,  0.1332), gr_complex(-0.0019,  0.0629),
      gr_complex(-0.0665, -0.0687), gr_complex( 0.0794,  0.0071), gr_complex( 0.0007, -0.0082),
      gr_complex( 0.0285, -0.0298), gr_complex( 0.0588,  0.0246), gr_complex(-0.0006,  0.0671),
      gr_complex( 0.0164,  0.0152), gr_complex(-0.0104, -0.0073), gr_complex( 0.0115, -0.0268),
      gr_complex(-0.0158, -0.0287), gr_complex( 0.0239, -0.0027), gr_complex(-0.0232, -0.0698),
      gr_complex(-0.0193,  0.0157), gr_complex( 0.0189, -0.0266), gr_complex(-0.0346,  0.0221),
      gr_complex( 0.0152, -0.0248), gr_complex( 0.0249,  0.0116), gr_complex( 0.0519,  0.0550),
      gr_complex( 0.0259,  0.0332), gr_complex( 0.0395, -0.0015), gr_complex( 0.0087, -0.0280),
      gr_complex( 0.0015, -0.0216), gr_complex(-0.0233, -0.0243), gr_complex( 0.0148,  0.0589),
      gr_complex( 0.0122,  0.0175), gr_complex(-0.0142, -0.0128), gr_complex( 0.0437, -0.0276),
      gr_complex( 0.0127, -0.0329), gr_complex( 0.0097,  0.0065), gr_complex( 0.0053, -0.0199),
      gr_complex(-0.0337,  0.0096), gr_complex(-0.0270, -0.0152), gr_complex(-0.0427, -0.0096),
      gr_complex( 0.0130, -0.0695), gr_complex( 0.0180, -0.0115), gr_complex( 0.0152, -0.0091),
      gr_complex(-0.0013, -0.0065), gr_complex(-0.0011,  0.0314), gr_complex(-0.0328,  0.0188),
      gr_complex(-0.0220,  0.0128), gr_complex( 0.0021, -0.0353), gr_complex( 0.0068, -0.0003),
      gr_complex( 0.0081, -0.0041), gr_complex(-0.0087,  0.0017), gr_complex( 0.0058,  0.0142),
      gr_complex(-0.0212, -0.0038), gr_complex( 0.0021, -0.0034), gr_complex( 0.0088, -0.0018),
      gr_complex(-0.0041,  0.0040), gr_complex( 0.0154, -0.0112), gr_complex(-0.0038,  0.0045),
      gr_complex( 0.0004, -0.0105), gr_complex(-0.0160, -0.0010), gr_complex(-0.0099, -0.0058),
      gr_complex(-0.0224,  0.0001), gr_complex(-0.0095, -0.0042), gr_complex( 0.0204,  0.0067),
      gr_complex( 0.0001,  0.0198), gr_complex( 0.0130,  0.0022), gr_complex(-0.0015,  0.0081),
      gr_complex(-0.0116,  0.0104), gr_complex(-0.0090, -0.0136), gr_complex(-0.0215, -0.0125),
      gr_complex( 0.0023,  0.0051), gr_complex(-0.0128,  0.0100), gr_complex( 0.0076,  0.0087),
      gr_complex( 0.0229,  0.0085)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_256_3_3[256] = {
      gr_complex( 0.0090, -0.0232), gr_complex(-0.0063,  0.0283), gr_complex( 0.0035, -0.0104),
      gr_complex(-0.0032,  0.0081), gr_complex( 0.0186,  0.0177), gr_complex( 0.0051, -0.0115),
      gr_complex( 0.0117,  0.0091), gr_complex(-0.0202, -0.0270), gr_complex(-0.0039,  0.0029),
      gr_complex( 0.0039, -0.0035), gr_complex(-0.0183, -0.0057), gr_complex( 0.0455,  0.0020),
      gr_complex(-0.0392,  0.0015), gr_complex( 0.0321,  0.0336), gr_complex( 0.0056,  0.0009),
      gr_complex(-0.0306, -0.0043), gr_complex(-0.0091, -0.0140), gr_complex(-0.0241,  0.0136),
      gr_complex( 0.0288, -0.0617), gr_complex(-0.0103,  0.0207), gr_complex(-0.0092, -0.0465),
      gr_complex( 0.0419,  0.0171), gr_complex(-0.0087, -0.0099), gr_complex(-0.0135, -0.0066),
      gr_complex( 0.0237,  0.0160), gr_complex(-0.0426, -0.0066), gr_complex( 0.0553, -0.0275),
      gr_complex(-0.0581,  0.0043), gr_complex( 0.0345, -0.0152), gr_complex(-0.0210,  0.0039),
      gr_complex(-0.0105,  0.0159), gr_complex( 0.0426,  0.0036), gr_complex(-0.0281,  0.0163),
      gr_complex( 0.0215, -0.0323), gr_complex(-0.0262,  0.0474), gr_complex( 0.0091,  0.0464),
      gr_complex(-0.0014, -0.0165), gr_complex(-0.0202, -0.0010), gr_complex(-0.0209, -0.0709),
      gr_complex(-0.0302,  0.0489), gr_complex( 0.0063, -0.0627), gr_complex(-0.0171,  0.0505),
      gr_complex(-0.0339, -0.0098), gr_complex( 0.0145,  0.0195), gr_complex( 0.0267, -0.0250),
      gr_complex(-0.0936, -0.0208), gr_complex( 0.0327,  0.0580), gr_complex(-0.0453, -0.0261),
      gr_complex( 0.0377, -0.0197), gr_complex(-0.1004,  0.0349), gr_complex( 0.0878, -0.0337),
      gr_complex( 0.0121, -0.0175), gr_complex( 0.0215,  0.0272), gr_complex( 0.0574,  0.0202),
      gr_complex( 0.0233,  0.0823), gr_complex( 0.0185, -0.0212), gr_complex( 0.0251,  0.0704),
      gr_complex(-0.0387, -0.0211), gr_complex(-0.0156,  0.0313), gr_complex(-0.0131, -0.0118),
      gr_complex(-0.0195, -0.0356), gr_complex( 0.0014,  0.0456), gr_complex( 0.0110,  0.0026),
      gr_complex(-0.0485,  0.0302), gr_complex( 0.0904,  0.0699), gr_complex(-0.0286,  0.0004),
      gr_complex(-0.0297,  0.0070), gr_complex(-0.0921, -0.0328), gr_complex(-0.0425,  0.0557),
      gr_complex( 0.0766, -0.0651), gr_complex(-0.0009,  0.0275), gr_complex( 0.0874, -0.0547),
      gr_complex( 0.1048,  0.0251), gr_complex( 0.0894, -0.0730), gr_complex(-0.0084, -0.1051),
      gr_complex(-0.1391,  0.0839), gr_complex( 0.0437, -0.0193), gr_complex( 0.0022, -0.0410),
      gr_complex( 0.0307,  0.0856), gr_complex( 0.0037, -0.0202), gr_complex( 0.0605,  0.0380),
      gr_complex( 0.0200, -0.0606), gr_complex( 0.0209,  0.0041), gr_complex(-0.0083, -0.0650),
      gr_complex(-0.0048,  0.0147), gr_complex( 0.0523,  0.0753), gr_complex( 0.0217, -0.0103),
      gr_complex( 0.0224,  0.0076), gr_complex(-0.0859,  0.0419), gr_complex(-0.0562, -0.0692),
      gr_complex(-0.0042,  0.0299), gr_complex( 0.0196,  0.0285), gr_complex( 0.0202, -0.1407),
      gr_complex( 0.0079, -0.1756), gr_complex(-0.0528, -0.0365), gr_complex(-0.1356, -0.0097),
      gr_complex(-0.0179,  0.0041), gr_complex(-0.0175,  0.0530), gr_complex( 0.1435,  0.0091),
      gr_complex(-0.0251,  0.0045), gr_complex(-0.0297, -0.0197), gr_complex( 0.0118,  0.0239),
      gr_complex( 0.0001, -0.0607), gr_complex( 0.0362,  0.0672), gr_complex(-0.0078, -0.0508),
      gr_complex(-0.0297,  0.0046), gr_complex( 0.0222,  0.0018), gr_complex(-0.0378,  0.0158),
      gr_complex( 0.0262,  0.0463), gr_complex(-0.0882, -0.0328), gr_complex( 0.0488, -0.0531),
      gr_complex(-0.0234, -0.0290), gr_complex(-0.0133,  0.0278), gr_complex(-0.1071,  0.0391),
      gr_complex( 0.0277,  0.0094), gr_complex( 0.0234,  0.0454), gr_complex(-0.0869, -0.0849),
      gr_complex(-0.1123,  0.0835), gr_complex( 0.0058,  0.0668), gr_complex( 0.0248, -0.0445),
      gr_complex(-0.0160,  0.0586), gr_complex( 0.0533,  0.1065), gr_complex(-0.0375,  0.0816),
      gr_complex( 0.1120, -0.0408), gr_complex( 0.0768,  0.0141), gr_complex(-0.1202, -0.0365),
      gr_complex(-0.0703, -0.0366), gr_complex(-0.0003, -0.0631), gr_complex(-0.0174, -0.0283),
      gr_complex(-0.0231, -0.0272), gr_complex( 0.0753, -0.0495), gr_complex( 0.0223,  0.1358),
      gr_complex(-0.0420,  0.0580), gr_complex(-0.0073,  0.0303), gr_complex(-0.0350, -0.0312),
      gr_complex(-0.0489, -0.0041), gr_complex(-0.0194, -0.0817), gr_complex( 0.0573,  0.0646),
      gr_complex( 0.0364, -0.0137), gr_complex( 0.0049,  0.0492), gr_complex(-0.1662, -0.0627),
      gr_complex(-0.0128, -0.0749), gr_complex( 0.0567, -0.0294), gr_complex( 0.0623,  0.1011),
      gr_complex(-0.0303,  0.0183), gr_complex( 0.0269,  0.0697), gr_complex(-0.0031,  0.0668),
      gr_complex( 0.0580, -0.0482), gr_complex( 0.0086,  0.0708), gr_complex( 0.0217,  0.0084),
      gr_complex(-0.0453,  0.0194), gr_complex(-0.0693, -0.0918), gr_complex(-0.0003, -0.0054),
      gr_complex(-0.0853, -0.0203), gr_complex(-0.0377,  0.0002), gr_complex(-0.0319, -0.1253),
      gr_complex( 0.0039, -0.0257), gr_complex(-0.0704,  0.0516), gr_complex( 0.0259,  0.0011),
      gr_complex( 0.0336, -0.0439), gr_complex(-0.0019,  0.1157), gr_complex( 0.0170,  0.1459),
      gr_complex( 0.0538,  0.0024), gr_complex(-0.0130,  0.0329), gr_complex( 0.0025,  0.0075),
      gr_complex( 0.0163, -0.0529), gr_complex( 0.0152,  0.1191), gr_complex( 0.0377,  0.0726),
      gr_complex( 0.0256, -0.0046), gr_complex(-0.0175,  0.0525), gr_complex(-0.0248, -0.0085),
      gr_complex( 0.0992,  0.0558), gr_complex( 0.0660,  0.0236), gr_complex(-0.0761,  0.0151),
      gr_complex( 0.0410, -0.0222), gr_complex( 0.0590,  0.0093), gr_complex(-0.0004,  0.0330),
      gr_complex( 0.0649,  0.0620), gr_complex(-0.0426, -0.0069), gr_complex(-0.0110,  0.0108),
      gr_complex( 0.0210, -0.0116), gr_complex(-0.0669,  0.0569), gr_complex( 0.0071, -0.0049),
      gr_complex(-0.0207, -0.0154), gr_complex(-0.0979,  0.0411), gr_complex( 0.0491,  0.0319),
      gr_complex(-0.0284,  0.0140), gr_complex(-0.0762,  0.0748), gr_complex(-0.0723,  0.0291),
      gr_complex(-0.1319, -0.0100), gr_complex(-0.0602, -0.0470), gr_complex( 0.0365, -0.0101),
      gr_complex(-0.0518,  0.0999), gr_complex(-0.0264, -0.0501), gr_complex(-0.0346, -0.0091),
      gr_complex(-0.0037,  0.0303), gr_complex( 0.0188, -0.0382), gr_complex(-0.0540, -0.0178),
      gr_complex( 0.0162,  0.0476), gr_complex( 0.0451, -0.0400), gr_complex(-0.0468,  0.0327),
      gr_complex(-0.0018,  0.0444), gr_complex( 0.0426,  0.0093), gr_complex(-0.0286,  0.0071),
      gr_complex(-0.0309, -0.0092), gr_complex( 0.0002, -0.0121), gr_complex( 0.0177, -0.0057),
      gr_complex( 0.0381, -0.0057), gr_complex(-0.0155,  0.0047), gr_complex(-0.0199, -0.0427),
      gr_complex( 0.0336, -0.0453), gr_complex( 0.0236, -0.0113), gr_complex( 0.0058, -0.0313),
      gr_complex( 0.0267, -0.0149), gr_complex(-0.0090,  0.0204), gr_complex(-0.0159,  0.0128),
      gr_complex( 0.0109,  0.0014), gr_complex(-0.0308,  0.0018), gr_complex(-0.0089,  0.0047),
      gr_complex(-0.0033,  0.0279), gr_complex(-0.0257,  0.0005), gr_complex(-0.0130, -0.0374),
      gr_complex( 0.0131,  0.0205), gr_complex(-0.0130,  0.0354), gr_complex(-0.0193, -0.0178),
      gr_complex(-0.0342,  0.0057), gr_complex( 0.0141, -0.0183), gr_complex( 0.0017, -0.0167),
      gr_complex(-0.0351,  0.0126), gr_complex(-0.0064, -0.0034), gr_complex( 0.0078, -0.0102),
      gr_complex(-0.0159,  0.0141), gr_complex(-0.0029, -0.0209), gr_complex( 0.0256, -0.0014),
      gr_complex(-0.0088,  0.0150), gr_complex(-0.0086, -0.0279), gr_complex(-0.0030, -0.0175),
      gr_complex( 0.0255, -0.0158), gr_complex( 0.0069, -0.0032), gr_complex(-0.0011,  0.0091),
      gr_complex(-0.0081, -0.0122), gr_complex(-0.0012, -0.0213), gr_complex( 0.0165,  0.0158),
      gr_complex( 0.0004, -0.0053), gr_complex(-0.0204, -0.0001), gr_complex(-0.0004, -0.0044),
      gr_complex( 0.0019, -0.0211), gr_complex( 0.0152, -0.0104), gr_complex( 0.0322,  0.0028),
      gr_complex(-0.0033,  0.0097), gr_complex(-0.0019,  0.0100), gr_complex( 0.0099, -0.0165),
      gr_complex( 0.0045, -0.0186), gr_complex( 0.0165,  0.0075), gr_complex( 0.0158,  0.0100),
      gr_complex(-0.0073,  0.0084)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_256_1_4[256] = {
      gr_complex( 0.0004,  0.0031), gr_complex(-0.0165,  0.0023), gr_complex( 0.0245, -0.0058),
      gr_complex( 0.0167,  0.0009), gr_complex( 0.0120,  0.0131), gr_complex(-0.0032,  0.0200),
      gr_complex(-0.0019, -0.0154), gr_complex( 0.0200,  0.0029), gr_complex( 0.0042, -0.0238),
      gr_complex( 0.0157, -0.0064), gr_complex(-0.0295, -0.0042), gr_complex( 0.0148, -0.0095),
      gr_complex( 0.0146,  0.0196), gr_complex( 0.0070, -0.0012), gr_complex( 0.0049,  0.0226),
      gr_complex( 0.0121, -0.0134), gr_complex( 0.0223,  0.0167), gr_complex(-0.0111, -0.0337),
      gr_complex( 0.0202, -0.0008), gr_complex(-0.0357,  0.0115), gr_complex(-0.0241,  0.0010),
      gr_complex(-0.0029, -0.0266), gr_complex( 0.0274, -0.0347), gr_complex(-0.0138,  0.0159),
      gr_complex(-0.0062, -0.0129), gr_complex(-0.0062,  0.0346), gr_complex(-0.0111, -0.0339),
      gr_complex( 0.0195,  0.0310), gr_complex(-0.0181,  0.0247), gr_complex( 0.0279, -0.0055),
      gr_complex( 0.0034, -0.0307), gr_complex( 0.0588, -0.0812), gr_complex(-0.0080,  0.0307),
      gr_complex( 0.0104, -0.0341), gr_complex( 0.0543,  0.0322), gr_complex(-0.0239, -0.0256),
      gr_complex( 0.0737, -0.0372), gr_complex(-0.0150,  0.0085), gr_complex( 0.0014, -0.0074),
      gr_complex(-0.0607, -0.0073), gr_complex(-0.0182, -0.0675), gr_complex( 0.0246,  0.0110),
      gr_complex(-0.0361,  0.0162), gr_complex( 0.0417,  0.0262), gr_complex(-0.0793, -0.0438),
      gr_complex( 0.0282, -0.0134), gr_complex(-0.0271,  0.0225), gr_complex(-0.0082,  0.0245),
      gr_complex(-0.0516,  0.0234), gr_complex(-0.0201, -0.0263), gr_complex( 0.0029,  0.0384),
      gr_complex(-0.0074, -0.0198), gr_complex( 0.0063,  0.0011), gr_complex(-0.0095, -0.0239),
      gr_complex(-0.0131,  0.0117), gr_complex( 0.0365,  0.0050), gr_complex( 0.0614, -0.0123),
      gr_complex( 0.0199,  0.0308), gr_complex(-0.0288, -0.0496), gr_complex(-0.0569, -0.0760),
      gr_complex(-0.0029, -0.0749), gr_complex(-0.0351,  0.0013), gr_complex(-0.0374,  0.0085),
      gr_complex(-0.0520, -0.0930), gr_complex( 0.0676, -0.0855), gr_complex( 0.0141,  0.0020),
      gr_complex(-0.0583, -0.0240), gr_complex(-0.0727,  0.0025), gr_complex(-0.0034, -0.0268),
      gr_complex(-0.0029, -0.0087), gr_complex(-0.0646,  0.0253), gr_complex(-0.0019,  0.0340),
      gr_complex(-0.0122,  0.0312), gr_complex( 0.0516,  0.0568), gr_complex(-0.0689, -0.0054),
      gr_complex( 0.0023,  0.0206), gr_complex( 0.0071,  0.0487), gr_complex( 0.0229,  0.1551),
      gr_complex( 0.0456, -0.0233), gr_complex(-0.0221, -0.1167), gr_complex(-0.0036, -0.0234),
      gr_complex(-0.1774,  0.0352), gr_complex(-0.0914, -0.0488), gr_complex(-0.0358, -0.0015),
      gr_complex(-0.0459, -0.0274), gr_complex( 0.0158, -0.0174), gr_complex( 0.0203,  0.0313),
      gr_complex( 0.0291, -0.0062), gr_complex(-0.0112,  0.0039), gr_complex(-0.0673, -0.0204),
      gr_complex(-0.0190,  0.0629), gr_complex(-0.0741, -0.0483), gr_complex( 0.0910,  0.0376),
      gr_complex(-0.0519, -0.0298), gr_complex(-0.0258,  0.0056), gr_complex( 0.0995, -0.0009),
      gr_complex(-0.0163,  0.0086), gr_complex(-0.0276, -0.0308), gr_complex(-0.0626,  0.1174),
      gr_complex(-0.0080,  0.1113), gr_complex( 0.1088, -0.0207), gr_complex(-0.0399, -0.0708),
      gr_complex( 0.0527,  0.1480), gr_complex(-0.0622, -0.0033), gr_complex( 0.0701, -0.0610),
      gr_complex( 0.0942,  0.0181), gr_complex(-0.0106, -0.0759), gr_complex( 0.0476,  0.0020),
      gr_complex(-0.0402,  0.0468), gr_complex( 0.0710,  0.1213), gr_complex( 0.0390, -0.0065),
      gr_complex(-0.0622,  0.0677), gr_complex( 0.0692,  0.0101), gr_complex(-0.0946, -0.0057),
      gr_complex(-0.0317,  0.0370), gr_complex( 0.0522,  0.0168), gr_complex( 0.0450,  0.0263),
      gr_complex( 0.0403, -0.0497), gr_complex(-0.0698, -0.0020), gr_complex( 0.0136, -0.1523),
      gr_complex(-0.0092, -0.0333), gr_complex(-0.0302, -0.1293), gr_complex(-0.1819,  0.0251),
      gr_complex( 0.0168, -0.0055), gr_complex(-0.0513,  0.0073), gr_complex( 0.0941, -0.0179),
      gr_complex(-0.0100, -0.0069), gr_complex( 0.0060,  0.0862), gr_complex( 0.0444,  0.0322),
      gr_complex(-0.0899,  0.0144), gr_complex(-0.0003, -0.0191), gr_complex( 0.0013, -0.0402),
      gr_complex(-0.0732,  0.0329), gr_complex( 0.0754,  0.0799), gr_complex( 0.0019,  0.0442),
      gr_complex(-0.1632,  0.1292), gr_complex(-0.0033, -0.0291), gr_complex( 0.0383, -0.0287),
      gr_complex(-0.0484, -0.0328), gr_complex(-0.0322, -0.0234), gr_complex(-0.0061, -0.0265),
      gr_complex(-0.0050, -0.0108), gr_complex(-0.0351,  0.0002), gr_complex(-0.0227,  0.2009),
      gr_complex( 0.0349, -0.0280), gr_complex( 0.0191, -0.0241), gr_complex(-0.0037,  0.0320),
      gr_complex( 0.0299,  0.0478), gr_complex(-0.0168,  0.0554), gr_complex(-0.0853,  0.0271),
      gr_complex( 0.0122, -0.1058), gr_complex(-0.0472, -0.0635), gr_complex( 0.0330, -0.0822),
      gr_complex(-0.0081, -0.0194), gr_complex( 0.0177,  0.0373), gr_complex( 0.0732,  0.0810),
      gr_complex(-0.0119, -0.0275), gr_complex(-0.1534, -0.0349), gr_complex( 0.0106, -0.0138),
      gr_complex(-0.0240,  0.0397), gr_complex(-0.0689,  0.0332), gr_complex(-0.0435,  0.0261),
      gr_complex(-0.1036, -0.0213), gr_complex( 0.0646, -0.0473), gr_complex(-0.0205, -0.0833),
      gr_complex(-0.0855,  0.0084), gr_complex(-0.0499, -0.0329), gr_complex( 0.0729, -0.0157),
      gr_complex(-0.0924, -0.1429), gr_complex( 0.1104,  0.0347), gr_complex( 0.0309, -0.0577),
      gr_complex(-0.0019, -0.0302), gr_complex(-0.0015,  0.0362), gr_complex( 0.0603,  0.0414),
      gr_complex(-0.0494, -0.0734), gr_complex(-0.0120, -0.0379), gr_complex( 0.0004, -0.0254),
      gr_complex(-0.0266,  0.0435), gr_complex(-0.0488,  0.0352), gr_complex( 0.1058, -0.0267),
      gr_complex( 0.0173, -0.0380), gr_complex( 0.0812,  0.0322), gr_complex( 0.0258,  0.0056),
      gr_complex( 0.0144,  0.0159), gr_complex( 0.0149,  0.0106), gr_complex(-0.0484, -0.0422),
      gr_complex( 0.0146, -0.0976), gr_complex( 0.0217,  0.0224), gr_complex(-0.0622, -0.0254),
      gr_complex( 0.0412, -0.0070), gr_complex( 0.0223,  0.0752), gr_complex(-0.0536,  0.0136),
      gr_complex( 0.0261, -0.0386), gr_complex( 0.0222, -0.0023), gr_complex(-0.0280, -0.0185),
      gr_complex( 0.0213, -0.0039), gr_complex( 0.0238, -0.0043), gr_complex( 0.0264,  0.0105),
      gr_complex(-0.0012,  0.0556), gr_complex(-0.0148, -0.0191), gr_complex(-0.0112,  0.0218),
      gr_complex(-0.0326,  0.0460), gr_complex(-0.0083, -0.0179), gr_complex( 0.0381, -0.0502),
      gr_complex(-0.0122,  0.0119), gr_complex(-0.0064,  0.0024), gr_complex( 0.0007, -0.0033),
      gr_complex(-0.0437,  0.0369), gr_complex(-0.0468,  0.0519), gr_complex( 0.0014, -0.0422),
      gr_complex( 0.0029, -0.0339), gr_complex(-0.0288,  0.0223), gr_complex( 0.0145, -0.0327),
      gr_complex( 0.0238,  0.0242), gr_complex(-0.0043,  0.0048), gr_complex(-0.0117, -0.0185),
      gr_complex(-0.0012, -0.0296), gr_complex( 0.0078, -0.0234), gr_complex( 0.0232, -0.0268),
      gr_complex(-0.0131,  0.0102), gr_complex( 0.0147, -0.0376), gr_complex( 0.0345, -0.0043),
      gr_complex(-0.0098, -0.0122), gr_complex( 0.0235, -0.0001), gr_complex( 0.0117,  0.0275),
      gr_complex(-0.0366,  0.0143), gr_complex(-0.0001, -0.0301), gr_complex( 0.0227,  0.0184),
      gr_complex(-0.0073, -0.0195), gr_complex( 0.0278, -0.0195), gr_complex( 0.0095, -0.0022),
      gr_complex(-0.0081,  0.0198), gr_complex( 0.0206, -0.0085), gr_complex(-0.0043, -0.0150),
      gr_complex( 0.0038,  0.0112), gr_complex( 0.0204,  0.0041), gr_complex(-0.0062, -0.0044),
      gr_complex(-0.0074,  0.0108), gr_complex( 0.0211, -0.0214), gr_complex( 0.0184, -0.0080),
      gr_complex( 0.0038,  0.0089), gr_complex( 0.0050,  0.0260), gr_complex(-0.0251,  0.0103),
      gr_complex(-0.0140, -0.0046), gr_complex( 0.0102, -0.0027), gr_complex(-0.0022,  0.0001),
      gr_complex( 0.0288,  0.0004), gr_complex( 0.0359,  0.0226), gr_complex(-0.0069,  0.0128),
      gr_complex( 0.0070,  0.0100), gr_complex(-0.0129,  0.0045), gr_complex(-0.0151, -0.0074),
      gr_complex(-0.0007, -0.0046), gr_complex(-0.0020,  0.0061), gr_complex(-0.0150,  0.0070),
      gr_complex(-0.0005, -0.0001)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_256_2_4[256] = {
      gr_complex(-0.0095, -0.0115), gr_complex( 0.0214, -0.0144), gr_complex( 0.0091, -0.0007),
      gr_complex(-0.0083, -0.0244), gr_complex(-0.0058,  0.0098), gr_complex(-0.0153,  0.0044),
      gr_complex( 0.0193, -0.0141), gr_complex( 0.0248,  0.0116), gr_complex(-0.0403,  0.0087),
      gr_complex( 0.0065, -0.0029), gr_complex(-0.0225,  0.0106), gr_complex( 0.0148, -0.0256),
      gr_complex( 0.0306,  0.0029), gr_complex(-0.0124, -0.0195), gr_complex(-0.0092, -0.0208),
      gr_complex( 0.0573, -0.0050), gr_complex( 0.0105, -0.0097), gr_complex( 0.0427,  0.0125),
      gr_complex( 0.0247,  0.0039), gr_complex( 0.0036,  0.0034), gr_complex(-0.0096,  0.0298),
      gr_complex( 0.0317,  0.0184), gr_complex( 0.0046,  0.0082), gr_complex(-0.0107, -0.0248),
      gr_complex( 0.0334, -0.0321), gr_complex( 0.0025,  0.0164), gr_complex( 0.0374, -0.0319),
      gr_complex( 0.0079, -0.0462), gr_complex( 0.0337, -0.0101), gr_complex( 0.0577,  0.0256),
      gr_complex( 0.0958,  0.0245), gr_complex(-0.0413,  0.0127), gr_complex( 0.0530, -0.0345),
      gr_complex(-0.0017,  0.0302), gr_complex( 0.0160,  0.0250), gr_complex( 0.0101,  0.0312),
      gr_complex(-0.0038,  0.0149), gr_complex(-0.0561, -0.0541), gr_complex( 0.0918, -0.0501),
      gr_complex(-0.0360,  0.0266), gr_complex( 0.0022,  0.0090), gr_complex( 0.0212,  0.0352),
      gr_complex( 0.0541, -0.0472), gr_complex( 0.0411,  0.0436), gr_complex( 0.0226,  0.1087),
      gr_complex( 0.0453,  0.0905), gr_complex( 0.0125,  0.0543), gr_complex(-0.0285,  0.0339),
      gr_complex(-0.0091,  0.0193), gr_complex(-0.0279,  0.0665), gr_complex( 0.0298,  0.0005),
      gr_complex(-0.0094, -0.0321), gr_complex(-0.0423,  0.0058), gr_complex( 0.1175,  0.0629),
      gr_complex( 0.0007,  0.0022), gr_complex(-0.0003, -0.0056), gr_complex( 0.0961,  0.0775),
      gr_complex(-0.0342,  0.0347), gr_complex(-0.0584,  0.0244), gr_complex( 0.0618,  0.0488),
      gr_complex( 0.0094, -0.0103), gr_complex( 0.0690, -0.0288), gr_complex(-0.0118,  0.0348),
      gr_complex(-0.0829, -0.0168), gr_complex(-0.0220,  0.0329), gr_complex( 0.0963, -0.0196),
      gr_complex( 0.0076, -0.0453), gr_complex(-0.0010,  0.0152), gr_complex(-0.1000,  0.0111),
      gr_complex( 0.0512, -0.1350), gr_complex( 0.0320,  0.0005), gr_complex(-0.0234, -0.0242),
      gr_complex(-0.1218,  0.1005), gr_complex( 0.0513,  0.0074), gr_complex(-0.0450,  0.0322),
      gr_complex(-0.0457, -0.0417), gr_complex( 0.0226,  0.0302), gr_complex(-0.0042, -0.0095),
      gr_complex(-0.0636,  0.0291), gr_complex( 0.0688,  0.0167), gr_complex( 0.0092,  0.0675),
      gr_complex( 0.0504, -0.0324), gr_complex(-0.0296,  0.0041), gr_complex(-0.0358,  0.0213),
      gr_complex( 0.0073, -0.0085), gr_complex(-0.0088, -0.0135), gr_complex( 0.0602, -0.0961),
      gr_complex( 0.0388,  0.0246), gr_complex( 0.0285,  0.0402), gr_complex( 0.0256,  0.0598),
      gr_complex( 0.0152, -0.0258), gr_complex( 0.0571,  0.1337), gr_complex( 0.0309, -0.0841),
      gr_complex(-0.0649, -0.0067), gr_complex( 0.0187,  0.0060), gr_complex( 0.0298, -0.0296),
      gr_complex( 0.0099, -0.0219), gr_complex( 0.0864,  0.0268), gr_complex( 0.1200, -0.0282),
      gr_complex(-0.0172, -0.0794), gr_complex(-0.0092, -0.0296), gr_complex( 0.0288,  0.0582),
      gr_complex(-0.0240,  0.0075), gr_complex( 0.0306, -0.0892), gr_complex( 0.0711,  0.0671),
      gr_complex(-0.0231, -0.1139), gr_complex(-0.0977,  0.0218), gr_complex(-0.0753,  0.0668),
      gr_complex( 0.0236,  0.1099), gr_complex( 0.0064,  0.0485), gr_complex(-0.0175,  0.1053),
      gr_complex(-0.0145,  0.0059), gr_complex( 0.0530, -0.0856), gr_complex(-0.0101,  0.0664),
      gr_complex( 0.0534, -0.0607), gr_complex( 0.0027, -0.0533), gr_complex(-0.1386, -0.0592),
      gr_complex( 0.0098, -0.0105), gr_complex(-0.0293, -0.0176), gr_complex(-0.1022, -0.0058),
      gr_complex( 0.0386, -0.0929), gr_complex(-0.0250,  0.0395), gr_complex( 0.1014, -0.0147),
      gr_complex(-0.0455, -0.0205), gr_complex(-0.0699, -0.0078), gr_complex( 0.1111,  0.0509),
      gr_complex( 0.0550, -0.0562), gr_complex( 0.0116, -0.0573), gr_complex( 0.0005, -0.0406),
      gr_complex(-0.0107, -0.0072), gr_complex( 0.0956,  0.0168), gr_complex(-0.0160, -0.0290),
      gr_complex( 0.0138, -0.0271), gr_complex( 0.0265,  0.0422), gr_complex(-0.0136,  0.1175),
      gr_complex( 0.0113,  0.0467), gr_complex( 0.0348, -0.0961), gr_complex( 0.0489,  0.0669),
      gr_complex(-0.0121,  0.0693), gr_complex(-0.0607,  0.0727), gr_complex( 0.0182, -0.0324),
      gr_complex(-0.0608, -0.0964), gr_complex( 0.0113, -0.0474), gr_complex(-0.0469,  0.0469),
      gr_complex( 0.0054, -0.0206), gr_complex(-0.0110,  0.0124), gr_complex(-0.0400, -0.0097),
      gr_complex( 0.0029,  0.0327), gr_complex( 0.1017, -0.0656), gr_complex(-0.0125, -0.0844),
      gr_complex( 0.0095,  0.0143), gr_complex(-0.0060, -0.0671), gr_complex( 0.0098, -0.0611),
      gr_complex(-0.0325, -0.0162), gr_complex( 0.0254,  0.0433), gr_complex( 0.0294,  0.0107),
      gr_complex(-0.0319,  0.0160), gr_complex( 0.0346, -0.0148), gr_complex(-0.0849,  0.0119),
      gr_complex(-0.1517,  0.0193), gr_complex(-0.0099, -0.1525), gr_complex(-0.0549,  0.0977),
      gr_complex( 0.0163, -0.0293), gr_complex(-0.0692, -0.0100), gr_complex(-0.0214,  0.0268),
      gr_complex( 0.0630,  0.0659), gr_complex(-0.0328,  0.0053), gr_complex( 0.1390, -0.0042),
      gr_complex(-0.0129,  0.0032), gr_complex(-0.0456, -0.0309), gr_complex(-0.0114, -0.0780),
      gr_complex(-0.0361, -0.0061), gr_complex( 0.1016,  0.0845), gr_complex(-0.0185,  0.0375),
      gr_complex( 0.0561,  0.0548), gr_complex(-0.0945,  0.0013), gr_complex( 0.0399, -0.0431),
      gr_complex(-0.0148, -0.0108), gr_complex(-0.0760, -0.0227), gr_complex(-0.0134,  0.0459),
      gr_complex(-0.0696,  0.0830), gr_complex(-0.0221,  0.0703), gr_complex(-0.0422, -0.1036),
      gr_complex( 0.0373,  0.0137), gr_complex(-0.0222, -0.0047), gr_complex( 0.0168,  0.0170),
      gr_complex( 0.1070,  0.0484), gr_complex( 0.0021,  0.1277), gr_complex( 0.0101,  0.0724),
      gr_complex(-0.0653, -0.0750), gr_complex( 0.0776,  0.0021), gr_complex(-0.0003, -0.0222),
      gr_complex( 0.0137, -0.0263), gr_complex( 0.0611,  0.0247), gr_complex(-0.0122,  0.0609),
      gr_complex( 0.0219,  0.0131), gr_complex(-0.0120, -0.0151), gr_complex( 0.0236, -0.0190),
      gr_complex(-0.0021, -0.0433), gr_complex( 0.0209, -0.0087), gr_complex(-0.0177, -0.0797),
      gr_complex(-0.0266,  0.0221), gr_complex( 0.0276, -0.0203), gr_complex(-0.0296,  0.0109),
      gr_complex( 0.0119, -0.0177), gr_complex( 0.0224,  0.0098), gr_complex( 0.0411,  0.0575),
      gr_complex( 0.0389,  0.0251), gr_complex( 0.0300,  0.0071), gr_complex( 0.0106, -0.0328),
      gr_complex(-0.0148, -0.0189), gr_complex(-0.0229, -0.0278), gr_complex(-0.0030,  0.0476),
      gr_complex( 0.0127,  0.0214), gr_complex(-0.0086, -0.0198), gr_complex( 0.0357, -0.0229),
      gr_complex( 0.0305, -0.0416), gr_complex(-0.0011,  0.0044), gr_complex( 0.0051, -0.0390),
      gr_complex(-0.0337,  0.0131), gr_complex(-0.0243, -0.0181), gr_complex(-0.0384,  0.0004),
      gr_complex( 0.0128, -0.0635), gr_complex( 0.0252, -0.0093), gr_complex( 0.0063,  0.0004),
      gr_complex( 0.0024, -0.0111), gr_complex(-0.0090,  0.0421), gr_complex(-0.0289,  0.0131),
      gr_complex(-0.0206,  0.0193), gr_complex(-0.0057, -0.0389), gr_complex( 0.0056, -0.0005),
      gr_complex(-0.0024,  0.0033), gr_complex(-0.0068,  0.0040), gr_complex(-0.0015,  0.0212),
      gr_complex(-0.0220, -0.0047), gr_complex( 0.0031,  0.0026), gr_complex( 0.0070, -0.0146),
      gr_complex(-0.0035, -0.0004), gr_complex( 0.0091, -0.0077), gr_complex( 0.0033,  0.0068),
      gr_complex(-0.0061, -0.0087), gr_complex(-0.0139, -0.0002), gr_complex(-0.0114,  0.0032),
      gr_complex(-0.0124, -0.0053), gr_complex(-0.0035, -0.0025), gr_complex( 0.0208,  0.0024),
      gr_complex( 0.0006,  0.0152), gr_complex( 0.0058,  0.0051), gr_complex(-0.0016,  0.0084),
      gr_complex(-0.0100,  0.0088), gr_complex(-0.0045, -0.0140), gr_complex(-0.0190, -0.0092),
      gr_complex( 0.0034,  0.0046), gr_complex(-0.0031,  0.0174), gr_complex( 0.0075,  0.0059),
      gr_complex( 0.0212,  0.0115)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_256_3_4[256] = {
      gr_complex( 0.0211, -0.0237), gr_complex(-0.0070,  0.0292), gr_complex( 0.0116, -0.0123),
      gr_complex(-0.0091,  0.0070), gr_complex( 0.0220,  0.0165), gr_complex(-0.0054, -0.0128),
      gr_complex( 0.0086,  0.0114), gr_complex(-0.0135, -0.0310), gr_complex(-0.0046,  0.0073),
      gr_complex(-0.0007, -0.0182), gr_complex(-0.0266,  0.0079), gr_complex( 0.0448,  0.0005),
      gr_complex(-0.0357,  0.0075), gr_complex( 0.0398,  0.0369), gr_complex( 0.0146, -0.0004),
      gr_complex(-0.0320, -0.0088), gr_complex(-0.0078, -0.0245), gr_complex(-0.0258,  0.0186),
      gr_complex( 0.0122, -0.0585), gr_complex(-0.0119,  0.0227), gr_complex(-0.0018, -0.0460),
      gr_complex( 0.0422,  0.0124), gr_complex(-0.0073, -0.0089), gr_complex(-0.0126, -0.0086),
      gr_complex( 0.0212,  0.0121), gr_complex(-0.0488, -0.0056), gr_complex( 0.0524, -0.0306),
      gr_complex(-0.0643,  0.0008), gr_complex( 0.0315, -0.0157), gr_complex(-0.0223, -0.0032),
      gr_complex(-0.0123,  0.0214), gr_complex( 0.0434,  0.0052), gr_complex(-0.0231,  0.0217),
      gr_complex( 0.0270, -0.0297), gr_complex(-0.0221,  0.0443), gr_complex( 0.0056,  0.0435),
      gr_complex(-0.0002, -0.0108), gr_complex(-0.0257, -0.0058), gr_complex(-0.0278, -0.0707),
      gr_complex(-0.0369,  0.0560), gr_complex( 0.0098, -0.0661), gr_complex(-0.0130,  0.0416),
      gr_complex(-0.0414, -0.0028), gr_complex( 0.0282,  0.0239), gr_complex( 0.0148, -0.0251),
      gr_complex(-0.0901, -0.0192), gr_complex( 0.0346,  0.0590), gr_complex(-0.0529, -0.0171),
      gr_complex( 0.0401, -0.0214), gr_complex(-0.1002,  0.0356), gr_complex( 0.0908, -0.0431),
      gr_complex( 0.0073, -0.0057), gr_complex( 0.0238,  0.0227), gr_complex( 0.0621,  0.0123),
      gr_complex( 0.0217,  0.0780), gr_complex( 0.0150, -0.0200), gr_complex( 0.0298,  0.0636),
      gr_complex(-0.0417, -0.0142), gr_complex(-0.0049,  0.0294), gr_complex(-0.0120, -0.0143),
      gr_complex(-0.0142, -0.0272), gr_complex(-0.0087,  0.0500), gr_complex( 0.0184,  0.0114),
      gr_complex(-0.0512,  0.0293), gr_complex( 0.0972,  0.0890), gr_complex(-0.0258, -0.0021),
      gr_complex(-0.0160,  0.0054), gr_complex(-0.0876, -0.0250), gr_complex(-0.0553,  0.0453),
      gr_complex( 0.0790, -0.0639), gr_complex(-0.0140,  0.0222), gr_complex( 0.0795, -0.0535),
      gr_complex( 0.1055,  0.0278), gr_complex( 0.0957, -0.0663), gr_complex(-0.0064, -0.1034),
      gr_complex(-0.1420,  0.0702), gr_complex( 0.0524, -0.0166), gr_complex(-0.0174, -0.0368),
      gr_complex( 0.0305,  0.0873), gr_complex( 0.0137, -0.0178), gr_complex( 0.0631,  0.0323),
      gr_complex( 0.0218, -0.0448), gr_complex( 0.0269, -0.0052), gr_complex(-0.0200, -0.0671),
      gr_complex(-0.0154,  0.0155), gr_complex( 0.0603,  0.0687), gr_complex( 0.0208, -0.0089),
      gr_complex( 0.0244,  0.0041), gr_complex(-0.0808,  0.0414), gr_complex(-0.0620, -0.0623),
      gr_complex( 0.0037,  0.0448), gr_complex( 0.0269,  0.0228), gr_complex( 0.0126, -0.1322),
      gr_complex( 0.0072, -0.1659), gr_complex(-0.0507, -0.0395), gr_complex(-0.1385, -0.0139),
      gr_complex(-0.0272,  0.0014), gr_complex(-0.0127,  0.0379), gr_complex( 0.1341,  0.0126),
      gr_complex(-0.0116, -0.0024), gr_complex(-0.0320, -0.0221), gr_complex( 0.0143,  0.0308),
      gr_complex( 0.0050, -0.0549), gr_complex( 0.0392,  0.0657), gr_complex(-0.0081, -0.0568),
      gr_complex(-0.0231,  0.0137), gr_complex( 0.0250, -0.0043), gr_complex(-0.0278,  0.0171),
      gr_complex( 0.0166,  0.0358), gr_complex(-0.0774, -0.0310), gr_complex( 0.0464, -0.0477),
      gr_complex(-0.0226, -0.0205), gr_complex(-0.0268,  0.0262), gr_complex(-0.1065,  0.0432),
      gr_complex( 0.0427, -0.0003), gr_complex( 0.0213,  0.0455), gr_complex(-0.0832, -0.0728),
      gr_complex(-0.1055,  0.0820), gr_complex( 0.0307,  0.0616), gr_complex( 0.0310, -0.0445),
      gr_complex(-0.0074,  0.0516), gr_complex( 0.0457,  0.1129), gr_complex(-0.0388,  0.0792),
      gr_complex( 0.1166, -0.0501), gr_complex( 0.0675,  0.0185), gr_complex(-0.1232, -0.0267),
      gr_complex(-0.0747, -0.0479), gr_complex(-0.0015, -0.0661), gr_complex(-0.0185, -0.0200),
      gr_complex(-0.0255, -0.0336), gr_complex( 0.0772, -0.0365), gr_complex( 0.0150,  0.1393),
      gr_complex(-0.0412,  0.0707), gr_complex( 0.0105,  0.0223), gr_complex(-0.0414, -0.0247),
      gr_complex(-0.0508, -0.0152), gr_complex(-0.0168, -0.0904), gr_complex( 0.0574,  0.0713),
      gr_complex( 0.0329, -0.0123), gr_complex(-0.0088,  0.0524), gr_complex(-0.1721, -0.0558),
      gr_complex(-0.0176, -0.0641), gr_complex( 0.0461, -0.0371), gr_complex( 0.0703,  0.1032),
      gr_complex(-0.0320,  0.0071), gr_complex( 0.0412,  0.0825), gr_complex( 0.0114,  0.0560),
      gr_complex( 0.0612, -0.0583), gr_complex(-0.0033,  0.0810), gr_complex( 0.0267,  0.0067),
      gr_complex(-0.0649,  0.0093), gr_complex(-0.0820, -0.0942), gr_complex(-0.0005, -0.0039),
      gr_complex(-0.0726, -0.0264), gr_complex(-0.0392, -0.0038), gr_complex(-0.0386, -0.1308),
      gr_complex( 0.0072, -0.0195), gr_complex(-0.0786,  0.0574), gr_complex( 0.0202, -0.0029),
      gr_complex( 0.0342, -0.0378), gr_complex(-0.0011,  0.1198), gr_complex( 0.0202,  0.1299),
      gr_complex( 0.0584,  0.0041), gr_complex(-0.0159,  0.0202), gr_complex( 0.0091,  0.0178),
      gr_complex( 0.0185, -0.0474), gr_complex( 0.0144,  0.1223), gr_complex( 0.0401,  0.0643),
      gr_complex( 0.0179,  0.0047), gr_complex(-0.0192,  0.0552), gr_complex(-0.0144, -0.0140),
      gr_complex( 0.0939,  0.0553), gr_complex( 0.0667,  0.0266), gr_complex(-0.0589,  0.0117),
      gr_complex( 0.0418, -0.0183), gr_complex( 0.0583,  0.0065), gr_complex( 0.0014,  0.0365),
      gr_complex( 0.0670,  0.0566), gr_complex(-0.0439, -0.0116), gr_complex(-0.0124,  0.0081),
      gr_complex( 0.0177, -0.0023), gr_complex(-0.0724,  0.0590), gr_complex( 0.0115, -0.0082),
      gr_complex(-0.0267,  0.0039), gr_complex(-0.0838,  0.0484), gr_complex( 0.0519,  0.0317),
      gr_complex(-0.0288,  0.0090), gr_complex(-0.0736,  0.0737), gr_complex(-0.0842,  0.0166),
      gr_complex(-0.1247, -0.0084), gr_complex(-0.0482, -0.0541), gr_complex( 0.0330, -0.0078),
      gr_complex(-0.0502,  0.1074), gr_complex(-0.0202, -0.0499), gr_complex(-0.0393, -0.0030),
      gr_complex(-0.0041,  0.0345), gr_complex( 0.0255, -0.0323), gr_complex(-0.0610, -0.0145),
      gr_complex( 0.0197,  0.0492), gr_complex( 0.0438, -0.0425), gr_complex(-0.0535,  0.0342),
      gr_complex( 0.0051,  0.0403), gr_complex( 0.0502,  0.0121), gr_complex(-0.0407,  0.0151),
      gr_complex(-0.0401, -0.0054), gr_complex( 0.0063, -0.0152), gr_complex( 0.0193,  0.0040),
      gr_complex( 0.0305, -0.0091), gr_complex(-0.0145,  0.0101), gr_complex(-0.0102, -0.0379),
      gr_complex( 0.0358, -0.0500), gr_complex( 0.0188, -0.0027), gr_complex( 0.0091, -0.0166),
      gr_complex( 0.0185, -0.0087), gr_complex(-0.0185,  0.0229), gr_complex(-0.0081,  0.0080),
      gr_complex( 0.0103,  0.0002), gr_complex(-0.0278, -0.0017), gr_complex(-0.0086,  0.0020),
      gr_complex(-0.0058,  0.0267), gr_complex(-0.0254, -0.0042), gr_complex(-0.0116, -0.0383),
      gr_complex( 0.0108,  0.0329), gr_complex(-0.0202,  0.0263), gr_complex(-0.0184, -0.0273),
      gr_complex(-0.0317,  0.0100), gr_complex( 0.0196, -0.0292), gr_complex(-0.0052, -0.0148),
      gr_complex(-0.0308,  0.0140), gr_complex(-0.0090, -0.0070), gr_complex( 0.0127, -0.0124),
      gr_complex(-0.0128,  0.0053), gr_complex(-0.0087, -0.0211), gr_complex( 0.0289,  0.0034),
      gr_complex(-0.0091, -0.0004), gr_complex(-0.0156, -0.0245), gr_complex( 0.0092, -0.0066),
      gr_complex( 0.0224, -0.0202), gr_complex( 0.0037,  0.0032), gr_complex( 0.0033,  0.0121),
      gr_complex(-0.0031, -0.0144), gr_complex(-0.0002, -0.0118), gr_complex( 0.0163,  0.0145),
      gr_complex( 0.0008, -0.0097), gr_complex(-0.0129, -0.0002), gr_complex(-0.0003, -0.0054),
      gr_complex( 0.0086, -0.0256), gr_complex( 0.0285, -0.0079), gr_complex( 0.0262,  0.0063),
      gr_complex(-0.0029,  0.0076), gr_complex( 0.0045,  0.0081), gr_complex( 0.0050, -0.0155),
      gr_complex(-0.0008, -0.0144), gr_complex( 0.0179,  0.0127), gr_complex( 0.0155,  0.0098),
      gr_complex(-0.0062,  0.0095)
    };

    const gr_complex pilotgenerator_cc_impl::miso_coefficients_256_4_4[256] = {
      gr_complex(-0.0128,  0.0022), gr_complex(-0.0144,  0.0046), gr_complex( 0.0102, -0.0130),
      gr_complex( 0.0048, -0.0072), gr_complex( 0.0065, -0.0035), gr_complex( 0.0098,  0.0061),
      gr_complex(-0.0074,  0.0324), gr_complex(-0.0179, -0.0038), gr_complex(-0.0142, -0.0178),
      gr_complex( 0.0133,  0.0090), gr_complex( 0.0004, -0.0078), gr_complex(-0.0136, -0.0148),
      gr_complex( 0.0230, -0.0002), gr_complex(-0.0029,  0.0075), gr_complex(-0.0186, -0.0010),
      gr_complex(-0.0057, -0.0272), gr_complex( 0.0203,  0.0036), gr_complex(-0.0039,  0.0237),
      gr_complex(-0.0068,  0.0133), gr_complex( 0.0095, -0.0003), gr_complex( 0.0188,  0.0256),
      gr_complex( 0.0138,  0.0215), gr_complex( 0.0018, -0.0024), gr_complex( 0.0203, -0.0007),
      gr_complex( 0.0360,  0.0254), gr_complex( 0.0037,  0.0131), gr_complex(-0.0168,  0.0072),
      gr_complex( 0.0255, -0.0002), gr_complex(-0.0028, -0.0044), gr_complex(-0.0355, -0.0171),
      gr_complex(-0.0141, -0.0232), gr_complex( 0.0025,  0.0042), gr_complex(-0.0118,  0.0155),
      gr_complex(-0.0467, -0.0381), gr_complex( 0.0085,  0.0129), gr_complex( 0.0118,  0.0267),
      gr_complex(-0.0522,  0.0478), gr_complex(-0.0066,  0.0363), gr_complex(-0.0050, -0.0138),
      gr_complex(-0.0168,  0.0023), gr_complex( 0.0033, -0.0110), gr_complex( 0.0333, -0.0079),
      gr_complex( 0.0402,  0.0284), gr_complex( 0.0216,  0.0573), gr_complex(-0.0060, -0.0064),
      gr_complex(-0.0233,  0.0283), gr_complex(-0.0079,  0.0045), gr_complex(-0.0055, -0.0450),
      gr_complex( 0.0126, -0.0068), gr_complex(-0.0211,  0.0305), gr_complex(-0.0040,  0.0159),
      gr_complex(-0.0390, -0.0801), gr_complex(-0.0363,  0.0340), gr_complex( 0.0000,  0.0104),
      gr_complex(-0.0431, -0.0395), gr_complex( 0.0296,  0.0585), gr_complex( 0.0005,  0.0314),
      gr_complex(-0.0879,  0.0294), gr_complex( 0.0353, -0.0193), gr_complex( 0.0246,  0.0276),
      gr_complex( 0.0653,  0.0968), gr_complex( 0.0260, -0.0060), gr_complex( 0.0323, -0.0155),
      gr_complex( 0.0557,  0.0025), gr_complex( 0.0015, -0.0066), gr_complex(-0.0344,  0.0180),
      gr_complex(-0.0183,  0.0599), gr_complex(-0.0084, -0.0042), gr_complex(-0.0711, -0.0018),
      gr_complex(-0.0218, -0.0898), gr_complex(-0.0171, -0.0459), gr_complex( 0.0344,  0.0802),
      gr_complex(-0.0760,  0.0634), gr_complex(-0.0346,  0.0226), gr_complex(-0.0124,  0.0585),
      gr_complex(-0.0432,  0.0644), gr_complex(-0.0762, -0.0508), gr_complex(-0.0288, -0.0024),
      gr_complex( 0.0972,  0.0269), gr_complex( 0.0025, -0.0101), gr_complex( 0.0278,  0.0907),
      gr_complex( 0.0737, -0.0002), gr_complex(-0.0744,  0.0119), gr_complex(-0.0087,  0.0293),
      gr_complex( 0.0091, -0.0154), gr_complex(-0.0211,  0.0470), gr_complex(-0.0259,  0.0374),
      gr_complex(-0.0365,  0.0115), gr_complex(-0.1417, -0.0547), gr_complex(-0.0717,  0.0396),
      gr_complex( 0.0187,  0.0036), gr_complex(-0.0888,  0.0405), gr_complex(-0.0574,  0.0049),
      gr_complex(-0.0518, -0.0061), gr_complex( 0.0202, -0.0141), gr_complex(-0.0533, -0.0075),
      gr_complex( 0.0206, -0.0127), gr_complex( 0.0621, -0.1175), gr_complex(-0.1313, -0.0155),
      gr_complex(-0.0265,  0.0178), gr_complex(-0.0194,  0.0200), gr_complex( 0.0274, -0.0186),
      gr_complex( 0.0177,  0.0390), gr_complex(-0.0234,  0.0317), gr_complex( 0.0089, -0.0008),
      gr_complex(-0.0606, -0.0170), gr_complex(-0.0566,  0.0596), gr_complex( 0.0872, -0.1098),
      gr_complex( 0.0477, -0.0136), gr_complex( 0.0887,  0.0408), gr_complex( 0.1657, -0.0408),
      gr_complex(-0.0307, -0.0767), gr_complex(-0.0776, -0.0526), gr_complex(-0.0856, -0.0313),
      gr_complex(-0.0014,  0.0291), gr_complex( 0.0812,  0.0323), gr_complex(-0.0846, -0.1057),
      gr_complex(-0.0551, -0.0410), gr_complex(-0.0377,  0.0158), gr_complex( 0.1031,  0.0823),
      gr_complex(-0.0074,  0.0167), gr_complex( 0.0298,  0.0468), gr_complex(-0.0603, -0.0793),
      gr_complex( 0.1176,  0.0203), gr_complex( 0.0036,  0.0357), gr_complex(-0.0224,  0.0801),
      gr_complex(-0.0650,  0.0651), gr_complex(-0.0464,  0.0179), gr_complex(-0.0845,  0.1009),
      gr_complex( 0.0186,  0.0074), gr_complex(-0.0049,  0.0505), gr_complex(-0.0724,  0.0740),
      gr_complex(-0.0311, -0.0019), gr_complex( 0.0256,  0.0078), gr_complex( 0.1163, -0.0532),
      gr_complex(-0.0193, -0.0867), gr_complex( 0.1069,  0.0163), gr_complex( 0.0417, -0.0084),
      gr_complex(-0.1473, -0.0163), gr_complex( 0.0471,  0.0245), gr_complex(-0.0345,  0.0275),
      gr_complex( 0.0811, -0.0023), gr_complex( 0.0889, -0.0048), gr_complex( 0.0584,  0.0125),
      gr_complex(-0.0197, -0.0390), gr_complex( 0.0107, -0.0147), gr_complex(-0.0554, -0.0271),
      gr_complex(-0.0593, -0.0340), gr_complex(-0.0785, -0.0083), gr_complex( 0.0780, -0.0940),
      gr_complex( 0.0223,  0.1641), gr_complex( 0.1162, -0.0177), gr_complex( 0.0019,  0.0079),
      gr_complex( 0.0028, -0.0016), gr_complex(-0.0185,  0.0874), gr_complex(-0.0081, -0.0029),
      gr_complex(-0.1267, -0.0138), gr_complex( 0.0145,  0.0262), gr_complex(-0.0726,  0.0598),
      gr_complex( 0.0454,  0.0474), gr_complex(-0.0273,  0.0298), gr_complex(-0.0009, -0.0804),
      gr_complex(-0.0094,  0.0020), gr_complex(-0.0091,  0.0600), gr_complex(-0.0389, -0.0208),
      gr_complex(-0.0103, -0.0089), gr_complex(-0.0351,  0.0082), gr_complex( 0.0990, -0.0370),
      gr_complex( 0.0110,  0.0131), gr_complex(-0.1548, -0.0949), gr_complex( 0.1390,  0.0395),
      gr_complex( 0.0637,  0.0974), gr_complex( 0.1185, -0.0294), gr_complex(-0.0497, -0.0050),
      gr_complex( 0.0020,  0.0127), gr_complex( 0.0349,  0.0165), gr_complex( 0.0555, -0.0116),
      gr_complex(-0.0109,  0.0746), gr_complex(-0.0719,  0.0407), gr_complex( 0.0912, -0.0295),
      gr_complex(-0.1047, -0.0247), gr_complex( 0.0595, -0.0057), gr_complex( 0.0540, -0.0299),
      gr_complex(-0.0337, -0.0118), gr_complex( 0.0949, -0.0806), gr_complex(-0.0282,  0.0736),
      gr_complex( 0.0260, -0.0386), gr_complex( 0.0868,  0.0786), gr_complex(-0.0041, -0.0161),
      gr_complex(-0.0439, -0.0327), gr_complex(-0.0150, -0.1164), gr_complex(-0.0010,  0.0316),
      gr_complex(-0.0402, -0.0263), gr_complex( 0.0195, -0.0416), gr_complex( 0.0462, -0.0085),
      gr_complex(-0.0013,  0.0083), gr_complex( 0.0473, -0.0001), gr_complex(-0.0198, -0.0593),
      gr_complex( 0.0399,  0.0305), gr_complex(-0.0264, -0.0572), gr_complex( 0.0387, -0.0018),
      gr_complex(-0.0857,  0.0098), gr_complex(-0.0581, -0.0174), gr_complex( 0.0857, -0.0301),
      gr_complex(-0.0047,  0.0318), gr_complex(-0.0071, -0.0411), gr_complex( 0.0157, -0.0136),
      gr_complex(-0.0672, -0.0296), gr_complex( 0.0063,  0.0325), gr_complex( 0.0022,  0.0659),
      gr_complex(-0.0320, -0.0227), gr_complex(-0.0199, -0.0471), gr_complex(-0.0331,  0.0026),
      gr_complex(-0.0280, -0.0222), gr_complex(-0.0103, -0.0005), gr_complex(-0.0375, -0.0327),
      gr_complex(-0.0207, -0.0002), gr_complex( 0.0006, -0.0300), gr_complex( 0.0313, -0.0094),
      gr_complex(-0.0429,  0.0186), gr_complex( 0.0275,  0.0011), gr_complex( 0.0068,  0.0368),
      gr_complex(-0.0257, -0.0128), gr_complex(-0.0148,  0.0089), gr_complex(-0.0331,  0.0263),
      gr_complex(-0.0043,  0.0231), gr_complex( 0.0055,  0.0115), gr_complex( 0.0199, -0.0391),
      gr_complex(-0.0201,  0.0412), gr_complex(-0.0018,  0.0373), gr_complex(-0.0171,  0.0203),
      gr_complex( 0.0143, -0.0111), gr_complex(-0.0017,  0.0153), gr_complex( 0.0006,  0.0331),
      gr_complex(-0.0093,  0.0003), gr_complex(-0.0059, -0.0063), gr_complex(-0.0174,  0.0213),
      gr_complex( 0.0157,  0.0087), gr_complex( 0.0032,  0.0060), gr_complex( 0.0102, -0.0135),
      gr_complex(-0.0061,  0.0076), gr_complex( 0.0368,  0.0192), gr_complex( 0.0034,  0.0037),
      gr_complex(-0.0199, -0.0199), gr_complex(-0.0065, -0.0138), gr_complex( 0.0144,  0.0109),
      gr_complex(-0.0062,  0.0088), gr_complex(-0.0054,  0.0072), gr_complex( 0.0274, -0.0110),
      gr_complex( 0.0147, -0.0075), gr_complex( 0.0022, -0.0041), gr_complex( 0.0092, -0.0105),
      gr_complex(-0.0046,  0.0081), gr_complex( 0.0018, -0.0135), gr_complex( 0.0255,  0.0003),
      gr_complex(-0.0089,  0.0243)
    };

  } /* namespace atsc3 */
} /* namespace gr */
