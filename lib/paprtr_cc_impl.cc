/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "paprtr_cc_impl.h"
#include <gnuradio/math.h>
#include <volk/volk.h>
#include <algorithm>

/* An early exit from the iteration loop is a very effective optimization */
#define EARLY_EXIT

namespace gr {
  namespace atsc3 {

    using input_type = gr_complex;
    using output_type = gr_complex;
    paprtr_cc::sptr
    paprtr_cc::make(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_pilotpattern_t pilotpattern, atsc3_first_sbs_t firstsbs, atsc3_papr_t paprmode, atsc3_reduced_carriers_t cred, float vclip, int iterations, unsigned int vlength)
    {
      return gnuradio::make_block_sptr<paprtr_cc_impl>(
        fftsize, numpayloadsyms, numpreamblesyms, pilotpattern, firstsbs, paprmode, cred, vclip, iterations, vlength);
    }


    /*
     * The private constructor
     */
    paprtr_cc_impl::paprtr_cc_impl(atsc3_fftsize_t fftsize, int numpayloadsyms, int numpreamblesyms, atsc3_pilotpattern_t pilotpattern, atsc3_first_sbs_t firstsbs, atsc3_papr_t paprmode, atsc3_reduced_carriers_t cred, float vclip, int iterations, unsigned int vlength)
      : gr::sync_block("paprtr_cc",
              gr::io_signature::make(1, 1, sizeof(input_type) * vlength),
              gr::io_signature::make(1, 1, sizeof(output_type) * vlength)),
        papr_fft(vlength, 1),
        papr_fft_size(vlength),
        fft_size(fftsize),
        papr_mode(paprmode),
        v_clip(vclip),
        num_iterations(iterations),
        ones_freq(papr_fft_size),
        ones_time(papr_fft_size),
        c(papr_fft_size),
        ctemp(papr_fft_size),
        magnitude(papr_fft_size)
    {
      symbols = numpreamblesyms + numpayloadsyms;
      preamble_symbols = numpreamblesyms;
      switch (fftsize) {
        case FFTSIZE_8K:
          N_TR = 72;
          tr_papr_map = trpapr_table_8K;
          carriers = carriers_table[FFTSIZE_8K][cred];
          max_carriers = carriers_table[FFTSIZE_8K][0];
          preamble_carriers = carriers_table[FFTSIZE_8K][4];
          break;
        case FFTSIZE_16K:
          N_TR = 144;
          tr_papr_map = trpapr_table_16K;
          carriers = carriers_table[FFTSIZE_16K][cred];
          max_carriers = carriers_table[FFTSIZE_16K][0];
          preamble_carriers = carriers_table[FFTSIZE_16K][4];
          break;
        case FFTSIZE_32K:
          N_TR = 288;
          tr_papr_map = trpapr_table_32K;
          carriers = carriers_table[FFTSIZE_32K][cred];
          max_carriers = carriers_table[FFTSIZE_32K][0];
          preamble_carriers = carriers_table[FFTSIZE_32K][4];
          break;
        default:
          N_TR = 72;
          tr_papr_map = trpapr_table_8K;
          carriers = carriers_table[FFTSIZE_8K][cred];
          max_carriers = carriers_table[FFTSIZE_8K][0];
          preamble_carriers = carriers_table[FFTSIZE_8K][4];
          break;
      }
      switch (pilotpattern) {
        case PILOT_SP3_2:
          dx = 3;
          dy = 2;
          break;
        case PILOT_SP3_4:
          dx = 3;
          dy = 4;
          break;
        case PILOT_SP4_2:
          dx = 4;
          dy = 2;
          break;
        case PILOT_SP4_4:
          dx = 4;
          dy = 4;
          break;
        case PILOT_SP6_2:
          dx = 6;
          dy = 2;
          break;
        case PILOT_SP6_4:
          dx = 6;
          dy = 4;
          break;
        case PILOT_SP8_2:
          dx = 8;
          dy = 2;
          break;
        case PILOT_SP8_4:
          dx = 8;
          dy = 4;
          break;
        case PILOT_SP12_2:
          dx = 12;
          dy = 2;
          break;
        case PILOT_SP12_4:
          dx = 12;
          dy = 4;
          break;
        case PILOT_SP16_2:
          dx = 16;
          dy = 2;
          break;
        case PILOT_SP16_4:
          dx = 16;
          dy = 4;
          break;
        case PILOT_SP24_2:
          dx = 24;
          dy = 2;
          break;
        case PILOT_SP24_4:
          dx = 24;
          dy = 4;
          break;
        case PILOT_SP32_2:
          dx = 32;
          dy = 2;
          break;
        case PILOT_SP32_4:
          dx = 32;
          dy = 4;
          break;
        default:
          dx = 3;
          dy = 2;
          break;
      }
      frame_symbols[0] = PREAMBLE_SYMBOL;
      for (int n = 1; n < numpreamblesyms; n++) {
        frame_symbols[n] = PREAMBLE_SYMBOL;
      }
      if (firstsbs == TRUE) {
        frame_symbols[numpreamblesyms] = SBS_SYMBOL;
        for (int n = 0; n < numpayloadsyms; n++) {
          frame_symbols[n + numpreamblesyms + 1] = DATA_SYMBOL;
        }
      }
      else {
        for (int n = 0; n < numpayloadsyms; n++) {
          frame_symbols[n + numpreamblesyms] = DATA_SYMBOL;
        }
      }
      frame_symbols[numpreamblesyms + numpayloadsyms - 1] = SBS_SYMBOL;
      left_nulls = ((vlength - carriers) / 2) + 1;
      right_nulls = (vlength - carriers) / 2;
      r.resize(N_TR);
      rNew.resize(N_TR);
      v.resize(N_TR);
      set_output_multiple(symbols);
    }

    /*
     * Our virtual destructor.
     */
    paprtr_cc_impl::~paprtr_cc_impl()
    {
    }

    void
    paprtr_cc_impl::init_pilots(int symbol)
    {
      for (int i = 0; i < carriers; i++) {
        tr_carrier_map[i] = DATA_CARRIER;
      }
      shift = dx * ((symbol - (preamble_symbols - 2)) % dy);
      if (frame_symbols[symbol] == SBS_SYMBOL || frame_symbols[symbol] == PREAMBLE_SYMBOL) {
        shift = 0;
      }
      switch (fft_size) {
        case FFTSIZE_8K:
          for (int i = 0; i < 72; i++) {
            tr_carrier_map[trpapr_table_8K[i] + shift] = TRPAPR_CARRIER;
          }
          break;
        case FFTSIZE_16K:
          for (int i = 0; i < 144; i++) {
            tr_carrier_map[trpapr_table_16K[i] + shift] = TRPAPR_CARRIER;
          }
          break;
        case FFTSIZE_32K:
          for (int i = 0; i < 288; i++) {
            tr_carrier_map[trpapr_table_32K[i] + shift] = TRPAPR_CARRIER;
          }
          break;
      }
    }

    int
    paprtr_cc_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      const int* papr_map;
      const gr_complex one(1.0, 0.0);
      const gr_complex zero(0.0, 0.0);
      const float normalization = 1.0 / N_TR;
      const int center = (carriers - 1) / 2;
      const float aMax = 5.0 * N_TR * std::sqrt(10.0 / (27.0 * carriers));
      gr_complex* dst;
      int m = 0, index, valid;
      float y, a, alpha;
      gr_complex u, result;
      double vtemp;

      for (int i = 0; i < noutput_items; i += symbols) {
        if (papr_mode == PAPR_TR) {
          for (int j = 0; j < symbols; j++) {
            init_pilots(j);
            valid = FALSE;
            if (j != 0) {
              index = 0;
              std::fill_n(&ones_freq[index], left_nulls, 0);
              index = left_nulls;
              for (int n = 0; n < carriers; n++) {
                if (tr_carrier_map[n] == TRPAPR_CARRIER) {
                  ones_freq[index++] = one;
                } 
                else {
                  ones_freq[index++] = zero;
                }
              }
              std::fill_n(&ones_freq[index], right_nulls, 0);
              papr_map = tr_papr_map;
              valid = TRUE;
            }
            if (valid == TRUE) {
              dst = papr_fft.get_inbuf();
              memcpy(&dst[papr_fft_size / 2], &ones_freq[0], sizeof(gr_complex) * papr_fft_size / 2);
              memcpy(&dst[0], &ones_freq[papr_fft_size / 2], sizeof(gr_complex) * papr_fft_size / 2);
              papr_fft.execute();
              memcpy(ones_time.data(), papr_fft.get_outbuf(), sizeof(gr_complex) * papr_fft_size);
              volk_32fc_s32fc_multiply_32fc(ones_time.data(), ones_time.data(), normalization, papr_fft_size);
              std::fill_n(&r[0], N_TR, 0);
              std::fill_n(&c[0], papr_fft_size, 0);
              for (int k = 1; k <= num_iterations; k++) {
                y = 0.0;
                volk_32f_x2_add_32f((float*)ctemp.data(), (float*)in, (float*)c.data(), papr_fft_size * 2);
                volk_32fc_magnitude_32f( magnitude.data(), ctemp.data(), papr_fft_size);
                for (int n = 0; n < papr_fft_size; n++) {
                  if (magnitude[n] > y) {
                    y = magnitude[n];
                    m = n;
                  }
                }
#ifdef EARLY_EXIT
                if (y < v_clip + 0.01) {
                  break;
                }
#else
                if (y < v_clip) {
                  break;
                }
#endif
                u = (in[m] + c[m]) / y;
                alpha = y - v_clip;
                for (int n = 0; n < N_TR; n++) {
                  vtemp = (-2.0 * GR_M_PI * m * ((papr_map[n] + shift) - center)) / papr_fft_size;
                  ctemp[n] = std::exp(gr_complexd(0.0, vtemp));
                }
                volk_32fc_s32fc_multiply_32fc(v.data(), ctemp.data(), u, N_TR);
                volk_32f_s32f_multiply_32f((float*)rNew.data(), (float*)v.data(), alpha, N_TR * 2);
                volk_32f_x2_subtract_32f((float*)rNew.data(), (float*)r.data(), (float*)rNew.data(), N_TR * 2);
                volk_32fc_x2_multiply_conjugate_32fc(ctemp.data(), r.data(), v.data(), N_TR);
                for (int n = 0; n < N_TR; n++) {
                  alphaLimit[n] = std::sqrt((aMax * aMax) - (ctemp[n].imag() * ctemp[n].imag())) + ctemp[n].real();
                }
                index = 0;
                volk_32fc_magnitude_32f(magnitude.data(), rNew.data(), N_TR);
                for (int n = 0; n < N_TR; n++) {
                  if (magnitude[n] > aMax) {
                    alphaLimitMax[index++] = alphaLimit[n];
                  }
                }
                if (index != 0) {
                  a = 1.0e+30;
                  for (int n = 0; n < index; n++) {
                    if (alphaLimitMax[n] < a) {
                      a = alphaLimitMax[n];
                    }
                  }
                  alpha = a;
                  volk_32f_s32f_multiply_32f((float*)rNew.data(), (float*)v.data(), alpha, N_TR * 2);
                  volk_32f_x2_subtract_32f((float*)rNew.data(), (float*)r.data(), (float*)rNew.data(), N_TR * 2);
                }
                for (int n = 0; n < papr_fft_size; n++) {
                  ones_freq[(n + m) % papr_fft_size] = ones_time[n];
                }
                result = u * alpha;
                volk_32fc_s32fc_multiply_32fc(ctemp.data(), ones_freq.data(), result, papr_fft_size);
                volk_32f_x2_subtract_32f((float*)c.data(), (float*)c.data(), (float*)ctemp.data(), papr_fft_size * 2);
                std::copy(std::begin(rNew), std::end(rNew), std::begin(r));
              }
              volk_32f_x2_add_32f((float*)out, (float*)in, (float*)c.data(), papr_fft_size * 2);
              in = in + papr_fft_size;
              out = out + papr_fft_size;
            } else {
              memcpy(out, in, sizeof(gr_complex) * papr_fft_size);
              in = in + papr_fft_size;
              out = out + papr_fft_size;
            }
          }
        }
        else {
          for (int j = 0; j < symbols; j++) {
            memcpy(out, in, sizeof(gr_complex) * papr_fft_size);
            in = in + papr_fft_size;
            out = out + papr_fft_size;
          }
        }
      }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    const int paprtr_cc_impl::carriers_table[3][5] = {
      {6913, 6817, 6721, 6625, 6529},
      {13825, 13633, 13441, 13249, 13057},
      {27649, 27265, 26881, 26497, 26113}
    };

    const int paprtr_cc_impl::trpapr_table_8K[72] = {
      250, 386, 407, 550, 591, 717, 763, 787, 797, 839, 950, 1090, 1105, 1199, 1738, 1867,
      1903, 1997, 2114, 2260, 2356, 2427, 2428, 2444, 2452, 2475, 2564, 2649, 2663, 2678, 2740, 2777,
      2819, 2986, 3097, 3134, 3253, 3284, 3323, 3442, 3596, 3694, 3719, 3751, 3763, 3836, 4154, 4257,
      4355, 4580, 4587, 4678, 4805, 5084, 5126, 5161, 5229, 5321, 5445, 5649, 5741, 5746, 5885, 5918,
      6075, 6093, 6319, 6421, 6463, 6511, 6517, 6577
    };

    const int paprtr_cc_impl::trpapr_table_16K[144] = {
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

    const int paprtr_cc_impl::trpapr_table_32K[288] = {
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

    const int paprtr_cc_impl::trpapr_alt_table_8K[72] = {
      295, 329, 347, 365, 463, 473, 481, 553, 578, 602, 742, 749, 829, 922, 941, 1115,
      1123, 1174, 1363, 1394, 1402, 1615, 1657, 1702, 1898, 1910, 1997, 2399, 2506, 2522, 2687, 2735,
      3043, 3295, 3389, 3454, 3557, 3647, 3719, 3793, 3794, 3874, 3898, 3970, 4054, 4450, 4609, 4666,
      4829, 4855, 4879, 4961, 4969, 5171, 5182, 5242, 5393, 5545, 5567, 5618, 5630, 5734, 5861, 5897,
      5987, 5989, 6002, 6062, 6074, 6205, 6334, 6497
    };

    const int paprtr_cc_impl::trpapr_alt_table_16K[144] = {
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

    const int paprtr_cc_impl::trpapr_alt_table_32K[288] = {
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

  } /* namespace atsc3 */
} /* namespace gr */
