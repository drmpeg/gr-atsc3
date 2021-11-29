/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "modulator_bc_impl.h"

namespace gr {
  namespace atsc3 {

    using input_type = unsigned short;
    using output_type = gr_complex;
    modulator_bc::sptr
    modulator_bc::make(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation)
    {
      return gnuradio::make_block_sptr<modulator_bc_impl>(
        framesize, rate, constellation);
    }


    /*
     * The private constructor
     */
    modulator_bc_impl::modulator_bc_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_constellation_t constellation)
      : gr::block("modulator_bc",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      double normalization;
      int rateindex, i, j;
      if (framesize == FECFRAME_NORMAL) {
        switch (constellation) {
          case MOD_QPSK:
            cell_size = 32400;
            break;
          case MOD_16QAM:
            cell_size = 16200;
            break;
          case MOD_64QAM:
            cell_size = 10800;
            break;
          case MOD_256QAM:
            cell_size = 8100;
            break;
          case MOD_1024QAM:
            cell_size = 6480;
            break;
          case MOD_4096QAM:
            cell_size = 5400;
            break;
          default:
            cell_size = 0;
            break;
        }
      }
      else {
        switch (constellation) {
          case MOD_QPSK:
            cell_size = 8100;
            break;
          case MOD_16QAM:
            cell_size = 4050;
            break;
          case MOD_64QAM:
            cell_size = 2700;
            break;
          case MOD_256QAM:
            cell_size = 2025;
            break;
          default:
            cell_size = 0;
            break;
        }
      }
      switch (rate) {
        case C2_15:
          rateindex = 0;
          break;
        case C3_15:
          rateindex = 1;
          break;
        case C4_15:
          rateindex = 2;
          break;
        case C5_15:
          rateindex = 3;
          break;
        case C6_15:
          rateindex = 4;
          break;
        case C7_15:
          rateindex = 5;
          break;
        case C8_15:
          rateindex = 6;
          break;
        case C9_15:
          rateindex = 7;
          break;
        case C10_15:
          rateindex = 8;
          break;
        case C11_15:
          rateindex = 9;
          break;
        case C12_15:
          rateindex = 10;
          break;
        case C13_15:
          rateindex = 11;
          break;
        default:
          rateindex = 0;
          break;
      }
      m_1024qam = &mod_table_1024QAM[rateindex][0];
      m_4096qam = &mod_table_4096QAM[rateindex][0];
      switch (constellation) {
        case MOD_QPSK:
          normalization = std::sqrt(2.0);
          m_qpsk[0] = gr_complex( 1.0 / normalization,  1.0 / normalization);
          m_qpsk[1] = gr_complex(-1.0 / normalization,  1.0 / normalization);
          m_qpsk[2] = gr_complex( 1.0 / normalization, -1.0 / normalization);
          m_qpsk[3] = gr_complex(-1.0 / normalization, -1.0 / normalization);
          break;
        case MOD_16QAM:
          m_16qam[0] = mod_table_16QAM[rateindex][0];
          m_16qam[1] = mod_table_16QAM[rateindex][1];
          m_16qam[2] = mod_table_16QAM[rateindex][2];
          m_16qam[3] = mod_table_16QAM[rateindex][3];
          m_16qam[4] = -std::conj(mod_table_16QAM[rateindex][0]);
          m_16qam[5] = -std::conj(mod_table_16QAM[rateindex][1]);
          m_16qam[6] = -std::conj(mod_table_16QAM[rateindex][2]);
          m_16qam[7] = -std::conj(mod_table_16QAM[rateindex][3]);
          m_16qam[8] = std::conj(mod_table_16QAM[rateindex][0]);
          m_16qam[9] = std::conj(mod_table_16QAM[rateindex][1]);
          m_16qam[10] = std::conj(mod_table_16QAM[rateindex][2]);
          m_16qam[11] = std::conj(mod_table_16QAM[rateindex][3]);
          m_16qam[12] = -mod_table_16QAM[rateindex][0];
          m_16qam[13] = -mod_table_16QAM[rateindex][1];
          m_16qam[14] = -mod_table_16QAM[rateindex][2];
          m_16qam[15] = -mod_table_16QAM[rateindex][3];
          break;
        case MOD_64QAM:
          for (i = 0, j = 0; i < 16; i++, j++) {
            m_64qam[i] = mod_table_64QAM[rateindex][j];
          }
          for (i = 16, j = 0; i < 32; i++, j++) {
            m_64qam[i] = -std::conj(mod_table_64QAM[rateindex][j]);
          }
          for (i = 32, j = 0; i < 48; i++, j++) {
            m_64qam[i] = std::conj(mod_table_64QAM[rateindex][j]);
          }
          for (i = 48, j = 0; i < 64; i++, j++) {
            m_64qam[i] = -mod_table_64QAM[rateindex][j];
          }
          break;
        case MOD_256QAM:
          for (i = 0, j = 0; i < 64; i++, j++) {
            m_256qam[i] = mod_table_256QAM[rateindex][j];
          }
          for (i = 64, j = 0; i < 128; i++, j++) {
            m_256qam[i] = -std::conj(mod_table_256QAM[rateindex][j]);
          }
          for (i = 128, j = 0; i < 192; i++, j++) {
            m_256qam[i] = std::conj(mod_table_256QAM[rateindex][j]);
          }
          for (i = 192, j = 0; i < 256; i++, j++) {
            m_256qam[i] = -mod_table_256QAM[rateindex][j];
          }
          break;
        default:
          normalization = std::sqrt(2.0);
          m_qpsk[0] = gr_complex( 1.0 / normalization,  1.0 / normalization);
          m_qpsk[1] = gr_complex(-1.0 / normalization,  1.0 / normalization);
          m_qpsk[2] = gr_complex( 1.0 / normalization, -1.0 / normalization);
          m_qpsk[3] = gr_complex(-1.0 / normalization, -1.0 / normalization);
          break;
      }
      signal_constellation = constellation;
      set_output_multiple(cell_size);
    }

    /*
     * Our virtual destructor.
     */
    modulator_bc_impl::~modulator_bc_impl()
    {
    }

    void
    modulator_bc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int
    modulator_bc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      int index, indexodd, indexeven;

      switch (signal_constellation) {
        case MOD_QPSK:
          for (int i = 0; i < noutput_items; i += cell_size) {
            for (int j = 0; j < cell_size; j++) {
              index = *in++;
              *out++ = m_qpsk[index & 0x3];
            }
          }
          break;
        case MOD_16QAM:
          for (int i = 0; i < noutput_items; i += cell_size) {
            for (int j = 0; j < cell_size; j++) {
              index = *in++;
              *out++ = m_16qam[index & 0xf];
            }
          }
          break;
        case MOD_64QAM:
          for (int i = 0; i < noutput_items; i += cell_size) {
            for (int j = 0; j < cell_size; j++) {
              index = *in++;
              *out++ = m_64qam[index & 0x3f];
            }
          }
          break;
        case MOD_256QAM:
          for (int i = 0; i < noutput_items; i += cell_size) {
            for (int j = 0; j < cell_size; j++) {
              index = *in++;
              *out++ = m_256qam[index & 0xff];
            }
          }
          break;
        case MOD_1024QAM:
          for (int i = 0; i < noutput_items; i += cell_size) {
            for (int j = 0; j < cell_size; j++) {
              index = *in++;
              indexodd = 0;
              for (int n = 4; n >= 0; n--) {
                indexodd |= (index & (0x1 << (n * 2))) >> n;
              }
              indexeven = 0;
              for (int n = 4; n >= 0; n--) {
                indexeven |= (index & (0x1 << ((n * 2) + 1))) >> (n + 1);
              }
              index = indexeven >> 4;
              index |= (indexodd & 0x10) >> 3;
              switch (index) {
                case 0:
                  *out++ = gr_complex(m_1024qam[map_table_1024QAM[indexodd & 0x1f]], m_1024qam[map_table_1024QAM[indexeven & 0x1f]]);
                  break;
                case 1:
                  *out++ = gr_complex(m_1024qam[map_table_1024QAM[indexodd & 0x1f]], -m_1024qam[map_table_1024QAM[indexeven & 0x1f]]);
                  break;
                case 2:
                  *out++ = gr_complex(-m_1024qam[map_table_1024QAM[indexodd & 0x1f]], m_1024qam[map_table_1024QAM[indexeven & 0x1f]]);
                  break;
                case 3:
                  *out++ = gr_complex(-m_1024qam[map_table_1024QAM[indexodd & 0x1f]], -m_1024qam[map_table_1024QAM[indexeven & 0x1f]]);
                  break;
                default:
                  break;
              }
            }
          }
          break;
        case MOD_4096QAM:
          for (int i = 0; i < noutput_items; i += cell_size) {
            for (int j = 0; j < cell_size; j++) {
              index = *in++;
              indexodd = 0;
              for (int n = 5; n >= 0; n--) {
                indexodd |= (index & (0x1 << (n * 2))) >> n;
              }
              indexeven = 0;
              for (int n = 5; n >= 0; n--) {
                indexeven |= (index & (0x1 << ((n * 2) + 1))) >> (n + 1);
              }
              index = indexeven >> 5;
              index |= (indexodd & 0x20) >> 4;
              switch (index) {
                case 0:
                  *out++ = gr_complex(m_4096qam[map_table_4096QAM[indexodd & 0x3f]], m_4096qam[map_table_4096QAM[indexeven & 0x3f]]);
                  break;
                case 1:
                  *out++ = gr_complex(m_4096qam[map_table_4096QAM[indexodd & 0x3f]], -m_4096qam[map_table_4096QAM[indexeven & 0x3f]]);
                  break;
                case 2:
                  *out++ = gr_complex(-m_4096qam[map_table_4096QAM[indexodd & 0x3f]], m_4096qam[map_table_4096QAM[indexeven & 0x3f]]);
                  break;
                case 3:
                  *out++ = gr_complex(-m_4096qam[map_table_4096QAM[indexodd & 0x3f]], -m_4096qam[map_table_4096QAM[indexeven & 0x3f]]);
                  break;
                default:
                  break;
              }
            }
          }
          break;
      }

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    const gr_complex modulator_bc_impl::mod_table_16QAM[12][4] = {
      {gr_complex(0.7062, 0.7075), gr_complex(0.7075, 0.7062), gr_complex(0.7072, 0.7077), gr_complex(0.7077, 0.7072)},
      {gr_complex(0.3620, 0.5534), gr_complex(0.5534, 0.3620), gr_complex(0.5940, 1.1000), gr_complex(1.1000, 0.5940)},
      {gr_complex(0.3412, 0.5241), gr_complex(0.5241, 0.3412), gr_complex(0.5797, 1.1282), gr_complex(1.1282, 0.5797)},
      {gr_complex(0.3192, 0.5011), gr_complex(0.5011, 0.3192), gr_complex(0.5575, 1.1559), gr_complex(1.1559, 0.5575)},
      {gr_complex(0.5115, 1.2092), gr_complex(1.2092, 0.5115), gr_complex(0.2663, 0.4530), gr_complex(0.4530, 0.2663)},
      {gr_complex(0.2592, 0.4888), gr_complex(0.4888, 0.2592), gr_complex(0.5072, 1.1980), gr_complex(1.1980, 0.5072)},
      {gr_complex(0.2535, 0.4923), gr_complex(0.4923, 0.2535), gr_complex(0.4927, 1.2044), gr_complex(1.2044, 0.4927)},
      {gr_complex(0.2386, 0.5296), gr_complex(0.5296, 0.2386), gr_complex(0.4882, 1.1934), gr_complex(1.1934, 0.4882)},
      {gr_complex(0.4487, 1.1657), gr_complex(1.2080, 0.5377), gr_complex(0.2213, 0.4416), gr_complex(0.6186, 0.2544)},
      {gr_complex(0.9342, 0.9847), gr_complex(0.9866, 0.2903), gr_complex(0.2716, 0.9325), gr_complex(0.2901, 0.2695)},
      {gr_complex(0.9555, 0.9555), gr_complex(0.9555, 0.2949), gr_complex(0.2949, 0.9555), gr_complex(0.2949, 0.2949)},
      {gr_complex(0.9517, 0.9511), gr_complex(0.9524, 0.3061), gr_complex(0.3067, 0.9524), gr_complex(0.3061, 0.3067)}
    };

    const gr_complex modulator_bc_impl::mod_table_64QAM[12][16] = {
      {gr_complex(0.6474, 0.9831), gr_complex(0.6438, 0.9829), gr_complex(0.6471, 0.9767), gr_complex(0.6444, 0.9762),
       gr_complex(0.9839, 0.6475), gr_complex(0.9778, 0.6474), gr_complex(0.9835, 0.6434), gr_complex(0.9777, 0.6433),
       gr_complex(0.4659, 0.6393), gr_complex(0.4643, 0.6386), gr_complex(0.4661, 0.6353), gr_complex(0.4639, 0.6350),
       gr_complex(0.6378, 0.4671), gr_complex(0.6352, 0.4673), gr_complex(0.6385, 0.4656), gr_complex(0.6353, 0.4653)},
      {gr_complex(0.5472, 1.1591), gr_complex(0.5473, 1.1573), gr_complex(0.5467, 1.1599), gr_complex(0.5479, 1.1585),
       gr_complex(1.1578, 0.5478), gr_complex(1.1576, 0.5475), gr_complex(1.1591, 0.5475), gr_complex(1.1591, 0.5475),
       gr_complex(0.3163, 0.5072), gr_complex(0.3163, 0.5072), gr_complex(0.3163, 0.5072), gr_complex(0.3163, 0.5072),
       gr_complex(0.5087, 0.3163), gr_complex(0.5087, 0.3163), gr_complex(0.5087, 0.3163), gr_complex(0.5087, 0.3163)},
      {gr_complex(0.5008, 1.2136), gr_complex(0.4994, 1.2194), gr_complex(0.5313, 1.1715), gr_complex(0.5299, 1.1788),
       gr_complex(1.2107, 0.5037), gr_complex(1.2209, 0.5008), gr_complex(1.1715, 0.5299), gr_complex(1.1802, 0.5270),
       gr_complex(0.2744, 0.4762), gr_complex(0.2729, 0.4762), gr_complex(0.2773, 0.4791), gr_complex(0.2773, 0.4791),
       gr_complex(0.4762, 0.2729), gr_complex(0.4762, 0.2729), gr_complex(0.4791, 0.2773), gr_complex(0.4791, 0.2758)},
      {gr_complex(1.4327, 0.3305), gr_complex(1.0909, 0.2971), gr_complex(1.2484, 0.7803), gr_complex(0.9762, 0.5715),
       gr_complex(0.3309, 1.4326), gr_complex(0.2979, 1.0923), gr_complex(0.7829, 1.2477), gr_complex(0.5739, 0.9763),
       gr_complex(0.3901, 0.2112), gr_complex(0.5317, 0.2475), gr_complex(0.3945, 0.2289), gr_complex(0.5236, 0.2894),
       gr_complex(0.2108, 0.3911), gr_complex(0.2475, 0.5327), gr_complex(0.2287, 0.3955), gr_complex(0.2898, 0.5246)},
      {gr_complex(1.4521, 0.3005), gr_complex(1.2657, 0.8178), gr_complex(1.0666, 0.2744), gr_complex(0.9500, 0.5641),
       gr_complex(0.3011, 1.4529), gr_complex(0.8202, 1.2651), gr_complex(0.2750, 1.0676), gr_complex(0.5656, 0.9499),
       gr_complex(0.3553, 0.1948), gr_complex(0.3569, 0.2094), gr_complex(0.5596, 0.2431), gr_complex(0.5410, 0.3002),
       gr_complex(0.1946, 0.3566), gr_complex(0.2094, 0.3579), gr_complex(0.2430, 0.5607), gr_complex(0.3004, 0.5417)},
      {gr_complex(0.1567, 0.3112), gr_complex(0.1709, 0.3037), gr_complex(0.2093, 0.6562), gr_complex(0.3315, 0.6038),
       gr_complex(0.3112, 0.1567), gr_complex(0.3037, 0.1709), gr_complex(0.6562, 0.2093), gr_complex(0.6038, 0.3315),
       gr_complex(0.2959, 1.4877), gr_complex(0.8427, 1.2612), gr_complex(0.2389, 1.0228), gr_complex(0.5559, 0.8912),
       gr_complex(1.4877, 0.2959), gr_complex(1.2612, 0.8427), gr_complex(1.0228, 0.2389), gr_complex(0.8912, 0.5559)},
      {gr_complex(1.4827, 0.2920), gr_complex(1.2563, 0.8411), gr_complex(1.0211, 0.2174), gr_complex(0.8798, 0.5702),
       gr_complex(0.2920, 1.4827), gr_complex(0.8410, 1.2563), gr_complex(0.2174, 1.0211), gr_complex(0.5702, 0.8798),
       gr_complex(0.3040, 0.1475), gr_complex(0.3028, 0.1691), gr_complex(0.6855, 0.1871), gr_complex(0.6126, 0.3563),
       gr_complex(0.1475, 0.3040), gr_complex(0.1691, 0.3028), gr_complex(0.1871, 0.6855), gr_complex(0.3563, 0.6126)},
      {gr_complex(0.1305, 0.3311), gr_complex(0.1633, 0.3162), gr_complex(0.1622, 0.7113), gr_complex(0.3905, 0.6163),
       gr_complex(0.3311, 0.1305), gr_complex(0.3162, 0.1633), gr_complex(0.7113, 0.1622), gr_complex(0.6163, 0.3905),
       gr_complex(0.2909, 1.4626), gr_complex(0.8285, 1.2399), gr_complex(0.2062, 1.0367), gr_complex(0.5872, 0.8789),
       gr_complex(1.4626, 0.2909), gr_complex(1.2399, 0.8285), gr_complex(1.0367, 0.2062), gr_complex(0.8789, 0.5872)},
      {gr_complex(0.1177, 0.1729), gr_complex(0.1601, 0.3212), gr_complex(0.1352, 0.7279), gr_complex(0.3246, 0.6148),
       gr_complex(0.4192, 0.1179), gr_complex(0.4033, 0.2421), gr_complex(0.7524, 0.1581), gr_complex(0.5996, 0.4330),
       gr_complex(0.2902, 1.4611), gr_complex(0.8180, 1.2291), gr_complex(0.2036, 1.0575), gr_complex(0.5641, 0.8965),
       gr_complex(1.4453, 0.2907), gr_complex(1.2157, 0.8186), gr_complex(1.0447, 0.2242), gr_complex(0.8497, 0.6176)},
      {gr_complex(1.4443, 0.2683), gr_complex(0.7471, 1.2243), gr_complex(1.1749, 0.7734), gr_complex(0.7138, 0.8201),
       gr_complex(0.1638, 1.0769), gr_complex(0.2927, 1.4217), gr_complex(0.1462, 0.7457), gr_complex(0.4134, 0.7408),
       gr_complex(1.0203, 0.1517), gr_complex(0.6653, 0.1357), gr_complex(0.9639, 0.4465), gr_complex(0.6746, 0.4339),
       gr_complex(0.1271, 0.1428), gr_complex(0.3782, 0.1406), gr_complex(0.1311, 0.4288), gr_complex(0.3919, 0.4276)},
      {gr_complex(1.4480, 0.2403), gr_complex(0.6406, 1.1995), gr_complex(1.0952, 0.9115), gr_complex(0.6868, 0.8108),
       gr_complex(1.0500, 0.1642), gr_complex(0.7170, 0.1473), gr_complex(1.0519, 0.5188), gr_complex(0.7146, 0.4532),
       gr_complex(0.1677, 1.0405), gr_complex(0.2402, 1.4087), gr_complex(0.1369, 0.7073), gr_complex(0.4044, 0.7057),
       gr_complex(0.1374, 0.1295), gr_complex(0.4185, 0.1357), gr_complex(0.1325, 0.3998), gr_complex(0.4122, 0.4120)},
      {gr_complex(1.4319, 0.2300), gr_complex(1.0762, 0.9250), gr_complex(0.6290, 1.1820), gr_complex(0.6851, 0.8072),
       gr_complex(1.0443, 0.1688), gr_complex(1.0635, 0.5305), gr_complex(0.7220, 0.1540), gr_complex(0.7151, 0.4711),
       gr_complex(0.2099, 1.4205), gr_complex(0.1190, 0.6677), gr_complex(0.2031, 1.0551), gr_complex(0.3722, 0.7548),
       gr_complex(0.1438, 0.1287), gr_complex(0.1432, 0.3903), gr_complex(0.4298, 0.1384), gr_complex(0.4215, 0.4279)}
    };

    const gr_complex modulator_bc_impl::mod_table_256QAM[12][64] = {
      {gr_complex(0.5553, 1.1262), gr_complex(0.5673, 1.1336), gr_complex(0.5593, 1.1204), gr_complex(0.5636, 1.1321),
       gr_complex(0.5525, 1.1249), gr_complex(0.5637, 1.1320), gr_complex(0.5598, 1.1181), gr_complex(0.5659, 1.1274),
       gr_complex(0.5579, 1.1381), gr_complex(0.5617, 1.1471), gr_complex(0.5593, 1.1346), gr_complex(0.5672, 1.1430),
       gr_complex(0.5533, 1.1355), gr_complex(0.5632, 1.1421), gr_complex(0.5567, 1.1325), gr_complex(0.5641, 1.1363),
       gr_complex(1.1309, 0.5597), gr_complex(1.1405, 0.5660), gr_complex(1.1348, 0.5588), gr_complex(1.1491, 0.5638),
       gr_complex(1.1245, 0.5615), gr_complex(1.1333, 0.5627), gr_complex(1.1284, 0.5578), gr_complex(1.1436, 0.5636),
       gr_complex(1.1196, 0.5620), gr_complex(1.1347, 0.5665), gr_complex(1.1379, 0.5611), gr_complex(1.1440, 0.5638),
       gr_complex(1.1221, 0.5594), gr_complex(1.1318, 0.5686), gr_complex(1.1302, 0.5619), gr_complex(1.1386, 0.5662),
       gr_complex(0.3394, 0.5381), gr_complex(0.3397, 0.5360), gr_complex(0.3387, 0.5324), gr_complex(0.3400, 0.5335),
       gr_complex(0.3374, 0.5306), gr_complex(0.3405, 0.5343), gr_complex(0.3379, 0.5324), gr_complex(0.3400, 0.5317),
       gr_complex(0.3397, 0.5370), gr_complex(0.3400, 0.5383), gr_complex(0.3381, 0.5347), gr_complex(0.3382, 0.5347),
       gr_complex(0.3379, 0.5342), gr_complex(0.3389, 0.5332), gr_complex(0.3402, 0.5347), gr_complex(0.3384, 0.5340),
       gr_complex(0.5350, 0.3394), gr_complex(0.5363, 0.3397), gr_complex(0.5342, 0.3389), gr_complex(0.5384, 0.3380),
       gr_complex(0.5329, 0.3363), gr_complex(0.5330, 0.3387), gr_complex(0.5311, 0.3389), gr_complex(0.5332, 0.3380),
       gr_complex(0.5313, 0.3397), gr_complex(0.5324, 0.3400), gr_complex(0.5339, 0.3402), gr_complex(0.5360, 0.3405),
       gr_complex(0.5285, 0.3397), gr_complex(0.5317, 0.3379), gr_complex(0.5319, 0.3381), gr_complex(0.5327, 0.3395)},
      {gr_complex(0.5229, 1.1810), gr_complex(0.5384, 1.1625), gr_complex(0.5148, 1.1943), gr_complex(0.5288, 1.1751),
       gr_complex(0.4985, 1.2202), gr_complex(0.5111, 1.1973), gr_complex(0.4889, 1.2357), gr_complex(0.5045, 1.2113),
       gr_complex(0.5222, 1.1817), gr_complex(0.5370, 1.1640), gr_complex(0.5133, 1.1950), gr_complex(0.5303, 1.1751),
       gr_complex(0.4971, 1.2216), gr_complex(0.5126, 1.1995), gr_complex(0.4882, 1.2371), gr_complex(0.5045, 1.2128),
       gr_complex(1.1795, 0.5251), gr_complex(1.1625, 0.5384), gr_complex(1.1914, 0.5133), gr_complex(1.1744, 0.5296),
       gr_complex(1.2209, 0.4993), gr_complex(1.2002, 0.5148), gr_complex(1.2342, 0.4882), gr_complex(1.2142, 0.5052),
       gr_complex(1.1803, 0.5229), gr_complex(1.1640, 0.5399), gr_complex(1.1921, 0.5133), gr_complex(1.1758, 0.5303),
       gr_complex(1.2209, 0.4971), gr_complex(1.2024, 0.5148), gr_complex(1.2349, 0.4889), gr_complex(1.2150, 0.5045),
       gr_complex(0.2740, 0.4771), gr_complex(0.2762, 0.4801), gr_complex(0.2733, 0.4757), gr_complex(0.2748, 0.4779),
       gr_complex(0.2703, 0.4742), gr_complex(0.2725, 0.4764), gr_complex(0.2696, 0.4727), gr_complex(0.2718, 0.4749),
       gr_complex(0.2740, 0.4779), gr_complex(0.2755, 0.4793), gr_complex(0.2725, 0.4757), gr_complex(0.2748, 0.4779),
       gr_complex(0.2711, 0.4734), gr_complex(0.2725, 0.4764), gr_complex(0.2696, 0.4720), gr_complex(0.2711, 0.4742),
       gr_complex(0.4771, 0.2740), gr_complex(0.4786, 0.2762), gr_complex(0.4764, 0.2725), gr_complex(0.4771, 0.2748),
       gr_complex(0.4734, 0.2703), gr_complex(0.4757, 0.2725), gr_complex(0.4734, 0.2696), gr_complex(0.4742, 0.2711),
       gr_complex(0.4771, 0.2740), gr_complex(0.4779, 0.2762), gr_complex(0.4764, 0.2725), gr_complex(0.4771, 0.2748),
       gr_complex(0.4742, 0.2703), gr_complex(0.4749, 0.2725), gr_complex(0.4734, 0.2696), gr_complex(0.4749, 0.2711)},
      {gr_complex(0.2975, 1.0564), gr_complex(0.5862, 0.9617), gr_complex(0.2909, 1.0696), gr_complex(0.5796, 0.9689),
       gr_complex(0.2953, 1.3357), gr_complex(0.7488, 1.2365), gr_complex(0.3004, 1.5114), gr_complex(0.8151, 1.3816),
       gr_complex(0.3004, 1.0535), gr_complex(0.5847, 0.9631), gr_complex(0.2931, 1.0659), gr_complex(0.5825, 0.9668),
       gr_complex(0.2953, 1.3189), gr_complex(0.7466, 1.2168), gr_complex(0.2960, 1.4654), gr_complex(0.8297, 1.3539),
       gr_complex(1.0637, 0.2960), gr_complex(0.9617, 0.5811), gr_complex(1.0732, 0.2931), gr_complex(0.9682, 0.5818),
       gr_complex(1.3619, 0.2997), gr_complex(1.2249, 0.7546), gr_complex(1.5427, 0.3106), gr_complex(1.3969, 0.8523),
       gr_complex(1.0615, 0.2945), gr_complex(0.9631, 0.5818), gr_complex(1.0710, 0.2924), gr_complex(0.9675, 0.5825),
       gr_complex(1.3255, 0.2975), gr_complex(1.1979, 0.7495), gr_complex(1.4560, 0.3040), gr_complex(1.3269, 0.8414),
       gr_complex(0.2493, 0.5585), gr_complex(0.2960, 0.5344), gr_complex(0.2450, 0.5417), gr_complex(0.2873, 0.5191),
       gr_complex(0.2049, 0.3922), gr_complex(0.2173, 0.3806), gr_complex(0.1990, 0.3755), gr_complex(0.2107, 0.3645),
       gr_complex(0.2493, 0.5599), gr_complex(0.2975, 0.5351), gr_complex(0.2450, 0.5439), gr_complex(0.2887, 0.5213),
       gr_complex(0.2056, 0.3937), gr_complex(0.2187, 0.3820), gr_complex(0.1998, 0.3762), gr_complex(0.2122, 0.3667),
       gr_complex(0.5607, 0.2486), gr_complex(0.5381, 0.2960), gr_complex(0.5439, 0.2442), gr_complex(0.5220, 0.2865),
       gr_complex(0.3908, 0.2049), gr_complex(0.3813, 0.2173), gr_complex(0.3740, 0.1998), gr_complex(0.3653, 0.2100),
       gr_complex(0.5643, 0.2486), gr_complex(0.5410, 0.2967), gr_complex(0.5475, 0.2435), gr_complex(0.5257, 0.2880),
       gr_complex(0.3937, 0.2049), gr_complex(0.3850, 0.2187), gr_complex(0.3762, 0.1998), gr_complex(0.3689, 0.2114)},
      {gr_complex(0.1524, 0.3087), gr_complex(0.1525, 0.3087), gr_complex(0.1513, 0.3043), gr_complex(0.1513, 0.3043),
       gr_complex(0.1682, 0.3004), gr_complex(0.1682, 0.3005), gr_complex(0.1663, 0.2964), gr_complex(0.1663, 0.2964),
       gr_complex(0.1964, 0.6584), gr_complex(0.1965, 0.6583), gr_complex(0.1967, 0.6652), gr_complex(0.1968, 0.6652),
       gr_complex(0.3371, 0.5987), gr_complex(0.3370, 0.5987), gr_complex(0.3414, 0.6039), gr_complex(0.3413, 0.6039),
       gr_complex(0.3087, 0.1524), gr_complex(0.3087, 0.1525), gr_complex(0.3043, 0.1513), gr_complex(0.3043, 0.1513),
       gr_complex(0.3004, 0.1682), gr_complex(0.3005, 0.1682), gr_complex(0.2964, 0.1663), gr_complex(0.2964, 0.1663),
       gr_complex(0.6584, 0.1964), gr_complex(0.6583, 0.1965), gr_complex(0.6652, 0.1967), gr_complex(0.6652, 0.1968),
       gr_complex(0.5987, 0.3371), gr_complex(0.5987, 0.3370), gr_complex(0.6039, 0.3414), gr_complex(0.6039, 0.3413),
       gr_complex(0.3183, 1.5992), gr_complex(0.3186, 1.5991), gr_complex(0.2756, 1.3848), gr_complex(0.2759, 1.3847),
       gr_complex(0.9060, 1.3557), gr_complex(0.9058, 1.3559), gr_complex(0.7846, 1.1739), gr_complex(0.7843, 1.1741),
       gr_complex(0.2257, 0.9956), gr_complex(0.2259, 0.9956), gr_complex(0.2276, 1.0326), gr_complex(0.2278, 1.0326),
       gr_complex(0.5446, 0.8635), gr_complex(0.5445, 0.8636), gr_complex(0.5694, 0.8910), gr_complex(0.5692, 0.8911),
       gr_complex(1.5992, 0.3183), gr_complex(1.5991, 0.3186), gr_complex(1.3848, 0.2756), gr_complex(1.3847, 0.2759),
       gr_complex(1.3557, 0.9060), gr_complex(1.3559, 0.9058), gr_complex(1.1739, 0.7846), gr_complex(1.1741, 0.7843),
       gr_complex(0.9956, 0.2257), gr_complex(0.9956, 0.2259), gr_complex(1.0326, 0.2276), gr_complex(1.0326, 0.2278),
       gr_complex(0.8635, 0.5446), gr_complex(0.8636, 0.5445), gr_complex(0.8910, 0.5694), gr_complex(0.8911, 0.5692)},
      {gr_complex(0.1430, 0.3078), gr_complex(0.1430, 0.3077), gr_complex(0.1413, 0.3003), gr_complex(0.1414, 0.3002),
       gr_complex(0.1637, 0.2973), gr_complex(0.1636, 0.2973), gr_complex(0.1604, 0.2905), gr_complex(0.1603, 0.2905),
       gr_complex(0.1768, 0.6686), gr_complex(0.1793, 0.6679), gr_complex(0.1769, 0.6707), gr_complex(0.1793, 0.6700),
       gr_complex(0.3506, 0.5961), gr_complex(0.3484, 0.5974), gr_complex(0.3523, 0.5975), gr_complex(0.3501, 0.5987),
       gr_complex(0.3078, 0.1430), gr_complex(0.3077, 0.1430), gr_complex(0.3003, 0.1413), gr_complex(0.3002, 0.1414),
       gr_complex(0.2973, 0.1637), gr_complex(0.2973, 0.1636), gr_complex(0.2905, 0.1604), gr_complex(0.2905, 0.1603),
       gr_complex(0.6686, 0.1768), gr_complex(0.6679, 0.1793), gr_complex(0.6707, 0.1769), gr_complex(0.6700, 0.1793),
       gr_complex(0.5961, 0.3506), gr_complex(0.5974, 0.3484), gr_complex(0.5975, 0.3523), gr_complex(0.5987, 0.3501),
       gr_complex(0.2071, 1.6690), gr_complex(0.4482, 1.6210), gr_complex(0.2080, 1.3641), gr_complex(0.3307, 1.3397),
       gr_complex(1.0341, 1.3264), gr_complex(0.8297, 1.4630), gr_complex(0.8178, 1.1114), gr_complex(0.7138, 1.1809),
       gr_complex(0.1957, 0.9674), gr_complex(0.2170, 0.9629), gr_complex(0.1977, 1.0341), gr_complex(0.2288, 1.0277),
       gr_complex(0.5458, 0.8224), gr_complex(0.5276, 0.8342), gr_complex(0.5916, 0.8709), gr_complex(0.5651, 0.8883),
       gr_complex(1.6690, 0.2071), gr_complex(1.6210, 0.4482), gr_complex(1.3641, 0.2080), gr_complex(1.3397, 0.3307),
       gr_complex(1.3264, 1.0341), gr_complex(1.4630, 0.8297), gr_complex(1.1114, 0.8178), gr_complex(1.1809, 0.7138),
       gr_complex(0.9674, 0.1957), gr_complex(0.9629, 0.2170), gr_complex(1.0341, 0.1977), gr_complex(1.0277, 0.2288),
       gr_complex(0.8224, 0.5458), gr_complex(0.8342, 0.5276), gr_complex(0.8709, 0.5916), gr_complex(0.8883, 0.5651)},
      {gr_complex(0.1170, 0.3003), gr_complex(0.1171, 0.3003), gr_complex(0.1204, 0.3233), gr_complex(0.1204, 0.3233),
       gr_complex(0.1454, 0.2877), gr_complex(0.1453, 0.2877), gr_complex(0.1566, 0.3074), gr_complex(0.1565, 0.3074),
       gr_complex(0.1427, 0.6856), gr_complex(0.1562, 0.6826), gr_complex(0.1422, 0.6584), gr_complex(0.1529, 0.6560),
       gr_complex(0.3840, 0.5856), gr_complex(0.3723, 0.5931), gr_complex(0.3651, 0.5660), gr_complex(0.3559, 0.5718),
       gr_complex(0.3003, 0.1170), gr_complex(0.3003, 0.1171), gr_complex(0.3233, 0.1204), gr_complex(0.3233, 0.1204),
       gr_complex(0.2877, 0.1454), gr_complex(0.2877, 0.1453), gr_complex(0.3074, 0.1566), gr_complex(0.3074, 0.1565),
       gr_complex(0.6856, 0.1427), gr_complex(0.6826, 0.1562), gr_complex(0.6584, 0.1422), gr_complex(0.6560, 0.1529),
       gr_complex(0.5856, 0.3840), gr_complex(0.5931, 0.3723), gr_complex(0.5660, 0.3651), gr_complex(0.5718, 0.3559),
       gr_complex(0.1683, 1.7041), gr_complex(0.4972, 1.6386), gr_complex(0.1495, 1.3560), gr_complex(0.3814, 1.3099),
       gr_complex(1.0862, 1.3238), gr_complex(0.8074, 1.5101), gr_complex(0.8534, 1.0644), gr_complex(0.6568, 1.1958),
       gr_complex(0.1552, 0.9481), gr_complex(0.2200, 0.9352), gr_complex(0.1577, 1.0449), gr_complex(0.2548, 1.0255),
       gr_complex(0.5609, 0.7800), gr_complex(0.5060, 0.8167), gr_complex(0.6276, 0.8501), gr_complex(0.5452, 0.9052),
       gr_complex(1.7041, 0.1683), gr_complex(1.6386, 0.4972), gr_complex(1.3560, 0.1495), gr_complex(1.3099, 0.3814),
       gr_complex(1.3238, 1.0862), gr_complex(1.5101, 0.8074), gr_complex(1.0644, 0.8534), gr_complex(1.1958, 0.6568),
       gr_complex(0.9481, 0.1552), gr_complex(0.9352, 0.2200), gr_complex(1.0449, 0.1577), gr_complex(1.0255, 0.2548),
       gr_complex(0.7800, 0.5609), gr_complex(0.8167, 0.5060), gr_complex(0.8501, 0.6276), gr_complex(0.9052, 0.5452)},
      {gr_complex(0.0995, 0.2435), gr_complex(0.0996, 0.2434), gr_complex(0.1169, 0.3886), gr_complex(0.1179, 0.3883),
       gr_complex(0.1192, 0.2345), gr_complex(0.1192, 0.2345), gr_complex(0.1953, 0.3558), gr_complex(0.1944, 0.3563),
       gr_complex(0.1293, 0.7217), gr_complex(0.1616, 0.7151), gr_complex(0.1287, 0.6355), gr_complex(0.1456, 0.6318),
       gr_complex(0.4191, 0.6016), gr_complex(0.3916, 0.6198), gr_complex(0.3585, 0.5403), gr_complex(0.3439, 0.5497),
       gr_complex(0.2435, 0.0995), gr_complex(0.2434, 0.0996), gr_complex(0.3886, 0.1169), gr_complex(0.3883, 0.1179),
       gr_complex(0.2345, 0.1192), gr_complex(0.2345, 0.1192), gr_complex(0.3558, 0.1953), gr_complex(0.3563, 0.1944),
       gr_complex(0.7217, 0.1293), gr_complex(0.7151, 0.1616), gr_complex(0.6355, 0.1287), gr_complex(0.6318, 0.1456),
       gr_complex(0.6016, 0.4191), gr_complex(0.6198, 0.3916), gr_complex(0.5403, 0.3585), gr_complex(0.5497, 0.3439),
       gr_complex(0.1665, 1.6859), gr_complex(0.4919, 1.6211), gr_complex(0.1360, 1.3498), gr_complex(0.3914, 1.2989),
       gr_complex(1.0746, 1.3096), gr_complex(0.7987, 1.4940), gr_complex(0.8585, 1.0504), gr_complex(0.6419, 1.1951),
       gr_complex(0.1334, 0.9483), gr_complex(0.2402, 0.9271), gr_complex(0.1323, 1.0786), gr_complex(0.2910, 1.0470),
       gr_complex(0.5764, 0.7648), gr_complex(0.4860, 0.8252), gr_complex(0.6693, 0.8561), gr_complex(0.5348, 0.9459),
       gr_complex(1.6859, 0.1665), gr_complex(1.6211, 0.4919), gr_complex(1.3498, 0.1360), gr_complex(1.2989, 0.3914),
       gr_complex(1.3096, 1.0746), gr_complex(1.4940, 0.7987), gr_complex(1.0504, 0.8585), gr_complex(1.1951, 0.6419),
       gr_complex(0.9483, 0.1334), gr_complex(0.9271, 0.2402), gr_complex(1.0786, 0.1323), gr_complex(1.0470, 0.2910),
       gr_complex(0.7648, 0.5764), gr_complex(0.8252, 0.4860), gr_complex(0.8561, 0.6693), gr_complex(0.9459, 0.5348)},
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
      {gr_complex(0.0754, 0.2310), gr_complex(0.0768, 0.2305), gr_complex(0.0924, 0.4136), gr_complex(0.1043, 0.4125),
       gr_complex(0.0829, 0.1135), gr_complex(0.0836, 0.1149), gr_complex(0.2682, 0.3856), gr_complex(0.2531, 0.3906),
       gr_complex(0.0836, 0.7817), gr_complex(0.2052, 0.7608), gr_complex(0.0838, 0.6034), gr_complex(0.1394, 0.5961),
       gr_complex(0.4861, 0.6331), gr_complex(0.3661, 0.7034), gr_complex(0.3732, 0.5159), gr_complex(0.3095, 0.5511),
       gr_complex(0.3030, 0.0811), gr_complex(0.3017, 0.0853), gr_complex(0.4758, 0.0932), gr_complex(0.4676, 0.1242),
       gr_complex(0.2425, 0.1081), gr_complex(0.2447, 0.1115), gr_complex(0.3837, 0.2813), gr_complex(0.3959, 0.2642),
       gr_complex(0.7929, 0.0859), gr_complex(0.7652, 0.2324), gr_complex(0.6365, 0.0872), gr_complex(0.6207, 0.1757),
       gr_complex(0.6149, 0.5145), gr_complex(0.6987, 0.3934), gr_complex(0.5063, 0.4029), gr_complex(0.5526, 0.3356),
       gr_complex(0.1598, 1.6262), gr_complex(0.4733, 1.5637), gr_complex(0.1307, 1.3502), gr_complex(0.3877, 1.2983),
       gr_complex(1.0328, 1.2617), gr_complex(0.7675, 1.4398), gr_complex(0.8496, 1.0508), gr_complex(0.6297, 1.1967),
       gr_complex(0.0910, 0.9531), gr_complex(0.2649, 0.9198), gr_complex(0.1080, 1.1340), gr_complex(0.3214, 1.0926),
       gr_complex(0.5941, 0.7527), gr_complex(0.4371, 0.8528), gr_complex(0.7093, 0.8880), gr_complex(0.5235, 1.0090),
       gr_complex(1.6180, 0.1602), gr_complex(1.5540, 0.4734), gr_complex(1.3411, 0.1336), gr_complex(1.2883, 0.3955),
       gr_complex(1.2561, 1.0337), gr_complex(1.4311, 0.7676), gr_complex(1.0362, 0.8626), gr_complex(1.1845, 0.6419),
       gr_complex(0.9546, 0.0957), gr_complex(0.9163, 0.2834), gr_complex(1.1282, 0.1128), gr_complex(1.0838, 0.3340),
       gr_complex(0.7329, 0.6204), gr_complex(0.8428, 0.4615), gr_complex(0.8680, 0.7295), gr_complex(0.9959, 0.5426)},
      {gr_complex(0.0593, 0.2193), gr_complex(0.0690, 0.3047), gr_complex(0.0663, 0.4879), gr_complex(0.1151, 0.4474),
       gr_complex(0.1689, 0.2163), gr_complex(0.1971, 0.2525), gr_complex(0.3096, 0.3796), gr_complex(0.2489, 0.3933),
       gr_complex(0.0790, 0.7970), gr_complex(0.2340, 0.7710), gr_complex(0.0723, 0.6395), gr_complex(0.1896, 0.6163),
       gr_complex(0.5090, 0.6272), gr_complex(0.3787, 0.7126), gr_complex(0.4079, 0.5049), gr_complex(0.3088, 0.5677),
       gr_complex(0.0675, 0.0626), gr_complex(0.3475, 0.0595), gr_complex(0.5482, 0.0626), gr_complex(0.4784, 0.1124),
       gr_complex(0.1674, 0.0751), gr_complex(0.2856, 0.1132), gr_complex(0.4134, 0.3028), gr_complex(0.4235, 0.2289),
       gr_complex(0.8258, 0.0840), gr_complex(0.7936, 0.2483), gr_complex(0.6788, 0.0783), gr_complex(0.6501, 0.2025),
       gr_complex(0.6246, 0.5211), gr_complex(0.7241, 0.3961), gr_complex(0.5144, 0.4089), gr_complex(0.5918, 0.3146),
       gr_complex(0.1631, 1.5801), gr_complex(0.4806, 1.5133), gr_complex(0.1260, 1.3365), gr_complex(0.3750, 1.2897),
       gr_complex(1.0324, 1.2029), gr_complex(0.7737, 1.3837), gr_complex(0.8350, 1.0529), gr_complex(0.6147, 1.1949),
       gr_complex(0.0929, 0.9596), gr_complex(0.2768, 0.9260), gr_complex(0.1095, 1.1349), gr_complex(0.3250, 1.0941),
       gr_complex(0.6086, 0.7556), gr_complex(0.4514, 0.8566), gr_complex(0.7161, 0.8933), gr_complex(0.5294, 1.0121),
       gr_complex(1.5809, 0.1471), gr_complex(1.5253, 0.4385), gr_complex(1.3380, 0.1363), gr_complex(1.2837, 0.4026),
       gr_complex(1.2476, 0.9785), gr_complex(1.4137, 0.7196), gr_complex(1.0246, 0.8681), gr_complex(1.1771, 0.6494),
       gr_complex(0.9782, 0.0985), gr_complex(0.9383, 0.2922), gr_complex(1.1455, 0.1158), gr_complex(1.0972, 0.3418),
       gr_complex(0.7446, 0.6273), gr_complex(0.8573, 0.4721), gr_complex(0.8767, 0.7377), gr_complex(1.0059, 0.5518)},
      {gr_complex(1.1980, 1.1541), gr_complex(0.9192, 1.2082), gr_complex(1.2778, 0.8523), gr_complex(1.0390, 0.9253),
       gr_complex(0.6057, 1.2200), gr_complex(0.7371, 1.4217), gr_complex(0.6678, 1.0021), gr_complex(0.8412, 0.9448),
       gr_complex(1.2128, 0.5373), gr_complex(1.0048, 0.5165), gr_complex(1.4321, 0.6343), gr_complex(1.0245, 0.7152),
       gr_complex(0.6384, 0.6073), gr_complex(0.8175, 0.5684), gr_complex(0.6568, 0.7801), gr_complex(0.8311, 0.7459),
       gr_complex(0.1349, 1.4742), gr_complex(0.1105, 1.2309), gr_complex(0.0634, 0.9796), gr_complex(0.1891, 1.0198),
       gr_complex(0.4142, 1.4461), gr_complex(0.3323, 1.2279), gr_complex(0.4998, 0.9827), gr_complex(0.3467, 1.0202),
       gr_complex(0.0680, 0.6501), gr_complex(0.2016, 0.6464), gr_complex(0.0719, 0.8075), gr_complex(0.2088, 0.8146),
       gr_complex(0.4809, 0.6296), gr_complex(0.3374, 0.6412), gr_complex(0.4955, 0.8008), gr_complex(0.3431, 0.8141),
       gr_complex(1.2731, 0.1108), gr_complex(1.0794, 0.0977), gr_complex(1.5126, 0.1256), gr_complex(0.9029, 0.0853),
       gr_complex(0.5429, 0.0694), gr_complex(0.6795, 0.0559), gr_complex(0.5628, 0.1945), gr_complex(0.7326, 0.1410),
       gr_complex(1.2283, 0.3217), gr_complex(1.0269, 0.3261), gr_complex(1.4663, 0.3716), gr_complex(0.9085, 0.2470),
       gr_complex(0.6160, 0.4549), gr_complex(0.7818, 0.4247), gr_complex(0.5938, 0.3170), gr_complex(0.7600, 0.2850),
       gr_complex(0.0595, 0.0707), gr_complex(0.1722, 0.0706), gr_complex(0.0599, 0.2119), gr_complex(0.1748, 0.2114),
       gr_complex(0.4134, 0.0701), gr_complex(0.2935, 0.0705), gr_complex(0.4231, 0.2066), gr_complex(0.2979, 0.2100),
       gr_complex(0.0638, 0.5002), gr_complex(0.1905, 0.4966), gr_complex(0.0612, 0.3552), gr_complex(0.1810, 0.3533),
       gr_complex(0.4630, 0.4764), gr_complex(0.3231, 0.4895), gr_complex(0.4416, 0.3397), gr_complex(0.3083, 0.3490)},
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

    const int modulator_bc_impl::map_table_1024QAM[32] = {
      15, 14, 12, 13, 8, 9, 11, 10, 0, 1, 3, 2, 7, 6, 4, 5, 15, 14, 12, 13, 8, 9, 11, 10, 0, 1, 3, 2, 7, 6, 4, 5
    };

    const float modulator_bc_impl::mod_table_1024QAM[12][16] = {
      {0.3317, 0.3321, 0.3322, 0.3321, 0.3327, 0.3328, 0.3322, 0.3322, 0.9369, 0.9418, 0.9514, 0.9471, 0.9448, 0.9492, 0.9394, 0.9349},
      {0.2382, 0.2556, 0.2749, 0.2558, 0.2748, 0.2949, 0.2749, 0.2558, 0.9486, 0.8348, 0.7810, 0.8348, 0.9463, 0.8336, 0.9459, 1.4299},
      {0.1924, 0.1940, 0.2070, 0.2050, 0.3056, 0.3096, 0.2890, 0.2854, 0.7167, 0.7362, 0.7500, 0.7326, 0.9667, 0.9665, 1.1332, 1.4761},
      {0.1313, 0.1311, 0.1269, 0.1271, 0.3516, 0.3504, 0.3569, 0.3581, 0.6295, 0.6301, 0.6953, 0.6903, 0.9753, 1.0185, 1.2021, 1.4981},
      {0.1275, 0.1276, 0.1294, 0.1295, 0.3424, 0.3431, 0.3675, 0.3666, 0.6097, 0.6072, 0.7113, 0.7196, 0.9418, 1.0048, 1.2286, 1.5031},
      {0.0951, 0.0949, 0.1319, 0.1322, 0.3170, 0.3174, 0.3936, 0.3921, 0.5786, 0.5789, 0.7205, 0.7456, 0.9299, 1.0084, 1.2349, 1.5118},
      {0.0773, 0.0773, 0.1614, 0.1614, 0.3086, 0.3085, 0.4159, 0.4163, 0.5810, 0.5872, 0.7213, 0.7604, 0.9212, 1.0349, 1.2281, 1.4800},
      {0.0638, 0.0638, 0.1757, 0.1756, 0.3069, 0.3067, 0.4333, 0.4343, 0.5765, 0.5862, 0.7282, 0.7705, 0.9218, 1.0364, 1.2234, 1.4646},
      {0.0592, 0.0594, 0.1780, 0.1790, 0.2996, 0.3041, 0.4241, 0.4404, 0.5561, 0.6008, 0.7141, 0.8043, 0.9261, 1.0639, 1.2285, 1.4309},
      {0.0502, 0.0637, 0.1615, 0.1842, 0.2760, 0.3178, 0.4040, 0.4686, 0.5535, 0.6362, 0.7293, 0.8302, 0.9432, 1.0704, 1.2158, 1.3884},
      {0.0354, 0.0921, 0.1602, 0.2185, 0.2910, 0.3530, 0.4264, 0.4947, 0.5763, 0.6531, 0.7417, 0.8324, 0.9386, 1.0529, 1.1917, 1.3675},
      {0.0325, 0.0967, 0.1623, 0.2280, 0.2957, 0.3645, 0.4361, 0.5100, 0.5878, 0.6696, 0.7566, 0.8497, 0.9498, 1.0588, 1.1795, 1.3184}
    };

    const int modulator_bc_impl::map_table_4096QAM[64] = {
      31, 30, 28, 29, 24, 25, 27, 26, 16, 17, 19, 18, 23, 22, 20, 21, 0, 1, 3, 2, 7, 6, 4, 5, 15, 14, 12, 13, 8, 9, 11, 10,
      31, 30, 28, 29, 24, 25, 27, 26, 16, 17, 19, 18, 23, 22, 20, 21, 0, 1, 3, 2, 7, 6, 4, 5, 15, 14, 12, 13, 8, 9, 11, 10
    };

    const float modulator_bc_impl::mod_table_4096QAM[12][32] = {
      {0.2826, 0.2885, 0.2944, 0.2885, 0.2944, 0.3003, 0.2944, 0.2885, 0.2944, 0.3003, 0.3003, 0.3003, 0.2944, 0.3003, 0.2944, 0.2885,
       0.9714, 0.8596, 0.7889, 0.8478, 0.8242, 0.7771, 0.8360, 0.9184, 1.1657, 0.9479, 0.8419, 0.9302, 0.9950, 0.8713, 1.0185, 1.4660},
      {0.2038, 0.2038, 0.2155, 0.2155, 0.2155, 0.2155, 0.2097, 0.2038, 0.2796, 0.2912, 0.3029, 0.2970, 0.2970, 0.3029, 0.2796, 0.2796,
       0.7222, 0.7397, 0.7455, 0.7339, 0.7397, 0.7513, 0.7455, 0.7339, 1.3046, 1.0833, 0.9785, 1.0134, 0.9901, 0.9610, 1.0658, 1.6424},
      {0.1508, 0.1468, 0.1456, 0.1479, 0.1491, 0.1444, 0.1491, 0.1508, 0.3368, 0.3368, 0.3334, 0.3363, 0.3386, 0.3357, 0.3340, 0.3374,
       0.6448, 0.6569, 0.7101, 0.6979, 0.6974, 0.7124, 0.6575, 0.6465, 1.3549, 1.1931, 1.0117, 0.9857, 0.9689, 0.9967, 1.1683, 1.6391},
      {0.1257, 0.1257, 0.1257, 0.1257, 0.1257, 0.1257, 0.1257, 0.1257, 0.3599, 0.3599, 0.3484, 0.3484, 0.3484, 0.3484, 0.3599, 0.3599,
       0.6112, 0.6112, 0.6969, 0.7026, 0.6969, 0.6969, 0.6112, 0.6112, 1.4052, 1.2281, 1.0054, 0.9482, 0.9425, 0.9939, 1.1882, 1.6566},
      {0.1041, 0.1041, 0.1087, 0.1089, 0.1094, 0.1094, 0.1094, 0.1109, 0.3319, 0.3319, 0.3348, 0.3348, 0.3657, 0.3657, 0.3657, 0.3657,
       0.5875, 0.5876, 0.5876, 0.5877, 0.6648, 0.6651, 0.6968, 0.7018, 0.9102, 0.9102, 0.9780, 0.9842, 1.1892, 1.2411, 1.4707, 1.7274},
      {0.0810, 0.0808, 0.0807, 0.0810, 0.1456, 0.1457, 0.1456, 0.1456, 0.3059, 0.3060, 0.3056, 0.3056, 0.4043, 0.4042, 0.4036, 0.4036,
       0.5684, 0.5682, 0.5700, 0.5704, 0.7155, 0.7186, 0.7425, 0.7385, 0.9163, 0.9089, 0.9771, 1.0012, 1.1766, 1.2355, 1.4381, 1.6851},
      {0.0501, 0.0553, 0.0562, 0.0562, 0.1677, 0.1687, 0.1687, 0.1718, 0.2963, 0.2963, 0.2963, 0.2968, 0.4234, 0.4240, 0.4248, 0.4248,
       0.5584, 0.5590, 0.5679, 0.5729, 0.7078, 0.7090, 0.7610, 0.7640, 0.8966, 0.8979, 1.0135, 1.0393, 1.1817, 1.2459, 1.4232, 1.6336},
      {0.0415, 0.0478, 0.0592, 0.0592, 0.1656, 0.1663, 0.1663, 0.1663, 0.2861, 0.2863, 0.2877, 0.2877, 0.4144, 0.4178, 0.4204, 0.4204,
       0.5352, 0.5370, 0.5673, 0.5683, 0.6848, 0.6848, 0.7694, 0.7838, 0.8808, 0.9039, 1.0050, 1.0619, 1.1797, 1.2898, 1.4381, 1.6223},
      {0.0397, 0.0397, 0.0659, 0.0659, 0.1443, 0.1453, 0.1819, 0.1826, 0.2591, 0.2591, 0.3128, 0.3128, 0.3872, 0.3872, 0.4549, 0.4549,
       0.5290, 0.5302, 0.6069, 0.6081, 0.6911, 0.6969, 0.7787, 0.8012, 0.8802, 0.9248, 1.0037, 1.0861, 1.1870, 1.2894, 1.4122, 1.5629},
      {0.0253, 0.0285, 0.0844, 0.0848, 0.1460, 0.1460, 0.2078, 0.2078, 0.2708, 0.2708, 0.3360, 0.3360, 0.4051, 0.4052, 0.4742, 0.4742,
       0.5417, 0.5446, 0.6118, 0.6209, 0.6857, 0.7107, 0.7734, 0.8174, 0.8791, 0.9425, 1.0131, 1.0904, 1.1787, 1.2766, 1.3852, 1.5162},
      {0.0262, 0.0262, 0.0828, 0.0842, 0.1337, 0.1389, 0.1887, 0.2018, 0.2466, 0.2675, 0.3096, 0.3393, 0.3796, 0.4118, 0.4506, 0.4897,
       0.5296, 0.5712, 0.6136, 0.6586, 0.7060, 0.7544, 0.8043, 0.8624, 0.9152, 0.9718, 1.0325, 1.1017, 1.1756, 1.2541, 1.3405, 1.4431},
      {0.0176, 0.0487, 0.0781, 0.1080, 0.1399, 0.1713, 0.2053, 0.2378, 0.2720, 0.3076, 0.3412, 0.3754, 0.4156, 0.4522, 0.4893, 0.5260,
       0.5643, 0.6051, 0.6469, 0.6885, 0.7336, 0.7790, 0.8255, 0.8776, 0.9254, 0.9749, 1.0276, 1.0870, 1.1474, 1.2121, 1.2835, 1.3644}
    };

  } /* namespace atsc3 */
} /* namespace gr */
