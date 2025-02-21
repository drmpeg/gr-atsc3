/* -*- c++ -*- */
/*
 * Copyright 2022 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "ldmcombiner_cc_impl.h"
#include <volk/volk.h>

#define num_items 5400

namespace gr {
  namespace atsc3 {

    using input_type = gr_complex;
    using output_type = gr_complex;
    ldmcombiner_cc::sptr
    ldmcombiner_cc::make(atsc3_ldm_injection_level_t level)
    {
      return gnuradio::make_block_sptr<ldmcombiner_cc_impl>(
        level);
    }


    /*
     * The private constructor
     */
    ldmcombiner_cc_impl::ldmcombiner_cc_impl(atsc3_ldm_injection_level_t level)
      : gr::sync_block("ldmcombiner_cc",
              gr::io_signature::make(2, 2, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      float alpha, beta;
      switch (level) {
        case LDM_LEVEL_00DB:
          alpha = 1.0000000;
          beta = 0.7071068;
          break;
        case LDM_LEVEL_05DB:
          alpha = 0.9440609;
          beta = 0.7271524;
          break;
        case LDM_LEVEL_10DB:
          alpha = 0.8912509;
          beta = 0.7465331;
          break;
        case LDM_LEVEL_15DB:
          alpha = 0.8413951;
          beta = 0.7651789;
          break;
        case LDM_LEVEL_20DB:
          alpha = 0.7943282;
          beta = 0.7830305;
          break;
        case LDM_LEVEL_25DB:
          alpha = 0.7498942;
          beta = 0.8000406;
          break;
        case LDM_LEVEL_30DB:
          alpha = 0.7079458;
          beta = 0.8161736;
          break;
        case LDM_LEVEL_35DB:
          alpha = 0.6683439;
          beta = 0.8314061;
          break;
        case LDM_LEVEL_40DB:
          alpha = 0.6309573;
          beta = 0.8457262;
          break;
        case LDM_LEVEL_45DB:
          alpha = 0.5956621;
          beta = 0.8591327;
          break;
        case LDM_LEVEL_50DB:
          alpha = 0.5623413;
          beta = 0.8716346;
          break;
        case LDM_LEVEL_60DB:
          alpha = 0.5011872;
          beta = 0.8940022;
          break;
        case LDM_LEVEL_70DB:
          alpha = 0.4466836;
          beta = 0.9130512;
          break;
        case LDM_LEVEL_80DB:
          alpha = 0.3981072;
          beta = 0.9290819;
          break;
        case LDM_LEVEL_90DB:
          alpha = 0.3548134;
          beta = 0.9424353;
          break;
        case LDM_LEVEL_100DB:
          alpha = 0.3162278;
          beta = 0.9534626;
          break;
        case LDM_LEVEL_110DB:
          alpha = 0.2818383;
          beta = 0.9625032;
          break;
        case LDM_LEVEL_120DB:
          alpha = 0.2511886;
          beta = 0.9698706;
          break;
        case LDM_LEVEL_130DB:
          alpha = 0.2238721;
          beta = 0.9758449;
          break;
        case LDM_LEVEL_140DB:
          alpha = 0.1995262;
          beta = 0.9806699;
          break;
        case LDM_LEVEL_150DB:
          alpha = 0.1778279;
          beta = 0.9845540;
          break;
        case LDM_LEVEL_160DB:
          alpha = 0.1584893;
          beta = 0.9876723;
          break;
        case LDM_LEVEL_170DB:
          alpha = 0.1412538;
          beta = 0.9901705;
          break;
        case LDM_LEVEL_180DB:
          alpha = 0.1258925;
          beta = 0.9921685;
          break;
        case LDM_LEVEL_190DB:
          alpha = 0.1122018;
          beta = 0.9937642;
          break;
        case LDM_LEVEL_200DB:
          alpha = 0.1000000;
          beta = 0.9950372;
          break;
        case LDM_LEVEL_210DB:
          alpha = 0.0891251;
          beta = 0.9960519;
          break;
        case LDM_LEVEL_220DB:
          alpha = 0.0794328;
          beta = 0.9968601;
          break;
        case LDM_LEVEL_230DB:
          alpha = 0.0707946;
          beta = 0.9975034;
          break;
        case LDM_LEVEL_240DB:
          alpha = 0.0630957;
          beta = 0.9980154;
          break;
        case LDM_LEVEL_250DB:
          alpha = 0.0562341;
          beta = 0.9984226;
          break;
        default:
          alpha = 1.0000000;
          beta = 0.7071068;
          break;
      }
      for (int i = 0; i < num_items; i++) {
        alpha_vector[i] = alpha;
        beta_vector[i] = beta;
      }
      set_tag_propagation_policy(TPP_DONT);
      set_output_multiple(num_items);
      set_max_noutput_items(num_items);
    }

    /*
     * Our virtual destructor.
     */
    ldmcombiner_cc_impl::~ldmcombiner_cc_impl()
    {
    }

    int
    ldmcombiner_cc_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      auto in0 = static_cast<const input_type*>(input_items[0]);
      auto in1 = static_cast<const input_type*>(input_items[1]);
      auto out = static_cast<output_type*>(output_items[0]);

      std::vector<tag_t> tags;
      uint64_t nread;
      uint64_t tagoffset;
      uint64_t tagvalue;

      nread = this->nitems_read(0); //number of items read on port 0
      // Read all tags on the input buffer
      this->get_tags_in_range(tags, 0, nread, nread + noutput_items, pmt::string_to_symbol("lls"));
      if ((int)tags.size()) {
        tagoffset = this->nitems_written(0);
        tagvalue = pmt::to_uint64(tags[0].value);
        pmt::pmt_t key = pmt::string_to_symbol("llscore");
        pmt::pmt_t value = pmt::from_uint64(tagvalue);
        this->add_item_tag(0, tagoffset, key, value);
      }
      nread = this->nitems_read(1); //number of items read on port 1
      // Read all tags on the input buffer
      this->get_tags_in_range(tags, 1, nread, nread + noutput_items, pmt::string_to_symbol("lls"));
      if ((int)tags.size()) {
        tagoffset = this->nitems_written(0);
        tagvalue = pmt::to_uint64(tags[0].value);
        pmt::pmt_t key = pmt::string_to_symbol("llsenh");
        pmt::pmt_t value = pmt::from_uint64(tagvalue);
        this->add_item_tag(0, tagoffset, key, value);
      }

      volk_32fc_32f_multiply_32fc(&out[0], &in1[0], &alpha_vector[0], noutput_items);
      volk_32fc_x2_add_32fc(&out[0], &out[0], &in0[0], noutput_items);
      volk_32fc_32f_multiply_32fc(&out[0], &out[0], &beta_vector[0], noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace atsc3 */
} /* namespace gr */
