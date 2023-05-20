/* -*- c++ -*- */
/*
 * Copyright 2021 Ron Economos.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "alpbbheader_bb_impl.h"

#define NUM_PACKETS 4
#define LONG_EXTENSION_BYTES 2
#define LLS_HEADER_LENGTH 4
#define ALP_HEADER_LENGTH 2
#define MPEG_PKT_LENGTH 188

namespace gr {
  namespace atsc3 {

    using input_type = unsigned char;
    using output_type = unsigned char;
    alpbbheader_bb::sptr
    alpbbheader_bb::make(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_lls_insertion_mode_t llsmode)
    {
      return gnuradio::make_block_sptr<alpbbheader_bb_impl>(
        framesize, rate, llsmode);
    }


    /*
     * The private constructor
     */
    alpbbheader_bb_impl::alpbbheader_bb_impl(atsc3_framesize_t framesize, atsc3_code_rate_t rate, atsc3_lls_insertion_mode_t llsmode)
      : gr::block("alpbbheader_bb",
              gr::io_signature::make(1, 1, sizeof(input_type)),
              gr::io_signature::make(1, 1, sizeof(output_type)))
    {
      char lls[1024];
      int offset = 0;
      z_stream zs;
      char src_address[] = {"44.4.15.9"};
      char dst_address[] = {"224.0.23.60"};

      for (int i = 0; i < 1024; i++) {
        lls[i] = 0;
      }
      memcpy(&lls[offset], &xml[0], strlen(xml));
      offset += strlen(xml);
      memcpy(&lls[offset], &SLT[0], strlen(SLT));
      offset += strlen(SLT);
      memcpy(&lls[offset], &Service[0], strlen(Service));
      offset += strlen(Service);
      memcpy(&lls[offset], &BroadcastSvcSignaling[0], strlen(BroadcastSvcSignaling));
      offset += strlen(BroadcastSvcSignaling);
      memcpy(&lls[offset], &Service_end[0], strlen(Service_end));
      offset += strlen(Service_end);
      memcpy(&lls[offset], &SLT_end[0], strlen(SLT_end));
      offset += strlen(SLT_end);

      zs.zalloc = Z_NULL;
      zs.zfree = Z_NULL;
      zs.opaque = Z_NULL;
      zs.avail_in = (uInt)offset;
      zs.next_in = (Bytef *)lls;
      zs.avail_out = (uInt)1024;
      zs.next_out = (Bytef *)llsgz;

      deflateInit2(&zs, Z_DEFAULT_COMPRESSION, Z_DEFLATED, 15 | 16, 8, Z_DEFAULT_STRATEGY);
      deflate(&zs, Z_FINISH);
      deflateEnd(&zs);
      lls_length = zs.total_out +  sizeof(struct ip) + sizeof(struct udphdr) + LLS_HEADER_LENGTH;
      llsgz_length = zs.total_out;

      identification = 0;
      inet_pton(AF_INET, src_address, &src_addr);
      inet_pton(AF_INET, dst_address, &dst_addr);

      count = 0;
      bbcount = 0;
      packets = 0;
      total = 0;
      segments = 0;
      dnp = FALSE;
      trigger = FALSE;
      lls_send = FALSE;
      lls_count = 0;
      remainder = 0;
      lls_mode = llsmode;
      if (framesize == FECFRAME_NORMAL) {
        switch (rate) {
          case C2_15:
            kbch = 8448;
            break;
          case C3_15:
            kbch = 12768;
            break;
          case C4_15:
            kbch = 17088;
            break;
          case C5_15:
            kbch = 21408;
            break;
          case C6_15:
            kbch = 25728;
            break;
          case C7_15:
            kbch = 30048;
            break;
          case C8_15:
            kbch = 34368;
            break;
          case C9_15:
            kbch = 38688;
            break;
          case C10_15:
            kbch = 43008;
            break;
          case C11_15:
            kbch = 47328;
            break;
          case C12_15:
            kbch = 51648;
            break;
          case C13_15:
            kbch = 55968;
            break;
          default:
            kbch = 0;
            break;
        }
      }
      else if (framesize == FECFRAME_SHORT) {
        switch (rate) {
          case C2_15:
            kbch = 1992;
            break;
          case C3_15:
            kbch = 3072;
            break;
          case C4_15:
            kbch = 4152;
            break;
          case C5_15:
            kbch = 5232;
            break;
          case C6_15:
            kbch = 6312;
            break;
          case C7_15:
            kbch = 7392;
            break;
          case C8_15:
            kbch = 8472;
            break;
          case C9_15:
            kbch = 9552;
            break;
          case C10_15:
            kbch = 10632;
            break;
          case C11_15:
            kbch = 11712;
            break;
          case C12_15:
            kbch = 12792;
            break;
          case C13_15:
            kbch = 13872;
            break;
          default:
            kbch = 0;
            break;
        }
      }
      set_output_multiple(kbch);
      clock_gettime(CLOCK_TAI, &tai_last);
    }

    /*
     * Our virtual destructor.
     */
    alpbbheader_bb_impl::~alpbbheader_bb_impl()
    {
    }

    void
    alpbbheader_bb_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items / 8;
    }

    void
    alpbbheader_bb_impl::sendbits(unsigned char b, unsigned char *out)
    {
      for (int n = 7; n >= 0; n--) {
        *out++ = b & (1 << n) ? 1 : 0;
      }
    }

    int
    alpbbheader_bb_impl::checksum(unsigned short *addr, int count, int sum)
    {
      while (count > 1) {
        sum += *addr++;
        count -= 2;
      }
      if (count > 0) {
        sum += *(unsigned char *)addr;
      }
      sum = (sum & 0xffff) + (sum >> 16);
      sum += (sum >> 16);
      return (sum);
    }

    void
    alpbbheader_bb_impl::sendlls(unsigned char *out)
    {
      struct ip *ip_ptr;
      struct udphdr *udp_ptr;
      unsigned short length, temp;
      unsigned char *saddr_ptr, *daddr_ptr;
      unsigned short source_port = 56797;
      unsigned short destination_port = 4937;
      unsigned short *csum_ptr;
      unsigned char pseudo[2] = {0x00, 0x11};
      unsigned short csum;

      ip_ptr = (struct ip*)(out);

      ip_ptr->ip_v = IPVERSION;
      ip_ptr->ip_hl = 5;
      ip_ptr->ip_tos = 0;
      length = lls_length;
      ip_ptr->ip_len = ((length & 0xff) << 8) | ((length & 0xff00) >> 8);
      ip_ptr->ip_id = ((identification & 0xff) << 8) | ((identification & 0xff00) >> 8);
      ip_ptr->ip_off = ((IP_DF & 0xff) << 8) | ((IP_DF & 0xff00) >> 8);
      ip_ptr->ip_ttl = 64;
      ip_ptr->ip_p = IPPROTO_UDP;

      saddr_ptr = (unsigned char *)&ip_ptr->ip_src;
      for (unsigned int i = 0; i < sizeof(in_addr); i++) {
        *saddr_ptr++ = src_addr[i];
      }

      daddr_ptr = (unsigned char *)&ip_ptr->ip_dst;
      for (unsigned int i = 0; i < sizeof(in_addr); i++) {
        *daddr_ptr++ = dst_addr[i];
      }

      ip_ptr->ip_sum = 0;
      csum_ptr = (unsigned short *)(out);
      csum = checksum(csum_ptr, 20, 0);
      ip_ptr->ip_sum = ~csum;
      identification++;

      udp_ptr = (struct udphdr*)(out + sizeof(struct ip));

      length -= 20;
      udp_ptr->source = ((source_port & 0xff) << 8) | ((source_port & 0xff00) >> 8);
      udp_ptr->dest = ((destination_port & 0xff) << 8) | ((destination_port & 0xff00) >> 8);
      udp_ptr->len = ((length & 0xff) << 8) | ((length & 0xff00) >> 8);
      udp_ptr->check = 0;
      out[sizeof(struct ip) + sizeof(struct udphdr)] = 0x01; /* SLT */
      out[sizeof(struct ip) + sizeof(struct udphdr) + 1] = 0x01;
      out[sizeof(struct ip) + sizeof(struct udphdr) + 2] = 0x00;
      out[sizeof(struct ip) + sizeof(struct udphdr) + 3] = 0x00;
      memcpy(out + sizeof(struct ip) + sizeof(struct udphdr) + LLS_HEADER_LENGTH, llsgz, llsgz_length);
      csum_ptr = (unsigned short *)(out + 12);
      csum = checksum(csum_ptr, 8, 0);
      csum_ptr = (unsigned short *)(pseudo);
      csum = checksum(csum_ptr, 2, csum);
      temp = ((length & 0xff) << 8) | ((length & 0xff00) >> 8);
      csum_ptr = (unsigned short *)(&temp);
      csum = checksum(csum_ptr, 2, csum);
      csum_ptr = (unsigned short *)(out + sizeof(struct ip));
      csum = checksum(csum_ptr, 8, csum);
      csum_ptr = (unsigned short *)(out + sizeof(struct ip) + sizeof(struct udphdr));
      csum = checksum(csum_ptr, llsgz_length + LLS_HEADER_LENGTH, csum);
      udp_ptr->check = ~csum;
    }

    int
    alpbbheader_bb_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      auto in = static_cast<const input_type*>(input_items[0]);
      auto out = static_cast<output_type*>(output_items[0]);
      int consumed = 0;
      int produced = 0;
      unsigned char bits;
      struct timespec tai;
      long long delta;
      int pcount;
      int stuffing, offset;

      if (ninput_items[0] > (noutput_items / 2)) {
        for (int i = 0; i < noutput_items; i += kbch) {
          pcount = 2;
          clock_gettime(CLOCK_TAI, &tai);
          if (tai.tv_sec > tai_last.tv_sec) {
            delta = (tai.tv_nsec + (1000000000 * (tai.tv_sec - tai_last.tv_sec))) - tai_last.tv_nsec;
          }
          else {
            delta = tai.tv_nsec - tai_last.tv_nsec;
          }
          if (delta > 1000000000) {
            clock_gettime(CLOCK_TAI, &tai_last);
            if (lls_mode == LLS_ON) {
              dnp = TRUE;
            }
          }
          for (int j = 0; j < (int)((kbch - 16) / 8); j++) {
            if (j == 0 && lls_count != lls_length + ALP_HEADER_LENGTH) {
              bits = (bbcount & 0x7f) | 0x80;
              sendbits(bits, out);
              out += 8;
              produced += 8;
              if (total == NUM_PACKETS) {
                offset = (NUM_PACKETS - 2) - segments;
                stuffing = (total * (MPEG_PKT_LENGTH - 1)) + (offset);
                stuffing = stuffing - lls_length - LONG_EXTENSION_BYTES;
                if (stuffing > (kbch / 8) - LONG_EXTENSION_BYTES - 3) {
                  remainder = stuffing - (kbch / 8) + 3;
                  stuffing = (kbch / 8) - LONG_EXTENSION_BYTES - 3;
                }
                else {
                  remainder = 0;
                }
                bits = (bbcount >> 5) & 0xfc;
                bits |= 0x2; /* OFI = long extension mode */
                sendbits(bits, out);
                out += 8;
                produced += 8;
                bits = (stuffing & 0x1f) | 0xe0; /* EXT_TYPE = all padding */
                sendbits(bits, out);
                out += 8;
                produced += 8;
                bits = stuffing >> 5;
                sendbits(bits, out);
                out += 8;
                produced += 8;
                for (int n = 0; n < stuffing; n++) {
                  sendbits(0, out);
                  out += 8;
                  produced += 8;
                }
                pcount += (stuffing) + LONG_EXTENSION_BYTES;
                j += (stuffing) + LONG_EXTENSION_BYTES;
                total = 0;
                segments = 0;
                if (remainder == 0) {
                  trigger = TRUE;
                }
              }
              else if (remainder) {
                if (remainder > (kbch / 8) - LONG_EXTENSION_BYTES - 3) {
                  remainder = remainder - (kbch / 8) + 3;
                  stuffing = (kbch / 8) - LONG_EXTENSION_BYTES - 3;
                }
                else {
                  stuffing = remainder;
                  remainder = 0;
                }
                bits = (bbcount >> 5) & 0xfc;
                bits |= 0x2; /* OFI = long extension mode */
                sendbits(bits, out);
                out += 8;
                produced += 8;
                bits = (stuffing & 0x1f) | 0xe0; /* EXT_TYPE = all padding */
                sendbits(bits, out);
                out += 8;
                produced += 8;
                bits = stuffing >> 5;
                sendbits(bits, out);
                out += 8;
                produced += 8;
                for (int n = 0; n < stuffing; n++) {
                  sendbits(0, out);
                  out += 8;
                  produced += 8;
                }
                pcount += (stuffing) + LONG_EXTENSION_BYTES;
                j += (stuffing) + LONG_EXTENSION_BYTES;
                if (remainder == 0) {
                  trigger = TRUE;
                }
              }
              else {
                bits = (bbcount >> 5) & 0xfc;
                sendbits(bits, out);
                out += 8;
                produced += 8;
              }
            }
            if (lls_send == TRUE) {
              lls_count--;
              if (lls_count == 0) {
                lls_send = FALSE;
              }
              bits = llstemp[lls_index++];
              sendbits(bits, out);
              out += 8;
              produced += 8;
            }
            else {
              if (count == 0) {
                if (*in != 0x47) {
                  GR_LOG_WARN(d_logger, "Transport Stream sync error!");
                }
                if (dnp == TRUE && j < (kbch / 8) - 3) {
                  if ((in[1] == 0x1f) && (in[2] == 0xff)) {
                    packets++;
                    total++;
                    in += MPEG_PKT_LENGTH;
                    consumed += MPEG_PKT_LENGTH;
                    if (packets > 1) {
                      j--;
                    }
                    if (total == NUM_PACKETS) {
                      dnp = FALSE;
                    }
                    goto skip;
                  }
                  else {
                    if (packets != 0) {
                      bits = 0xe3; /* AHF - Additional Header Flag*/
                      sendbits(bits, out);
                      out += 8;
                      produced += 8;
                      bits = packets; /* DNP - Deleted Null Packets */
                      packets = 0;
                      segments++;
                      in++;
                      pcount += (MPEG_PKT_LENGTH + 1);
                    }
                    else {
                      bits = 0xe2; /* one TS packet per ALP packet */
                      in++;
                      pcount += MPEG_PKT_LENGTH;
                    }
                  }
                }
                else {
                  if (packets != 0) {
                    bits = 0xe3;
                    sendbits(bits, out);
                    out += 8;
                    produced += 8;
                    bits = packets;
                    packets = 0;
                    segments++;
                    in++;
                    pcount += (MPEG_PKT_LENGTH + 1);
                  }
                  else {
                    if (trigger == TRUE) {
                      trigger = FALSE;
                      bits = 0 | ((lls_length >> 8) & 0x7);
                      llstemp[0] = bits;
                      bits = lls_length & 0xff;
                      llstemp[1] = bits;
                      sendlls(&llstemp[2]);
                      lls_count = lls_length + ALP_HEADER_LENGTH;
                      lls_index = 0;
                      lls_send = TRUE;
                      pcount += lls_length + ALP_HEADER_LENGTH;
                      j--;
                      goto skip;
                    }
                    else {
                      bits = 0xe2;
                      in++;
                      pcount += MPEG_PKT_LENGTH;
                    }
                  }
                }
              }
              else {
                bits = *in++;
              }
              count = (count + 1) % MPEG_PKT_LENGTH;
              consumed++;
              sendbits(bits, out);
              out += 8;
              produced += 8;
skip:
              asm("nop");
            }
          }
          bbcount += pcount - (kbch / 8);
        }
      }
      if (produced != noutput_items) {
        if (produced != 0) {
          printf("produced = %d, expected = %d\n", produced, noutput_items);
        }
      }

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (consumed);

      // Tell runtime system how many output items we produced.
      return produced;
    }

    const char alpbbheader_bb_impl::xml[] = {"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"};
    const char alpbbheader_bb_impl::SLT[] = {"<SLT bsid=\"8086\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"tag:atsc.org,2016:XMLSchemas/ATSC3/Delivery/SLT/1.0/ SLT-1.0-20211209.xsd\">\n"};
    const char alpbbheader_bb_impl::Service[] = {"    <Service globalServiceID=\"urn:atsc:serviceid:drmpeg\" majorChannelNo=\"37\" minorChannelNo=\"1\" serviceCategory=\"1\" serviceId=\"73\" shortServiceName=\"W6RZ-DT\" sltSvcSeqNum=\"0\">\n"};
    const char alpbbheader_bb_impl::BroadcastSvcSignaling[] = {"        <BroadcastSvcSignaling slsDestinationIpAddress=\"239.255.37.1\" slsDestinationUdpPort=\"8000\" slsMajorProtocolVersion=\"0\" slsMinorProtocolVersion=\"0\" slsProtocol=\"1\" slsSourceIpAddress=\"44.4.15.9\"/>\n"};
    const char alpbbheader_bb_impl::Service_end[] = {"    </Service>\n"};
    const char alpbbheader_bb_impl::SLT_end[] = {"</SLT>\n"};

  } /* namespace atsc3 */
} /* namespace gr */
