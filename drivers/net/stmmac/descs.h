/*******************************************************************************
  Header File to describe the DMA descriptors.
  Enhanced descriptors have been in case of DWMAC1000 Cores.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>


  References:
    [1] - "Ethernet MAC Universal Databook", Version 3.61a
*******************************************************************************/
struct dma_desc {
	/* Receive descriptor */
	union {
		struct {
			/* RDES0 */
#if defined(__LITTLE_ENDIAN_BITFIELD)
			u32 reserved1:1;
			u32 crc_error:1;
			u32 dribbling:1;
			u32 mii_error:1;
			u32 receive_watchdog:1;
			u32 frame_type:1;
			u32 collision:1;
			u32 frame_too_long:1;
			u32 last_descriptor:1;
			u32 first_descriptor:1;
			u32 multicast_frame:1;
			u32 run_frame:1;
			u32 length_error:1;
			u32 partial_frame_error:1;
			u32 descriptor_error:1;
			u32 error_summary:1;
			u32 frame_length:14;
			u32 filtering_fail:1;
			u32 own:1;
#elif defined (__BIG_ENDIAN_BITFIELD)
			u32 own:1;
			u32 filtering_fail:1;
			u32 frame_length:14;
			u32 error_summary:1;
			u32 descriptor_error:1;
			u32 partial_frame_error:1;
			u32 length_error:1;
			u32 run_frame:1;
			u32 multicast_frame:1;
			u32 first_descriptor:1;
			u32 last_descriptor:1;
			u32 frame_too_long:1;
			u32 collision:1;
			u32 frame_type:1;
			u32 receive_watchdog:1;
			u32 mii_error:1;
			u32 dribbling:1;
			u32 crc_error:1;
			u32 reserved1:1;
#else
#error  "Please fix <asm/byteorder.h>"
#endif
			/* RDES1 */
#if defined(__LITTLE_ENDIAN_BITFIELD)
			u32 buffer1_size:11;
			u32 buffer2_size:11;
			u32 reserved2:2;
			u32 second_address_chained:1;
			u32 end_ring:1;
			u32 reserved3:5;
			u32 disable_ic:1;
#elif defined (__BIG_ENDIAN_BITFIELD)
			u32 disable_ic:1;
			u32 reserved3:5;
			u32 end_ring:1;
			u32 second_address_chained:1;
			u32 reserved2:2;
			u32 buffer2_size:11;
			u32 buffer1_size:11;
#else
#error  "Please fix <asm/byteorder.h>"
#endif
		} rx;
		struct {
			/* RDES0 */
			/* In version 3.50a and newer payload_csum_error field
			 * is actually Extended Status Available. See [1], p.302, 
			 * "Table 8-22  Receive Descriptor Fields (RDES0)"
			 */
			u32 payload_csum_error:1;
			u32 crc_error:1;
			u32 dribbling:1;
			u32 error_gmii:1;
			u32 receive_watchdog:1;
			u32 frame_type:1;
			u32 late_collision:1;
			u32 ipc_csum_error:1;
			u32 last_descriptor:1;
			u32 first_descriptor:1;
			u32 vlan_tag:1;
			u32 overflow_error:1;
			u32 length_error:1;
			u32 sa_filter_fail:1;
			u32 descriptor_error:1;
			u32 error_summary:1;
			u32 frame_length:14;
			u32 da_filter_fail:1;
			u32 own:1;
			/* RDES1 */
			u32 buffer1_size:13;
			u32 reserved1:1;
			u32 second_address_chained:1;
			u32 end_ring:1;
			u32 buffer2_size:13;
			u32 reserved2:2;
			u32 disable_ic:1;
		} erx;		/* -- enhanced -- */

		/* Transmit descriptor */
		struct {
			/* TDES0 */
#if defined(__LITTLE_ENDIAN_BITFIELD)
			u32 deferred:1;
			u32 underflow_error:1;
			u32 excessive_deferral:1;
			u32 collision_count:4;
			u32 heartbeat_fail:1;
			u32 excessive_collisions:1;
			u32 late_collision:1;
			u32 no_carrier:1;
			u32 loss_carrier:1;
			u32 reserved1:3;
			u32 error_summary:1;
			u32 reserved2:15;
			u32 own:1;
#elif defined (__BIG_ENDIAN_BITFIELD)
			u32 own:1;
			u32 reserved2:15;
			u32 error_summary:1;
			u32 reserved1:3;
			u32 loss_carrier:1;
			u32 no_carrier:1;
			u32 late_collision:1;
			u32 excessive_collisions:1;
			u32 heartbeat_fail:1;
			u32 collision_count:4;
			u32 excessive_deferral:1;
			u32 underflow_error:1;
			u32 deferred:1;
#else
#error  "Please fix <asm/byteorder.h>"
#endif

			/* TDES1 */
#if defined(__LITTLE_ENDIAN_BITFIELD)
			u32 buffer1_size:11;
			u32 buffer2_size:11;
			u32 reserved3:1;
			u32 disable_padding:1;
			u32 second_address_chained:1;
			u32 end_ring:1;
			u32 crc_disable:1;
			u32 reserved4:2;
			u32 first_segment:1;
			u32 last_segment:1;
			u32 interrupt:1;
#elif defined (__BIG_ENDIAN_BITFIELD)
			u32 interrupt:1;
			u32 last_segment:1;
			u32 first_segment:1;
			u32 reserved4:2;
			u32 crc_disable:1;
			u32 end_ring:1;
			u32 second_address_chained:1;
			u32 disable_padding:1;
			u32 reserved3:1;
			u32 buffer2_size:11;
			u32 buffer1_size:11;
#else
#error  "Please fix <asm/byteorder.h>"
#endif
		} tx;
		struct {
			/* TDES0 */
			u32 deferred:1;
			u32 underflow_error:1;
			u32 excessive_deferral:1;
			u32 collision_count:4;
			u32 vlan_frame:1;
			u32 excessive_collisions:1;
			u32 late_collision:1;
			u32 no_carrier:1;
			u32 loss_carrier:1;
			u32 payload_error:1;
			u32 frame_flushed:1;
			u32 jabber_timeout:1;
			u32 error_summary:1;
			u32 ip_header_error:1;
			u32 time_stamp_status:1;
			u32 reserved1:2;
			u32 second_address_chained:1;
			u32 end_ring:1;
			u32 checksum_insertion:2;
			u32 reserved2:1;
			u32 time_stamp_enable:1;
			u32 disable_padding:1;
			u32 crc_disable:1;
			u32 first_segment:1;
			u32 last_segment:1;
			u32 interrupt:1;
			u32 own:1;
			/* TDES1 */
			u32 buffer1_size:13;
			u32 reserved3:3;
			u32 buffer2_size:13;
			u32 reserved4:3;
		} etx;		/* -- enhanced -- */
		struct {
			uint32_t	w0;
			uint32_t	w1;
		} words;
	} des01;
	unsigned int des2;
	unsigned int des3;

	/* [1], p.302, "Table 6-5 Register 0 (Bus Mode Register)" says that
	 * after version 3.50a enhanced descriptor size is 32 bytes
	 * instead of 16 as earlier.
	 */
	union {
		struct {
			u32 ip_payload_type:3;
			u32 ip_header_error:1;
			u32 ip_payload_error:1;
			u32 ip_csum_bypass:1;
			u32 ipv4_pkt_rx:1;
			u32 ipv6_pkt_rx:1;
			u32 msg_type:4;
			u32 ptp_frame_type:1;
			u32 ptp_version:1;
			u32 time_stamp_dropped:1;
			u32 reserved1:1;
			u32 av_pkt_rx:1;
			u32 av_tagged_pkt_rx:1;
			u32 vlan_tag_prio:3;
			u32 reserved2:11;
		} ext_stat;
		uint32_t		w2;
	} des04;
	unsigned int des5;
	unsigned int des6;
	unsigned int des7;
};

/* Transmit checksum insertion control */
enum tdes_csum_insertion {
	cic_disabled = 0,	/* Checksum Insertion Control */
	cic_only_ip = 1,	/* Only IP header */
	cic_no_pseudoheader = 2,	/* IP header but pseudoheader
					 * is not calculated */
	cic_full = 3,		/* IP header and pseudoheader */
};


static inline void dma_desc_le2cpu(struct dma_desc *pdesc, struct dma_desc *pcpu)
{
	pcpu->des01.words.w0 =	le32_to_cpu(pdesc->des01.words.w0);
	pcpu->des01.words.w1 =	le32_to_cpu(pdesc->des01.words.w1);
	pcpu->des2 =		le32_to_cpu(pdesc->des2);
	pcpu->des3 =		le32_to_cpu(pdesc->des3);
}

static inline void dma_desc_cpu2le(struct dma_desc *pdesc, struct dma_desc *pcpu)
{
	pdesc->des01.words.w0 =	cpu_to_le32(pcpu->des01.words.w0);
	pdesc->des01.words.w1 =	cpu_to_le32(pcpu->des01.words.w1);
	pdesc->des2 =		cpu_to_le32(pcpu->des2);
	pdesc->des3 =		cpu_to_le32(pcpu->des3);
}

