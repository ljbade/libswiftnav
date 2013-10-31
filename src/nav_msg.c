/*
 * Copyright (C) 2010 Swift Navigation Inc.
 * Contact: Henry Hallam <henry@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "nav_msg.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define NAV_MSG_BIT_PHASE_THRES 5
#define SUBFRAME_START_BUFFER_OFFSET (NAV_MSG_SUBFRAME_BITS_LEN * 32 - 360)

// LB: Add forward function  definitions
int nav_parity(u32 *word);
int parity(u32 x);

void nav_msg_init(nav_msg_t *n)
{
  /* Initialize the necessary parts of the nav message state structure. */
  n->subframe_bit_index = 0;
  n->bit_phase = 0;
  n->bit_phase_ref = 0;
  n->bit_phase_count = 0;
  n->nav_bit_integrate = 0;
  n->subframe_start_index = 0;
  memset(n->subframe_bits, 0, sizeof(n->subframe_bits));
  n->next_subframe_id = 1;
}

u32 extract_word(nav_msg_t *n, u16 bit_index, u8 n_bits, u8 invert)
{
  /* Extract a word of n_bits length (n_bits <= 32) at position bit_index into
   * the subframe. Takes account of the offset stored in n, and the circular
   * nature of the n->subframe_bits buffer. */

  /* Offset for the start of the subframe in the buffer. */
  if (n->subframe_start_index) {
    if (n->subframe_start_index > 0)
      bit_index += n->subframe_start_index; /* Standard. */
    else {
      bit_index -= n->subframe_start_index; /* Bits are inverse! */
      invert = !invert;
    }

    bit_index--;
  }

  /* Wrap if necessary. */
  if (bit_index > NAV_MSG_SUBFRAME_BITS_LEN * 32)
    bit_index -= NAV_MSG_SUBFRAME_BITS_LEN * 32;

  u8 bix_hi = bit_index >> 5;
  u8 bix_lo = bit_index & 0x1F;
  u32 word = n->subframe_bits[bix_hi] << bix_lo;

  if (bix_lo) {
    bix_hi++;
    if (bix_hi == NAV_MSG_SUBFRAME_BITS_LEN)
      bix_hi = 0;
    word |=  n->subframe_bits[bix_hi] >> (32 - bix_lo);
  }

  if (invert)
    word = ~word;

  return word >> (32 - n_bits);
}

s32 nav_msg_update(nav_msg_t *n, s32 corr_prompt_real)
{
  /* Called once per tracking loop update (atm fixed at 1 PRN [1 ms]). Performs
   * the necessary steps to recover the nav bit clock, store the nav bits and
   * decode them. */

  s32 TOW_ms = -1;

  /* Do we have bit phase lock yet? (Do we know which of the 20 possible PRN
   * offsets corresponds to the nav bit edges?) */
  n->bit_phase++;
  n->bit_phase %= 20;

  if (n->bit_phase_count < NAV_MSG_BIT_PHASE_THRES) {

    /* No bit phase lock yet. */
    if ((n->nav_bit_integrate > 0) != (corr_prompt_real > 0)) {
      /* Edge detected. */
      if (n->bit_phase == n->bit_phase_ref)
        /* This edge came N*20 ms after the last one. */
        n->bit_phase_count++;
      else {
        /* Store the bit phase hypothesis. */
        n->bit_phase_ref = n->bit_phase;
        n->bit_phase_count = 1;
      }
    }

    /* Store the correlation for next time. */
    n->nav_bit_integrate = corr_prompt_real;

  } else {

    /* We have bit phase lock. */
    if (n->bit_phase != n->bit_phase_ref) {
      /* Sum the correlations over the 20 ms bit period. */
      n->nav_bit_integrate += corr_prompt_real;
    } else {
      /* Dump the nav bit, i.e. determine the sign of the correlation over the
       * nav bit period. */

      /* Is bit 1? */
      if (n->nav_bit_integrate > 0) {
        n->subframe_bits[n->subframe_bit_index >> 5] |= \
          1 << (31 - (n->subframe_bit_index & 0x1F));
      } else {
        /* Integrated correlation is negative, so bit is 0. */
        n->subframe_bits[n->subframe_bit_index >> 5] &= \
          ~(1 << (31 - (n->subframe_bit_index & 0x1F)));
      }

      /* Zero the integrator for the next nav bit. */
      n->nav_bit_integrate = 0;

      n->subframe_bit_index++;
      if (n->subframe_bit_index == NAV_MSG_SUBFRAME_BITS_LEN * 32)
        n->subframe_bit_index = 0;

      /* Yo dawg, are we still looking for the preamble? */
      if (!n->subframe_start_index) {
        /* We're going to look for the preamble at a time 360 nav bits ago,
         * then again 60 nav bits ago.
         * See SUBFRAME_START_BUFFER_OFFSET */

        /* Check whether there's a preamble at the start of the circular
         * subframe_bits buffer. */
        u8 preamble_candidate = extract_word(n, n->subframe_bit_index + SUBFRAME_START_BUFFER_OFFSET, 8, 0);

        if (preamble_candidate == 0x8B) {
           n->subframe_start_index = n->subframe_bit_index + SUBFRAME_START_BUFFER_OFFSET + 1;
        }
        else if (preamble_candidate == 0x74) {
           n->subframe_start_index = -(n->subframe_bit_index + SUBFRAME_START_BUFFER_OFFSET + 1);
        }

        if (n->subframe_start_index) {
          // LB: Check the parity
          u32 sf_tlm1 = extract_word(n, -2, 32, 0);
          u32 sf_tlm2 = extract_word(n, -270, 32, 0);
          u32 sf_how = extract_word(n, 28, 32, 0);

          // Looks like we found a preamble, but let's confirm.
          if (nav_parity(&sf_tlm1) && nav_parity(&sf_tlm2) && nav_parity(&sf_how) && (extract_word(n, 300, 8, 0) == 0x8B)) {
            // There's another preamble in the following subframe.  Looks good so far.

            // Extract the TOW:
            TOW_trunc--;  // LB: Decrement it, to see what we expect at the start of the previous subframe
            signed int TOW_trunc = extract_word(n, 30, 17, extract_word(n, 29, 1, 0)); // bit 29 is D30* for the second word, where the TOW resides.
            if (TOW_trunc <= -1)  // Handle end of week rollover
              TOW_trunc = 7 * 24 * 60 * 10 - 1;

            // LB: Check against the previous frame
            if (TOW_trunc == extract_word(n, -270, 17, extract_word(n, 329, 1, 0))) {
              // We got two appropriately spaced preambles, and two matching TOW counts.  Pretty certain now.

              // LB: Reset the TOW to the correct number
              TOW_trunc++;
              if (TOW_trunc >= 7 * 24 * 60 * 10)  // Handle end of week rollover
                            TOW_trunc = 0;
              TOW_trunc++;
              if (TOW_trunc >= 7 * 24 * 60 * 10)  // Handle end of week rollover
                            TOW_trunc = 0;

              // The TOW in the message is for the start of the NEXT subframe.
              // That is, 240 nav bits' time from now, since we are 60 nav bits into the second subframe that we recorded.
              if (TOW_trunc)
                TOW_ms = TOW_trunc * 6000 - (300 - 60) * 20;
              else  // end of week special case
                TOW_ms = 7 * 24 * 60 * 60 * 1000 - (300 - 60) * 20;
              //printf("TOW = hh:%02d:%02d.%03d\n", (int) (TOW_ms / 60000 % 60), (int)(TOW_ms / 1000 % 60), (int)(TOW_ms % 1000));

              // LB: Find best place for this info
              // LB: All subframes: TLM word, HOW, parity
              // LB: Preamble: TLM word (1), bits 1-8, "1001011"
              // LB: TLM message: TLM word (1), bits 9-22
              // LB: Integrity status flag: TLM word (1), bit 23
              // LB: Reserved: TLM word (1), bit 24
              // LB: Parity: All words, bits 25-30, see IS for algorithm
              // LB: TOW count (truncated): HOW (2), bits 1-17, 17 MSBs of full TOW count
              // LB: Note: TOW count is the 19 LSBs of the 29 bit Z count
              // LB: Alert flag: HOW(2), bit 18 (was momentum dump flag on Block I SVs)
              // LB: Anti spoof flag: HOW(2), bit 19 (was TLM word synchronization flag on Block I SVs)
              // LB: Subframe ID: HOW(2), bits 20-22
              // LB: Parity fixup bits: HOW(2), bit 23-24, solved for parity bits 29-30 being 0

              // LB: Extract the TLM and integrity status flag from the TLM word
              n->tlm = extract_word(n, 308, 14, extract_word(n, 299, 1, 0));
              n->integrity_status_flag = extract_word(n, 322, 1, extract_word(n, 299, 1, 0)); // LB: TODO: Use ISF in RAIM (if implemented)

              // LB: Extract the alert flag and the anti-spoof flag from the TLM word
              // LB: TODO: Use alert flag in position solution
              n->alert_flag = extract_word(n, 317, 1, extract_word(n, 329, 1, 0)); // LB: QZSS: Set when URA > 9.64m? (max of NMCT correction data), compatible with GPS
              n->anti_spoof_flag = extract_word(n, 317, 1, extract_word(n, 329, 1, 0)); // LB: QZSS: Fixed to 0, no AS mode on QZSS (also same with GPS), compatible with GPS
              if (!n->alert_flag)
                printf("SV ALERT\n");

            } else
              n->subframe_start_index = 0;  // the TOW counts didn't match - disregard.
          } else
            n->subframe_start_index = 0;    // didn't find a second preamble in the right spot - disregard.
        }
      }
    }
  }

  return TOW_ms;
}

int parity(u32 x)
{
  /* Returns 1 if there are an odd number of bits set. */
  x ^= x >> 1;
  x ^= x >> 2;
  x ^= x >> 4;
  x ^= x >> 8;
  x ^= x >> 16;
  return (x & 1);
}

int nav_parity(u32 *word) {
// expects a word where MSB = D29*, bit 30 = D30*, bit 29 = D1, ... LSB = D30 as described in IS-GPS-200E Table 20-XIV
// Inverts the bits if necessary, and checks the parity.
// Returns 0 for success, 1 for fail.

  if (*word & 1 << 30)     // inspect D30*
    *word ^= 0x3FFFFFC0; // invert all the data bits!

  //printf("w=%08X  ", (unsigned int)word);

  if (parity(*word & 0xBB1F34A0 /* 0b10111011000111110011010010100000 */)) // check d25 (see IS-GPS-200E Table 20-XIV)
    return 25;

  if (parity(*word & 0x5D8F9A50 /* 0b01011101100011111001101001010000 */)) // check d26
    return 26;

  if (parity(*word & 0xAEC7CD08 /* 0b10101110110001111100110100001000 */)) // check d27
    return 27;

  if (parity(*word & 0x5763E684 /* 0b01010111011000111110011010000100 */)) // check d28
    return 28;

  if (parity(*word & 0x6BB1F342 /* 0b01101011101100011111001101000010 */)) // check d29
    return 29;

  if (parity(*word & 0x8B7A89C1 /* 0b10001011011110101000100111000001 */)) // check d30
    return 30;

  return 0;
}

bool subframe_ready(nav_msg_t *n) {
  return (n->subframe_start_index != 0);
}

s8 process_subframe(nav_msg_t *n, ephemeris_t *e) {
  // Check parity and parse out the ephemeris from the most recently received subframe

  // LB: TODO: How to improve TTFF:
  // LB: TODO: enable incomplete, out of order subframes, so that we can use ephemeris with any 3 subframes that match IODE/IODC, incase some subframes are dropped due to noise
  // LB: TODO: also handle case if TLM or HOW fails parity, but remaining frame is good data
  // LB: TODO: perhaps we can predict TOW if we lose it? HOW can remain static but need to make sure we check alert/ISF asap

  // First things first - check the parity, and invert bits if necessary.
  // process the data, skipping the first word, TLM, and starting with HOW

  //printf("  %d  ", (n->subframe_start_index > 0));

  /* TODO: Check if inverted has changed and detect half cycle slip. */
  if (n->inverted != (n->subframe_start_index < 0))
    printf("Nav phase flip\n");

  n->inverted = (n->subframe_start_index < 0);

  if (!e) {
    printf(" process_subframe: CALLED WITH e = NULL!\n");
    n->subframe_start_index = 0;  // Mark the subframe as processed
    n->next_subframe_id = 1;      // Make sure we start again next time
    return -1;
  }

  // LB: Add parity check for word 1
  u32 sf_word1 = extract_word(n, -2, 32, 0);

  if (nav_parity(&sf_word1)) {
      printf("SUBFRAME PARITY ERROR (word 1)\n");
      n->subframe_start_index = 0;  // Mark the subframe as processed
      n->next_subframe_id = 1;      // Make sure we start again next time
      return -2;
  }

  u32 sf_word2 = extract_word(n, 28, 32, 0);

  if (nav_parity(&sf_word2)) {
      printf("SUBFRAME PARITY ERROR (word 2)\n");
      n->subframe_start_index = 0;  // Mark the subframe as processed
      n->next_subframe_id = 1;      // Make sure we start again next time
      return -2;
  }

  u8 sf_id = sf_word2 >> 8 & 0x07;    // Which of 5 possible subframes is it?

  /*printf("sf_id = %d, nsf = %d\n",sf_id, n->next_subframe_id);*/

  if (sf_id <= 3 && sf_id == n->next_subframe_id) {  // Is it the one that we want next?

    for (int w = 0; w < 8; w++) {   // For words 3..10
      n->frame_words[sf_id - 1][w] = extract_word(n, 30 * (w + 2) - 2, 32, 0);    // Get the bits

      // MSBs are D29* and D30*.  LSBs are D1...D30
      if (nav_parity(&n->frame_words[sf_id - 1][w])) {  // Check parity and invert bits if D30*
        printf("SUBFRAME PARITY ERROR (word %d)\n", w + 3);
        n->next_subframe_id = 1;      // Make sure we start again next time
        n->subframe_start_index = 0;  // Mark the subframe as processed
        return -3;
      }
    }

    n->subframe_start_index = 0;  // Mark the subframe as processed
    n->next_subframe_id++;

    if (sf_id == 3) {
      // Got all of subframes 1 to 3
      n->next_subframe_id = 1;      // Make sure we start again next time

      // Now let's actually go through the parameters...

      // These unions facilitate signed/unsigned conversion and sign extension
      // TODO: Use types from common.h here
      union {
        char s8;
        unsigned char u8;
      } onebyte;

      union
      {
        short s16;
        unsigned short u16;
      } twobyte;

      union
      {
        int s32;
        unsigned u32;
      } fourbyte;

      // LB: All subframes: parity
      // LB: Parity: All words, bits 25-30, see IS for algorithm
      // LB: Parity fixup bits: Word 10, bit 23-24, for parity computation

      // Subframe 1: WN, C/A or P on L2, URA index, SV health, IODC, L2 P data flag, T_GD, t_oc, a_f2, a_f1, a_f0

      // LB: TODO: need to handle week roll over, should save the current week epoch permanently, somewhere like flash
      // LB: TODO: can cross check it against other GNSS is using multiple
      // LB: TODO: also the UTC leap seconds should be saved permanetly somewhere too
      e->toe.wn = (n->frame_words[0][3 - 3] >> (30 - 10) & 0x3FF);          // GPS week number (mod 1024): Word 3, bits 1-10
      e->toe.wn += GPS_WEEK_CYCLE * 1024;                                   // LB: Note: WN is the 10 MSBs of the 29 bit Z count
      e->toc.wn = e->toe.wn;

      e->ca_or_p_on_l2 = n->frame_words[0][3 - 3] >> (30 - 12) & 0x3;       // LB: C/A or P on L2: Word 3, bits 11-12
                                                                            // LB: QZSS: Fixed to "10" (C/A on L2) as no L2P code (or L2 C/A code?), compatible with GPS

      e->ura_index = n->frame_words[0][3 - 3] >> (30 - 6) & 0xF;            // LB: URA index: Word 3, bits 13-16
      if (e->ura_index < 6) {
        e->ura = pow(2, 1.0 + (double)(e->ura_index) / 2.0);
        if (e->ura_index == 1)
          e->ura = 2.8;
        if (e->ura_index == 3)
          e->ura = 5.7;
        if (e->ura_index == 5)
          e->ura = 11.3;
      } else if (e->ura_index < 15) {
        e->ura = pow(2, (double)(e->ura_index) - 2.0);
      } else {
        // LB: Do not use this SV!
        // LB: TODO: add to logic, inc alert bits, health bits (eph and alm), parity, non std code
        e->ura = INFINITY;
      }

      // LB: Changed to the 6 bit SV health bits
      e->sv_health = n->frame_words[0][3 - 3] >> (30 - 22) & 0x3F;          // SV health status bits: Word 3, bits 17-22
      e->healthy = !(e->sv_health & 0x20);                                  // LB: QZSS: MSB nav status is same as GPS, but the 5 LSBs are not the same, incompatible with GPS
      if (!e->healthy)
        printf("SV NAV UNHEALTHY\n");
      if (((e->sv_health & 0x1F) == 0x01) || // LB: All signals weak
          ((e->sv_health & 0x1F) == 0x02) || // LB: All signals dead
          ((e->sv_health & 0x1F) == 0x04) || // LB: All signals have no data modulation
          ((e->sv_health & 0x1F) == 0x02) || // LB: All signals dead
          ((e->sv_health & 0x1F) == 0x02) || // LB: All signals dead
          ((e->sv_health & 0x1F) == 0x0A) || // LB: L1 C signal weak
          ((e->sv_health & 0x1F) == 0x0B) || // LB: L1 C signal dead
          ((e->sv_health & 0x1F) == 0x0C) || // LB: L1 C signal has no data modulation
          ((e->sv_health & 0x1F) == 0x13) || // LB: L1 & L2 C signal weak
          ((e->sv_health & 0x1F) == 0x14) || // LB: L1 & L2 C signal dead
          ((e->sv_health & 0x1F) == 0x15) || // LB: L1 & L2 C signal has no data modulation
          ((e->sv_health & 0x1F) == 0x16) || // LB: L1 signal weak
          ((e->sv_health & 0x1F) == 0x17) || // LB: L1 signal dead
          ((e->sv_health & 0x1F) == 0x18) || // LB: L1 signal has no data modulation
          ((e->sv_health & 0x1F) == 0x1C) || // LB: SV is temporarily out (do not use this SV during current pass)
          ((e->sv_health & 0x1F) == 0x1D) || // LB: SV will be temporarily out (use with caution)
          ((e->sv_health & 0x1F) == 0x1E) || // LB: One or more signals deformed, however the relevant URA paramters are valid
          ((e->sv_health & 0x1F) == 0x1F)) { // LB: More than one combination would be required to describe the anomalies
        printf("SV L1 UNHEALTHY\n");
        e->healthy = 0;
      }
      if (e->ura_index == 15) {
        printf("SV URA UNHEALTHY\n");
        e->healthy = 0;
      }
      // LB: TODO: also need to set unhealthy if alternating ones and zeros, non standard code, SatZap (PRN 37)

      e->iodc_1 = ((n->frame_words[0][3 - 3] >> (30 - 24) & 0x3) << 8)      // LB: IODC for subframe 1: Word 3, bits 23-24
             | (n->frame_words[0][8 - 3] >> (30 - 8) & 0xFF);               // and word 8, bits 1-8

      e->l2_p_data_flag = n->frame_words[0][4 - 3] >> (30 - 1) & 0x1;       // LB: L2 P data flag: Word 4, bit 1
                                                                            // LB: QZSS: Fixed to 0 as no L2P code, compatible with GPS

      // LB: Reserved: Word 4, bits 2-24

      // LB: Reserved: Word 5, bits 1-24

      // LB: Reserved: Word 6, bits 1-24

      // LB: Reserved: Word 7, bits 1-16 

      onebyte.u8 = n->frame_words[0][7 - 3] >> (30 - 24) & 0xFF;            // t_gd: Word 7, bits 17-24
      e->tgd = onebyte.s8 * pow(2, -31);                                    // LB: QZSS: Adds "10000000" to indicate t_gd cannot be use, incompatible with GPS

      e->toc.tow = (n->frame_words[0][8 - 3] >> (30 - 24) & 0xFFFF) * 16;   // t_oc: Word 8, bits 8-24

      onebyte.u8 = n->frame_words[0][9 - 3] >> (30 - 8) & 0xFF;             // a_f2: Word 9, bits 1-8
      e->af2 = onebyte.s8 * pow(2, -55);

      twobyte.u16 = n->frame_words[0][9 - 3] >> (30 - 24) & 0xFFFF;         // a_f1: Word 9, bits 9-24
      e->af1 = twobyte.s16 * pow(2, -43);

      fourbyte.u32 = n->frame_words[0][10 - 3] >> (30 - 22) & 0x3FFFFF;     // a_f0: Word 10, bits 1-22
      fourbyte.u32 <<= 10; // Shift to the left for sign extension
      fourbyte.s32 >>= 10; // Carry the sign bit back down and reduce to signed 22 bit value
      e->af0 = fourbyte.s32 * pow(2, -31);

      // Subframe 2: IODE, crs, dn, m0, cuc, ecc, cus, sqrta, toe, fit interval flag, AODO

      e->iode_2 = (n->frame_words[1][3 - 3] >> (30 - 8) & 0xF);             // LB: IODE for subframe 2: Word 3, bits 1-8

      twobyte.u16 = n->frame_words[1][3 - 3] >> (30 - 24) & 0xFFFF;         // crs: Word 3, bits 9-24
      e->crs = twobyte.s16 * pow(2, -5);

      twobyte.u16 = n->frame_words[1][4 - 3] >> (30 - 16) & 0xFFFF;         // dn: Word 4, bits 1-16
      e->dn = twobyte.s16 * pow(2, -43) * M_PI;

      fourbyte.u32 = ((n->frame_words[1][4 - 3] >> (30 - 24) & 0xFF) << 24) // m0: Word 4, bits 17-24
                  | (n->frame_words[1][5 - 3] >> (30 - 24) & 0xFFFFFF);     // and word 5, bits 1-24
      e->m0 = fourbyte.s32 * pow(2, -31) * M_PI;

      twobyte.u16 = n->frame_words[1][6 - 3] >> (30 - 16) & 0xFFFF;         // cuc: Word 6, bits 1-16
      e->cuc = twobyte.s16 * pow(2, -29);

      fourbyte.u32 = ((n->frame_words[1][6 - 3] >> (30 - 24) & 0xFF) << 24) // ecc: Word 6, bits 17-24
                  | (n->frame_words[1][7 - 3] >> (30 - 24) & 0xFFFFFF);     // and word 7, bits 1-24
      e->ecc = fourbyte.u32 * pow(2, -33);                                  // LB: QZSS: This value can go to 0.5 (0.03 max for GPS), partially incompatible with GPS

      twobyte.u16 = n->frame_words[1][8 - 3] >> (30 - 16) & 0xFFFF;         // cus: Word 8, bits 1-16
      e->cus = twobyte.s16 * pow(2, -29);

      fourbyte.u32 = ((n->frame_words[1][8 - 3] >> (30 - 24) & 0xFF) << 24) // sqrta: Word 8, bits 17-24
                  | (n->frame_words[1][9 - 3] >> (30 - 24) & 0xFFFFFF);     // and word 9, bits 1-24
      e->sqrta = fourbyte.u32 * pow(2, -19);

      e->toe.tow = (n->frame_words[1][10 - 3] >> (30 - 16) & 0xFFFF) * 16;  // t_oe: Word 10, bits 1-16

      e->fit_interval_flag = n->frame_words[1][10 - 3] >> (30 - 17) & 0x1;  // LB: Fit interval flag: Word 10, bit 17

      e->aodo = (n->frame_words[1][10 - 3] >> (30 - 18) & 0x1F) * 900;      // LB: AODO: Word 10, bits 18-22

      // Subframe 3: cic, omega0, cis, inc, crc, w, omegadot, IODE, inc_dot

      twobyte.u16 = n->frame_words[2][3 - 3] >> (30 - 16) & 0xFFFF;         // cic: Word 3, bits 1-16
      e->cic = twobyte.s16 * pow(2, -29);

      fourbyte.u32 = ((n->frame_words[2][3 - 3] >> (30 - 24) & 0xFF) << 24) // omega0: Word 3, bits 17-24
                  | (n->frame_words[2][4 - 3] >> (30 - 24) & 0xFFFFFF);     // and word 4, bits 1-24
      e->omega0 = fourbyte.s32 * pow(2, -31) * M_PI;

      twobyte.u16 = n->frame_words[2][5 - 3] >> (30 - 16) & 0xFFFF;         // cis: Word 5, bits 1-16
      e->cis = twobyte.s16 * pow(2, -29);

      fourbyte.u32 = ((n->frame_words[2][5 - 3] >> (30 - 24) & 0xFF) << 24) // inc (i0): Word 5, bits 17-24
                  | (n->frame_words[2][6 - 3] >> (30 - 24) & 0xFFFFFF);     // and word 6, bits 1-24
      e->inc = fourbyte.s32 * pow(2, -31) * M_PI;

      twobyte.u16 = n->frame_words[2][7 - 3] >> (30 - 16) & 0xFFFF;         // crc: Word 7, bits 1-16
      e->crc = twobyte.s16 * pow(2, -5);

      fourbyte.u32 = ((n->frame_words[2][7 - 3] >> (30 - 24) & 0xFF) << 24) // w (omega): Word 7, bits 17-24
                  | (n->frame_words[2][8 - 3] >> (30 - 24) & 0xFFFFFF);     // and word 8, bits 1-24
      e->w = fourbyte.s32 * pow(2, -31) * M_PI;

      fourbyte.u32 = n->frame_words[2][9 - 3] >> (30 - 24) & 0xFFFFFF;      // Omega_dot: Word 9, bits 1-24
      fourbyte.u32 <<= 8; // shift left for sign extension
      fourbyte.s32 >>= 8; // sign-extend it
      e->omegadot = fourbyte.s32 * pow(2, -43) * M_PI;

      e->iode_3 = (n->frame_words[2][10 - 3] >> (30 - 8) & 0xF);            // LB: IODE for subframe 3: Word 10, bits 1-8

      twobyte.u16 = n->frame_words[2][10 - 3] >> (30 - 22) & 0x3FFF;        // inc_dot (IDOT): Word 10, bits 9-22
      twobyte.u16 <<= 2;
      twobyte.s16 >>= 2;  // sign-extend
      e->inc_dot = twobyte.s16 * pow(2, -43) * M_PI;

      // LB: TODO: Should continue to use the old ephemeris if the new one is invalid
      // LB: TODO: Same with almanac
      // LB: TODO: But if old emphemeris is too old should stop using until new one is available
      // LB: TODO: Need to check age of toa (WNA) and tou and ionospheric age, almanac validity

      // LB: Ensure the IODC and IODEs are consistent, this means all the other ephemeris data is consistent too
      if (((e->iodc_1 & 0x0F) == e->iode_2) && (e->iode_2 == e->iode_3))
        e->valid = 1;
      else
        e->valid = 0;

      // LB: Detect if there has been a new upload in the last 4 hours
      // LB: t_oe will be slightly offset from the hour for the first (and possibly second) data set after upload
      // LB: If another upload happens it will be offset even more
      e->upload = fmod(e->toe.tow, 3600) != 0;

      // LB: Calculate the ephemeris data fit interval
      // LB: Calculate the start of the ephemeris transmission interval
      e->start_time.wn = e->toe.wn;
      e->start_time.tow = e->toe.tow;
      if (!e->fit_interval_flag) {
        // LB: Normal operations
        // LB: Fit interval is 4 hours
        // LB: IODC/IODE will be incremented on each new data set cutover (both from new upload and from automatic cutover)
        // LB: Except that IODE is not allowed to  have 8 LSBs be >=240 and <=255 (because of old receivers still using the Block I ICD definitions,
        // LB: as they only used 8 bit IODC/IODE vs the current 10 bit IODC)
        e->fit_interval = 4 * 3600;
        e->start_time.tow -= 2 * 3600;
      } else {
        // LB: Note that the 98, 122, and 146 hour fits are from earlier ICDs, they have been removed from newer ICDs but remain backwards compatible
        if ((e->iodc_1 >= 240) && (e->iodc_1 <= 247)) {
          // LB: Long term extended operations
          e->fit_interval = 8 * 3600;
          e->start_time.tow -= 4 * 3600;
        } else if (((e->iodc_1 >= 248) && (e->iodc_1 <= 255)) || (e->iodc_1 == 496)) {
          e->fit_interval = 14 * 3600;
          e->start_time.tow -= 7 * 3600;
        } else if (((e->iodc_1 >= 497) && (e->iodc_1 <= 503)) || ((e->iodc_1 >= 1021) && (e->iodc_1 <= 1023))) {
          e->fit_interval = 26 * 3600;
          e->start_time.tow -= 13 * 3600;
        } else if ((e->iodc_1 >= 504) && (e->iodc_1 <= 510)) {
          e->fit_interval = 50 * 3600;
          e->start_time.tow -= 25 * 3600;
        } else if ((e->iodc_1 == 511) || ((e->iodc_1 >= 752) && (e->iodc_1 <= 756))) {
          e->fit_interval = 74 * 3600;
          e->start_time.tow -= 37 * 3600;
        } else if ((e->iodc_1 >= 757) && (e->iodc_1 <= 763)) {
          e->fit_interval = 98 * 3600;
          e->start_time.tow -= 49 * 3600;
        } else if (((e->iodc_1 >= 764) && (e->iodc_1 <= 767)) || ((e->iodc_1 >= 1008) && (e->iodc_1 <= 1010))) {
          e->fit_interval = 122 * 3600;
          e->start_time.tow -= 61 * 3600;
        } else if ((e->iodc_1 >= 1011) && (e->iodc_1 <= 1020)) {
          e->fit_interval = 146 * 3600;
          e->start_time.tow -= 73 * 3600;
        } else {
          // LB: Short term extended operations
          // LB: IODC/IODE will be incremented on each new data set cutover (both from new upload and from automatic cutover)
          // LB: (Same rule as above about 8 LSBs)
          e->fit_interval = 6 * 3600;
          e->start_time.tow -= 3 * 3600;
        }
      }
      e->start_time = normalize_gps_time(e->start_time);

      // LB: Caculate the age of the emphemeris
      update_eph_age(e, e->start_time);

      /*printf("SV health 0x%X\n", e->sv_health);*/
      /*printf("IODC 1 0x%X\n", e->iodc_1);*/
      /*printf("IODE 2 0x%X\n", e->iode_2);*/
      /*printf("IODE 3 0x%X\n", e->iode_3);*/
      /*printf("URA (%d) %16g\n", e->ura_index, e->ura);*/
      /*printf("TGD %16g\n", e->tgd);*/
      /*printf("TOC %16u\n", (unsigned int)e->toc);*/
      /*printf("af2 %16g\n", e->af2);*/
      /*printf("af1 %16g\n", e->af1);*/
      /*printf("af0 %16g\n", e->af0);*/
      /*printf("CRS %16g\n", e->crs);*/
      /*printf("DN %16g\n", e->dn);*/
      /*printf("M0 %16g\n", e->m0);*/
      /*printf("CUC %16g\n", e->cuc);*/
      /*printf("Ecc %16g\n", e->ecc);*/
      /*printf("CUS %16g\n", e->cus);*/
      /*printf("SQRT A %16g\n", e->sqrta);*/
      /*printf("TOE %16u\n", (unsigned int)e->toe);*/
      /*printf("CIC %16g\n", e->cic);*/
      /*printf("omega0 %16g\n", e->omega0);*/
      /*printf("CIS %16g\n", e->cis);*/
      /*printf("Inc %16g\n", e->inc);*/
      /*printf("CRC %16g\n", e->crc);*/
      /*printf("W %16g\n", e->w);*/
      /*printf("omegadot %16g\n", e->omegadot);*/
      /*printf("inc_dot %16g\n", e->inc_dot);*/

      return 1;
    }
  } else {  // didn't get the subframe that we want next
      n->next_subframe_id = 1;      // Make sure we start again next time
      n->subframe_start_index = 0;  // Mark the subframe as processed
  }

  return 0;
}

