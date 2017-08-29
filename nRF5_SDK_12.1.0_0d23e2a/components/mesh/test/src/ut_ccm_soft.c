/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "packet.h"
#include "ccm_soft.h"

#define NET_PAYLOAD_INDEX 7
#define NET_MSG_LEN       4+2+2
#define MIC_INDEX NET_PAYLOAD_INDEX + NET_MSG_LEN
#define MIC_LEN 4

/* Bluetooth mesh test vectors */
static uint8_t net_unenc_packet[] = {0x4a, 0x07, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0xa1, 0x01, 0x3f, 0x5e, 0x05, 0xd2, 0xbc, 0x8c, 0x26, 0x03};
static uint8_t net_enc_packet[]   = {0x4a, 0x07, 0x01, 0x02, 0x03, 0x04, 0x05, 0x21, 0x31, 0x93, 0xf0, 0x4f, 0xba, 0x43, 0xee, 0xbc, 0x8c, 0x26, 0x03};
static uint8_t enc_key[] = {0xac, 0x16, 0x1f, 0x58, 0x9e, 0x5d, 0xe7, 0x45, 0x6d, 0x2c, 0x1a, 0x5f, 0x49, 0x72, 0x12, 0x6b};

static uint8_t net_nonce[] = {0x07, 0x01, 0x02, 0x03, 0x04, 0x05, 0x26, 0xd2, 0xa6, 0x9d, 0xa0, 0x82, 0xe0};

/* RFC test vectors */


/* Static variables */
static uint8_t out1[16];
static uint8_t mic[MIC_LEN];

static ccm_soft_data_t  enc_data = {.mic_len = MIC_LEN};
static ccm_soft_data_t  dec_data = {.mic_len = MIC_LEN};

void setUp(void)
{
    memset(out1, 0x00, 16);
    memset(mic, 0x00, MIC_LEN);
}

void tearDown(void)
{
}


void test_ccm_soft_encrypt(void)
{
    memcpy(out1, &net_unenc_packet[NET_PAYLOAD_INDEX], NET_MSG_LEN);

    enc_data.p_key   = enc_key;
    enc_data.p_nonce = net_nonce;
    enc_data.p_m     = out1;
    enc_data.m_len   = NET_MSG_LEN;
    enc_data.p_mic   = mic;
    enc_data.p_out   = out1;

    ccm_soft_encrypt(&enc_data);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&net_enc_packet[MIC_INDEX], mic, MIC_LEN);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&net_enc_packet[NET_PAYLOAD_INDEX], out1, NET_MSG_LEN);
}

void test_ccm_soft_decrypt(void)
{
    dec_data.p_key   = enc_key;
    dec_data.p_nonce = net_nonce;
    dec_data.p_m     = &net_enc_packet[NET_PAYLOAD_INDEX];
    dec_data.m_len   = NET_MSG_LEN;
    dec_data.p_mic   = &net_enc_packet[MIC_INDEX];
    dec_data.p_out   = out1;

    bool mic_passed = false;
    ccm_soft_decrypt(&dec_data, &mic_passed);

    TEST_ASSERT_EQUAL(true, mic_passed);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&net_unenc_packet[NET_PAYLOAD_INDEX], out1, NET_MSG_LEN);
}
