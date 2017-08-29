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

#include "log.h"
#include "utils.h"
#include "packet.h"
#include "enc.h"

#define ENC_TEST_S1_INPUT_DATA  { 't', 'e', 's', 't' }
#define ENC_TEST_S1_RESULT_DATA { 0xb7, 0x3c, 0xef, 0xbd, 0x64, 0x1e, 0xf2, 0xea, 0x59, 0x8c, 0x2b, 0x6e, 0xfb, 0x62, 0xf7, 0x9c }

#define ENC_TEST_K2_INPUT_NETKEY { 0xf7, 0xa2, 0xa4, 0x4f, 0x8e, 0x8a, 0x80, 0x29, 0x06, 0x4f, 0x17, 0x3d, 0xdc, 0x1e, 0x2b, 0x00 }
#define ENC_TEST_K2_INPUT_P      { 0x00 }
#define ENC_TEST_K2_RESULT_DATA  { 0x7f, \
    0x9f, 0x58, 0x91, 0x81, 0xa0, 0xf5, 0x0d, 0xe7, 0x3c, 0x80, 0x70, 0xc7, 0xa6, 0xd2, 0x7f, 0x46, \
    0x4c, 0x71, 0x5b, 0xd4, 0xa6, 0x4b, 0x93, 0x8f, 0x99, 0xb4, 0x53, 0x35, 0x16, 0x53, 0x12, 0x4f }

#define ENC_TEST_K3_INPUT_DATA  { 0xf7, 0xa2, 0xa4, 0x4f, 0x8e, 0x8a, 0x80, 0x29, 0x06, 0x4f, 0x17, 0x3d, 0xdc, 0x1e, 0x2b, 0x00 }
#define ENC_TEST_K3_RESULT_DATA { 0xff, 0x04, 0x69, 0x58, 0x23, 0x3d, 0xb0, 0x14 }

#define ENC_TEST_K4_INPUT_DATA_1  { 0x32, 0x16, 0xd1, 0x50, 0x98, 0x84, 0xb5, 0x33, 0x24, 0x85, 0x41, 0x79, 0x2b, 0x87, 0x7f, 0x98 }
#define ENC_TEST_K4_RESULT_DATA_1 0x01
#define ENC_TEST_K4_INPUT_DATA_2  { 0x63, 0x96, 0x47, 0x71, 0x73, 0x4f, 0xbd, 0x76, 0xe3, 0xb4, 0x05, 0x19, 0xd1, 0xd9, 0x4a, 0x48 }
#define ENC_TEST_K4_RESULT_DATA_2 0x14

static uint8_t m_app_nonce[] = {0x01, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00, 0x00, 0x00, 0x02};
static uint8_t m_net_nonce[] = {0x00, 0x07, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};

static uint32_t iv_index = LE2BE32(0x00000002);
static uint8_t m_output_buf[13];

/* length | adv_type | header | payload */
static uint8_t m_net_pkt[] = {0x15, 0xf0, 0x4a, 0x07, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x28, 0x0b, 0xf2, 0xb7, 0xd9, 0x18, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00};
packet_net_t * mp_packet;

void setUp()
{
    __LOG_INIT(LOG_SRC_ENC | LOG_SRC_TEST, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    mp_packet = (packet_net_t *) m_net_pkt;
}

void tearDown()
{
}

void test_enc_nonce_generate_app0(void)
{
    enc_nonce_generate(&mp_packet->header, iv_index, ENC_NONCE_APP, m_output_buf);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(m_app_nonce, m_output_buf, 13);
}

void test_enc_nonce_generate_net0(void)
{
    enc_nonce_generate(&mp_packet->header, iv_index, ENC_NONCE_NET, m_output_buf);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(m_net_nonce, m_output_buf, 13);
}

void test_s1(void)
{
    const uint8_t input[] = ENC_TEST_S1_INPUT_DATA;
    const uint8_t expected[] = ENC_TEST_S1_RESULT_DATA;
    uint8_t result[16];

    enc_s1(input, sizeof(input), result);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, result, sizeof(expected));
}

void test_k2_master(void)
{
    const uint8_t input_netkey[] = ENC_TEST_K2_INPUT_NETKEY;
    const uint8_t input_p[] = ENC_TEST_K2_INPUT_P;
    uint8_t expected[] = ENC_TEST_K2_RESULT_DATA;
    nrf_mesh_network_secmat_t result;

    enc_k2(input_netkey, input_p, sizeof(input_p), &result);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, (uint8_t *) &result, sizeof(expected));
}

void test_k3(void)
{
    const uint8_t input[] = ENC_TEST_K3_INPUT_DATA;
    const uint8_t expected[] = ENC_TEST_K3_RESULT_DATA;
    uint8_t result[8];

    enc_k3(input, result);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, result, sizeof(expected));
}

void test_k4_1(void)
{
    const uint8_t input[] = ENC_TEST_K4_INPUT_DATA_1;
    const uint8_t expected = ENC_TEST_K4_RESULT_DATA_1;
    uint8_t result;

    enc_k4(input, &result);
    TEST_ASSERT_EQUAL_UINT8(expected, result);
}

void test_k4_2(void)
{
    const uint8_t input[] = ENC_TEST_K4_INPUT_DATA_2;
    const uint8_t expected = ENC_TEST_K4_RESULT_DATA_2;
    uint8_t result;

    enc_k4(input, &result);
    TEST_ASSERT_EQUAL_UINT8(expected, result);
}

