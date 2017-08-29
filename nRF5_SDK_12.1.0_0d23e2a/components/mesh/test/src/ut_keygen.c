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

#include <unity.h>
#include <cmock.h>

#include "nrf_mesh_keygen.h"
#include "enc_mock.h"



/*****************************************************************************
* Defines
*****************************************************************************/


/*****************************************************************************
* UT globals
*****************************************************************************/


/*****************************************************************************
* Mocks
*****************************************************************************/


/*****************************************************************************
* Setup functions
*****************************************************************************/

void setUp(void)
{
    CMOCK_SETUP(enc);
}

void tearDown(void)
{
    CMOCK_TEARDOWN();
}

/*****************************************************************************
* Tests
*****************************************************************************/
void test_keygen(void)
{
    uint8_t key[NRF_MESH_KEY_SIZE];
    /* Testing nrf_mesh_keygen_aid */
    uint8_t aid;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_aid(NULL, &aid));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_aid(key, NULL));
    /* AID generation: Section 3.8.6.2 in core spec d09r22*/
    enc_k4_Expect(key, &aid);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_keygen_aid(key, &aid));

    /* Testing nrf_mesh_keygen_network_secmat */
    nrf_mesh_network_secmat_t secmat;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_network_secmat(NULL, &secmat));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_network_secmat(key, NULL));
    /* NID, Encryption Key, and Privacy Key generation: Section 3.8.6.3.1 in core spec d09r22*/
    enc_k2_Expect(key, NULL, 1, &secmat);
    enc_k2_IgnoreArg_p_p();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_keygen_network_secmat(key, &secmat));

    /* Testing nrf_mesh_keygen_beacon_secmat */
    nrf_mesh_beacon_secmat_t beacon_secmat;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_beacon_secmat(NULL, &beacon_secmat));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_beacon_secmat(key, NULL));
    /* Network ID generation: Section 3.8.6.3.2 in core spec d09r22*/
    enc_k3_Expect(key, beacon_secmat.net_id);
    /* Beacon Key generation: Section 3.8.6.3.4 in core spec d09r22*/
    enc_s1_Expect(NULL, 4, NULL);
    enc_s1_IgnoreArg_p_in();
    enc_s1_IgnoreArg_p_out();
    enc_k1_Expect(key, NRF_MESH_KEY_SIZE, NULL, NULL, 6, beacon_secmat.key);
    enc_k1_IgnoreArg_p_salt();
    enc_k1_IgnoreArg_p_info();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_keygen_beacon_secmat(key, &beacon_secmat));


    /* Testing nrf_mesh_keygen_identitykey */
    uint8_t identity_key[NRF_MESH_KEY_SIZE];
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_identitykey(NULL, identity_key));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_identitykey(key, NULL));
    enc_s1_Expect(NULL, 4, NULL);
    enc_s1_IgnoreArg_p_out();
    enc_s1_IgnoreArg_p_in();
    enc_k1_Expect(key, NRF_MESH_KEY_SIZE, NULL, NULL, 6, identity_key);
    enc_k1_IgnoreArg_p_info();
    enc_k1_IgnoreArg_p_salt();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_keygen_identitykey(key, identity_key));

    /* Testing nrf_mesh_keygen_virtual_address*/
    uint16_t address;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_virtual_address(NULL, &address));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_virtual_address(key, NULL));
    enc_s1_Expect(NULL, 4, NULL);
    enc_s1_IgnoreArg_p_out();
    enc_s1_IgnoreArg_p_in();
    enc_aes_cmac_Expect(NULL, key, NRF_MESH_KEY_SIZE, NULL);
    enc_aes_cmac_IgnoreArg_p_key();
    enc_aes_cmac_IgnoreArg_p_result();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_keygen_virtual_address(key, &address));
    /* See Figure 3-4: Virtual Address Format in mesh core spec d09r22 */
    TEST_ASSERT_TRUE((address & (3 << 14)) == (1<<15));


}
