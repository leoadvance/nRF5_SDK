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


/** Override enable/disable IRQ macros */
#define TOOLCHAIN_H__
#include <stdint.h>
#include <string.h>

#include <unity.h>

uint32_t m_was_masked;
void (*irq_enable_cb)(void);

#include "fifo.h"
#include "nrf_error.h"

fifo_t* mp_fifo;

typedef struct
{
    uint8_t param1;
    uint32_t param2;
    uint8_t param3;
} test_struct_t;

test_struct_t m_race_struct;

void setUp(void)
{
    m_was_masked = 0;
    mp_fifo = NULL;
    irq_enable_cb = NULL;
    m_race_struct.param1 = 11;
    m_race_struct.param2 = 12;
    m_race_struct.param3 = 13;
}

void tearDown(void)
{

}

void race_condition_write(void)
{
    irq_enable_cb = NULL; /* only trigger once per call */
    if (mp_fifo == NULL)
    {
        TEST_FAIL_MESSAGE("REWRITE THE TEST: Didn't set mp_fifo\n");
    }

    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_push(mp_fifo, &m_race_struct));
}

void race_condition_read(void)
{
    irq_enable_cb = NULL; /* only trigger once per call */
    if (mp_fifo == NULL)
    {
        TEST_FAIL_MESSAGE("REWRITE THE TEST: Didn't set mp_fifo\n");
    }
    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_pop(mp_fifo, &m_race_struct));
}

/*************** tests ***************/

void test_normal_usage(void)
{
    fifo_t fifo;
    test_struct_t buf[16];
    fifo.elem_array = buf;
    fifo.elem_size = sizeof(test_struct_t);
    fifo.array_len = 16;

    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_init(&fifo));
    TEST_ASSERT_EQUAL(0, fifo.head);
    TEST_ASSERT_EQUAL(0, fifo.tail);
    TEST_ASSERT_EQUAL(16, fifo.array_len);
    TEST_ASSERT_EQUAL(12, fifo.elem_size);
    TEST_ASSERT_EQUAL(buf, fifo.elem_array);

    test_struct_t test_elem = {1, 2, 3};
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, fifo_pop(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, fifo_peek(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, fifo_peek_at(&fifo, &test_elem, 1));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_push(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(1, test_elem.param1);
    TEST_ASSERT_EQUAL(2, test_elem.param2);
    TEST_ASSERT_EQUAL(3, test_elem.param3);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_pop(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(1, test_elem.param1);
    TEST_ASSERT_EQUAL(2, test_elem.param2);
    TEST_ASSERT_EQUAL(3, test_elem.param3);
    for (uint32_t i = 0; i < 16; ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_push(&fifo, &test_elem));
    }
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, fifo_push(&fifo, &test_elem));
    for (uint32_t i = 0; i < 16; ++i)
    {
        memset(&test_elem, 0, sizeof(test_struct_t));
        TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_peek_at(&fifo, &test_elem, i));
        TEST_ASSERT_EQUAL(1, test_elem.param1);
        TEST_ASSERT_EQUAL(2, test_elem.param2);
        TEST_ASSERT_EQUAL(3, test_elem.param3);
    }
    for (uint32_t i = 0; i < 16; ++i)
    {
        memset(&test_elem, 0, sizeof(test_struct_t));
        TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_pop(&fifo, &test_elem));
        TEST_ASSERT_EQUAL(1, test_elem.param1);
        TEST_ASSERT_EQUAL(2, test_elem.param2);
        TEST_ASSERT_EQUAL(3, test_elem.param3);
    }
    memset(&test_elem, 0, sizeof(test_struct_t));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, fifo_pop(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(0, test_elem.param1);
    TEST_ASSERT_EQUAL(0, test_elem.param2);
    TEST_ASSERT_EQUAL(0, test_elem.param3);
}

void test_memcpy(void)
{
    fifo_t fifo;
    test_struct_t buf[16];
    fifo.elem_array = buf;
    fifo.elem_size = sizeof(test_struct_t);
    fifo.array_len = 16;

    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_init(&fifo));
    TEST_ASSERT_EQUAL(0, fifo.head);
    TEST_ASSERT_EQUAL(0, fifo.tail);
    TEST_ASSERT_EQUAL(16, fifo.array_len);
    TEST_ASSERT_EQUAL(12, fifo.elem_size);
    TEST_ASSERT_EQUAL(buf, fifo.elem_array);

    test_struct_t test_elem = {1, 2, 3};
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, fifo_pop(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, fifo_peek(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, fifo_peek_at(&fifo, &test_elem, 1));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_push(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(1, test_elem.param1);
    TEST_ASSERT_EQUAL(2, test_elem.param2);
    TEST_ASSERT_EQUAL(3, test_elem.param3);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_pop(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(1, test_elem.param1);
    TEST_ASSERT_EQUAL(2, test_elem.param2);
    TEST_ASSERT_EQUAL(3, test_elem.param3);
    for (uint32_t i = 0; i < 16; ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_push(&fifo, &test_elem));
    }
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, fifo_push(&fifo, &test_elem));
    for (uint32_t i = 0; i < 16; ++i)
    {
        memset(&test_elem, 0, sizeof(test_struct_t));
        TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_peek_at(&fifo, &test_elem, i));
        TEST_ASSERT_EQUAL(1, test_elem.param1);
        TEST_ASSERT_EQUAL(2, test_elem.param2);
        TEST_ASSERT_EQUAL(3, test_elem.param3);
    }
    for (uint32_t i = 0; i < 16; ++i)
    {
        memset(&test_elem, 0, sizeof(test_struct_t));
        TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_pop(&fifo, &test_elem));
        TEST_ASSERT_EQUAL(1, test_elem.param1);
        TEST_ASSERT_EQUAL(2, test_elem.param2);
        TEST_ASSERT_EQUAL(3, test_elem.param3);
    }
    memset(&test_elem, 0, sizeof(test_struct_t));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, fifo_pop(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(0, test_elem.param1);
    TEST_ASSERT_EQUAL(0, test_elem.param2);
    TEST_ASSERT_EQUAL(0, test_elem.param3);
}

void test_failures(void)
{
    fifo_t fifo;
    test_struct_t buf[16];
    fifo.elem_array = buf;
    fifo.elem_size = sizeof(test_struct_t);
    fifo.array_len = 17; /* NOT POWER OF TWO, INVALID */
    test_struct_t test_elem = {1, 2, 3};

    /* test init function */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, fifo_init(NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, fifo_init(&fifo));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, fifo_push(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, fifo_pop(&fifo, &test_elem));
    fifo.array_len = 16;
    fifo.elem_array = NULL;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, fifo_init(&fifo));
    fifo.elem_array = buf;
    fifo.elem_size = 0;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, fifo_init(&fifo));

    /* test access functions */
    fifo.elem_size = sizeof(test_struct_t);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_init(&fifo));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_push(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_pop(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, fifo_push(NULL, &test_elem));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, fifo_push(&fifo, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, fifo_pop(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_push(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_pop(&fifo, NULL)); /* don't really have to copy into a buffer*/
    memset(&test_elem, 0, sizeof(test_struct_t));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, fifo_pop(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(0, test_elem.param1);
    TEST_ASSERT_EQUAL(0, test_elem.param2);
    TEST_ASSERT_EQUAL(0, test_elem.param3);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, fifo_push(&fifo, &test_elem));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, fifo_peek(&fifo, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, fifo_peek(NULL, &test_elem));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, fifo_peek_at(&fifo, NULL, 1));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, fifo_peek_at(NULL, &test_elem, 1));
}

