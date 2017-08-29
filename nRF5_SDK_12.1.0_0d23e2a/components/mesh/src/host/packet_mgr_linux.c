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

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include <pthread.h>

#include "packet_mgr.h"
#include "nrf_error.h"
#include "nrf_mesh_assert.h"
#include "utils.h"
#include "log.h"

#if PACKET_MGR_DEBUG_MODE
#   include <stdio.h>
#   define PACKET_MGR_LOG(...) printf(__VA_ARGS__)
#else
#   define PACKET_MGR_LOG(...)
#endif

#define PACKET_MGR_PACKET_MAGIC_1_BASE  0xbeeffeeb
#define PACKET_MGR_PACKET_MAGIC_2_BASE  0xaa55aa55

/**
 * Structure of the header for each allocated packet buffer.
 */
typedef struct
{
    uint32_t magic_1;            /**< Magic word 1, used for locating the header of a block. */
    uint32_t magic_2;            /**< Magic word 2, used for locating the header of a block. */
    pthread_mutex_t block_mutex; /**< Mutex ensuring atomic access to block headers. */
    uint16_t size;               /**< Size of the block in bytes */
    uint16_t ref_count;          /**< Reference count */
} buffer_header_t;

/********************
 * Static functions *
 ********************/

/**
 * Gets the buffer header from a packet buffer.
 * @param p_buffer Packet buffer.
 * @return Returns a pointer to the buffer header of the packet buffer.
 */
static inline buffer_header_t * buffer_header_get(packet_generic_t * p_buffer)
{
    return (buffer_header_t *) (((uint8_t *) p_buffer) - sizeof(buffer_header_t));
}

/**
 * Gets the start of the memory region of a buffer.
 * @param p_header Pointer to the buffer header.
 * @return Returns a pointer to the start of the memory for a buffer.
 */
static inline packet_generic_t * buffer_get_mem(buffer_header_t * p_header)
{
    return (packet_generic_t *) (((uint8_t *) p_header) + sizeof(buffer_header_t));
}

#if PACKET_MGR_DEBUG_MODE
/**
 * Gets the offset of a buffer relative to the start of the memory pool.
 * @param p_header header of the buffer.
 * @return offset of the buffer relative to the start of the memory pool.
 */
static inline unsigned int buffer_offset_get(buffer_header_t * p_header)
{
    return (unsigned int) p_header;
}
#endif

/******************************
 * Public interface functions *
 ******************************/

uint32_t packet_mgr_init(const nrf_mesh_init_params_t * p_init_params)
{
    PACKET_MGR_LOG("Packet manager initialized, using malloc() and free() as backend:\n");
    PACKET_MGR_LOG("\tmaximum buffer size: %d\n", PACKET_MGR_PACKET_MAXLEN);
    return NRF_SUCCESS;
}

uint32_t packet_mgr_alloc(packet_generic_t ** pp_buffer, uint16_t size)
{
    buffer_header_t * current;
    uint32_t retval = NRF_SUCCESS;

    if(size > PACKET_MGR_PACKET_MAXLEN)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    current = malloc(size + sizeof(buffer_header_t));
    if(current == NULL)
    {
        retval = NRF_ERROR_NO_MEM;
        goto _alloc_return;
    }

    current->size = size;
    current->ref_count = 1;
    current->magic_2 = current->size + PACKET_MGR_PACKET_MAGIC_2_BASE;
    current->magic_1 = current->size + current->magic_2 + PACKET_MGR_PACKET_MAGIC_1_BASE;
    *pp_buffer = buffer_get_mem(current);
    pthread_mutex_init(&current->block_mutex, NULL);

    PACKET_MGR_LOG("Allocated block of size %d (actual %d)\n", size, current->size);

_alloc_return:
    return retval;
}


void packet_mgr_incref(packet_generic_t * p_buffer)
{
    buffer_header_t * header = buffer_header_get(p_buffer);
    pthread_mutex_lock(&header->block_mutex);
    header->ref_count++;
    pthread_mutex_unlock(&header->block_mutex);

    PACKET_MGR_LOG("Increased refcount for buffer at offset %d to %d\n", buffer_offset_get(header), header->ref_count);
}

void packet_mgr_decref(packet_generic_t * p_buffer)
{
    buffer_header_t * header = buffer_header_get(p_buffer);

    pthread_mutex_lock(&header->block_mutex);
    NRF_MESH_ASSERT(header->ref_count != 0);

    header->ref_count--;
    if(header->ref_count != 0)
    {
        pthread_mutex_unlock(&header->block_mutex);
        PACKET_MGR_LOG("Decreased refcount for buffer at offset %d to %d\n", buffer_offset_get(header), header->ref_count);
    }
    else
    {
        pthread_mutex_unlock(&header->block_mutex);
        pthread_mutex_destroy(&header->block_mutex);
        free(header);
    }
}

uint32_t packet_mgr_bufstart_get(packet_generic_t * p_buffer, packet_generic_t ** pp_start)
{
    packet_generic_t * p_packet = NULL;
    for(int i = 0; i < PACKET_MGR_PACKET_MAXLEN; ++i)
    {
        buffer_header_t * p_header = (buffer_header_t *) (((uint8_t *) p_buffer) - i);
        if(p_header->magic_2 == PACKET_MGR_PACKET_MAGIC_2_BASE + p_header->size
                && p_header->magic_1 == PACKET_MGR_PACKET_MAGIC_1_BASE + p_header->magic_2 + p_header->size)
        {
            PACKET_MGR_LOG("Found buffer start at %x (%x)\n", p_header, buffer_get_mem(p_header));
            p_packet = buffer_get_mem(p_header);
            break;
        }
    }

    *pp_start = p_packet;
    return NRF_SUCCESS;
}

uint8_t packet_mgr_refcount_get(packet_generic_t * p_packet)
{
    buffer_header_t * header = buffer_header_get(p_packet);
    return header->ref_count;
}


