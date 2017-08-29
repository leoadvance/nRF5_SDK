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
#ifndef MESH_TOOLCHAIN_H__
#define MESH_TOOLCHAIN_H__

#include "nrf_mesh_hw.h"

void toolchain_init_irqs(void);

#define POPCOUNT64(n) __builtin_popcountll(n)

#if defined(UNIT_TEST)
    #define _DISABLE_IRQS(_was_masked) (void)_was_masked /* avoid "not used" warning */
    #define _ENABLE_IRQS(_was_masked)
    #define _GET_LR(lr) lr = (uint32_t) __builtin_return_address(0);
    #define FORCEINLINE inline
#elif defined(MTT_TEST)
    #include <pthread.h>
    extern pthread_mutex_t irq_mutex;
    #define _GET_LR(lr) lr = (uint32_t) __builtin_return_address(0);
    #define _DISABLE_IRQS(_was_masked) pthread_mutex_lock(&irq_mutex); (void) _was_masked;
    #define _ENABLE_IRQS(_was_masked) pthread_mutex_unlock(&irq_mutex);
    #define FORCEINLINE inline
#elif defined(__CC_ARM)
/** ARMCC specific implementations */

/** Disable all interrupts and get whether it was masked. */
    #define _DISABLE_IRQS(_was_masked) _was_masked = __disable_irq()

/** Enable all interrupts if they weren't masked. */
    #define _ENABLE_IRQS(_was_masked) do{ if (!(_was_masked)) { __enable_irq(); } } while(0)


/** Get the value of the link register. */
    #define _GET_LR(lr) do { lr = __return_address(); } while (0)

/** Force inline attribute for functions. */
    #define FORCEINLINE __forceinline

#elif defined(__GNUC__)
/** GCC specific implementations */

/** Disable all interrupts and get whether it was masked. */
    #define _DISABLE_IRQS(_was_masked) do{ \
            __ASM volatile ("MRS %0, primask" : "=r" (_was_masked) );\
            __ASM volatile ("cpsid i" : : : "memory");\
        } while(0)

/** Enable all interrupts if they weren't masked. */
    #define _ENABLE_IRQS(_was_masked) do{ if (!(_was_masked)) { __enable_irq(); } } while(0)

/** Get the value of the link register. */
    #define _GET_LR(lr) do { lr = (uint32_t) __builtin_return_address(0); } while (0)

/** Force inline attribute for functions. */
    #define FORCEINLINE __attribute__((always_inline))

#endif


#endif /* TOOLCHAIN_H__ */
