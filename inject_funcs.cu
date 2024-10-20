/*
 * SPDX-FileCopyrightText: Copyright (c) 2019 NVIDIA CORPORATION & AFFILIATES.
 * All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <stdio.h>
#include "common.h"
#include "utils/utils.h"
#include "utils/channel.hpp"

extern "C" __device__ __noinline__ void instrument_mem(int pred, int opcode_id,
                                                       uint64_t addr,
                                                       uint64_t grid_launch_id,
                                                       uint64_t pchannel_dev) {
    char formatted_msg[] = "your formtted message\n";
    /* if thread is predicated off, return */
    if (!pred) {
        return;
    }

    // formatted_msg = your_sprintf("opcode: %d, addr %x\n", opcode_id, addr);
    ((ChannelDev*)pchannel_dev)->push(formatted_msg, sizeof(formatted_msg));
}

extern "C" __device__ __noinline__ void instrument_call(int pred, int opcode_id,
                                                       uint64_t addr,
                                                       uint64_t grid_launch_id,
                                                       uint64_t pchannel_dev) {
    char formatted_msg[] = "your formtted message\n";
    //char formatted_msg[100];
    /* if thread is predicated off, return */
    if (!pred) {
        return;
    }
    call_trace_t ct;
    ct.opcode_id = opcode_id;
    int active_mask = __ballot_sync(__activemask(), 1);
    const int laneid = get_laneid();
    const int first_laneid = __ffs(active_mask) - 1;

    /* collect memory address information from other threads */
    //for (int i = 0; i < 32; i++) {
    //    ct.addrs[i] = __shfl_sync(active_mask, addr, i);
    //}
    ct.grid_launch_id = grid_launch_id;
    ct.cta_id_x = get_ctaid().x;
    ct.cta_id_y = get_ctaid().y;
    ct.cta_id_z = get_ctaid().z;
    ct.warp_id = get_warpid();
    int character_limit = 2048;
    int current_character = 0;
    while(character_limit > current_character)
    {
        unsigned long instruction = *((unsigned long *)addr+current_character*1);
        ct.call_sass[current_character] = instruction;
        current_character += 1;
    }
    //ct.call_sass[0] = (unsigned long) *((unsigned long*)addr);
    //ct.call_sass[0] = *((unsigned long*) 0x700b6f400);

    //while(character_limit > current_character)
    //{
    //    unsigned long instruction = *((unsigned long *)ct.addrs[0]+current_character*16);
    //    ct.call_sass[current_character] = instruction;
    //    current_character += 1;
    //}

    // formatted_msg = your_sprintf("opcode: %d, addr %x\n", opcode_id, addr);
    ((ChannelDev*)pchannel_dev)->push(&ct, sizeof(call_trace_t));
}
