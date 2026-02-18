/*
Copyright(c) 2015-2026 Panos Karabelas

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// gpu bc3 texture compression using amd compressonator kernels
// each 64-thread group compresses 4 bc blocks (4x4 pixels each)

#ifndef ASPM_HLSL
#define ASPM_HLSL
#endif

#include "compressonator/bcn_common_kernel.h"
#include "common_resources.hlsl"

// compression parameters packed into the push constant values:
// values[0].x = num_block_x     (as uint via asuint)
// values[0].y = num_total_blocks (as uint via asuint)
// values[0].z = quality          (float)
// values[0].w = mip_level        (as uint via asuint)
// values[1].x = buffer_offset    (as uint via asuint)
uint  get_num_block_x()      { return asuint(pass_get_f3_value().x); }
uint  get_num_total_blocks() { return asuint(pass_get_f3_value().y); }
float get_quality()          { return pass_get_f3_value().z; }
uint  get_mip_level()        { return asuint(buffer_pass.values[0].w); }
uint  get_buffer_offset()    { return asuint(buffer_pass.values[1].x); }

#define MAX_USED_THREAD   16
#define BLOCK_IN_GROUP    4
#define THREAD_GROUP_SIZE 64
#define BLOCK_SIZE_Y      4
#define BLOCK_SIZE_X      4

groupshared float4 shared_temp[THREAD_GROUP_SIZE];

[numthreads(THREAD_GROUP_SIZE, 1, 1)]
void main_cs(uint GI : SV_GroupIndex, uint3 groupID : SV_GroupID)
{
    uint num_block_x      = get_num_block_x();
    uint num_total_blocks = get_num_total_blocks();
    uint mip_level        = get_mip_level();

    uint blockInGroup = GI / MAX_USED_THREAD;
    uint blockID      = groupID.x * BLOCK_IN_GROUP + blockInGroup;
    uint pixelBase    = blockInGroup * MAX_USED_THREAD;
    uint pixelInBlock = GI - pixelBase;

    if (blockID >= num_total_blocks)
        return;

    uint block_y = blockID / num_block_x;
    uint block_x = blockID - block_y * num_block_x;
    uint base_x  = block_x * BLOCK_SIZE_X;
    uint base_y  = block_y * BLOCK_SIZE_Y;

    // load 4x4 pixel block from the source texture at the specified mip level
    if (pixelInBlock < 16)
    {
        shared_temp[GI] = float4(tex.Load(uint3(base_x + pixelInBlock % 4, base_y + pixelInBlock / 4, mip_level)));
    }

    GroupMemoryBarrierWithGroupSync();

    // thread 0 of each block compresses and writes the result
    if (pixelInBlock == 0)
    {
        float3 blockRGB[16];
        float  blockA[16];
        for (int i = 0; i < 16; i++)
        {
            blockRGB[i].x = shared_temp[pixelBase + i].x;
            blockRGB[i].y = shared_temp[pixelBase + i].y;
            blockRGB[i].z = shared_temp[pixelBase + i].z;
            blockA[i]     = shared_temp[pixelBase + i].w;
        }

        tex_compress_out[get_buffer_offset() + blockID] = CompressBlockBC3_UNORM(blockRGB, blockA, get_quality(), false);
    }
}
