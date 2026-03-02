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

// GPU skinning compute shader - processes all skinned meshes in one indirect dispatch

struct SkinningBone
{
    float4 r[3];        // current frame row-major 3x4 (rows 0-2, 4 floats each)
    float4 r_prev[3];   // previous frame row-major 3x4
};

struct SkinningJob
{
    uint vertex_start;      // header: job_count, job: global start (inclusive)
    uint vertex_count;      // header: total_vertices, job: count
    uint vertex_in_offset;  // element offset into sb_skinning_vertices_in/indices/weights
    uint vertex_out_offset; // element offset into uav_skinning_vertices_out
    uint bone_offset;       // element offset into sb_skinning_bones
    uint pad0, pad1, pad2;
};

struct VertexIn
{
    float3 position;
    float2 uv;
    float3 normal;
    float3 tangent;
};

struct SkinnedVertex
{
    float3 position;
    float  padding0;
    float3 position_prev;
    float  padding1;
    float3 normal;
    float  padding2;
    float4 tangent;
    float2 uv;
    float  padding3;
    float  padding4;
};

StructuredBuffer<SkinningJob>  sb_jobs          : register(t27);  // slot 27
StructuredBuffer<SkinningBone> sb_bones         : register(t28);  // slot 28
StructuredBuffer<VertexIn>     sb_vertices_in   : register(t29);  // slot 29
StructuredBuffer<uint>         sb_indices       : register(t30);  // slot 30 (4 x uint8 packed into uint32)
StructuredBuffer<float4>       sb_weights       : register(t31);  // slot 31

RWStructuredBuffer<SkinnedVertex> uav_vertices_out : register(u43); // slot 43

float3 mul_pos(float4 r[3], float3 p)
{
    float4 v = float4(p, 1.0);
    return float3(dot(r[0], v), dot(r[1], v), dot(r[2], v));
}

float3 mul_dir(float4 r[3], float3 d)
{
    return float3(dot(r[0].xyz, d), dot(r[1].xyz, d), dot(r[2].xyz, d));
}

uint find_job(uint global_vertex, uint job_count)
{
    // Binary search for job containing this vertex
    uint lo = 1;
    uint hi = job_count;
    while (lo < hi)
    {
        uint mid = (lo + hi + 1) >> 1;
        uint start = sb_jobs[mid].vertex_start;
        if (start <= global_vertex) lo = mid;
        else hi = mid - 1;
    }
    return lo;
}

[numthreads(64, 1, 1)]
void main_cs(uint3 dtid : SV_DispatchThreadID)
{
    uint job_count      = sb_jobs[0].vertex_start;
    uint total_vertices = sb_jobs[0].vertex_count;

    uint gv = dtid.x;
    if (gv >= total_vertices || job_count == 0)
        return;

    uint j = find_job(gv, job_count);
    SkinningJob job = sb_jobs[j];

    uint local = gv - job.vertex_start;
    if (local >= job.vertex_count)
        return;

    uint in_idx  = job.vertex_in_offset  + local;
    uint out_idx = job.vertex_out_offset + local;

    VertexIn vin = sb_vertices_in[in_idx];
    uint packed_bi = sb_indices[in_idx];
    uint4 bi = uint4(
        packed_bi & 0xFFu,
        (packed_bi >> 8u) & 0xFFu,
        (packed_bi >> 16u) & 0xFFu,
        (packed_bi >> 24u) & 0xFFu
    );
    float4 bw    = sb_weights[in_idx];

    float3 p  = 0;
    float3 pp = 0;
    float3 n  = 0;
    float3 t  = 0;

    [unroll] for (int k = 0; k < 4; k++)
    {
        float w = bw[k];
        if (w == 0.0) continue;
        uint bone_index = job.bone_offset + bi[k];

        SkinningBone b = sb_bones[bone_index];

        p  += w * mul_pos(b.r,      vin.position);
        pp += w * mul_pos(b.r_prev, vin.position);
        n  += w * mul_dir(b.r,      vin.normal);
        t  += w * mul_dir(b.r,      vin.tangent);
    }

    SkinnedVertex vout;
    vout.position      = p;
    vout.position_prev = pp;
    vout.normal        = normalize(n);
    vout.tangent       = float4(normalize(t), 1.0f);
    vout.uv            = vin.uv;
    vout.padding0      = 0;
    vout.padding1      = 0;
    vout.padding2      = 0;
    vout.padding3      = 0;
    vout.padding4      = 0;

    uav_vertices_out[out_idx] = vout;
}
