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

#include "Renderer.h"
#include "Renderer_Buffers.h"
#include "../RHI/RHI_CommandList.h"
#include "../RHI/RHI_AccelerationStructure.h"
#include "../RHI/RHI_Shader.h"
#include "../World/Components/Renderable.h"
#include "../Geometry/Mesh.h"

//= NAMESPACES ===============
using namespace std;
using namespace spartan::math;
//============================

namespace spartan
{
    // Internal tracking structure for skinned entities to be processed this frame
    struct SkinningDrawable
    {
        uint32_t vertex_in_offset;   // into sb_skinning_vertices_in/indices/weights
        uint32_t vertex_out_offset;  // into uav_skinning_vertices_out
        uint32_t vertex_count;
        uint32_t bone_offset;        // into sb_skinning_bones
        uint32_t bone_count;
        RHI_AccelerationStructure* blas; // per-entity BLAS
        RHI_Buffer* index_buffer;       // for BLAS build
        uint32_t index_offset;
        uint32_t index_count;
    };

    void Renderer::Pass_GpuSkinning(RHI_CommandList* cmd)
    {
        // Gather skinned drawables from this frame's draw calls
        vector<SkinningDrawable> drawables;

        for (uint32_t i = 0; i < m_draw_call_count; i++)
        {
            const Renderer_DrawCall& dc = m_draw_calls[i];
            Renderable* renderable = dc.renderable;

            // Skip if not a skinned mesh
            if (!renderable->IsSkinned())
                continue;

            SkinningDrawable drawable{};
            drawable.vertex_in_offset  = renderable->GetSkinningVertexInputOffset();
            drawable.vertex_count      = renderable->GetSkinningVertexCount();
            if (drawable.vertex_count == 0 || renderable->GetBoneCount() == 0)
                continue;
            
            // Output offset was already allocated earlier in the frame (before UpdateDrawCalls)
            drawable.vertex_out_offset = renderable->GetSkinningVertexOutputOffset();
            
            drawable.bone_offset       = renderable->GetSkinningBoneOffset();
            drawable.bone_count        = renderable->GetBoneCount();
            drawable.blas              = renderable->GetSkinnedBlas();
            drawable.index_buffer      = renderable->GetIndexBuffer();
            drawable.index_offset      = renderable->GetIndexOffset();
            drawable.index_count       = renderable->GetIndexCount();

            // Skip if BLAS or index buffer doesn't exist yet
            if (!drawable.blas || !drawable.index_buffer)
                continue;

            drawables.push_back(drawable);
        }

        if (drawables.empty())
            return;

        // 1) Build jobs buffer (header + jobs)
        vector<Sb_SkinningJob> jobs;
        jobs.resize(drawables.size() + 1);

        uint32_t total_vertices = 0;
        jobs[0].vertex_start = static_cast<uint32_t>(drawables.size()); // job_count
        jobs[0].vertex_count = 0;                                       // total_vertices (filled later)

        for (uint32_t i = 0; i < drawables.size(); i++)
        {
            auto& d = drawables[i];

            Sb_SkinningJob j{};
            j.vertex_start      = total_vertices;
            j.vertex_count      = d.vertex_count;
            j.vertex_in_offset  = d.vertex_in_offset;
            j.vertex_out_offset = d.vertex_out_offset;
            j.bone_offset       = d.bone_offset;

            jobs[i + 1] = j;
            total_vertices += d.vertex_count;
        }
        jobs[0].vertex_count = total_vertices;

        if (total_vertices == 0)
            return;

        FrameResource& fr = m_frame_resources[m_frame_resource_index];

        // 2) Upload jobs
        RHI_Buffer* sb_skinning_jobs = fr.skinning_jobs.get();
        cmd->UpdateBuffer(sb_skinning_jobs, 0, static_cast<uint32_t>(jobs.size() * sizeof(Sb_SkinningJob)), jobs.data());

        // 3) Fill indirect dispatch args: ceil(total_vertices/64)
        RHI_Buffer* sb_skinning_dispatch_args = fr.skinning_dispatch_args.get();
        Sb_SkinningDispatchArgs args{};
        args.x = (total_vertices + 63u) / 64u;
        args.y = 1;
        args.z = 1;

        // Update the dispatch args buffer
        cmd->UpdateBuffer(sb_skinning_dispatch_args, 0, sizeof(args), &args);

        // 4) Barriers before compute reads
        cmd->InsertBarrier(sb_skinning_jobs);
        cmd->InsertBarrier(fr.skinning_bones.get());
        // Barrier for indirect dispatch args: ensure transfer write is visible before indirect execution
        cmd->InsertBarrier(RHI_Barrier::buffer_sync(sb_skinning_dispatch_args).from(RHI_Barrier_Scope::Transfer).to(RHI_Barrier_Scope::Indirect));
        cmd->FlushBarriers();

        // 5) Bind compute pipeline + resources
        RHI_Shader* shader = GetShader(Renderer_Shader::skinning_c);
        if (!shader || !shader->IsCompiled())
            return;

        RHI_Buffer* buf_skinning_bones       = fr.skinning_bones.get();
        RHI_Buffer* buf_skinning_vertices_in = GetBuffer(Renderer_Buffer::SkinningVerticesIn);
        RHI_Buffer* buf_skinning_indices     = GetBuffer(Renderer_Buffer::SkinningIndices);
        RHI_Buffer* buf_skinning_weights     = GetBuffer(Renderer_Buffer::SkinningWeights);
        RHI_Buffer* buf_skinning_vertices_out = GetBuffer(Renderer_Buffer::SkinningVerticesOut);
        if (!buf_skinning_bones || !buf_skinning_vertices_in || !buf_skinning_indices || 
            !buf_skinning_weights || !buf_skinning_vertices_out || !sb_skinning_jobs)
        {
            SP_LOG_WARNING("GPU skinning skipped: one or more buffers not initialized");
            return;
        }

        RHI_PipelineState pso;
        pso.name = "gpu_skinning";
        pso.shaders[RHI_Shader_Type::Compute] = shader;
        cmd->SetPipelineState(pso);

        cmd->SetBuffer(Renderer_BindingsSrv::skinning_jobs,        sb_skinning_jobs);
        cmd->SetBuffer(Renderer_BindingsSrv::skinning_bones,       buf_skinning_bones);
        cmd->SetBuffer(Renderer_BindingsSrv::skinning_vertices_in, buf_skinning_vertices_in);
        cmd->SetBuffer(Renderer_BindingsSrv::skinning_indices,     buf_skinning_indices);
        cmd->SetBuffer(Renderer_BindingsSrv::skinning_weights,     buf_skinning_weights);
        cmd->SetBuffer(Renderer_BindingsUav::skinning_vertices_out,                       buf_skinning_vertices_out);

        // 6) Single indirect dispatch for the whole frame
        cmd->DispatchIndirect(sb_skinning_dispatch_args, 0);

        // 7) Barriers: compute writes -> AS build reads AND vertex shader reads
        cmd->InsertBarrier(RHI_Barrier::buffer_sync(buf_skinning_vertices_out).from(RHI_Barrier_Scope::Compute).to(RHI_Barrier_Scope::AccelerationStructureBuild));
        cmd->InsertBarrier(RHI_Barrier::buffer_sync(buf_skinning_vertices_out).from(RHI_Barrier_Scope::Compute).to(RHI_Barrier_Scope::Graphics));
        cmd->FlushBarriers();

        // 8) Rebuild skinned BLAS (per-entity)
        for (const SkinningDrawable& d : drawables)
        {
            RHI_AccelerationStructureGeometry geom{};
            geom.vertex_buffer_address = buf_skinning_vertices_out->GetDeviceAddress() +
                                         uint64_t(d.vertex_out_offset) * sizeof(Sb_SkinnedVertex);
            geom.vertex_stride         = sizeof(Sb_SkinnedVertex);
            geom.vertex_format         = RHI_Format::R32G32B32_Float; // position at offset 0
            geom.max_vertex            = d.vertex_count - 1;

            geom.index_buffer_address  = d.index_buffer->GetDeviceAddress() + uint64_t(d.index_offset) * sizeof(uint32_t);
            geom.index_format          = RHI_Format::R32_Uint;
            geom.transparent           = false;

            uint32_t triangle_count = d.index_count / 3;

            vector<RHI_AccelerationStructureGeometry> geometries = { geom };
            vector<uint32_t> primitive_counts                    = { triangle_count };

            d.blas->BuildBottomLevel(cmd, geometries, primitive_counts);
        }

        // 9) Rebuild TLAS to pick up updated BLAS addresses
        // BuildOrUpdateTlas(cmd); - called later in frame
    }
}
