/*
Copyright(c) 2015-2026 Panos Karabelas

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

//= INCLUDES ============================
#include "../../Math/Matrix.h"
#include "../../Math/Transform.h"
#include "../../Rendering/Animation/AnimationClip.h"
#include "../../Rendering/Animation/SkeletalMeshBinding.h"
#include "../../Rendering/Animation/Skeleton.h"
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
//=======================================

struct aiScene;
struct aiNode;
struct aiMesh;

namespace spartan::animation_cooker
{
    struct SourceBoneNode
    {
        std::string name;
        const aiNode* node = nullptr;
        bool influences_vertices = false;
        bool required_for_evaluation = false;
    };

    struct SourceSkeletonGraph
    {
        std::unordered_map<const aiNode*, SourceBoneNode> nodes;
        std::unordered_map<std::string, const aiNode*> nodes_by_name;
        std::vector<std::string> runtime_bone_names;
    };

    struct SourceAnimationChannel
    {
        struct VectorKey
        {
            float time_seconds = 0.0f;
            math::Vector3 value = math::Vector3::Zero;
        };

        struct QuaternionKey
        {
            float time_seconds = 0.0f;
            math::Quaternion value = math::Quaternion::Identity;
        };

        std::string node_name;
        std::vector<VectorKey> position_keys;
        std::vector<QuaternionKey> rotation_keys;
        std::vector<VectorKey> scale_keys;
    };

    struct SourceAnimationClip
    {
        std::string name;
        float duration_seconds = 0.0f;
        float ticks_per_second = 0.0f;
        std::unordered_map<std::string, SourceAnimationChannel> channels_by_node;
    };

    struct SkeletonBuildResult
    {
        std::shared_ptr<Skeleton> skeleton;
        std::unordered_map<std::string, uint32_t> source_name_to_runtime_bone;
        std::unordered_map<std::string, math::Matrix> source_inverse_bind_overrides;
        std::vector<const aiNode*> runtime_source_nodes;
        math::Matrix skeleton_space_to_mesh_space = math::Matrix::Identity;
    };

    struct CookedSkinnedSubMesh
    {
        uint32_t source_mesh_index = std::numeric_limits<uint32_t>::max();
        uint32_t sub_mesh_index = 0;
        uint32_t vertex_input_offset = 0;
        uint32_t vertex_count = 0;
        std::vector<uint32_t> source_vertex_remap;
    };

    struct RuntimeBoneCandidate
    {
        std::string name;
        const aiNode* source_node = nullptr;
        int32_t parent_index = -1;
        math::Matrix source_global_bind = math::Matrix::Identity;
        math::Matrix engine_global_bind = math::Matrix::Identity;
    };

    math::Matrix ComputeSourceToEngineConversion(const aiScene* scene);
    math::Matrix ComputeSourceGlobalBind(const aiNode* node);
    SourceSkeletonGraph CollectSourceSkeletonGraph(const aiScene* scene);
    std::vector<SourceAnimationClip> CollectSourceAnimationClips(const aiScene* scene);

    SkeletonBuildResult BuildRuntimeSkeleton(
        const aiScene* scene,
        const SourceSkeletonGraph& source_graph,
        const math::Matrix& source_to_engine,
        const std::string& asset_name,
        const std::string& output_directory
    );

    bool BuildSkeletalMeshBinding(
        const aiScene* scene,
        const std::vector<CookedSkinnedSubMesh>& cooked_submeshes,
        const SkeletonBuildResult& skeleton_build,
        SkeletalMeshBinding& binding);
    void ValidateSourceBoneMappingOffsets(
        const aiScene* scene,
        const CookedSkinnedSubMesh& cooked_submesh,
        const SkeletonBuildResult& skeleton_build,
        const math::Matrix& source_to_engine,
        float epsilon = 0.01f
    );

    Transform EvaluateSourceNodeLocal(const aiNode* node, const SourceAnimationClip& clip, float time_seconds);
    math::Matrix EvaluateSourceNodeGlobal(const aiNode* node, const SourceAnimationClip& clip, float time_seconds, std::unordered_map<const aiNode*, math::Matrix>& cache);
    std::vector<float> CollectUniformSampleTimes(const SourceAnimationClip& clip, float sample_rate);
    std::unique_ptr<AnimationClip> CookRuntimeClip(
        const aiScene* scene,
        const SourceAnimationClip& source_clip,
        const SkeletonBuildResult& skeleton_build,
        const math::Matrix& source_to_engine,
        const std::string& output_directory
    );
}
