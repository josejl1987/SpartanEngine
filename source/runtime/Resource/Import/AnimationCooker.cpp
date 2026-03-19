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

#include "pch.h"
#include "AnimationCooker.h"
#include "../../Resource/Animation/AnimationAssetValidation.h"
#include "../../Resource/Animation/AnimationClipWriter.h"
#include "../../Resource/Animation/AnimationLimits.h"
#include "../../Resource/Animation/SkeletonWriter.h"
#include "../../Logging/Log.h"
#include "../../Profiling/Profiler.h"
#include <algorithm>
#include <array>
#include <string_view>
#include <span>
SP_WARNINGS_OFF
#include "assimp/scene.h"
SP_WARNINGS_ON

namespace
{
    struct Influence
    {
        uint16_t bone = 0;
        float weight = 0.0f;
    };

    struct VertexInfluences
    {
        static constexpr size_t capacity = 8;

        std::array<Influence, capacity> data = {};
        size_t count = 0;

        void add(const Influence& influence)
        {
            if (count < capacity)
            {
                data[count++] = influence;
                return;
            }

            auto weakest = std::min_element(data.begin(), data.begin() + count, [](const Influence& a, const Influence& b)
            {
                if (a.weight != b.weight)
                    return a.weight < b.weight;
                return a.bone > b.bone;
            });

            const bool replace_weakest =
                influence.weight > weakest->weight ||
                (influence.weight == weakest->weight && influence.bone < weakest->bone);

            if (replace_weakest)
                *weakest = influence;
        }
    };

    struct SourceAnimationChannelCursor
    {
        size_t position = 0;
        size_t rotation = 0;
        size_t scale = 0;
    };

    static void collect_node_lookup(const aiNode* node, std::unordered_map<std::string, std::vector<const aiNode*>>& lookup)
    {
        if (!node)
            return;

        const std::string node_name = node->mName.C_Str();
        lookup[node_name].push_back(node);

        for (uint32_t i = 0; i < node->mNumChildren; i++)
        {
            collect_node_lookup(node->mChildren[i], lookup);
        }
    }

    static const spartan::animation_cooker::SourceBoneNode* find_source_bone_node(
        const spartan::animation_cooker::SourceSkeletonGraph& graph,
        const std::string& name)
    {
        auto name_it = graph.nodes_by_name.find(name);
        if (name_it == graph.nodes_by_name.end())
            return nullptr;

        auto node_it = graph.nodes.find(name_it->second);
        if (node_it == graph.nodes.end())
            return nullptr;

        return &node_it->second;
    }

    static spartan::math::Vector3 to_vector3(const aiVector3D& value)
    {
        return spartan::math::Vector3(value.x, value.y, value.z);
    }

    static spartan::math::Quaternion to_quaternion(const aiQuaternion& value)
    {
        return spartan::math::Quaternion(value.x, value.y, value.z, value.w);
    }

    static spartan::math::Matrix to_matrix(const aiMatrix4x4& transform)
    {
        return spartan::math::Matrix(
            transform.a1, transform.b1, transform.c1, transform.d1,
            transform.a2, transform.b2, transform.c2, transform.d2,
            transform.a3, transform.b3, transform.c3, transform.d3,
            transform.a4, transform.b4, transform.c4, transform.d4
        );
    }

    static const aiMesh* resolve_source_mesh(const aiScene* scene, const spartan::animation_cooker::CookedSkinnedSubMesh& cooked_submesh)
    {
        if (!scene || cooked_submesh.source_mesh_index >= scene->mNumMeshes)
            return nullptr;

        return scene->mMeshes[cooked_submesh.source_mesh_index];
    }

    template <typename TKey, typename TValue, typename TNormalize, typename TInterpolate>
    static TValue sample_keys(
        const std::vector<TKey>& keys,
        const float time_seconds,
        size_t* cursor,
        const TValue& default_value,
        TNormalize&& normalize,
        TInterpolate&& interpolate)
    {
        if (keys.empty())
            return default_value;

        if (keys.size() == 1 || time_seconds <= keys.front().time_seconds)
        {
            if (cursor) *cursor = 0;
            return normalize(keys.front().value);
        }

        if (time_seconds >= keys.back().time_seconds)
        {
            if (cursor) *cursor = keys.size() - 2;
            return normalize(keys.back().value);
        }

        size_t index = 0;
        if (cursor)
        {
            const size_t last_interval = keys.size() - 2;
            index = std::min(*cursor, last_interval);

            while (index < last_interval && time_seconds > keys[index + 1].time_seconds)
                ++index;

            while (index > 0 && time_seconds < keys[index].time_seconds)
                --index;

            *cursor = index;
        }
        else
        {
            const auto it = std::lower_bound(keys.begin(), keys.end(), time_seconds, [](const TKey& key, const float time)
            {
                return key.time_seconds < time;
            });
            index = std::distance(keys.begin(), it) - 1;
        }

        const TKey& a = keys[index];
        const TKey& b = keys[index + 1];
        const float dt = b.time_seconds - a.time_seconds;
        if (dt <= 0.0f)
            return normalize(b.value);

        const float alpha = std::clamp((time_seconds - a.time_seconds) / dt, 0.0f, 1.0f);
        return interpolate(a.value, b.value, alpha);
    }

    static spartan::math::Vector3 sample_vector_keys(
        const std::vector<spartan::animation_cooker::SourceAnimationChannel::VectorKey>& keys,
        const float time_seconds,
        size_t* cursor)
    {
        return sample_keys(
            keys,
            time_seconds,
            cursor,
            spartan::math::Vector3::Zero,
            [](const spartan::math::Vector3& value) { return value; },
            [](const spartan::math::Vector3& a, const spartan::math::Vector3& b, const float alpha)
            {
                return spartan::math::Vector3::Lerp(a, b, alpha);
            }
        );
    }

    static spartan::math::Quaternion sample_quaternion_keys(
        const std::vector<spartan::animation_cooker::SourceAnimationChannel::QuaternionKey>& keys,
        const float time_seconds,
        size_t* cursor)
    {
        return sample_keys(
            keys,
            time_seconds,
            cursor,
            spartan::math::Quaternion::Identity,
            [](spartan::math::Quaternion value)
            {
                value.Normalize();
                return value;
            },
            [](const spartan::math::Quaternion& a, const spartan::math::Quaternion& b, const float alpha)
            {
                spartan::math::Quaternion q0 = a;
                spartan::math::Quaternion q1 = b;
                if (spartan::math::Quaternion::Dot(q0, q1) < 0.0f)
                    q1 = -q1;

                return spartan::math::Quaternion::Slerp(q0, q1, alpha);
            }
        );
    }

    static spartan::Transform evaluate_source_node_local(
        const aiNode* node,
        const spartan::animation_cooker::SourceAnimationClip& clip,
        const float time_seconds,
        std::unordered_map<const aiNode*, SourceAnimationChannelCursor>* cursors)
    {
        if (!node)
            return { spartan::math::Vector3::Zero, spartan::math::Quaternion::Identity, spartan::math::Vector3::One };

        spartan::Transform local = spartan::Transform::FromMatrix(to_matrix(node->mTransformation));
        auto it = clip.channels_by_node.find(node->mName.C_Str());
        if (it == clip.channels_by_node.end())
        {
            return local;
        }

        size_t* pos_cursor   = nullptr;
        size_t* rot_cursor   = nullptr;
        size_t* scale_cursor = nullptr;

        if (cursors)
        {
            auto& c      = (*cursors)[node];
            pos_cursor   = &c.position;
            rot_cursor   = &c.rotation;
            scale_cursor = &c.scale;
        }

        const spartan::animation_cooker::SourceAnimationChannel& channel = it->second;
        if (!channel.position_keys.empty())
            local.position = sample_vector_keys(channel.position_keys, time_seconds, pos_cursor);
        if (!channel.rotation_keys.empty())
            local.rotation = sample_quaternion_keys(channel.rotation_keys, time_seconds, rot_cursor);
        if (!channel.scale_keys.empty())
            local.scale = sample_vector_keys(channel.scale_keys, time_seconds, scale_cursor);

        return local;
    }

    static spartan::math::Matrix evaluate_source_node_global(
        const aiNode* node,
        const spartan::animation_cooker::SourceAnimationClip& clip,
        const float time_seconds,
        std::unordered_map<const aiNode*, spartan::math::Matrix>& cache,
        std::unordered_map<const aiNode*, SourceAnimationChannelCursor>* cursors)
    {
        if (!node)
            return spartan::math::Matrix::Identity;

        auto it = cache.find(node);
        if (it != cache.end())
            return it->second;

        const spartan::Transform local = evaluate_source_node_local(node, clip, time_seconds, cursors);
        const spartan::math::Matrix local_matrix = local.ToMatrix();
        const spartan::math::Matrix parent_global = evaluate_source_node_global(node->mParent, clip, time_seconds, cache, cursors);
        const spartan::math::Matrix global = local_matrix * parent_global;
        cache[node] = global;
        return global;
    }
}

namespace spartan::animation_cooker
{
    math::Matrix ComputeSourceToEngineConversion(const aiScene* scene)
    {
        static_cast<void>(scene);
        // Identity is intentional for now: the importer currently relies on Assimp post-process flags
        // and assumes the asset basis is already compatible with the runtime basis.
        return math::Matrix::Identity;
    }

    SourceSkeletonGraph CollectSourceSkeletonGraph(const aiScene* scene)
    {
        SourceSkeletonGraph graph;
        if (!scene || !scene->mRootNode)
            return graph;

        graph.runtime_bone_names.reserve(64);

        std::unordered_map<std::string, std::vector<const aiNode*>> node_lookup;
        collect_node_lookup(scene->mRootNode, node_lookup);

        for (uint32_t mesh_index = 0; mesh_index < scene->mNumMeshes; mesh_index++)
        {
            const aiMesh* mesh = scene->mMeshes[mesh_index];
            if (!mesh || mesh->mNumBones == 0)
                continue;

            for (uint32_t bone_index = 0; bone_index < mesh->mNumBones; bone_index++)
            {
                const aiBone* bone = mesh->mBones[bone_index];
                if (!bone)
                    continue;

                const std::string bone_name = bone->mName.C_Str();
                if (bone_name.empty())
                    continue;

                auto node_it = node_lookup.find(bone_name);
                if (node_it == node_lookup.end() || node_it->second.empty())
                {
                    SP_LOG_WARNING("AnimationCooker: source bone '%s' has no matching aiNode", bone_name.c_str());
                    continue;
                }

                if (node_it->second.size() > 1)
                {
                    SP_LOG_ERROR("AnimationCooker: source bone '%s' matches %zu aiNode entries; names must be unique", bone_name.c_str(), node_it->second.size());
                    return {};
                }

                const aiNode* bone_node = node_it->second.front();
                SourceBoneNode& deform = graph.nodes[bone_node];
                if (deform.name.empty())
                    deform.name = bone_name;

                auto [name_it, inserted] = graph.nodes_by_name.emplace(bone_name, bone_node);
                if (!inserted && name_it->second != bone_node)
                {
                    SP_LOG_ERROR("AnimationCooker: source node name '%s' resolves to multiple aiNode pointers", bone_name.c_str());
                    return {};
                }

                if (!deform.influences_vertices)
                    graph.runtime_bone_names.push_back(bone_name);

                deform.influences_vertices = true;
                deform.required_for_evaluation = true;

                deform.node = bone_node;

                const aiNode* ancestor = deform.node->mParent;
                while (ancestor)
                {
                    const std::string ancestor_name = ancestor->mName.C_Str();
                    if (ancestor_name.empty())
                    {
                        ancestor = ancestor->mParent;
                        continue;
                    }

                    SourceBoneNode& entry = graph.nodes[ancestor];
                    if (entry.name.empty())
                        entry.name = ancestor_name;

                    if (!entry.node)
                        entry.node = ancestor;

                    auto [ancestor_name_it, ancestor_inserted] = graph.nodes_by_name.emplace(ancestor_name, ancestor);
                    if (!ancestor_inserted && ancestor_name_it->second != ancestor)
                    {
                        SP_LOG_ERROR("AnimationCooker: source node name '%s' resolves to multiple aiNode pointers", ancestor_name.c_str());
                        return {};
                    }

                    entry.required_for_evaluation = true;
                    ancestor = ancestor->mParent;
                }
            }
        }

        return graph;
    }

    std::vector<SourceAnimationClip> CollectSourceAnimationClips(const aiScene* scene)
    {
        std::vector<SourceAnimationClip> clips;
        if (!scene || scene->mNumAnimations == 0)
            return clips;

        clips.reserve(scene->mNumAnimations);

        for (uint32_t animation_index = 0; animation_index < scene->mNumAnimations; animation_index++)
        {
            const aiAnimation* animation = scene->mAnimations[animation_index];
            if (!animation)
                continue;

            const float ticks_per_second = animation->mTicksPerSecond != 0.0 ? static_cast<float>(animation->mTicksPerSecond) : 25.0f;
            const float inv_ticks_per_second = ticks_per_second > 0.0f ? 1.0f / ticks_per_second : 0.0f;

            SourceAnimationClip clip;
            clip.name = animation->mName.length != 0 ? animation->mName.C_Str() : ("anim_" + std::to_string(animation_index));
            clip.duration_seconds = static_cast<float>(animation->mDuration) * inv_ticks_per_second;
            clip.ticks_per_second = ticks_per_second;

            for (uint32_t channel_index = 0; channel_index < animation->mNumChannels; channel_index++)
            {
                const aiNodeAnim* channel = animation->mChannels[channel_index];
                if (!channel)
                    continue;

                const std::string node_name = channel->mNodeName.C_Str();
                SourceAnimationChannel source_channel;
                source_channel.node_name = node_name;
                source_channel.position_keys.reserve(channel->mNumPositionKeys);
                source_channel.rotation_keys.reserve(channel->mNumRotationKeys);
                source_channel.scale_keys.reserve(channel->mNumScalingKeys);

                for (uint32_t i = 0; i < channel->mNumPositionKeys; i++)
                {
                    const aiVectorKey& key = channel->mPositionKeys[i];
                    source_channel.position_keys.push_back({ static_cast<float>(key.mTime) * inv_ticks_per_second, to_vector3(key.mValue) });
                }

                for (uint32_t i = 0; i < channel->mNumRotationKeys; i++)
                {
                    const aiQuatKey& key = channel->mRotationKeys[i];
                    math::Quaternion rotation = to_quaternion(key.mValue);
                    rotation.Normalize();
                    source_channel.rotation_keys.push_back({ static_cast<float>(key.mTime) * inv_ticks_per_second, rotation });
                }

                for (uint32_t i = 0; i < channel->mNumScalingKeys; i++)
                {
                    const aiVectorKey& key = channel->mScalingKeys[i];
                    source_channel.scale_keys.push_back({ static_cast<float>(key.mTime) * inv_ticks_per_second, to_vector3(key.mValue) });
                }

                auto compare_time = [](const auto& a, const auto& b) { return a.time_seconds < b.time_seconds; };
                std::sort(source_channel.position_keys.begin(), source_channel.position_keys.end(), compare_time);
                std::sort(source_channel.rotation_keys.begin(), source_channel.rotation_keys.end(), compare_time);
                std::sort(source_channel.scale_keys.begin(), source_channel.scale_keys.end(), compare_time);

                for (size_t i = 0; i < source_channel.rotation_keys.size(); i++)
                {
                    math::Quaternion& q = source_channel.rotation_keys[i].value;
                    q.Normalize();

                    if (i > 0)
                    {
                        const math::Quaternion& previous = source_channel.rotation_keys[i - 1].value;
                        if (math::Quaternion::Dot(previous, q) < 0.0f)
                        {
                            q = -q;
                        }
                    }
                }

                clip.channels_by_node[source_channel.node_name] = std::move(source_channel);
            }

            clips.push_back(std::move(clip));
        }

        return clips;
    }

    math::Matrix ComputeSourceGlobalBind(const aiNode* node)
    {
        math::Matrix global = math::Matrix::Identity;
        const aiNode* current = node;
        std::vector<const aiNode*> chain;

        while (current)
        {
            chain.push_back(current);
            current = current->mParent;
        }

        for (auto it = chain.rbegin(); it != chain.rend(); ++it)
        {
            global = to_matrix((*it)->mTransformation) * global;
        }

        return global;
    }

    SkeletonBuildResult BuildRuntimeSkeleton(
        const aiScene* scene,
        const SourceSkeletonGraph& source_graph,
        const math::Matrix& source_to_engine,
        const std::string& asset_name,
        const std::string& output_directory
    )
    {
        SP_PROFILE_CPU();
        SkeletonBuildResult result;

        if (scene)
        {
            const math::Matrix source_to_engine_inverse = math::Matrix::Invert(source_to_engine);
            for (uint32_t mesh_index = 0; mesh_index < scene->mNumMeshes; mesh_index++)
            {
                const aiMesh* mesh = scene->mMeshes[mesh_index];
                if (!mesh)
                    continue;

                for (uint32_t bone_index = 0; bone_index < mesh->mNumBones; bone_index++)
                {
                    const aiBone* bone = mesh->mBones[bone_index];
                    if (!bone)
                        continue;

                    const std::string bone_name = bone->mName.C_Str();
                    if (bone_name.empty() || result.source_inverse_bind_overrides.find(bone_name) != result.source_inverse_bind_overrides.end())
                        continue;

                    const math::Matrix source_inverse_bind = to_matrix(bone->mOffsetMatrix);
                    result.source_inverse_bind_overrides[bone_name] = source_to_engine_inverse * source_inverse_bind;
                }
            }
        }

        math::Matrix skeleton_space_to_mesh_space = math::Matrix::Identity;
        bool skeleton_space_binding_found = false;
        for (const std::string& deform_name : source_graph.runtime_bone_names)
        {
            auto override_it = result.source_inverse_bind_overrides.find(deform_name);
            const SourceBoneNode* source_bone = find_source_bone_node(source_graph, deform_name);
            if (override_it == result.source_inverse_bind_overrides.end() || !source_bone || !source_bone->node)
                continue;

            const math::Matrix desired_global_bind = math::Matrix::Invert(override_it->second);
            const math::Matrix source_global_bind  = ComputeSourceGlobalBind(source_bone->node) * source_to_engine;
            skeleton_space_to_mesh_space           = desired_global_bind * math::Matrix::Invert(source_global_bind);
            skeleton_space_binding_found           = true;
            break;
        }
        if (skeleton_space_binding_found)
        {
            for (const std::string& deform_name : source_graph.runtime_bone_names)
            {
                auto override_it = result.source_inverse_bind_overrides.find(deform_name);
                const SourceBoneNode* source_bone = find_source_bone_node(source_graph, deform_name);
                if (override_it == result.source_inverse_bind_overrides.end() || !source_bone || !source_bone->node)
                    continue;

                const math::Matrix desired_global_bind = math::Matrix::Invert(override_it->second);
                const math::Matrix source_global_bind  = ComputeSourceGlobalBind(source_bone->node) * source_to_engine;
                const math::Matrix derived_binding     = desired_global_bind * math::Matrix::Invert(source_global_bind);
                if (!derived_binding.IsClose(skeleton_space_to_mesh_space, 1e-4f))
                {
                    SP_LOG_ERROR(
                        "AnimationCooker: skeleton '%s' has non-uniform skeleton-to-mesh basis between deform bones",
                        asset_name.c_str()
                    );
                    return {};
                }
            }
        }
        result.skeleton_space_to_mesh_space = skeleton_space_to_mesh_space;

        std::vector<RuntimeBoneCandidate> candidates;
        candidates.reserve(source_graph.runtime_bone_names.size() * 2);
        std::unordered_map<std::string, uint32_t> candidate_index_by_name;

        auto try_add_candidate = [&](const std::string& name) -> bool
        {
            if (candidate_index_by_name.find(name) != candidate_index_by_name.end())
                return true;

            const SourceBoneNode* source_bone = find_source_bone_node(source_graph, name);
            if (!source_bone || !source_bone->node)
            {
                SP_LOG_ERROR("AnimationCooker: runtime deform bone '%s' has no source aiNode", name.c_str());
                return false;
            }

            RuntimeBoneCandidate candidate;
            candidate.name = name;
            candidate.source_node = source_bone->node;
            candidate.source_global_bind = ComputeSourceGlobalBind(candidate.source_node);
            candidate.engine_global_bind = candidate.source_global_bind * source_to_engine * skeleton_space_to_mesh_space;

            candidate_index_by_name[name] = static_cast<uint32_t>(candidates.size());
            candidates.push_back(candidate);
            return true;
        };

        for (const std::string& name : source_graph.runtime_bone_names)
        {
            if (!try_add_candidate(name))
                return {};
        }

        for (const std::string& deform_name : source_graph.runtime_bone_names)
        {
            const SourceBoneNode* deform_bone = find_source_bone_node(source_graph, deform_name);
            if (!deform_bone || !deform_bone->node)
                continue;

            const aiNode* ancestor = deform_bone->node->mParent;
            while (ancestor)
            {
                const std::string ancestor_name = ancestor->mName.C_Str();
                const SourceBoneNode* source_bone = find_source_bone_node(source_graph, ancestor_name);
                if (source_bone && source_bone->required_for_evaluation && source_bone->node)
                {
                    if (!try_add_candidate(ancestor_name))
                        return {};
                }
                ancestor = ancestor->mParent;
            }
        }

        for (uint32_t i = 0; i < static_cast<uint32_t>(candidates.size()); i++)
        {
            const aiNode* ancestor = candidates[i].source_node ? candidates[i].source_node->mParent : nullptr;
            while (ancestor)
            {
                const std::string ancestor_name = ancestor->mName.C_Str();
                auto candidate_it = candidate_index_by_name.find(ancestor_name);
                if (candidate_it != candidate_index_by_name.end())
                {
                    candidates[i].parent_index = static_cast<int32_t>(candidate_it->second);
                    break;
                }
                ancestor = ancestor->mParent;
            }
        }

        std::vector<uint32_t> roots;
        for (uint32_t i = 0; i < static_cast<uint32_t>(candidates.size()); i++)
        {
            if (candidates[i].parent_index < 0)
                roots.push_back(i);
        }

        if (roots.empty())
        {
            SP_LOG_ERROR("AnimationCooker: skeleton '%s' has no root bones", asset_name.c_str());
            return {};
        }

        if (roots.size() > 1)
        {
            RuntimeBoneCandidate virtual_root;
            virtual_root.name = "__spartan_virtual_root__";
            virtual_root.source_node = nullptr;
            virtual_root.parent_index = -1;
            virtual_root.source_global_bind = math::Matrix::Identity;
            virtual_root.engine_global_bind = math::Matrix::Identity;

            const uint32_t virtual_root_index = static_cast<uint32_t>(candidates.size());
            candidates.push_back(virtual_root);
            for (const uint32_t root_index : roots)
            {
                candidates[root_index].parent_index = static_cast<int32_t>(virtual_root_index);
            }

            roots.clear();
            roots.push_back(virtual_root_index);
        }

        const uint32_t candidate_count = static_cast<uint32_t>(candidates.size());
        if (candidate_count == 0 || candidate_count > static_cast<uint32_t>(std::numeric_limits<int16_t>::max()))
            return {};

        std::vector<int32_t> parents(candidates.size(), -1);
        std::vector<math::Matrix> global_bind(candidates.size(), math::Matrix::Identity);
        std::vector<math::Matrix> inverse_bind(candidates.size(), math::Matrix::Identity);

        for (uint32_t i = 0; i < candidate_count; i++)
        {
            parents[i] = candidates[i].parent_index;

            if (auto inverse_it = result.source_inverse_bind_overrides.find(candidates[i].name); inverse_it != result.source_inverse_bind_overrides.end())
            {
                inverse_bind[i] = inverse_it->second;
                global_bind[i]  = math::Matrix::Invert(inverse_bind[i]);
            }
            else
            {
                global_bind[i]  = candidates[i].engine_global_bind;
                inverse_bind[i] = math::Matrix::Invert(global_bind[i]);
            }
        }

        std::vector<uint32_t> order;
        order.reserve(candidate_count);
        std::vector<std::vector<uint32_t>> children(candidate_count);
        for (uint32_t bone = 0; bone < candidate_count; ++bone)
        {
            const int32_t parent = parents[bone];
            if (parent >= 0)
            {
                children[static_cast<uint32_t>(parent)].push_back(bone);
            }
        }

        std::vector<uint32_t> traversal_stack;
        traversal_stack.reserve(candidate_count);
        for (auto root_it = roots.rbegin(); root_it != roots.rend(); ++root_it)
        {
            traversal_stack.push_back(*root_it);
        }

        while (!traversal_stack.empty())
        {
            const uint32_t bone = traversal_stack.back();
            traversal_stack.pop_back();
            order.push_back(bone);

            const auto& bone_children = children[bone];
            for (auto child_it = bone_children.rbegin(); child_it != bone_children.rend(); ++child_it)
            {
                traversal_stack.push_back(*child_it);
            }
        }

        if (order.size() != candidate_count)
            return {};

        auto skeleton = std::make_shared<Skeleton>();
        skeleton->Allocate(static_cast<uint16_t>(candidate_count));

        std::vector<uint32_t> remap(candidate_count, 0);
        result.runtime_source_nodes.reserve(candidate_count);
        for (uint32_t new_index = 0; new_index < candidate_count; ++new_index)
        {
            const uint32_t old_index = order[new_index];
            remap[old_index] = new_index;

            const RuntimeBoneCandidate& candidate = candidates[old_index];
            result.source_name_to_runtime_bone[candidate.name] = new_index;
            result.runtime_source_nodes.push_back(candidate.source_node);

            const int32_t old_parent = parents[old_index];
            const int32_t new_parent = old_parent < 0 ? -1 : static_cast<int32_t>(remap[static_cast<uint32_t>(old_parent)]);
            skeleton->parent_indices[new_index] = static_cast<int16_t>(new_parent);

            Transform local_bind = old_parent < 0
                ? Transform::FromMatrix(global_bind[old_index])
                : Transform::FromMatrix(global_bind[old_index] * math::Matrix::Invert(global_bind[static_cast<uint32_t>(old_parent)]));

            if (!local_bind.position.IsFinite() ||
                !local_bind.scale.IsFinite() ||
                !std::isfinite(local_bind.rotation.x) || !std::isfinite(local_bind.rotation.y) ||
                !std::isfinite(local_bind.rotation.z) || !std::isfinite(local_bind.rotation.w) ||
                !inverse_bind[old_index].IsFinite())
            {
                return {};
            }

            skeleton->bind_positions[new_index] = local_bind.position;
            skeleton->bind_rotations[new_index] = local_bind.rotation;
            skeleton->bind_scales[new_index] = local_bind.scale;
            skeleton->inverse_bind_matrices[new_index] = inverse_bind[old_index];
        }

        std::string error;
        if (!ValidateSkeleton(*skeleton, &error))
        {
            SP_LOG_ERROR("AnimationCooker: cooked skeleton failed validation: %s", error.c_str());
            return {};
        }

        if (!output_directory.empty())
        {
            const std::string file_name = asset_name + ".skeleton";
            const std::string output_path = output_directory + file_name;
            if (!SkeletonWriter::WriteToFile(*skeleton, output_path))
            {
                SP_LOG_WARNING("AnimationCooker: failed to save cooked skeleton to '%s'", output_path.c_str());
            }
        }

        result.skeleton = skeleton;
        return result;
    }

    Transform EvaluateSourceNodeLocal(const aiNode* node, const SourceAnimationClip& clip, float time_seconds)
    {
        return evaluate_source_node_local(node, clip, time_seconds, nullptr);
    }

    math::Matrix EvaluateSourceNodeGlobal(const aiNode* node, const SourceAnimationClip& clip, float time_seconds, std::unordered_map<const aiNode*, math::Matrix>& cache)
    {
        return evaluate_source_node_global(node, clip, time_seconds, cache, nullptr);
    }

    std::vector<float> CollectUniformSampleTimes(const SourceAnimationClip& clip, const float sample_rate)
    {
        const uint32_t sample_count = std::max(1u, static_cast<uint32_t>(std::ceil(std::max(clip.duration_seconds, 0.0f) * sample_rate)) + 1u);
        std::vector<float> times(sample_count, 0.0f);
        const float sample_interval = 1.0f / sample_rate;
        for (uint32_t i = 0; i < sample_count; ++i)
        {
            times[i] = std::min(static_cast<float>(i) * sample_interval, clip.duration_seconds);
        }

        if (!times.empty())
            times.back() = clip.duration_seconds;

        return times;
    }

    std::unique_ptr<AnimationClip> CookRuntimeClip(
        const aiScene* scene,
        const SourceAnimationClip& source_clip,
        const SkeletonBuildResult& skeleton_build,
        const math::Matrix& source_to_engine,
        const std::string& output_directory)
    {
        SP_PROFILE_CPU();
        static_cast<void>(scene);
        if (!skeleton_build.skeleton)
            return nullptr;

        const Skeleton& skeleton = *skeleton_build.skeleton;
        const uint32_t bone_count = static_cast<uint32_t>(skeleton.joint_count);

        auto clip = std::make_unique<AnimationClip>();
        clip->duration_seconds = source_clip.duration_seconds;
        // Foundation first: keep the cooked format uniformly sampled and deterministic.
        clip->sample_rate      = source_clip.ticks_per_second > 0.0f ? source_clip.ticks_per_second : 30.0f;
        clip->joint_count      = bone_count;

        auto& position_stream = clip->position_stream;
        auto& rotation_stream = clip->rotation_stream;
        auto& scale_stream = clip->scale_stream;
        auto& base_local_positions = clip->base_local_positions;
        auto& base_local_rotations = clip->base_local_rotations;
        auto& base_local_scales = clip->base_local_scales;
        auto& sampled_bones = clip->sampled_bones;

        const std::vector<float> sample_times = CollectUniformSampleTimes(source_clip, clip->sample_rate);
        if (sample_times.empty())
            return clip;

        const size_t sample_count = sample_times.size();
        clip->sample_count = static_cast<uint32_t>(sample_count);
        std::vector<Transform> sampled_locals(bone_count * sample_count);

        std::unordered_map<const aiNode*, SourceAnimationChannelCursor> channel_cursors;
        channel_cursors.reserve(source_clip.channels_by_node.size());

        std::unordered_map<const aiNode*, math::Matrix> cache;
        cache.reserve(skeleton_build.runtime_source_nodes.size());

        std::vector<math::Matrix> runtime_globals(bone_count, math::Matrix::Identity);

        for (size_t sample_index = 0; sample_index < sample_count; ++sample_index)
        {
            const float time_seconds = sample_times[sample_index];
            cache.clear();

            for (uint32_t bone = 0; bone < bone_count; bone++)
            {
                const aiNode* source_node = bone < skeleton_build.runtime_source_nodes.size() ? skeleton_build.runtime_source_nodes[bone] : nullptr;
                if (!source_node)
                {
                    runtime_globals[bone] = math::Matrix::Identity;
                }
                else
                {
                    const math::Matrix source_global = evaluate_source_node_global(source_node, source_clip, time_seconds, cache, &channel_cursors);
                    runtime_globals[bone] = source_global * source_to_engine * skeleton_build.skeleton_space_to_mesh_space;
                }
            }

            for (uint32_t bone = 0; bone < bone_count; bone++)
            {
                const int32_t parent = skeleton.parent_indices[bone];
                const math::Matrix local_matrix = parent < 0 ? runtime_globals[bone] : runtime_globals[bone] * math::Matrix::Invert(runtime_globals[parent]);
                Transform local = Transform::FromMatrix(local_matrix);
                sampled_locals[bone * sample_count + sample_index] = local;
            }
        }

        const std::span<const math::Vector3> bind_positions = skeleton.bind_positions;
        const std::span<const math::Quaternion> bind_rotations = skeleton.bind_rotations;
        const std::span<const math::Vector3> bind_scales = skeleton.bind_scales;
        const float epsilon = 1e-4f;

        auto append_track = [&](const uint32_t bone, const auto& keys, auto&& is_bind_equal, auto&& is_constant, auto&& push_constant,
                                auto& track_stream) -> bool
        {
            if (is_bind_equal())
                return true;

            if (is_constant())
            {
                push_constant(keys.front().value);
                return true;
            }

            const uint64_t first_sample = track_stream.values.size();
            const uint64_t sample_count = keys.size();
            if (first_sample > std::numeric_limits<uint32_t>::max() || sample_count > std::numeric_limits<uint32_t>::max())
                return false;

            track_stream.channels.push_back({ bone, static_cast<uint32_t>(first_sample), static_cast<uint32_t>(sample_count) });
            for (const auto& key : keys)
            {
                track_stream.values.push_back(key.value);
            }

            return true;
        };

        auto is_vector_equal = [&](const math::Vector3& a, const math::Vector3& b) { return (a - b).LengthSquared() <= epsilon * epsilon; };
        auto is_quaternion_equal = [&](const math::Quaternion& a, const math::Quaternion& b) { return 1.0f - std::abs(math::Quaternion::Dot(a, b)) <= epsilon; };

        for (uint32_t bone = 0; bone < bone_count; bone++)
        {
            std::vector<SourceAnimationChannel::VectorKey> position_keys;
            std::vector<SourceAnimationChannel::QuaternionKey> rotation_keys;
            std::vector<SourceAnimationChannel::VectorKey> scale_keys;
            position_keys.reserve(sample_times.size());
            rotation_keys.reserve(sample_times.size());
            scale_keys.reserve(sample_times.size());

            for (size_t i = 0; i < sample_times.size(); i++)
            {
                const Transform& t = sampled_locals[bone * sample_count + i];
                position_keys.push_back({ sample_times[i], t.position });
                math::Quaternion q = t.rotation;
                if (!rotation_keys.empty() && math::Quaternion::Dot(rotation_keys.back().value, q) < 0.0f)
                    q = -q;
                rotation_keys.push_back({ sample_times[i], q });
                scale_keys.push_back({ sample_times[i], t.scale });
            }

            const auto& bind_pos = bind_positions[bone];
            const auto& bind_rot = bind_rotations[bone];
            const auto& bind_scl = bind_scales[bone];

            if (!append_track(
                    bone, position_keys,
                    [&]() { return std::all_of(position_keys.begin(), position_keys.end(), [&](const auto& k) { return is_vector_equal(k.value, bind_pos); }); },
                    [&]() { return !position_keys.empty() && std::all_of(position_keys.begin(), position_keys.end(), [&](const auto& k) { return is_vector_equal(k.value, position_keys.front().value); }); },
                    [&](const math::Vector3& value) { position_stream.constants.push_back({ bone, value }); },
                    position_stream
                ) ||
                !append_track(
                    bone, rotation_keys,
                    [&]() { return std::all_of(rotation_keys.begin(), rotation_keys.end(), [&](const auto& k) { return is_quaternion_equal(k.value, bind_rot); }); },
                    [&]() { return !rotation_keys.empty() && std::all_of(rotation_keys.begin(), rotation_keys.end(), [&](const auto& k) { return is_quaternion_equal(k.value, rotation_keys.front().value); }); },
                    [&](const math::Quaternion& value)
                    {
                        math::Quaternion normalized = value;
                        normalized.Normalize();
                        rotation_stream.constants.push_back({ bone, normalized });
                    },
                    rotation_stream
                ) ||
                !append_track(
                    bone, scale_keys,
                    [&]() { return std::all_of(scale_keys.begin(), scale_keys.end(), [&](const auto& k) { return is_vector_equal(k.value, bind_scl); }); },
                    [&]() { return !scale_keys.empty() && std::all_of(scale_keys.begin(), scale_keys.end(), [&](const auto& k) { return is_vector_equal(k.value, scale_keys.front().value); }); },
                    [&](const math::Vector3& value) { scale_stream.constants.push_back({ bone, value }); },
                    scale_stream
                ))
            {
                return nullptr;
            }
        }

        base_local_positions.assign(bind_positions.begin(), bind_positions.end());
        base_local_rotations.assign(bind_rotations.begin(), bind_rotations.end());
        base_local_scales.assign(bind_scales.begin(), bind_scales.end());

        for (const ConstantPosition& channel : position_stream.constants)
        {
            if (channel.bone_index < bone_count)
                base_local_positions[channel.bone_index] = channel.value;
        }

        for (const ConstantRotation& channel : rotation_stream.constants)
        {
            if (channel.bone_index < bone_count)
                base_local_rotations[channel.bone_index] = channel.value;
        }

        for (const ConstantScale& channel : scale_stream.constants)
        {
            if (channel.bone_index < bone_count)
                base_local_scales[channel.bone_index] = channel.value;
        }

        std::vector<uint8_t> animated_mask(bone_count, 0);
        auto mark_animated = [&](const auto& channels)
        {
            for (const auto& channel : channels)
                animated_mask[channel.bone_index] = 1;
        };

        mark_animated(position_stream.channels);
        mark_animated(rotation_stream.channels);
        mark_animated(scale_stream.channels);

        sampled_bones.clear();
        sampled_bones.reserve(position_stream.channels.size() + rotation_stream.channels.size() + scale_stream.channels.size());
        for (uint32_t bone = 0; bone < bone_count; ++bone)
        {
            if (animated_mask[bone] != 0)
                sampled_bones.push_back(bone);
        }

        std::string error;
        if (!ValidateClip(*clip, skeleton, &error))
        {
            SP_LOG_ERROR("AnimationCooker: cooked clip '%s' failed validation: %s", source_clip.name.c_str(), error.c_str());
            return nullptr;
        }

        if (!output_directory.empty())
        {
            const std::string output_path = output_directory + source_clip.name + ".clipanim";
            if (!AnimationClipWriter::WriteToFile(*clip, output_path))
            {
                SP_LOG_WARNING("AnimationCooker: failed to save cooked clip to '%s'", output_path.c_str());
            }
        }

        return clip;
    }

    bool BuildSkeletalMeshBinding(const aiScene* scene, const std::vector<CookedSkinnedSubMesh>& cooked_submeshes, const SkeletonBuildResult& skeleton_build, SkeletalMeshBinding& binding)
    {
        binding.Clear();

        for (const CookedSkinnedSubMesh& cooked_submesh : cooked_submeshes)
        {
            const aiMesh* source_mesh = resolve_source_mesh(scene, cooked_submesh);
            if (!source_mesh)
            {
                SP_LOG_ERROR("AnimationCooker: failed to resolve source mesh for cooked skinned sub-mesh %u", cooked_submesh.sub_mesh_index);
                return false;
            }

            if (source_mesh->mNumBones == 0)
            {
                SP_LOG_ERROR("AnimationCooker: source mesh for sub-mesh %u has no bones but was marked as skinned", cooked_submesh.sub_mesh_index);
                return false;
            }

            std::vector<VertexInfluences> per_vertex_influences(cooked_submesh.vertex_count);
            std::vector<uint32_t> mesh_bone_to_runtime_bone(source_mesh->mNumBones, std::numeric_limits<uint32_t>::max());

            for (uint32_t bone_index = 0; bone_index < source_mesh->mNumBones; bone_index++)
            {
                const aiBone* source_bone = source_mesh->mBones[bone_index];
                if (!source_bone)
                    continue;

                const std::string source_bone_name = source_bone->mName.C_Str();
                const auto mapping_it = skeleton_build.source_name_to_runtime_bone.find(source_bone_name);
                if (mapping_it == skeleton_build.source_name_to_runtime_bone.end())
                {
                    SP_LOG_ERROR("AnimationCooker: source bone '%s' has no runtime mapping", source_bone_name.c_str());
                    return false;
                }

                const uint32_t runtime_bone_index = mapping_it->second;
                if (runtime_bone_index > static_cast<uint32_t>(std::numeric_limits<uint16_t>::max()))
                {
                    SP_LOG_ERROR("AnimationCooker: runtime bone index %u exceeds uint16 range for bone '%s'", runtime_bone_index, source_bone_name.c_str());
                    return false;
                }

                mesh_bone_to_runtime_bone[bone_index] = runtime_bone_index;
            }

            for (uint32_t bone_index = 0; bone_index < source_mesh->mNumBones; bone_index++)
            {
                const uint32_t runtime_bone_index = mesh_bone_to_runtime_bone[bone_index];
                if (runtime_bone_index == std::numeric_limits<uint32_t>::max())
                    continue;

                const aiBone* source_bone = source_mesh->mBones[bone_index];
                if (!source_bone)
                    continue;

                for (uint32_t weight_index = 0; weight_index < source_bone->mNumWeights; weight_index++)
                {
                    const aiVertexWeight& source_weight = source_bone->mWeights[weight_index];
                    uint32_t cooked_vertex_index = source_weight.mVertexId;
                    if (!cooked_submesh.source_vertex_remap.empty())
                    {
                        if (source_weight.mVertexId >= cooked_submesh.source_vertex_remap.size())
                        {
                            SP_LOG_ERROR("AnimationCooker: vertex index %u out of remap range for bone '%s'", source_weight.mVertexId, source_bone->mName.C_Str());
                            return false;
                        }

                        cooked_vertex_index = cooked_submesh.source_vertex_remap[source_weight.mVertexId];
                        if (cooked_vertex_index == std::numeric_limits<uint32_t>::max() || cooked_vertex_index >= cooked_submesh.vertex_count)
                        {
                            SP_LOG_ERROR("AnimationCooker: vertex index %u remapped to invalid or out-of-range index %u for bone '%s'", source_weight.mVertexId, cooked_vertex_index, source_bone->mName.C_Str());
                            return false;
                        }
                    }
                    else if (source_weight.mVertexId >= cooked_submesh.vertex_count)
                    {
                        SP_LOG_ERROR(
                            "AnimationCooker: vertex influence for sub-mesh %u, vertex %u is out of range (vertex_count=%u) for bone '%s'",
                            cooked_submesh.sub_mesh_index,
                            source_weight.mVertexId,
                            cooked_submesh.vertex_count,
                            source_bone->mName.C_Str()
                        );
                        return false;
                    }

                    per_vertex_influences[cooked_vertex_index].add(
                        {
                            static_cast<uint16_t>(runtime_bone_index),
                            source_weight.mWeight
                        }
                    );
                }
            }

            SkeletalMeshSection section;
            section.sub_mesh_index = cooked_submesh.sub_mesh_index;
            section.vertex_input_offset = cooked_submesh.vertex_input_offset;
            section.vertex_count = cooked_submesh.vertex_count;
            section.influences.resize(cooked_submesh.vertex_count);

            std::unordered_map<uint16_t, uint16_t> skeleton_to_palette;
            skeleton_to_palette.reserve(source_mesh->mNumBones);
            section.palette_bone_indices.reserve(source_mesh->mNumBones);
            bool palette_limit_exceeded = false;

            auto get_palette_index = [&](const uint16_t runtime_bone_index) -> uint16_t
            {
                auto it = skeleton_to_palette.find(runtime_bone_index);
                if (it != skeleton_to_palette.end())
                    return it->second;

                if (section.palette_bone_indices.size() >= animation_limits::joint_count)
                {
                    palette_limit_exceeded = true;
                    return 0;
                }

                const uint16_t palette_index = static_cast<uint16_t>(section.palette_bone_indices.size());
                section.palette_bone_indices.push_back(runtime_bone_index);
                skeleton_to_palette[runtime_bone_index] = palette_index;
                return palette_index;
            };

            for (uint32_t vertex_index = 0; vertex_index < cooked_submesh.vertex_count; vertex_index++)
            {
                VertexInfluences& influences = per_vertex_influences[vertex_index];
                const size_t influence_count = std::min<size_t>(4, influences.count);
                std::partial_sort(influences.data.begin(), influences.data.begin() + influence_count, influences.data.begin() + influences.count, [](const Influence& a, const Influence& b)
                {
                    if (a.weight != b.weight)
                        return a.weight > b.weight;
                    return a.bone < b.bone;
                });

                float weight_sum = 0.0f;
                for (size_t slot = 0; slot < influence_count; ++slot)
                {
                    const Influence& influence = influences.data[slot];
                    if (std::isfinite(influence.weight) && influence.weight > 0.0f)
                        weight_sum += influence.weight;
                }

                if (influence_count > 0 && weight_sum <= 0.0f)
                {
                    SP_LOG_ERROR(
                        "AnimationCooker: sub-mesh %u vertex %u has bone influences but zero total weight",
                        cooked_submesh.sub_mesh_index,
                        vertex_index
                    );
                    binding.Clear();
                    return false;
                }

                SkeletalVertexInfluence packed = {};
                for (uint32_t slot = 0; slot < static_cast<uint32_t>(influence_count); slot++)
                {
                    const Influence& influence = influences.data[slot];
                    packed.bone_indices[slot] = get_palette_index(influence.bone);
                    if (palette_limit_exceeded)
                        break;
                    packed.bone_weights[slot] = (weight_sum > 0.0f && std::isfinite(influence.weight) && influence.weight > 0.0f) ? influence.weight / weight_sum : 0.0f;
                }

                if (palette_limit_exceeded)
                {
                    SP_LOG_ERROR(
                        "AnimationCooker: sub-mesh %u uses %zu bone palette entries, exceeding the runtime limit of %u",
                        cooked_submesh.sub_mesh_index,
                        section.palette_bone_indices.size(),
                        animation_limits::joint_count
                    );
                    binding.Clear();
                    return false;
                }

                section.influences[vertex_index] = packed;
            }

            binding.GetSectionsUnsafeMutable().push_back(std::move(section));
        }

        if (!binding.IsValid())
        {
            SP_LOG_ERROR("AnimationCooker: BuildSkeletalMeshBinding produced an invalid binding");
            return false;
        }

        return true;
    }

    void ValidateSourceBoneMappingOffsets(
        const aiScene* scene,
        const CookedSkinnedSubMesh& cooked_submesh,
        const SkeletonBuildResult& skeleton_build,
        const math::Matrix& source_to_engine,
        float epsilon)
    {
        const aiMesh* source_mesh = resolve_source_mesh(scene, cooked_submesh);
        if (!source_mesh || !skeleton_build.skeleton)
            return;

        const Skeleton& skeleton = *skeleton_build.skeleton;
        const uint32_t bone_count = static_cast<uint32_t>(skeleton.joint_count);
        const math::Matrix source_to_engine_inverse = math::Matrix::Invert(source_to_engine);
        std::vector<math::Matrix> local_bind_matrices(bone_count);
        for (uint32_t bone = 0; bone < bone_count; ++bone)
        {
            local_bind_matrices[bone] = math::Matrix(
                skeleton.bind_positions[bone],
                skeleton.bind_rotations[bone],
                skeleton.bind_scales[bone]
            );
        }

        std::vector<math::Matrix> global_bind_matrices(bone_count, math::Matrix::Identity);
        skeleton.ComputeGlobalPose(local_bind_matrices, global_bind_matrices);

        for (uint32_t bone_index = 0; bone_index < source_mesh->mNumBones; bone_index++)
        {
            const aiBone* source_bone = source_mesh->mBones[bone_index];
            if (!source_bone)
                continue;

            const std::string source_bone_name = source_bone->mName.C_Str();
            const auto mapping_it = skeleton_build.source_name_to_runtime_bone.find(source_bone_name);
            if (mapping_it == skeleton_build.source_name_to_runtime_bone.end())
                continue;

            const uint32_t runtime_bone_index = mapping_it->second;
            if (runtime_bone_index >= global_bind_matrices.size())
                continue;

            const math::Matrix source_offset = source_to_engine_inverse * to_matrix(source_bone->mOffsetMatrix);
            const math::Matrix cooked_inverse_bind = math::Matrix::Invert(global_bind_matrices[runtime_bone_index]);
            const float translation_error = (source_offset.GetTranslation() - cooked_inverse_bind.GetTranslation()).Length();
            const float scale_error = (source_offset.GetScale() - cooked_inverse_bind.GetScale()).Length();
            math::Quaternion source_rotation = source_offset.GetRotation();
            math::Quaternion cooked_rotation = cooked_inverse_bind.GetRotation();
            source_rotation.Normalize();
            cooked_rotation.Normalize();
            const float rotation_error = 1.0f - std::abs(math::Quaternion::Dot(source_rotation, cooked_rotation));

            if (translation_error > epsilon || scale_error > epsilon || rotation_error > epsilon)
            {
                SP_LOG_WARNING(
                    "AnimationCooker: source offset differs from cooked inverse bind for bone '%s' (t=%.6f r=%.6f s=%.6f)",
                    source_bone_name.c_str(),
                    translation_error,
                    rotation_error,
                    scale_error
                );
            }
        }
    }
}
