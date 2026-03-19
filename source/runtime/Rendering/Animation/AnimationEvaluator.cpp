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

//= INCLUDES ==========================
#include "../../Core/pch.h"
#include "AnimationEvaluator.h"
#include "AnimationSamplingCommon.h"
#include <span>
//=====================================

namespace
{
    [[nodiscard]] spartan::math::Quaternion nlerp_quaternion(const spartan::math::Quaternion& a, const spartan::math::Quaternion& b, const float alpha)
    {
        const spartan::math::Quaternion q1 = spartan::math::Quaternion::Dot(a, b) < 0.0f ? -b : b;
        return (a * (1.0f - alpha) + q1 * alpha).Normalized();
    }

    void copy_base_local_pose(const spartan::AnimationClip& clip, spartan::AnimationPose& pose, const uint32_t bone_count)
    {
        SP_ASSERT(clip.base_local_positions.size() == bone_count);
        SP_ASSERT(clip.base_local_rotations.size() == bone_count);
        SP_ASSERT(clip.base_local_scales.size() == bone_count);

        std::copy(clip.base_local_positions.begin(), clip.base_local_positions.end(), pose.local_positions.begin());
        std::copy(clip.base_local_rotations.begin(), clip.base_local_rotations.end(), pose.local_rotations.begin());
        std::copy(clip.base_local_scales.begin(), clip.base_local_scales.end(), pose.local_scales.begin());
    }
}

namespace spartan::animation_runtime
{
    void AnimationEvaluator::SampleLocalPose(const Skeleton& skeleton, const AnimationClip& clip, AnimationPlayer& player, AnimationPose& pose)
    {
        const uint32_t bone_count = static_cast<uint32_t>(skeleton.parent_indices.size());
        if (bone_count == 0)
            return;

        if (player.GetClip() != &clip)
        {
            player.ResetForClip(clip);
        }

        SP_ASSERT(clip.base_local_positions.size() == bone_count);
        SP_ASSERT(clip.base_local_rotations.size() == bone_count);
        SP_ASSERT(clip.base_local_scales.size() == bone_count);

        // Rebuild from base pose every evaluation so output depends only on clip + time.
        copy_base_local_pose(clip, pose, bone_count);

        const float time_seconds = player.GetTimeSeconds();
        const float sample_position = std::max(time_seconds * clip.sample_rate, 0.0f);
        const uint32_t sample_index = static_cast<uint32_t>(std::floor(sample_position));
        const float alpha = sample_position - static_cast<float>(sample_index);

        ApplyConstantChannels(
            clip.position_stream.constants,
            pose.local_positions, bone_count);

        ApplyConstantChannels(
            clip.rotation_stream.constants,
            pose.local_rotations, bone_count);

        ApplyConstantChannels(
            clip.scale_stream.constants,
            pose.local_scales, bone_count);

        ApplySampledChannels(
            clip.position_stream.channels,
            clip.position_stream.values,
            pose.local_positions, bone_count, sample_index, alpha,
            [](const math::Vector3& a, const math::Vector3& b, float alpha) { return math::Vector3::Lerp(a, b, alpha); });

        ApplySampledChannels(
            clip.rotation_stream.channels,
            clip.rotation_stream.values,
            pose.local_rotations, bone_count, sample_index, alpha,
            [](const math::Quaternion& a, const math::Quaternion& b, float alpha) { return nlerp_quaternion(a, b, alpha); });

        ApplySampledChannels(
            clip.scale_stream.channels,
            clip.scale_stream.values,
            pose.local_scales, bone_count, sample_index, alpha,
            [](const math::Vector3& a, const math::Vector3& b, float alpha) { return math::Vector3::Lerp(a, b, alpha); });

        player.SetPreviousTimeSeconds(time_seconds);
    }

    void AnimationEvaluator::BuildGlobalMatrices(const Skeleton& skeleton, AnimationPose& pose)
    {
        const uint32_t bone_count = static_cast<uint32_t>(skeleton.parent_indices.size());
        if (bone_count == 0)
            return;

        thread_local std::vector<math::Matrix> local_matrices;
        local_matrices.resize(bone_count);

        for (uint32_t bone_index = 0; bone_index < bone_count; ++bone_index)
        {
            local_matrices[bone_index] = math::Matrix(
                pose.local_positions[bone_index],
                pose.local_rotations[bone_index],
                pose.local_scales[bone_index]
            );
        }

        skeleton.ComputeGlobalPose(local_matrices, pose.global_matrices);
    }

    void AnimationEvaluator::BuildSkinningMatrices(const Skeleton& skeleton, AnimationPose& pose)
    {
        const uint32_t bone_count = static_cast<uint32_t>(skeleton.parent_indices.size());
        for (uint32_t i = 0; i < bone_count; ++i)
        {
            pose.skinning_matrices[i] = skeleton.inverse_bind_matrices[i] * pose.global_matrices[i];
        }
    }

    void AnimationEvaluator::Evaluate(const Skeleton& skeleton, const AnimationClip& clip, AnimationPlayer& player, AnimationPose& pose)
    {
        SP_ASSERT(skeleton.parent_indices.size() > 0);
        SP_ASSERT(clip.joint_count == skeleton.parent_indices.size());

        SampleLocalPose(skeleton, clip, player, pose);
        BuildGlobalMatrices(skeleton, pose);
    }

    void AnimationEvaluator::EvaluateForSkinning(const Skeleton& skeleton, const AnimationClip& clip, AnimationPlayer& player, AnimationPose& pose)
    {
        SP_ASSERT(skeleton.parent_indices.size() > 0);
        SP_ASSERT(clip.joint_count == skeleton.parent_indices.size());

        SampleLocalPose(skeleton, clip, player, pose);
        BuildGlobalMatrices(skeleton, pose);
        BuildSkinningMatrices(skeleton, pose);
    }

    void AnimationEvaluator::BuildCompactSkinningPalette(
        const Skeleton& skeleton,
        const AnimationPose& pose,
        const std::vector<uint16_t>& palette_bone_indices,
        std::vector<math::Matrix>& out_palette
    )
    {
        const uint32_t bone_count = static_cast<uint32_t>(skeleton.parent_indices.size());
        if (bone_count == 0)
        {
            out_palette.clear();
            return;
        }

        // Requires pose.global_matrices valid for all palette bones; use EvaluateForSkinning().
        const uint32_t palette_size = static_cast<uint32_t>(palette_bone_indices.size());
        if (out_palette.capacity() < palette_size)
        {
            out_palette.reserve(palette_size);
        }
        out_palette.resize(palette_size);

        for (uint32_t palette_index = 0; palette_index < palette_size; ++palette_index)
        {
            const uint32_t bone_index = static_cast<uint32_t>(palette_bone_indices[palette_index]);
            if (bone_index >= bone_count || bone_index >= pose.global_matrices.size())
            {
                out_palette[palette_index] = math::Matrix::Identity;
                continue;
            }

            if (bone_index < pose.skinning_matrices.size())
                out_palette[palette_index] = pose.skinning_matrices[bone_index];
            else
                out_palette[palette_index] = skeleton.inverse_bind_matrices[bone_index] * pose.global_matrices[bone_index];
        }
    }
}
