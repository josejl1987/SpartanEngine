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
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

//= INCLUDES ===========================
#include "Component.h"
#include "../../Rendering/Animation/AnimationPlayer.h"
#include "../../Rendering/Animation/AnimationPose.h"
#include <memory>
//======================================

namespace spartan
{
    class Skeleton;
    class AnimationClip;

    class Animator : public Component
    {
    public:
        Animator(Entity* entity);
        ~Animator() = default;

        static void RegisterForScripting(sol::state_view State);
        sol::reference AsLua(sol::state_view state) override;

        // lifecycle
        void Initialize() override;
        void Start() override;
        void Stop() override;
        void Tick() override;

        // serialization
        void Save(pugi::xml_node& node) override;
        void Load(pugi::xml_node& node) override;

        // skeleton
        void SetSkeleton(const std::shared_ptr<Skeleton>& skeleton);
        const std::shared_ptr<Skeleton>& GetSkeleton() const { return m_skeleton; }

        // animation clip
        void SetAnimationClip(const std::shared_ptr<AnimationClip>& clip);
        void SetAnimationClips(const std::vector<std::shared_ptr<AnimationClip>>& clips, uint32_t default_index = 0);
        const std::shared_ptr<AnimationClip>& GetAnimationClip() const { return m_clip; }
        const std::vector<std::shared_ptr<AnimationClip>>& GetAnimationClips() const { return m_clips; }
        uint32_t GetAnimationClipIndex() const { return m_clip_index; }
        bool SetAnimationClipByIndex(uint32_t index);

        // playback controls
        void Play();
        void Pause();

        // playback state
        bool IsPlaying() const { return m_is_playing; }
        void SetLooping(bool enabled) { m_player.SetLooping(enabled); }
        bool GetLooping() const { return m_player.GetLooping(); }
        void SetPlaybackRate(float rate) { m_player.SetPlaybackRate(rate); }
        float GetPlaybackRate() const { return m_player.GetPlaybackRate(); }
        void SetTimeSeconds(float time_seconds);
        float GetTimeSeconds() const { return m_player.GetTimeSeconds(); }
        float GetDurationSeconds() const;

        // evaluated data
        const AnimationPose& GetPose() const { return m_pose; }
        const AnimationPose& GetPreviousPose() const { return m_pose_previous; }
        const std::vector<math::Matrix>& GetSkinningMatrices() const { return m_pose.skinning_matrices; }
        const std::vector<math::Matrix>& GetPreviousSkinningMatrices() const { return m_pose_previous.skinning_matrices.empty() ? m_pose.skinning_matrices : m_pose_previous.skinning_matrices; }

        // explicit evaluation
        void Evaluate();
        void AdvanceAndEvaluate(float delta_time);

        // validation
        bool IsReady() const;
        bool IsCompatible() const;

    private:
        void TickInternal(float delta_time);
        bool ValidateReadyState(bool log_errors) const;
        void RefreshCompatibility();
        void InitializeRuntime();
        void ResetPlayerForCurrentClip();
        void ResetToBindPose();
        void EvaluateInternal(float delta_time, bool advance_player);

        std::shared_ptr<Skeleton> m_skeleton;
        std::shared_ptr<AnimationClip> m_clip;
        std::vector<std::shared_ptr<AnimationClip>> m_clips;
        uint32_t m_clip_index = 0;
        AnimationPlayer m_player;
        AnimationPose m_pose;
        AnimationPose m_pose_previous;
        bool m_is_playing = false;
        bool m_runtime_initialized = false;
        bool m_is_compatible = false;
    };
}
