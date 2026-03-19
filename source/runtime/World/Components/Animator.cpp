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

#include "../../Core/pch.h"
#include "Animator.h"
#include "../../Core/Timer.h"
#include "../../Core/Engine.h"
#include "../../Rendering/Animation/Skeleton.h"
#include "../../Rendering/Animation/AnimationClip.h"
#include "../../Resource/Animation/AnimationAssetValidation.h"
#include "../../Rendering/Animation/AnimationEvaluator.h"
#include "../../Logging/Log.h"
#include <pugixml.hpp>

namespace spartan
{
    Animator::Animator(Entity* entity) : Component(entity)
    {
    }

    void Animator::RegisterForScripting(sol::state_view State)
    {
        State.new_usertype<Animator>("Animator",
            "Play",                 &Animator::Play,
            "Pause",                &Animator::Pause,
            "IsPlaying",            &Animator::IsPlaying,
            "SetLooping",           &Animator::SetLooping,
            "GetLooping",           &Animator::GetLooping,
            "SetPlaybackRate",      &Animator::SetPlaybackRate,
            "GetPlaybackRate",      &Animator::GetPlaybackRate,
            "SetTimeSeconds",       &Animator::SetTimeSeconds,
            "GetTimeSeconds",       &Animator::GetTimeSeconds,
            "GetDurationSeconds",   &Animator::GetDurationSeconds,
            "GetAnimationClipIndex",&Animator::GetAnimationClipIndex,
            "SetAnimationClipByIndex", &Animator::SetAnimationClipByIndex,
            sol::base_classes,      sol::bases<Component>()
        );
    }

    sol::reference Animator::AsLua(sol::state_view state)
    {
        return sol::make_reference(state, this);
    }

    void Animator::Initialize()
    {
        if (m_skeleton)
        {
            InitializeRuntime();
        }
    }

    void Animator::Start()
    {
        if (!m_runtime_initialized || !m_skeleton)
            return;

        if (m_clip && IsCompatible())
        {
            Evaluate();
        }
        else
        {
            ResetToBindPose();
        }
    }

    void Animator::Stop()
    {
        m_is_playing = false;

        m_player.Reset();

        if (m_runtime_initialized && m_skeleton)
        {
            ResetToBindPose();
        }
    }

    void Animator::Tick()
    {
        float delta_time = static_cast<float>(Timer::GetDeltaTimeSec());
        TickInternal(delta_time);
    }

    void Animator::Save(pugi::xml_node& node)
    {
        node.append_attribute("looping") = m_player.GetLooping();
        node.append_attribute("playback_rate") = m_player.GetPlaybackRate();
    }

    void Animator::Load(pugi::xml_node& node)
    {
        m_player.SetLooping(node.attribute("looping").as_bool(true));
        m_player.SetPlaybackRate(node.attribute("playback_rate").as_float(1.0f));
    }

    void Animator::SetSkeleton(const std::shared_ptr<Skeleton>& skeleton)
    {
        m_skeleton = skeleton;
        m_runtime_initialized = false;
        RefreshCompatibility();

        if (m_skeleton)
        {
            InitializeRuntime();
        }
    }

    void Animator::SetAnimationClip(const std::shared_ptr<AnimationClip>& clip)
    {
        if (clip)
        {
            auto it = std::find(m_clips.begin(), m_clips.end(), clip);
            if (it == m_clips.end())
            {
                m_clips.push_back(clip);
                m_clip_index = static_cast<uint32_t>(m_clips.size() - 1);
            }
            else
            {
                m_clip_index = static_cast<uint32_t>(std::distance(m_clips.begin(), it));
            }
        }

        m_clip = clip;
        RefreshCompatibility();

        if (m_clip)
        {
            ResetPlayerForCurrentClip();

            if (IsReady() && IsCompatible())
            {
                Evaluate();
            }
        }
        else
        {
            if (m_runtime_initialized && m_skeleton)
            {
                ResetToBindPose();
            }
        }
    }

    void Animator::SetAnimationClips(const std::vector<std::shared_ptr<AnimationClip>>& clips, uint32_t default_index)
    {
        m_clips.clear();
        for (const std::shared_ptr<AnimationClip>& clip : clips)
        {
            if (clip)
            {
                m_clips.push_back(clip);
            }
        }

        if (m_clips.empty())
        {
            m_clip_index = 0;
            SetAnimationClip(nullptr);
            return;
        }

        m_clip_index = std::min(default_index, static_cast<uint32_t>(m_clips.size() - 1));
        SetAnimationClip(m_clips[m_clip_index]);
    }

    bool Animator::SetAnimationClipByIndex(uint32_t index)
    {
        if (index >= m_clips.size())
            return false;

        m_clip_index = index;
        SetAnimationClip(m_clips[index]);
        return true;
    }

    void Animator::Play()
    {
        if (IsReady() && !IsCompatible())
        {
            SP_LOG_ERROR("Animator clip/skeleton combination is incompatible");
            m_is_playing = false;
            return;
        }

        m_is_playing = true;

        if (IsReady() && IsCompatible())
        {
            Evaluate();
        }
    }

    void Animator::Pause()
    {
        m_is_playing = false;
    }

    void Animator::SetTimeSeconds(float time_seconds)
    {
        if (!m_clip)
            return;

        m_player.SetTimeSeconds(time_seconds, *m_clip);

        if (IsReady() && IsCompatible())
        {
            Evaluate();
        }
    }

    float Animator::GetDurationSeconds() const
    {
        return m_clip ? m_clip->duration_seconds : 0.0f;
    }

    void Animator::Evaluate()
    {
        EvaluateInternal(0.0f, false);
    }

    void Animator::AdvanceAndEvaluate(float delta_time)
    {
        EvaluateInternal(delta_time, true);
    }

    bool Animator::IsReady() const
    {
        return m_skeleton != nullptr && m_clip != nullptr && m_runtime_initialized;
    }

    bool Animator::IsCompatible() const
    {
        return m_is_compatible;
    }

    bool Animator::ValidateReadyState(bool log_errors) const
    {
        if (!m_skeleton)
        {
            if (log_errors)
                SP_LOG_WARNING("Animator is missing a skeleton");
            return false;
        }

        if (!m_clip)
        {
            if (log_errors)
                SP_LOG_WARNING("Animator is missing an animation clip");
            return false;
        }

        if (!m_runtime_initialized)
        {
            if (log_errors)
                SP_LOG_WARNING("Animator runtime is not initialized");
            return false;
        }

        if (!IsCompatible())
        {
            if (log_errors)
                SP_LOG_ERROR("Animator clip/skeleton combination is incompatible");
            return false;
        }

        return true;
    }

    void Animator::RefreshCompatibility()
    {
        if (!m_skeleton || !m_clip)
        {
            m_is_compatible = false;
            return;
        }

        std::string error;
        m_is_compatible = ValidateClip(*m_clip, *m_skeleton, &error);
        if (!m_is_compatible && !error.empty())
        {
            SP_LOG_ERROR("Animator clip/skeleton combination is incompatible: %s", error.c_str());
        }
    }

    void Animator::InitializeRuntime()
    {
        if (!m_skeleton)
            return;

        const uint32_t bone_count = static_cast<uint32_t>(m_skeleton->parent_indices.size());
        m_pose.Resize(bone_count);
        m_pose_previous.Resize(bone_count);

        m_player.Reset();
        m_runtime_initialized = true;

        ResetToBindPose();

        if (m_clip && IsCompatible())
        {
            if (m_is_playing)
            {
                m_player.ResetForClip(*m_clip);
            }

            Evaluate();
        }
    }

    void Animator::ResetToBindPose()
    {
        if (!m_skeleton || !m_runtime_initialized)
            return;

        m_pose.ResetToBindPose(*m_skeleton);
        m_pose_previous = m_pose;
    }

    void Animator::ResetPlayerForCurrentClip()
    {
        SP_ASSERT(m_clip != nullptr);
        if (!m_clip)
            return;

        m_player.Reset();
        m_player.ResetForClip(*m_clip);
    }

    void Animator::TickInternal(float delta_time)
    {
        if (!m_is_playing)
            return;

        if (!IsReady() || !IsCompatible())
            return;

        AdvanceAndEvaluate(delta_time);
    }

    void Animator::EvaluateInternal(float delta_time, bool advance_player)
    {
        if (!ValidateReadyState(true))
            return;

        std::swap(m_pose_previous, m_pose);
        if (advance_player)
            m_player.Advance(*m_clip, delta_time);

        animation_runtime::AnimationEvaluator::EvaluateForSkinning(*m_skeleton, *m_clip, m_player, m_pose);
    }
}
