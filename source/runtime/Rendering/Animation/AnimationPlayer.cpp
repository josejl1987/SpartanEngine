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
#include "AnimationPlayer.h"
//=====================================

namespace spartan
{
    AnimationPlayer::AnimationPlayer()
    = default;

    void AnimationPlayer::Reset()
    {
        this->clip = nullptr;

        this->time_seconds = 0.0f;
        this->previous_time_seconds = 0.0f;
    }

    void AnimationPlayer::ResetForClip(const AnimationClip& clip_to_set)
    {
        this->clip = &clip_to_set;
        this->time_seconds = 0.0f;
        this->previous_time_seconds = 0.0f;
    }

    void AnimationPlayer::SetTimeSeconds(const float time_to_set, const AnimationClip& clip_to_use)
    {
        if (this->clip != &clip_to_use)
            ResetForClip(clip_to_use);

        this->previous_time_seconds = this->time_seconds;
        this->time_seconds = std::clamp(time_to_set, 0.0f, this->clip->duration_seconds);
    }

    float AnimationPlayer::Advance(const AnimationClip& clip_to_use, const float delta_seconds)
    {
        if (this->clip != &clip_to_use)
            ResetForClip(clip_to_use);

        this->previous_time_seconds = this->time_seconds;

        const float safe_delta = std::isfinite(delta_seconds) ? delta_seconds : 0.0f;

        const float duration_seconds = this->clip->duration_seconds;
        if (!std::isfinite(duration_seconds) || duration_seconds <= 0.0f)
        {
            this->time_seconds = 0.0f;
            return this->time_seconds;
        }

        const float advanced_time = this->time_seconds + (safe_delta * this->playback_rate);
        if (this->looping)
        {
            this->time_seconds = std::fmod(std::fmod(advanced_time, duration_seconds) + duration_seconds, duration_seconds);
        }
        else
        {
            this->time_seconds = std::clamp(advanced_time, 0.0f, duration_seconds);
        }

        return this->time_seconds;
    }
}
