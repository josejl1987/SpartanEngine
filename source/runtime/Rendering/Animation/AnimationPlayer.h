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

#pragma once

//= INCLUDES =================
#include "AnimationClip.h"
#include <algorithm>
#include <cmath>
//============================

namespace spartan
{
    class AnimationPlayer
    {
    public:
        AnimationPlayer();
        void Reset();
        void ResetForClip(const AnimationClip& clip);
        void SetTimeSeconds(float time_seconds, const AnimationClip& clip);
        float Advance(const AnimationClip& clip, float delta_seconds);
        void SetPlaybackRate(float in_playback_rate) { playback_rate = std::isfinite(in_playback_rate) ? in_playback_rate : 1.0f; }
        void SetLooping(bool in_looping) { looping = in_looping; }
        float GetTimeSeconds() const { return time_seconds; }
        float GetPlaybackRate() const { return playback_rate; }
        bool GetLooping() const { return looping; }
        const AnimationClip* GetClip() const { return clip; }
        float GetPreviousTimeSeconds() const { return previous_time_seconds; }
        void SetPreviousTimeSeconds(float time_seconds) { previous_time_seconds = time_seconds; }

    private:
        const AnimationClip* clip = nullptr;
        float time_seconds          = 0.0f;
        float previous_time_seconds = 0.0f;
        float playback_rate         = 1.0f;
        bool looping                = true;
    };
}
