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
#include "AnimationPlayer.h"
#include "AnimationPose.h"
//============================

namespace spartan::animation_runtime
{
    class AnimationEvaluator
    {
    public:
        static void SampleLocalPose(const Skeleton& skeleton, const AnimationClip& clip, AnimationPlayer& player, AnimationPose& pose);
        static void BuildGlobalMatrices(const Skeleton& skeleton, AnimationPose& pose);
        static void BuildSkinningMatrices(const Skeleton& skeleton, AnimationPose& pose);
        static void BuildCompactSkinningPalette(
            const Skeleton& skeleton,
            const AnimationPose& pose,
            const std::vector<uint16_t>& palette_bone_indices,
            std::vector<math::Matrix>& out_palette
        );
        static void Evaluate(const Skeleton& skeleton, const AnimationClip& clip, AnimationPlayer& player, AnimationPose& pose);
        static void EvaluateForSkinning(const Skeleton& skeleton, const AnimationClip& clip, AnimationPlayer& player, AnimationPose& pose);
    };
}
