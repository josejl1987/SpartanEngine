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

//= INCLUDES ==========
#include "Matrix.h"
#include <cassert>
#include <cmath>
//======================

namespace spartan
{
    // M0 convention reminder:
    // - Row-vector: v_world = v_local * M
    // - Hierarchy: global_child = local_child * global_parent
    struct Transform
    {
        math::Vector3 position = math::Vector3::Zero;
        math::Quaternion rotation = math::Quaternion::Identity;
        math::Vector3 scale = math::Vector3::One;

        static Transform Identity()
        {
            return Transform{};
        }

        bool IsFinite() const
        {
            return position.IsFinite() &&
                   scale.IsFinite() &&
                   rotation.IsFinite();
        }

        void NormalizeRotation()
        {
            rotation.Normalize();
        }

        math::Matrix ToMatrix() const
        {
            assert(rotation.IsFinite());
            math::Quaternion normalized_rotation = rotation;
            normalized_rotation.Normalize();
            return math::Matrix(position, normalized_rotation, scale);
        }

        static Transform FromMatrix(const math::Matrix& matrix)
        {
            Transform transform;
            transform.position = matrix.GetTranslation();
            transform.scale    = matrix.GetScale();
            transform.rotation = matrix.GetRotation();
            transform.NormalizeRotation();

            return transform;
        }
    };
}
