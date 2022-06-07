#pragma once

#include "framebuffer.h"

namespace VCL
{
    namespace Draw2D
    {
        void Point(Framebuffer *framebuffer, const Vec4f &color, const Vec2f &pos);
        void Line(Framebuffer *framebuffer, const Vec4f &color, const Vec2f &point1,
                  const Vec2f &point2);
        void Triangle(Framebuffer *framebuffer, const Vec4f &color, const Vec2f &point1,
                      const Vec2f &point2, const Vec2f &point3, bool culling = false);
    }; // namespace Draw2D
};     // namespace VCL