#pragma once

#include "graphics/draw2d.h"
#include "graphics/model.h"
#include "graphics/framebuffer.h"

namespace VCL
{
    namespace Draw3D
    {
        void WireFrame(Framebuffer *framebuffer, Model *model, const Mat4f &proj_view, const Vec4f &color, bool culling = true);
        void Line3D(Framebuffer* framebuffer, const Mat4f& proj_view, const Vec3f& v1, const Vec3f& v2, const Vec4f& color);
        void Cone3D(Framebuffer* framebuffer, const Mat4f& proj_view, const Vec3f& a, const Vec3f& b, float width, int line_count, const Vec4f& color);
        void Circle3DPlane(Framebuffer* framebuffer, const Mat4f& proj_view, const Vec3f& a, float width, int line_count, const Vec4f& color);
    }; // namespace Draw3D
}; // namespace VCL