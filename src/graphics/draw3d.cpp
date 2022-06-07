#include "draw3d.h"
#include "common/helperfunc.h"
#include <cmath>

namespace VCL::Draw3D
{
  void WireFrame(Framebuffer *framebuffer, Model *model, const Mat4f &proj_view,
                 const Vec4f &color, bool culling)
  {
    for (size_t t = 0; t < model->n_tri_; ++t)
    {
      uint32_t idx1 = model->indices_[3 * t];
      uint32_t idx2 = model->indices_[3 * t + 1];
      uint32_t idx3 = model->indices_[3 * t + 2];
      Vec3f v1 = model->vertices_[idx1];
      Vec3f v2 = model->vertices_[idx2];
      Vec3f v3 = model->vertices_[idx3];
      Vec4f p1(v1[0], v1[1], v1[2], 1.0);
      Vec4f p2(v2[0], v2[1], v2[2], 1.0);
      Vec4f p3(v3[0], v3[1], v3[2], 1.0);
      p1 = proj_view * p1;
      p1 /= p1[3];
      p2 = proj_view * p2;
      p2 /= p2[3];
      p3 = proj_view * p3;
      p3 /= p3[3];
      Draw2D::Triangle(framebuffer, color, p1.segment<2>(0), p2.segment<2>(0),
                       p3.segment<2>(0), culling);
    }
  }

  void Line3D(Framebuffer* framebuffer, const Mat4f& proj_view, const Vec3f& v1, const Vec3f& v2, const Vec4f & color)
  {
      Vec4f p1(v1[0], v1[1], v1[2], 1.0);
      Vec4f p2(v2[0], v2[1], v2[2], 1.0);
      p1 = proj_view * p1;
      p1 /= p1[3];
      p2 = proj_view * p2;
      p2 /= p2[3];
      Draw2D::Line(framebuffer, color, p1.segment<2>(0), p2.segment<2>(0));
  }

  void Cone3D(Framebuffer* framebuffer, const Mat4f& proj_view, const Vec3f& a, const Vec3f& b, float width, int line_count, const Vec4f& color)
  {
      Vec3f dir0, dir1, dir2;
      GetOrthoCoordinate(a, b, dir0, dir1, dir2);
      for (int i = 0; i < line_count; i++)
      {
          float angle = 2 * PI_ * i / line_count;
          Vec3f pos = b + std::cos(angle) * width * dir1 + std::sin(angle) * width * dir2;
          Line3D(framebuffer, proj_view, a, pos, color);
          Line3D(framebuffer, proj_view, b, pos, color);
      }
  }

  void Circle3DPlane(Framebuffer* framebuffer, const Mat4f& proj_view, const Vec3f& a, float width, int line_count, const Vec4f& color)
  {
      Vec3f prev_pos = a + width * Vec3f::UnitX();
      for (int i = 0; i < line_count; i++)
      {
          float angle = 2 * PI_ * i / line_count;
          Vec3f pos = a + std::cos(angle) * width * Vec3f::UnitX() + std::sin(angle) * width * Vec3f::UnitZ();
          Line3D(framebuffer, proj_view, prev_pos, pos, color);
          prev_pos = pos;
      }
  }
}; // namespace VCL::Draw3D