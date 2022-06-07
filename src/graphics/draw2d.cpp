#include "draw2d.h"

#include <algorithm>

#include "common/helperfunc.h"

namespace VCL::Draw2D
{
  static Vec2i ConvertPoint(Framebuffer *framebuffer, const Vec2f &point)
  {
    Vec2i cpos; // row col
    float tx = std::clamp((point[0] + 1.0f) / 2, 0.0f, 1.0f);
    float ty = std::clamp((point[1] + 1.0f) / 2, 0.0f, 1.0f);
    cpos[0] = int((framebuffer->height_ - 1) * ty + 0.5f);
    cpos[1] = int((framebuffer->width_ - 1) * tx + 0.5f);
    return cpos;
  }

  static void DrawPoint(Framebuffer *framebuffer, unsigned char color[4], int row,
                        int col)
  {
    int index = (row * framebuffer->width_ + col) * 4;
    for (int i = 0; i < 4; i++)
    {
      framebuffer->color_[index + i] = color[i];
    }
  }

  static void DrawLine(Framebuffer *framebuffer, unsigned char color[4], int row0,
                       int col0, int row1, int col1)
  {
    int row_distance = abs(row0 - row1);
    int col_distance = abs(col0 - col1);
    if (row_distance == 0 && col_distance == 0)
    {
      DrawPoint(framebuffer, color, row0, col0);
    }
    else if (row_distance > col_distance)
    {
      int row;
      if (row0 > row1)
      {
        std::swap(row0, row1);
        std::swap(col0, col1);
      }
      for (row = row0; row <= row1; row++)
      {
        float t = (float)(row - row0) / (float)row_distance;
        int col = LerpInt(col0, col1, t);
        DrawPoint(framebuffer, color, row, col);
      }
    }
    else
    {
      int col;
      if (col0 > col1)
      {
        std::swap(col0, col1);
        std::swap(row0, row1);
      }
      for (col = col0; col <= col1; col++)
      {
        float t = (float)(col - col0) / (float)col_distance;
        int row = LerpInt(row0, row1, t);
        DrawPoint(framebuffer, color, row, col);
      }
    }
  }

  void Point(Framebuffer *framebuffer, const Vec4f &color, const Vec2f &pos)
  {
    if (pos[0] > -1.0f && pos[0] < 1.0f && pos[1] > -1.0f && pos[1] < 1.0f)
    {
      unsigned char ucolor[4];
      ConvertColor(color, ucolor);
      Vec2i cpos = ConvertPoint(framebuffer, pos);
      DrawPoint(framebuffer, ucolor, cpos[0], cpos[1]);
    }
  };

  void Line(Framebuffer *framebuffer, const Vec4f &color, const Vec2f &point1,
            const Vec2f &point2)
  {
    // TODO: clip the line rather than raise error if intersecting with boundary
    assert(point1[0] > -1.0f && point1[0] < 1.0f && point1[1] > -1.0f &&
           point1[1] < 1.0f);
    assert(point2[0] > -1.0f && point2[0] < 1.0f && point2[1] > -1.0f &&
           point1[1] < 1.0f);
    unsigned char ucolor[4];
    ConvertColor(color, ucolor);
    Vec2i cpos1 = ConvertPoint(framebuffer, point1);
    Vec2i cpos2 = ConvertPoint(framebuffer, point2);
    DrawLine(framebuffer, ucolor, cpos1[0], cpos1[1], cpos2[0], cpos2[1]);
  };
  void Triangle(Framebuffer *framebuffer, const Vec4f &color, const Vec2f &point1,
                const Vec2f &point2, const Vec2f &point3, bool culling)
  {
    if (culling)
    {
      Vec2f dp1 = point2 - point1;
      Vec2f dp2 = point3 - point1;
      if (dp1[0] * dp2[1] - dp1[1] * dp2[0] < 0)
        return;
    }
    Line(framebuffer, color, point1, point2);
    Line(framebuffer, color, point2, point3);
    Line(framebuffer, color, point3, point1);
  };
}; // namespace VCL::Draw2D