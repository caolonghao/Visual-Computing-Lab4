#pragma once

#include "mathtype.h"

namespace VCL {
    unsigned char FloatToUChar(float x);
    int LerpInt(int a, int b, float t);
    void ConvertColor(Vec4f input, unsigned char output[4]);
    void GetOrthoCoordinate(const Vec3f& a, const Vec3f& b, Vec3f& dir0, Vec3f& dir1, Vec3f& dir2);
};