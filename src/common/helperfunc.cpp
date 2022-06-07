#include "helperfunc.h"

namespace VCL {
    unsigned char FloatToUChar(float x) { return (unsigned char)(x * 255.999); }

    int LerpInt(int a, int b, float t) { return int(a + (b - a) * t); }

    void ConvertColor(Vec4f input, unsigned char output[4]) {
        output[0] = FloatToUChar(input[0]);
        output[1] = FloatToUChar(input[1]);
        output[2] = FloatToUChar(input[2]);
        output[3] = FloatToUChar(input[3]);
    }

    void GetOrthoCoordinate(const Vec3f& a, const Vec3f& b, Vec3f& dir0, Vec3f& dir1, Vec3f& dir2)
    {
        // Get a ortho coordinate
        dir0 = (b - a).normalized();
        dir1 = Vec3f::UnitX();
        dir2 = dir0.cross(dir1);
        if (std::abs(dir2.y()) < 1e-4 && std::abs(dir2.z()) < 1e-4)
        {
            dir1 = Vec3f::UnitY();
            dir2 = dir0.cross(dir1);
        }
        dir2.normalized();
        dir1 = dir0.cross(dir2);
        dir1.normalize();
    }
};  // namespace VCL