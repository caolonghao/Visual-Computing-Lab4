#pragma once

#include <string>

#include "common/mathtype.h"

namespace VCL
{
    class Model
    {
    public:
        size_t n_vert_;
        Vec3f* vertices_ = nullptr;
        size_t n_tri_;
        uint32_t* indices_ = nullptr;

        Model()
        {
            
        };

        ~Model()
        {
            if (vertices_)
                delete[] vertices_;
            if (indices_)
                delete[] indices_;
        }

    public:
        static Model* create_cube()
        {
            Model* model = new Model();
            model->n_vert_ = 8;
            model->vertices_ = new Vec3f[8]{ {0, 0, 0}, {0, 0, 1}, {1, 0, 1}, {1, 0, 0}, {0, 1, 0}, {0, 1, 1}, {1, 1, 1}, {1, 1, 0} };
            model->n_tri_ = 12;
            model->indices_ =
                new uint32_t[36]{ 0, 2, 1, 0, 3, 2, 0, 1, 5, 0, 5, 4, 1, 2, 5, 2, 6, 5,
                                 2, 7, 6, 2, 3, 7, 0, 7, 3, 0, 4, 7, 5, 6, 7, 5, 7, 4 };
            return model;
        }
    };
}; // namespace VCL