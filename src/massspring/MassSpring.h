/***
 * Task: 4 corners are fixed
 * 
 * 
 * 
***/

# pragma once
# include <vector>
# include "common/mathtype.h"
#include "graphics/camera.h"
#include "graphics/framebuffer.h"
#include "graphics/model.h"

namespace VCL
{
    namespace MassSpring
    {
        enum class IntegrationMode : unsigned char
        {
            ExplicitEuler = 0,
            ImplicitEuler = 1
        };

        struct Node;
        struct Edge;

        struct Node
        {
            float mass;
            bool is_fixed = false;

            Vec3f position;
            Vec3f velocity;
            Vec3f acceleration;
            Vec3f force;

            std::vector<Edge*> edges;
            Node() = default;
            Node(float mass_, Vec3f position_, bool is_fixed_ = false);
        };

        struct Edge
        {
            float init_length;
            float k;
            int node_id[2];
            Node* nodes[2] = {nullptr, nullptr};
            Vec4 render_color;
            Edge() = default;
            Edge(float init_length_, float k_, int pid0, int pid1, Node* node0, Node* node1);
            float get_length() const;

            void compute_spring_force(Vec3f& force);
            void compute_hessian_matrix(Mat3f& force);
        };

        struct DynamicSystem
        {
            public:
                std::vector<Node*> nodes;
                std::vector<Edge*> edges;
                float dt = 5e-3f;
                Vec3f gravity = Vec3f(0.0f, -9.8f, 0.0f);
                IntegrationMode integration_mode = IntegrationMode::ImplicitEuler;

            protected:
                Framebuffer* framebuffer = nullptr;
                Camera* camera = nullptr;

            public:
                DynamicSystem(Framebuffer* framebuffer_, Camera* camera_);
                ~DynamicSystem();
                int num_nodes() const;
                int num_edges() const;

                void create_line(int num_node, float mass, float length, float k, bool fix_last_node = true);
                void create_mesh_grid(int row, float mass, float length, float k, float k_diag);

                void render();
                void simulation();
                void update();
            private:
                void add_matrix_block(const Mat3f&mat, std::vector<Tripletf>* mat_eles, int row, int col);
                void explicit_euler_step(float dt);
                void implicit_euler_step(float dt);
        };

        struct SparseSolver
        {
            static const float tolerance;
            static const int max_iter_num;

            SparseSolver() = default;
            static bool CG(const SpMatf& A, VecXf& x, const VecXf& b);
        };
    };
};