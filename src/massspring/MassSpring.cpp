# include "MassSpring.h"
# include "graphics/draw3d.h"
# include <iostream>

namespace VCL::MassSpring
{
    Node::Node(float mass_, Vec3f position_, bool is_fixed_): mass(mass_), is_fixed(is_fixed_)
    {
        this->position = position_;
        this->velocity = Vec3f::Zero();
        this->acceleration = Vec3f::Zero();
        this->force = Vec3f::Zero();
    }

    Edge::Edge(float init_length_, float k_, int pid0, int pid1, Node* node0, Node* node1) :
        init_length(init_length_),
        k(k_)
    {
        node_id[0] = pid0;
        node_id[1] = pid1;
        nodes[0] = node0;
        nodes[1] = node1;
        if (node0 != nullptr)
        {
            node0->edges.push_back(this);
        }
        if (node1 != nullptr)
        {
            node1->edges.push_back(this);
        }
        this->render_color = 0.3f * Vec4f::Random() + 1.0f * Vec4f::Ones();
    }

    float Edge::get_length() const
    {
        if (this->nodes[0] != nullptr && this->nodes[1] != nullptr)
        {
            return (this->nodes[0]->position - this->nodes[1]->position).norm();
        }
        return this->init_length;
    }

    /*homework: compute the force on the single spring*/
    void Edge::compute_spring_force(Vec3f& force) 
    {
        float length_now = this->get_length();
        // 根据下面的解释，node1是+，因此计算node1收到的力
        force = (this->nodes[1]->position - this->nodes[0]->position).normalized();
        force *= (this->init_length - length_now) * this->k;
    }

    /* homework: compute the single spring on the Hessian matrix: \partial f^i / \partial x^i*/
    void Edge::compute_hessian_matrix(Mat3f& hessian) 
    {
        Vec3f x_ij = this->nodes[0]->position - this->nodes[1]->position;
        Mat3f Mat_I = Mat3f::Identity();
        // Mat_I << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        hessian = (this->k * x_ij * x_ij.transpose()) / x_ij.norm();
        hessian += this->k * (1 - this->init_length / x_ij.norm()) * (Mat_I - x_ij * x_ij.transpose());
        hessian *= -1.0;
    }

    DynamicSystem::DynamicSystem(Framebuffer* framebuffer_, Camera* camera_) :
        framebuffer(framebuffer_),
        camera(camera_)
    {
        // this->create_line(10, 1, 0.15f, 1000.0f);
        this->create_mesh_grid(9, 1, 0.1f, 2000.f, 100.f);
    }

    DynamicSystem::~DynamicSystem()
    {
        for (int i = 0; i < this->nodes.size(); i++)
        {
            delete this->nodes[i];
        }
        for (int i = 0; i < this->edges.size(); i++)
        {
            delete this->edges[i];
        }
    }

    int DynamicSystem::num_nodes() const
    {
        return static_cast<int>(this->nodes.size());
    }

    int DynamicSystem::num_edges() const
    {
        return static_cast<int>(this->edges.size());
    }

    void DynamicSystem::render()
    {
        // render the xyz axis
        Vec4f axis_color(0, 1.0f, 0, 1);
        Draw3D::Line3D(this->framebuffer, this->camera->proj_view_, Vec3f::Zero(), 1.5f * Vec3f::UnitX(), axis_color);
        Draw3D::Line3D(this->framebuffer, this->camera->proj_view_, Vec3f::Zero(), 1.5f * Vec3f::UnitY(), axis_color);
        Draw3D::Line3D(this->framebuffer, this->camera->proj_view_, Vec3f::Zero(), 1.5f * Vec3f::UnitZ(), axis_color);

        // simply render edge and nodes.
        for (int i = 0; i < this->edges.size(); i++)
        {
            Edge* edge = this->edges[i];
            Draw3D::Line3D(this->framebuffer, this->camera->proj_view_, edge->nodes[0]->position, edge->nodes[1]->position, edge->render_color);
        }
    }

    /*
    * Parameters:
    * int num_node: number of nodes in the chain
    * float mass: mass for each node
    * float length: length between two nodes
    * float k: the Stiffness coefficient of spring.
    */
    void DynamicSystem::create_line(int num_node, float mass, float length, float k, bool fix_last_node)
    {
        const float init_height = 1.5f;
        Node * prev_node = new Node(mass, Vec3f(0.5f, init_height, 0.5f), true);
        nodes.push_back(prev_node);

        for (int i = 1; i < num_node; i++)
        {
            Node * node = new Node(mass, Vec3f(0.5f + i * length, init_height, 0.5f));
            Edge* edge = new Edge(length, k, i - 1, i, prev_node, node);
            nodes.push_back(node);
            edges.push_back(edge);
            prev_node = node;
        }
        if (fix_last_node)
        {
            nodes[num_node - 1]->is_fixed = true;
        }
    }

    void DynamicSystem::create_mesh_grid(int row, float mass, float length, float k, float k_diag)
    {
        const float height = 1.0f;
        // create nodes
        for (int i = 0; i < row; i++)
        {
            for (int j = 0; j < row; j++)
            {
                Node* node = new Node(mass, Vec3f(i * length, height, j * length));
                nodes.push_back(node);
            }
        }
        nodes[0]->is_fixed = true;
        nodes[row - 1]->is_fixed = true;
        nodes[row * (row - 1)]->is_fixed = true;
        nodes[row * row - 1]->is_fixed = true;

        // create edges 
        for (int i = 0; i < row; i++)
        {
            for (int j = 0; j < row - 1; j++)
            {
                edges.push_back(new Edge(length, k, i * row + j, i * row + j + 1, nodes[i * row + j], nodes[i * row + j + 1]));
                edges.push_back(new Edge(length, k, i + j * row, i + (j + 1) * row, nodes[i + j * row], nodes[i + (j + 1) * row]));
            }
        }

        float sqr2len = std::sqrt(2) * length;
        for (int i = 0; i < row - 1; i++)
        {
            for (int j = 0; j < row - 1; j++)
            {
                edges.push_back(new Edge(sqr2len, k, i * row + j, (i + 1) * row + j + 1, nodes[i * row + j], nodes[(i + 1) * row + j + 1]));
                edges.push_back(new Edge(sqr2len, k, i * row + j + 1, (i + 1) * row + j, nodes[i * row + j + 1], nodes[(i + 1) * row + j]));
            }
        }
    }

    void DynamicSystem::update()
    {
        this->simulation();
        this->render();
    }

    void DynamicSystem::simulation()
    {
        if (this->integration_mode == IntegrationMode::ExplicitEuler) // solve inverse kinematics
        {
            this->explicit_euler_step(this->dt);
        }
        else if (this->integration_mode == IntegrationMode::ImplicitEuler)
        {
            this->implicit_euler_step(this->dt);
        }
    }

    void DynamicSystem::explicit_euler_step(float dt)
    {
        for (int i = 0; i < this->num_nodes(); i++)
            nodes[i]->force = gravity;

        for (int i = 0; i < this->num_edges(); i++)
        {
            // compute force for each edge
            Edge* edge = edges[i];
            Vec3f force(0, 0, 0);
            edge->compute_spring_force(force);
            edge->nodes[0]->force -= force;
            edge->nodes[1]->force += force;
        }

        // here we use Euler method to solve.
        // compute acc from force
        for (int i = 0; i < this->num_nodes(); i++)
        {
            /* homework: explicit euler (update acceleration, velocity, position) */
            if (!nodes[i]->is_fixed)
            {
                Vec3f acc = nodes[i]->force / nodes[i]->mass;
                nodes[i]->velocity += acc * dt;
                nodes[i]->position += nodes[i]->velocity * dt;
            }
        }
    }

    void DynamicSystem::implicit_euler_step(float dt)
    {
        for (int i = 0; i < this->num_nodes(); i++)
            nodes[i]->force.setZero();

        std::vector<Tripletf> matrix_elements;
        matrix_elements.clear(); 
        for (int i = 0; i < this->num_edges(); i++)
        {
            // compute force for each edge
            Edge* edge = edges[i];
            Vec3f force(0, 0, 0);
            Mat3f hessian(Mat3f::Zero());
            edge->compute_spring_force(force);
            edge->compute_hessian_matrix(hessian);

            int pid[2] = { edge->node_id[0],edge->node_id[1] };
            if (nodes[pid[0]]->is_fixed && nodes[pid[1]]->is_fixed)continue;

            edge->nodes[0]->force -= force;
            edge->nodes[1]->force += force;

            if (nodes[pid[1]]->is_fixed)
                add_matrix_block(-hessian, &matrix_elements, pid[0], pid[0]);
            else if (nodes[pid[0]]->is_fixed)
                add_matrix_block(-hessian, &matrix_elements, pid[1], pid[1]);
            else {
                add_matrix_block(-hessian, &matrix_elements, pid[0], pid[0]);
                add_matrix_block(-hessian, &matrix_elements, pid[1], pid[1]);
                add_matrix_block(hessian, &matrix_elements, pid[1], pid[0]);
                add_matrix_block(hessian, &matrix_elements, pid[0], pid[1]);
            }
        }

        VecXf deriv = VecXf::Zero(this->num_nodes() * 3);
        VecXf delta_x = VecXf::Zero(this->num_nodes() * 3);
        SpMatf Hess(this->num_nodes() * 3, this->num_nodes() * 3);

        for (int i = 0; i < this->num_nodes(); i++) {
            // 大 Hess 保存了 F 关于 x 的Hessian矩阵
            for (int row = 0; row < 3; row++)
                matrix_elements.push_back(Tripletf(3 * i + row, 3 * i + row, nodes[i]->mass / dt / dt));
            Vec3f y = nodes[i]->position + dt * nodes[i]->velocity + dt * dt * gravity;
            // F 关于 x 的导数
            if(!nodes[i]->is_fixed)deriv.segment<3>(3 * i) = (nodes[i]->position - y) * nodes[i]->mass / dt / dt - nodes[i]->force;
        }
        Hess.setFromTriplets(matrix_elements.begin(), matrix_elements.end());

        SparseSolver::CG(Hess, delta_x, -deriv);

        /* homework: read the code of implicit euler method in this function, and update the system state*/
        for (int i = 0; i < this->num_nodes(); i++) {
            Vec3f delta_x_i = delta_x.segment<3>(3 * i);
            nodes[i]->position += delta_x_i;
            // Vec3f pre_velocity = nodes[i]->velocity;
            nodes[i]->velocity = delta_x_i / dt;
        }

    }

    void DynamicSystem::add_matrix_block(const Mat3f& mat, std::vector<Tripletf>* mat_eles, int row, int col) 
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                mat_eles->push_back(Tripletf(3 * row + i, 3 * col + j, mat(i, j)));
    }

    bool SparseSolver::CG(const SpMatf& A, VecXf& x, const VecXf& b)
    {
        Eigen::ConjugateGradient<SpMatf, Eigen::Upper, Eigen::IdentityPreconditioner> cg;
        cg.setMaxIterations(max_iter_num);
        cg.setTolerance(tolerance);
        cg.compute(A);
        if (cg.info() != Eigen::Success) { std::cerr << "Error: [Sparse] Eigen CG solver factorization failed." << std::endl; return false; }
        x = cg.solve(b);
        if (cg.info() != Eigen::Success) { std::cerr << "Error: [Sparse] Eigen CG solver failed." << std::endl; return false; }
        //std::cout << "Eigen CG solver converge in " << cg.iterations() << " iterations, error: " << cg.error() << std::endl; 
        return true;
    }
    const float SparseSolver::tolerance = (float)1e-8;
    const int SparseSolver::max_iter_num = 1000;
};