# include "SimpleArm.h"
# include "graphics/draw3d.h"
# include <iostream>

namespace VCL::IK
{

    SimpleArm::SimpleArm(Framebuffer* framebuffer_, Camera* camera_, IKType ik_type_):
        framebuffer(framebuffer_),
        camera(camera_),
        ik_type(ik_type_)
    {
        this->cube = Model::create_cube();
        // create arm by default offsets.
        joint_offset.emplace_back(Vec3f(1, 1, 1));
        // joint_offset.emplace_back(Vec3f(-0.6f, -0.2f, -0.4f));
        // joint_offset.emplace_back(Vec3f(0.4f, -0.3f, -0.4f));
        // joint_offset.emplace_back(Vec3f(-0.4f, -0.5f, 0.0f));
        // TODO: add new arm here
        joint_offset.emplace_back(Vec3f(2.0f, 1.0f, 0.7f));
        joint_offset.emplace_back(Vec3f(2.0f, 1.0f, 0.7f));
        joint_offset.emplace_back(Vec3f(-0.2f, -0.3f, 0.5f));
        joint_offset.emplace_back(Vec3f(0.1f, -0.3f, -0.5f));
        joint_offset.emplace_back(Vec3f(0.1f, -0.3f, -0.5f));

        size_t n_joints = this->num_joints();
        for (int i = 0; i < n_joints - 1; i++)
        {
            joint_offset_len.push_back(joint_offset[i+1].norm());
        }

        set_unit_joint_rotation(n_joints);
        set_unit_joint_orientation(n_joints);
        set_zero_joint_position(n_joints);
        forward_kinematics(0);
        this->create_trajectory();
    }

    SimpleArm::~SimpleArm()
    {
        delete this->cube;
    }

    int SimpleArm::num_joints() const
    {
        return static_cast<int>(this->joint_offset.size());
    }

    Vec3f& SimpleArm::end_effector_pos()
    {
        return this->joint_position[this->joint_position.size() - 1];
    }

    void SimpleArm::forward_kinematics(int start_index)
    {
        // using forward kinematics to compute joint global position here.
        if (start_index == 0)
        {
            joint_position[0] = joint_offset[0];
            joint_orientation[0] = joint_rotation[0];
            start_index = 1;
        }
        size_t num_joint = this->joint_offset.size();
        for (size_t i = start_index; i < num_joint; i++)
        {
            joint_position[i] = joint_position[i - 1] + joint_orientation[i - 1] * joint_offset[i];
            joint_orientation[i] = joint_orientation[i - 1] * joint_rotation[i];
        }
    }

    void SimpleArm::inverse_kinematics(const Vec3f& end_position)
    {
        // You can choose one of inverse kinematics to implement
        if (this->ik_type == IKType::CCD)
        {
            this->ccd_ik(end_position, 500, 1e-4);
        }
        else if (this->ik_type == IKType::FABR)
        {
            this->fabr_ik(end_position, 10, 1e-4);
        }
    }

    void SimpleArm::ccd_ik(const Vec3f& end_position, int maxCCDIKIteration, float eps)
    {
        // Implement CCD IK here
        this->forward_kinematics(0);
        for(int CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && (end_effector_pos() - end_position).norm() > eps; CCDIKIteration++)
        {
            for (int i = this->num_joints() - 2; i >= 0; i--)
            {
                // homework: create rotation of i-th joint
                this->forward_kinematics(i);
                Vec3f chain_top_point = joint_position[this->num_joints() - 1];
                Vec3f link_root = joint_position[i];
                Vec3f root_to_end = (end_position - link_root).normalized();
                Vec3f root_to_top = (chain_top_point - link_root).normalized();


            /*  // Simple Rotation 
                // 利用叉积找到法向量作为旋转轴
                Vec3f rotation_axis = root_to_top.cross(root_to_end);
                float k = root_to_end.dot(root_to_top) + 1.0f;
                float s = 1.0 / sqrt(k + k);
                Quatf result(k * s, s * rotation_axis(0), s * rotation_axis(1), s * rotation_axis(2));
                // 后施加新的 result 旋转，换序问题要考虑
                joint_rotation[i] = joint_rotation[i] * result;
            */
                
                // More Robust rotation
                Vec3f w;
                float norm_root_end = sqrt(root_to_end.dot(root_to_end) * root_to_top.dot(root_to_top));
                float real_part = norm_root_end + root_to_end.dot(root_to_top);
                if (real_part < 1.e-6f * norm_root_end) {
                    real_part = 0.0f;
                    w = abs(root_to_top(0)) > abs(root_to_top(2)) ? Vec3f(-root_to_top(1), root_to_top(0), 0.f)
                                                                  : Vec3f(0.f, -root_to_top(2), root_to_top(1));
                }
                else {
                    w = root_to_top.cross(root_to_end);
                }
                Quatf result(real_part, w(0), w(1), w(2));
                joint_rotation[i] = joint_rotation[i] * result.normalized();
            }
        }
        this->forward_kinematics(0);
    }

    void SimpleArm::fabr_ik(const Vec3f& end_position, int maxFABRIKIteration, float eps)
    {
        // Implement fabr ik here
        this->forward_kinematics(0);
        int n_joints = this->num_joints();
        std::vector<Vec3f> backward_positions(n_joints, Vec3f::Zero()), forward_positions(n_joints, Vec3f::Zero());
        for (int IKIteration = 0; IKIteration < maxFABRIKIteration && (end_effector_pos() - end_position).norm() > eps; IKIteration++)
        {
            // backward update
            Vec3f next_position = end_position;
            backward_positions[n_joints - 1] = end_position;
            for (int i = n_joints - 2; i >= 0; i--)
            {
                // homework: compute the positions in backward processing
                Vec3f dir = (joint_position[i] - next_position).normalized();
                next_position = next_position + dir * joint_offset_len[i];
                backward_positions[i] = next_position;
            }

            // forward update
            Vec3f now_position = this->joint_position[0];
            forward_positions[0] = this->joint_position[0];

            for (int i = 0; i < n_joints - 1; i++)
            {
                // homework: compute the position in forward processing
                Vec3f dir = (backward_positions[i+1] - now_position).normalized();
                now_position = now_position + joint_offset_len[i] * dir;
                forward_positions[i+1] = now_position;
            }

            // copy forward positions to joint_positions
            joint_position = forward_positions;
        }
        // Compute joint rotation by position here.
        for (int i = 0; i < n_joints - 1; i++)
        {
            this->joint_orientation[i] = Quatf::FromTwoVectors(this->joint_offset[i + 1], this->joint_position[i + 1] - this->joint_position[i]);
        }
        this->joint_rotation[0] = this->joint_orientation[0];
        for (int i = 1; i < n_joints - 1; i++)
        {
            this->joint_rotation[i] = this->joint_orientation[i - 1].conjugate() * this->joint_orientation[i];
        }
        this->forward_kinematics(0);
    }

    void SimpleArm::set_zero_joint_position(size_t size_)
    {
        joint_position.resize(size_, Vec3f::Zero());
    }

    void SimpleArm::set_unit_joint_rotation(size_t size_)
    {
        joint_rotation.resize(size_, Quatf::Identity());
    }

    void SimpleArm::set_unit_joint_orientation(size_t size_)
    {
        joint_orientation.resize(size_, Quatf::Identity());
    }

    /*
    * homework: change the trajectory for end effector
    */
    void SimpleArm::create_trajectory()
    {
        // create a simple circle as end effector trajectory
        // You can modify the trajectory here.
        
        // circle
        const int max_count = 300;
        for (int i = 0; i < max_count; i++)
        {
            float theta = (2 * PI_ * i) / max_count;
            this->trajectory.emplace_back(Vec3f(0.5f + 0.5f * std::cos(theta), 0, 0.5f + 0.5f * std::sin(theta)));
        }

        // CLH logo draw
        // const int max_count = 1000;
        // float total_t = PI_ + 8.0;
        // float delta_t = total_t / max_count;
        // for (float t=0.0; t < total_t; t += delta_t)
        // {
        //     if(t<=PI_)
        //     {
        //         this->trajectory.emplace_back(Vec3f(-std::sin(t), 0, 1 + std::cos(t)));
        //     }
        //     else if(t>PI_ && t<=PI_+2.0)
        //     {
        //         this->trajectory.emplace_back(Vec3f(0, 0, PI_ + 2.0 - t));
        //     }
        //     else if(t>PI_+2.0 && t<=PI_+3.0)
        //     {
        //         this->trajectory.emplace_back(Vec3f(t - PI_ - 2.0, 0, 0));
        //     }
        //     else if(t>PI_+3.0 && t<=PI_+5.0)
        //     {
        //         this->trajectory.emplace_back(Vec3f(1, 0, PI_ + 5.0 - t));
        //     }
        //     else if(t>PI_+5.0 && t<=PI_+6.0)
        //     {
        //         this->trajectory.emplace_back(Vec3f(t - PI_ - 4.0, 0, 1));
        //     }
        //     else if(t>PI_ + 6.0)
        //     {
        //         this->trajectory.emplace_back(Vec3f(2, 0, PI_ + 8.0 - t));
        //     }
        // }

        // Number 2
        // const int max_count = 500;
        // float total_t = 5.0;
        // float delta_t = total_t / max_count;
        // for (float t=0.0; t < total_t; t += delta_t)
        // {
        //     if(t <= 1.0){
        //         this->trajectory.emplace_back(Vec3f(t, 0, 2));
        //     }
        //     else if(t > 1.0 && t <= 2.0){
        //         this->trajectory.emplace_back(Vec3f(1, 0, 3 - t));
        //     }
        //     else if(t > 2.0 && t <= 3.0){
        //         this->trajectory.emplace_back(Vec3f(3.0 - t, 0, 1));
        //     }
        //     else if(t > 3.0 && t <= 4.0){
        //         this->trajectory.emplace_back(Vec3f(0, 0, 4.0 - t));
        //     }
        //     else if(t > 4.0 && t <= 5.0){
        //         this->trajectory.emplace_back(Vec3f(t - 4.0, 0, 0));
        //     }
        // }
    }

    void SimpleArm::render()
    {
        // render grid on the x-z plane
        if (false)
        {
            Vec4f xz_color(0.8f, 0.8f, 0.8f, 1.0f);
            for (float i = -2.5; i <= 2.5; i += 0.25)
            {
                Draw3D::Line3D(this->framebuffer, this->camera->proj_view_, Vec3f(-2.5f, 0, i), Vec3f(2.5f, 0, i), xz_color);
                Draw3D::Line3D(this->framebuffer, this->camera->proj_view_, Vec3f(i, 0, -2.5f), Vec3f(i, 0, 2.5f), xz_color);
            }
        }

        // render a cube for showing the boundary.
        Draw3D::WireFrame(this->framebuffer, this->cube, this->camera->proj_view_, Vec4f::Ones(), true);
        
        // render x, y, z axis
        Vec4f axis_color(0, 1, 0, 1);
        Draw3D::Line3D(this->framebuffer, this->camera->proj_view_, Vec3f::Zero(), 1.5f * Vec3f::UnitX(), axis_color);
        Draw3D::Line3D(this->framebuffer, this->camera->proj_view_, Vec3f::Zero(), 1.5f * Vec3f::UnitY(), axis_color);
        Draw3D::Line3D(this->framebuffer, this->camera->proj_view_, Vec3f::Zero(), 1.5f * Vec3f::UnitZ(), axis_color);

        // render each bone
        size_t num_joint = this->joint_offset.size();
        for (size_t i = 1; i < num_joint; i++)
        {
            Draw3D::Cone3D(this->framebuffer, this->camera->proj_view_, joint_position[i-1], joint_position[i], 0.02f, 10, Vec4f::Ones());
        }

        // render the end effector position
        Draw3D::Circle3DPlane(this->framebuffer, this->camera->proj_view_, joint_position[joint_position.size() - 1], 0.02f, 10, Vec4f(1.0f, 0.0f, 0.0f, 1.0f));

        // render the trace in history
        for (size_t i = 1; i < end_position_hist.size(); i++)
        {
            Draw3D::Line3D(this->framebuffer, this->camera->proj_view_, end_position_hist[i - 1], end_position_hist[i], Vec4f(0.6f, 0.0f, 0.0f, 0.0f));
        }
    }

    void SimpleArm::update()
    {
        this->inverse_kinematics(this->trajectory[this->curr_frame]);
        this->end_position_hist.push_back(this->end_effector_pos());
        this->render();
        this->curr_frame += 1;
        if (this->curr_frame == this->trajectory.size())
        {
            this->curr_frame = 0;
            this->end_position_hist.clear();
        }
    }
}