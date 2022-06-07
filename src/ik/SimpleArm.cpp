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
        joint_offset.emplace_back(Vec3f(-0.6f, -0.2f, -0.4f));
        joint_offset.emplace_back(Vec3f(0.4f, -0.3f, -0.4f));
        joint_offset.emplace_back(Vec3f(-0.4f, -0.5f, 0.0f));
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
            this->ccd_ik(end_position, 100, 1e-4);
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
            }
        }
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
            }

            // forward update
            Vec3f now_position = this->joint_position[0];
            forward_positions[0] = this->joint_position[0];
            for (int i = 0; i < n_joints - 1; i++)
            {
                // homework: compute the position in forward processing
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
        const int max_count = 100;
        for (int i = 0; i < max_count; i++)
        {
            float theta = (2 * PI_ * i) / max_count;
            this->trajectory.emplace_back(Vec3f(0.5f + 0.5f * std::cos(theta), 0, 0.5f + 0.5f * std::sin(theta)));
        }
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