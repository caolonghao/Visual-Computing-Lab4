#pragma once
# include <vector>
# include "common/mathtype.h"
#include "graphics/camera.h"
#include "graphics/framebuffer.h"
#include "graphics/model.h"

namespace VCL
{
    namespace IK
    {
        enum class IKType : unsigned char
        {
            CCD = 0,
            FABR = 1
        };

        class SimpleArm
        {
            public:
                std::vector<Vec3f> joint_offset; // the local offset between joints
                std::vector<float> joint_offset_len; // length of each bone
                std::vector<Quatf> joint_rotation; // local rotation of each joint
                std::vector<Quatf> joint_orientation; // global rotation of each joint
                std::vector<Vec3f> joint_position; // global position 

                std::vector<Vec3f> end_position_hist;  // we can render the history of end position as points
                std::vector<Vec3f> trajectory;
                int curr_frame = 0;
                IKType ik_type = IKType::FABR;

            protected:
                Framebuffer* framebuffer = nullptr;
                Camera* camera = nullptr;
                Model * cube = nullptr;

            public:
                SimpleArm(Framebuffer * framebuffer, Camera * camera, IKType ik_type_);
                ~SimpleArm();
            public:
                int num_joints() const;
                Vec3f& end_effector_pos();

                /*
                * input: rotation of each joint
                * output: position of each joint, including end effector
                */
                void forward_kinematics(int start_index);

                /*
                * input: the end effector position
                * output: rotation for each joint
                */
                void inverse_kinematics(const Vec3f& end_position);


                void ccd_ik(const Vec3f& end_position, int maxCCDIKIteration, float eps);

                void fabr_ik(const Vec3f& end_position, int maxFABRIKIteration, float eps);

                void set_zero_joint_position(size_t size_);

                void set_unit_joint_rotation(size_t size_);

                void set_unit_joint_orientation(size_t size_);

                void create_trajectory();
                /*
                * render the simple arm
                */
                void render();

                void update();
        };
    }
}