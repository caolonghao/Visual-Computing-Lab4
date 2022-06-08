#pragma once

#include <string>
#include <vector>

#include "graphics/camera.h"
#include "graphics/framebuffer.h"
#include "graphics/model.h"
#include "graphics/platform.h"

#include "ik/SimpleArm.h"
#include "MassSpring/MassSpring.h"

namespace VCL
{
    enum class BUTTON : unsigned char
    {
        Left = 0,
        Right,
        Middle,
        NUM
    };

    enum class Lab4Mode : unsigned char
    {
        IK = 0,
        MassSpring = 1
    };

    class Renderer
    {
    public:
        VWindow* window_ = nullptr;
        Framebuffer* framebuffer_ = nullptr;
        Camera* camera_ = nullptr;

        Vec2f last_mouse_pos_;
        bool button_pressed_[size_t(BUTTON::NUM)] = {};

        int width_ = 800;
        int height_ = 600;

        Lab4Mode lab4_mode = Lab4Mode::IK;
        IK::SimpleArm * simple_arm = nullptr;
        MassSpring::DynamicSystem * mass_spring = nullptr;
        
    public:
        void Init(const std::string& title, int width, int height);
        void MainLoop();
        void Destroy();

        // callbacks
        void MouseBottonCallback(BUTTON button, bool pressed);
    };
}; // namespace VCL