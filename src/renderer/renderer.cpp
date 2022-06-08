#include "renderer.h"

#include "graphics/draw2d.h"
#include "graphics/draw3d.h"

namespace VCL {
    void Renderer::Init(const std::string& title, int width, int height) {
        width_ = width;
        height_ = height;
        InitPlatform();
        window_ = CreateVWindow(title, width_, height_, this);
        framebuffer_ = new Framebuffer(width_, height_);

        camera_ = new Camera;
        camera_->InitData((float)width_ / height_, 0.25f * PI_, 1.0f, 1000.0f, 5.0f,
            0.2f * PI_, 0.3f * PI_, Vec3f::Zero());

        if (this->lab4_mode == Lab4Mode::IK)
        {
            this->simple_arm = new IK::SimpleArm(this->framebuffer_, this->camera_, IK::IKType::FABR);
        }
        else if (this->lab4_mode == Lab4Mode::MassSpring)
        {
            this->mass_spring = new MassSpring::DynamicSystem(this->framebuffer_, this->camera_);
        }
    }

    void Renderer::MainLoop() {
        while (!window_->should_close_) {
            framebuffer_->Clear();
            PollInputEvents();
            camera_->UpdateData();
            if (this->lab4_mode == Lab4Mode::IK) // solve inverse kinematics
            {
                this->simple_arm->update();
            }
            else if (this->lab4_mode == Lab4Mode::MassSpring)
            {
                this->mass_spring->update();
            }
            window_->DrawBuffer(framebuffer_);
        }
    }

    void Renderer::Destroy() {
        if (simple_arm) delete simple_arm;
        if (camera_) delete camera_;
        if (framebuffer_) delete framebuffer_;
        window_->Destroy();
        if (window_) delete window_;
        DestroyPlatform();
    }
};  // namespace VCL