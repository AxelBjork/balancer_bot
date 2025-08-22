// XboxController.h
#pragma once

#include <SDL2/SDL.h>
#include <stdexcept>

class XboxController {
public:
    XboxController() {
        if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
            throw std::runtime_error("SDL_Init failed");
        }

        if (SDL_NumJoysticks() < 1) {
            throw std::runtime_error("No joystick found");
        }

        joystick_ = SDL_JoystickOpen(0);
        if (!joystick_) {
            throw std::runtime_error("Failed to open joystick 0");
        }
    }

    ~XboxController() {
        if (joystick_) SDL_JoystickClose(joystick_);
        SDL_Quit();
    }

    // Poll the current state
    void update() {
        SDL_JoystickUpdate();
    }

    // Return normalized stick values [-1.0 .. +1.0]
    float leftY() const {
        return normalize(SDL_JoystickGetAxis(joystick_, 1)); // left stick vertical
    }

    float rightY() const {
        return normalize(SDL_JoystickGetAxis(joystick_, 3)); // right stick vertical
    }

private:
    SDL_Joystick* joystick_{nullptr};

    static float normalize(Sint16 value) {
        // Axis is -32768..32767, normalize to -1..+1
        return static_cast<float>(value) / 32768.0f;
    }
};
