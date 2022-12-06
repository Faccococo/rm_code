#pragma once
#include "Transform.hpp"
#include <algorithm>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <random>

using MotionState = Transform<FrameOfRef::Robot, FrameOfRef::Ground, true>;

enum class TargetType { Infantry, Hero, BalancedInfantry, Sentry, Outpost, BaseClosed, BaseExpanded, Fans };

enum class TargetMotionType { Static, Spinning, Translate2D, Translate3D, LargeCircle, Fans, Sentry };

enum class SourceMotionType { Static, Vibration, Translate2D, Translate3D, Sentry, UAV };

class MotionController {
public:
    virtual void step(MotionState& motionState, double dt) = 0;
    virtual ~MotionController() = default;
};

class StaticMotionController final : public MotionController {
public:
    void step(MotionState& motionState, double dt) override{};
};

class SpinMotionController final : public MotionController {
    double mSpinningSpeed;

public:
    explicit SpinMotionController(const double spinningSpeed) : mSpinningSpeed(spinningSpeed){};
    void step(MotionState& motionState, double dt) override {
        motionState =
            glm::rotate(glm::identity<glm::dmat4>(), glm::two_pi<double>() * mSpinningSpeed * dt, glm::dvec3{ 0.0, 1.0, 0.0 }) *
            motionState.raw();
    };
};

class LargeCircleMotionController final : public MotionController {
    double mSpinningSpeed;

public:
    explicit LargeCircleMotionController(const double spinningSpeed) : mSpinningSpeed(spinningSpeed){};
    void step(MotionState& motionState, const double dt) override {
        motionState =
            glm::rotate(glm::identity<glm::dmat4>(), glm::two_pi<double>() * mSpinningSpeed * dt, glm::dvec3{ 0.0, 1.0, 0.0 }) *
            motionState.raw();
    };
};

class SentryMotionController final : public MotionController {
    bool mMovingDirection = false;
    static constexpr double speed = 0.5f;

public:
    void step(MotionState& motionState, const double dt) override {
        auto position = motionState.translatePoint();
        if(position.mVal.x > 2.0)
            mMovingDirection = false;
        else if(position.mVal.x < -2.0)
            mMovingDirection = true;

        position.mVal.x += (mMovingDirection ? speed * dt : -speed * dt);
    };
};

class Translate2DMotionController final : public MotionController {
    static constexpr double alpha = 0.001;

    std::default_random_engine mGenerator;
    double mMaxV;
    std::normal_distribution<double> mDistribution;
    double mSpeedX, mSpeedZ, mT = 0.0;
    double mSmoothX = 0.0, mSmoothZ = 0.0;

public:
    explicit Translate2DMotionController(const double maxV) : mMaxV{ maxV }, mDistribution{ 0, maxV / 3.0 } {};
    void step(MotionState& motionState, const double dt) override {
        mT += dt;
        if(mT >= 2.0) {                                                      // Random generation velocity every two seconds
            mSpeedX = std::clamp(mDistribution(mGenerator), -mMaxV, mMaxV);  // Limit the speed of random generation
            mSpeedZ = std::clamp(mDistribution(mGenerator), -mMaxV, mMaxV);

            mT -= 2.0;
        }

        mSmoothX = (1.0 - alpha) * mSmoothX + alpha * mSpeedX;
        mSmoothZ = (1.0 - alpha) * mSmoothZ + alpha * mSpeedZ;

        motionState =
            glm::translate(glm::identity<glm::dmat4>(), glm::dvec3{ mSmoothX * dt, 0.0, mSmoothZ * dt }) * motionState.raw();
    };
};