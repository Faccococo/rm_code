#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "SimulatorMotionType.hpp"
#include "SimulatorWorldInfo.hpp"
#include "Transform.hpp"
#include "Utility.hpp"
#include <random>
#include <tuple>

#include "SuppressWarningBegin.hpp"

#include <caf/blocking_actor.hpp>
#include <fmt/format.h>
#include <glm/gtc/type_ptr.hpp>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

struct SimulatorSettings final {
    double step;

    double v0;
    double v0Std;
    double shootInterval;

    double maxTime;
    uint32_t bulletCount;

    double vibrationLinearRange;
    double vibrationAngleRange;  // in degrees
    double spinningSpeed;        // in circles/s

    double standardDistance;
    double sourceHeight;
    double targetHeight;

    std::string targetType;
    std::string targetMotionType;
    std::string sourceMotionType;

    uint32_t expectedCount;

    bool printBulletPos;
};

template <class Inspector>
bool inspect(Inspector& f, SimulatorSettings& x) {
    return f.object(x).fields(
        f.field("step", x.step).invariant([](const double v) { return v >= 0.001 && v <= 0.01; }), f.field("v0", x.v0),
        f.field("v0Std", x.v0Std), f.field("shootInterval", x.shootInterval), f.field("maxTime", x.maxTime),
        f.field("bulletCount", x.bulletCount), f.field("vibrationLinearRange", x.vibrationLinearRange),
        f.field("vibrationAngleRange", x.vibrationAngleRange), f.field("spinningSpeed", x.spinningSpeed),
        f.field("standardDistance", x.standardDistance), f.field("sourceHeight", x.sourceHeight),
        f.field("targetHeight", x.targetHeight), f.field("targetType", x.targetType),
        f.field("targetMotionType", x.targetMotionType), f.field("sourceMotionType", x.sourceMotionType),
        f.field("expectedCount", x.expectedCount), f.field("printBulletPos", x.printBulletPos).fallback(false));
}

class Simulator final : public HubHelper<caf::blocking_actor, SimulatorSettings, simulator_step_atom> {
    Identifier mKey, mHeadKey{};

    std::vector<std::pair<Point<UnitType::Distance, FrameOfRef::Ground>, Vector<UnitType::LinearVelocity, FrameOfRef::Ground>>>
        mBullets;  // pair : [pose velocity]
    std::pair<MotionState, std::unique_ptr<MotionController>> mTarget;
    std::vector<std::pair<Transform<FrameOfRef::Armor, FrameOfRef::Robot, true>, std::pair<double, double>>>
        mTargetArmors;  // pair : (relatedPosition [width height])
    std::pair<MotionState, std::unique_ptr<MotionController>> mSource;
    std::mt19937_64 mEngine{ static_cast<uint64_t>(Clock::now().time_since_epoch().count()) };

    const double mNorThresholdVel = 6.0;

    void initializeTestCase() {
        {
            // SourceMotion
            mSource.first = glm::translate(glm::identity<glm::dmat4>(), { 0.0, mConfig.sourceHeight, 0.0 });

            switch(magic_enum::enum_cast<SourceMotionType>(mConfig.sourceMotionType).value()) {
                case SourceMotionType::Static:
                    mSource.second = std::make_unique<StaticMotionController>();
                    break;

                case SourceMotionType::Sentry:
                    mSource.second = std::make_unique<SentryMotionController>();
                    break;
                case SourceMotionType::Translate2D:
                    mSource.second = std::make_unique<Translate2DMotionController>(mConfig.vibrationLinearRange);
                    break;
                case SourceMotionType::UAV:
                    [[fallthrough]];
                case SourceMotionType::Vibration:
                    [[fallthrough]];
                case SourceMotionType::Translate3D:
                    throw NotImplemented{};
            }
        }

        // Target
        {
            switch(magic_enum::enum_cast<TargetType>(mConfig.targetType).value()) {
                case TargetType::Infantry: {
                    const glm::dmat4 baseTransform =
                        glm::rotate(glm::translate(glm::identity<glm::dmat4>(), { 0.0, 0.0, -radiusOfInfantry }),
                                    15.0 / 180.0 * glm::pi<double>(), { 1.0, 0.0, 0.0 });
                    mTargetArmors.clear();
                    mTargetArmors.reserve(4);
                    mTargetArmors.emplace_back(baseTransform, std::make_pair(widthOfSmallArmor, heightOfSmallArmor));
                    mTargetArmors.emplace_back(
                        glm::rotate(glm::identity<glm::dmat4>(), glm::half_pi<double>(), { 0.0, 1.0, 0.0 }) * baseTransform,
                        std::make_pair(widthOfSmallArmor, heightOfSmallArmor));
                    mTargetArmors.emplace_back(glm::rotate(glm::identity<glm::dmat4>(), glm::pi<double>(), { 0.0, 1.0, 0.0 }) *
                                                   baseTransform,
                                               std::make_pair(widthOfSmallArmor, heightOfSmallArmor));
                    mTargetArmors.emplace_back(
                        glm::rotate(glm::identity<glm::dmat4>(), glm::three_over_two_pi<double>(), { 0.0, 1.0, 0.0 }) *
                            baseTransform,
                        std::make_pair(widthOfSmallArmor, heightOfSmallArmor));
                    break;
                }
                case TargetType::Sentry: {
                    const glm::dmat4 baseTransform =
                        glm::rotate(glm::translate(glm::identity<glm::dmat4>(), { 0.0, 0.0, -radiusOfInfantry * 0.5 }),
                                    -15.0 / 180.0 * glm::pi<double>(), { 1.0, 0.0, 0.0 });
                    mTargetArmors.clear();
                    mTargetArmors.reserve(2);
                    mTargetArmors.emplace_back(baseTransform, std::make_pair(widthOfLargeArmor, heightOfLargeArmor));
                    mTargetArmors.emplace_back(glm::rotate(glm::identity<glm::dmat4>(), glm::pi<double>(), { 0.0, 1.0, 0.0 }) *
                                                   baseTransform,
                                               std::make_pair(widthOfLargeArmor, heightOfLargeArmor));
                    break;
                }
                case TargetType::Outpost: {
                    const glm::dmat4 baseTransform =
                        glm::translate((glm::rotate(glm::translate(glm::identity<glm::dmat4>(), { 0.0, 0.0, -radiusOfOutpost }),
                                                    -15.0 / 180.0 * glm::pi<double>(), { 1.0, 0.0, 0.0 })),
                                       { 0.0, -heightOfSmallArmor / 2, 0.0 });
                    mTargetArmors.clear();
                    mTargetArmors.reserve(3);
                    mTargetArmors.emplace_back(baseTransform, std::make_pair(widthOfSmallArmor, heightOfSmallArmor));
                    mTargetArmors.emplace_back(
                        glm::rotate(glm::identity<glm::dmat4>(), glm::pi<double>() * 2 / 3, { 0.0, 1.0, 0.0 }) * baseTransform,
                        std::make_pair(widthOfSmallArmor, heightOfSmallArmor));
                    mTargetArmors.emplace_back(
                        glm::rotate(glm::identity<glm::dmat4>(), glm::pi<double>() * 4 / 3, { 0.0, 1.0, 0.0 }) * baseTransform,
                        std::make_pair(widthOfSmallArmor, heightOfSmallArmor));
                    break;
                }

                case TargetType::Hero:
                    [[fallthrough]];
                case TargetType::BalancedInfantry:
                    [[fallthrough]];
                case TargetType::BaseClosed:
                    [[fallthrough]];
                case TargetType::BaseExpanded:
                    [[fallthrough]];
                case TargetType::Fans:
                    throw NotImplemented{};
            }
        }

        // TargetMotion
        {
            auto& [motion, controller] = mTarget;
            motion = glm::translate(glm::identity<glm::dmat4>(), { 0.0, mConfig.targetHeight, -mConfig.standardDistance });

            switch(magic_enum::enum_cast<TargetMotionType>(mConfig.targetMotionType).value()) {
                case TargetMotionType::Static: {
                    controller = std::make_unique<StaticMotionController>();
                } break;
                case TargetMotionType::Spinning: {
                    controller = std::make_unique<SpinMotionController>(mConfig.spinningSpeed);
                } break;
                case TargetMotionType::LargeCircle: {
                    controller = std::make_unique<LargeCircleMotionController>(mConfig.spinningSpeed);
                } break;
                case TargetMotionType::Translate2D: {
                    controller = std::make_unique<Translate2DMotionController>(mConfig.vibrationLinearRange);
                } break;
                case TargetMotionType::Translate3D:
                    [[fallthrough]];
                case TargetMotionType::Fans:
                    [[fallthrough]];
                case TargetMotionType::Sentry:
                    throw NotImplemented{};
            }
        }
    }

public:
    Simulator(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {
        initializeTestCase();

        Timer::instance().addTimer(this->address(), 10ms);
    }

    void act() override {
        Scalar<UnitType::Time> time{ 0.0 };
        bool runFlag = true;
        bool shoot = false;
        Scalar<UnitType::Time> lastShoot{ -2 * mConfig.shootInterval };
        uint32_t hitCount = 0;
        uint32_t bulletCount = 0;
        auto& globalSettings = GlobalSettings::get();
        const Scalar<UnitType::Time> dt{ mConfig.step };

        globalSettings.bulletSpeed = mConfig.v0;
        std::normal_distribution vGen{ mConfig.v0, std::fmax(mConfig.v0Std, 1e-3) };
        const Scalar<UnitType::LinearVelocity> maxVelocity{ mConfig.v0 + 3.0 * mConfig.v0Std };
        const Scalar<UnitType::LinearVelocity> minVelocity{ mConfig.v0 - 3.0 * mConfig.v0Std };
        const Scalar<UnitType::Distance> bulletRadius{ GlobalSettings::get().bulletRadius() };

        while(runFlag) {
            for(auto& [pos, v] : mBullets) {
                if(pos.mVal.y < 0.0)
                    continue;

                if(mConfig.printBulletPos) {
                    logInfo(fmt::format("bullet position: {:.2f} {:.2f} {:.2f}", pos.mVal.x, pos.mVal.y, pos.mVal.z));
                    logInfo(fmt::format("bullet velocity: {:.2f} {:.2f} {:.2f}", v.mVal.x, v.mVal.y, v.mVal.z));
                }
            }

            // update drag forces

            // step:update source pose and velocity
            Vector<UnitType::LinearVelocity, FrameOfRef::Ground> vSrc;
            {
                const auto p1 = mSource.first.translatePoint();
                mSource.second->step(mSource.first, dt.mVal);
                const auto p2 = mSource.first.translatePoint();
                vSrc = (p2 - p1) / dt;
            }
            mTarget.second->step(mTarget.first, dt.mVal);

            for(auto& [pos, v] : mBullets) {
                if(pos.mVal.y < 0.0)
                    continue;
                pos += v * dt;
                v += glm::dvec3{ 0.0, GlobalSettings::get().gForce * dt.mVal, 0.0 };
            }

            time += dt;

            // update world info
            {
                SimulatorWorldInfo info;

                info.lastUpdate = TimePoint{ static_cast<Duration>(
                    static_cast<Clock::rep>(time.mVal * Clock::period::den / Clock::period::num)) };
                SynchronizedClock::instance().setSimulationTime(info.lastUpdate);

                info.tfGround2Robot = mSource.first.invTransformObj();

                {
                    const auto& targetMotion = mTarget.first;

                    for(auto& [armorRelatedMotion, threshold] : mTargetArmors) {
                        info.targets.emplace_back(combine(targetMotion, armorRelatedMotion).translatePoint());
                    }
                }

                sendAll(simulator_step_atom_v, BlackBoard::instance().updateSync(mKey, std::move(info)));
            }

            // update collisions
            {
                const auto& tfRobot2Ground = mTarget.first;
                for(size_t i = 0; i < mTargetArmors.size(); i++) {
                    auto& tfArmor2Robot = mTargetArmors[i].first;
                    auto& area = mTargetArmors[i].second;
                    const auto tfArmor2Ground = combine(tfRobot2Ground, tfArmor2Robot);
                    const auto tfGround2Armor = tfArmor2Ground.invTransformObj();

                    for(auto& bullet : mBullets) {
                        if(bullet.first.mVal.y < 0.0)
                            continue;
                        const auto posRefArmor = tfGround2Armor(bullet.first);
                        const auto velRefArmor = tfGround2Armor(bullet.second);

                        /* If the projection of the velocity of the bullet in the normal direction of the armor plate is less
                        than a certain threshold and it's projection is within armor, the bullet can hit */
                        if(posRefArmor.mVal.z > -bulletRadius.mVal && std::fabs(posRefArmor.mVal.x) <= area.first / 2 &&
                           std::fabs(posRefArmor.mVal.y) <= area.second / 2 && velRefArmor.mVal.z >= mNorThresholdVel) {
                            bullet.first.mVal.y = -1.0;
                            logInfo(fmt::format("Hit at armor[{}] ({:.2f},{:.2f},{:.2f})", i,
                                                tfArmor2Ground.translatePoint().mVal.x, tfArmor2Ground.translatePoint().mVal.y,
                                                tfArmor2Ground.translatePoint().mVal.z));
                            ++hitCount;
                        }
                    }
                }
            }

            // update events
            receive(
                [&](set_target_info_atom, GroupMask, Clock::rep, const double, const double, const bool isFire) {
                    ACTOR_PROTOCOL_CHECK(set_target_info_atom, GroupMask, Clock::rep, double, double, bool);
                    shoot = isFire;
                },
                [&](update_head_atom, GroupMask, Identifier key) {
                    ACTOR_PROTOCOL_CHECK(update_head_atom, GroupMask, TypedIdentifier<HeadInfo>);
                    mHeadKey = key;
                },
                [&](const caf::down_msg&) { runFlag = false; }, [&](const caf::exit_msg&) { runFlag = false; },
                [&](timer_atom) { ACTOR_PROTOCOL_CHECK(timer_atom); });

            const auto headData = BlackBoard::instance().get<HeadInfo>(mHeadKey);
            Transform<FrameOfRef::Robot, FrameOfRef::Gun, true> tfRobot2Gun{ glm::identity<glm::dmat4>() };
            if(headData.has_value()) {
                tfRobot2Gun = headData.value().tfRobot2Gun;
            }

            const auto tfGun2Ground = combine(tfRobot2Gun.invTransformObj(), mSource.first);

            // shoot
            if(shoot && bulletCount < mConfig.bulletCount && time - lastShoot > mConfig.shootInterval) {
                const Scalar<UnitType::LinearVelocity> v{ std::clamp(vGen(mEngine), minVelocity.mVal, maxVelocity.mVal) };
                GlobalSettings::get().bulletSpeed = v.mVal;

                const auto velocity = tfGun2Ground(Normal<FrameOfRef::Gun>({ 0.0, 0.0, -1.0f }, Normalized())) * v;

                mBullets.emplace_back(tfGun2Ground.translatePoint(), vSrc + velocity);

                lastShoot = time;
                ++bulletCount;
            }

            logInfo(
                fmt::format("Simulator time {:.3f}s bullet count {} hit {} shoot {}", time.mVal, bulletCount, hitCount, shoot));
            {
                const auto posSrc = mSource.first.translatePoint();

                logInfo(fmt::format("Source position   {:.3f} {:.3f} {:.3f}", posSrc.mVal.x, posSrc.mVal.y, posSrc.mVal.z));

                const auto posDst = mTarget.first.translatePoint();

                logInfo(fmt::format("Target position  {:.3f} {:.3f} {:.3f}", posDst.mVal.x, posDst.mVal.y, posDst.mVal.z));

                auto diff = normalize(posDst - posSrc);

                logInfo(fmt::format("Ref dir(Dst ref Src) {:.3f} {:.3f} {:.3f}", diff.raw().x, diff.raw().y, diff.raw().z));

                const auto real = normalize(tfGun2Ground(Vector<UnitType::Distance, FrameOfRef::Gun>{ 0.0, 0.0, -1.0f }));
                logInfo(fmt::format("Gun dir {:.3f} {:.3f} {:.3f}", real.raw().x, real.raw().y, real.raw().z));

                Scalar<UnitType::Distance> minDist{ 1e10 };
                std::optional<
                    std::pair<Point<UnitType::Distance, FrameOfRef::Ground>, Point<UnitType::Distance, FrameOfRef::Ground>>>
                    closest = std::nullopt;

                for(auto& trans : mTargetArmors) {
                    const auto pos = combine(mTarget.first, std::get<0>(trans)).translatePoint();

                    for(const auto& [p, v] : mBullets) {
                        if(p.mVal.y < 0.0)
                            continue;
                        if(const auto dist = distance(p, pos); dist < minDist) {
                            minDist = dist;
                            closest = { pos, p };
                        }
                    }
                }

                if(closest) {
                    const auto [p1, p2] = closest.value();
                    logInfo(fmt::format("Closest pair armor {:.3f} {:.3f} {:.3f} <-> bullet {:.3f} {:.3f} {:.3f} : {:.3f} m",
                                        p1.mVal.x, p1.mVal.y, p1.mVal.z, p2.mVal.x, p2.mVal.y, p2.mVal.z, minDist.mVal));
                }
            }

            if(time - mConfig.maxTime > -1e-4) {
                runFlag = false;
            }
            if(runFlag && bulletCount == mConfig.bulletCount) {
                runFlag = false;
                for(auto& [p, v] : mBullets) {
                    if(p.mVal.y >= 0.0) {
                        runFlag = true;
                        break;
                    }
                }
            }
            std::this_thread::sleep_for(10ms);
        }

        logInfo(fmt::format("Expected {} Result {}", mConfig.expectedCount, hitCount));
        appendTestResult(fmt::format("Result {}/{} (Require {}, Shoot {})", hitCount, mConfig.bulletCount, mConfig.expectedCount,
                                     bulletCount));

        if(hitCount < mConfig.expectedCount) {
            logError("Test failed");
            terminateSystem(*this, false);
        } else
            terminateSystem(*this, true);
    }
};

HUB_REGISTER_CLASS(Simulator);
