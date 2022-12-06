#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "ExceptionProbe.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"
#include "Utility.hpp"
#include <cstdint>
#include <utility>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <eigen3/Eigen/Dense>
#include <fmt/format.h>
#include <magic_enum.hpp>

constexpr double maxDeltaTime = 0.2;
constexpr int32_t maxCntWrongData = 5;

constexpr double maxJumpXDist = 0.5;
constexpr double maxJumpYDist = 0.5;
constexpr double maxJumpZDist = 0.5;

constexpr double maxXVel = 3.0;
constexpr double maxYVel = 3.0;
constexpr double maxZVel = 1.0;

struct ArmorPredictorSettings final {
    bool enablePredictor;
};

template <class Inspector>
bool inspect(Inspector& f, ArmorPredictorSettings& x) {
    return f.object(x).fields(f.field("enalePredictor", x.enablePredictor));
}

class ArmorPredictor final : public HubHelper<caf::event_based_actor, ArmorPredictorSettings, predict_success_atom> {
    Identifier mKey, mIMUKey, mHeadKey;
    GroupMask mGroupMask;

    bool mInitFlag = false;
    int32_t mCntWrongData = 0;
    glm::dvec3 mLastPos;
    TimePoint mLastTimePoint;
    glm::dvec3 mPredictedVel;
    Eigen::VectorXd mX;  // State vector(Position & Velocity)
    Eigen::MatrixXd mF;  // State tfRobot2Gun mat
    Eigen::MatrixXd mP;  // State covariance mat
    Eigen::MatrixXd mQ;  // Process covariance mat
    Eigen::MatrixXd mH;  // Measurement mat
    Eigen::MatrixXd mR;  // Measurement covariance mat

    void setX(const glm::dvec3& measuredPos) {
        Eigen::VectorXd initX(6, 1);
        initX << measuredPos.x, measuredPos.y, measuredPos.z, 0.0, 0.0, 0.0;
        mX = initX;
    }

    //检查数据是否跳变，跳变数据连续出现五次，则重新初始化滤波器
    glm::dvec3 filterWrongData(const glm::dvec3& measuredPos) {
        glm::dvec3 filterRes = measuredPos;
        bool isWrongData = false;
        if(mCntWrongData < maxCntWrongData) {  //如果跳变数据小于maxCntWrongData,则继续计数
            if(std::fabs(measuredPos.x - mLastPos.x) > maxJumpXDist) {
                isWrongData = true;
                filterRes.x = mLastPos.x;
            }
            if(std::fabs(measuredPos.y - mLastPos.y) > maxJumpYDist) {
                isWrongData = true;
                filterRes.y = mLastPos.y;
            }
            if(std::fabs(measuredPos.z - mLastPos.z) > maxJumpXDist) {
                isWrongData = true;
                filterRes.z = mLastPos.z;
            }
        } else {  //跳变数据个数超过maxCntWrongData, 则重新初始化滤波器
            mInitFlag = false;
            mCntWrongData = 0;
        }

        if(isWrongData) {  //无跳变数据出现,为正常数据，对目标进行速度预测
            mCntWrongData = 0;
        } else {
            ++mCntWrongData;  //跳变数据出现，进行计数
        }
        return filterRes;
    }

    void initialKalmanFilter(const glm::dvec3& measuredPos, const TimePoint& curTimePoint) {
        mX.resize(6);
        mF.setIdentity(6, 6);
        mP.setIdentity(6, 6);
        mQ.setIdentity(6, 6);
        mH.resize(3, 6);
        mH << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,  //
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,    //
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0;    //
        mR.resize(3, 3);
        mR << 0.01, 0.0, 0.0,  //
            0.0, 0.01, 0.0,    //
            0.0, 0.0, 0.01;

        mLastPos = measuredPos;
        mLastTimePoint = curTimePoint;
        setX(measuredPos);
        mInitFlag = true;
    }

    void Prediction() {
        mX = mF * mX;
        mP = mF * mP * mF.transpose() + mQ;
    }

    void UpdateMeasurement(const Eigen::VectorXd z) {
        const auto y = z - mH * mX;  // Measure
        const auto S = mH * mP * mH.transpose() + mR;
        const auto K = mP * mH.transpose() * S.inverse();  // Kalman Gain
        mX = mX + (K * y);                                 // Optimal estimate
        const auto I = Eigen::MatrixXd::Identity(mX.size(), mX.size());
        mP = (I - K * mH) * mP;
    }

    void KmFilter(const glm::dvec3& pos, double dt) {
        Eigen::MatrixXd inputF(6, 6);
        inputF << 1.0, 0.0, 0.0, dt, 0.0, 0.0,  //
            0.0, 1.0, 0.0, 0.0, dt, 0.0,        //
            0.0, 0.0, 1.0, 0.0, 0.0, dt,        //
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,       //
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,       //
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;       //
        mF << inputF;

        Prediction();
        Eigen::VectorXd measuredZ(3, 1);
        measuredZ << pos.x, pos.y, pos.z;
        UpdateMeasurement(measuredZ);

        mPredictedVel = { mX(3), mX(4), mX(5) };
        if(mPredictedVel.x > maxXVel) {  //限幅滤波
            mPredictedVel.x = maxXVel;
        }
        if(mPredictedVel.y > maxYVel) {
            mPredictedVel.y = maxYVel;
        }
        if(mPredictedVel.z > maxZVel) {
            mPredictedVel.z = maxZVel;
        }
    }

    void runFilter(const glm::dvec3& measuredPos, const TimePoint& curTimePoint) {
        if(!mInitFlag) {
            initialKalmanFilter(measuredPos, curTimePoint);
            return;
        }

        double deltaTime =
            static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(curTimePoint - mLastTimePoint).count()) /
            1e6;
        if(deltaTime > maxDeltaTime) {
            initialKalmanFilter(measuredPos, curTimePoint);
            return;
        }
        const auto filterPos = filterWrongData(measuredPos);
        if(!mInitFlag) {
            initialKalmanFilter(measuredPos, curTimePoint);
            return;
        }
        KmFilter(filterPos, deltaTime);
    }

public:
    ArmorPredictor(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}

    caf::behavior make_behavior() override {
        return {
            [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](set_target_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(set_target_atom, TypedIdentifier<SelectedTarget>);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<SelectedTarget>(key);
                const auto dataHeadInfo = BlackBoard::instance().get<HeadInfo>(mHeadKey);
                const auto dataPosture = BlackBoard::instance().get<PostureData>(mIMUKey);
                if(!(data.has_value() && data.value().selected.has_value() && dataHeadInfo.has_value() &&
                     dataPosture.has_value()))
                    return;

                Vector<UnitType::Distance, FrameOfRef::Gun> posOfRefGun(data.value().selected.value().center.mVal);
                Vector<UnitType::Distance, FrameOfRef::Robot> posRefRobot =
                    dataHeadInfo.value().tfRobot2Gun.invTransform(posOfRefGun);
                Vector<UnitType::LinearVelocity, FrameOfRef::Ground> linearVelocity(
                    dataPosture.value().linearVelocityOfRobot.mVal);
                data.value().position = posRefRobot;

                if(mConfig.enablePredictor) {  //如果使用预测功能的话，目标相对机器人的速度即为机器人坐标系下，相机所观测的速度
                    glm::dvec3 measuredPos = posRefRobot.mVal;
                    runFilter(measuredPos, data.value().lastUpdate);
                    data.value().selected.value().velocity.mVal = mPredictedVel;
                } else {  //如果不使用预测功能的话，将目标看作为静止状态，目标相对机器人的速度即为机器人自身速度取反
                    data.value().selected.value().velocity.mVal = -linearVelocity.mVal;
                }

                sendAll(predict_success_atom_v,
                        BlackBoard::instance().updateSync<SelectedTarget>(Identifier{ mKey.val }, data.value()));
            },
            [this](update_head_atom, GroupMask, Identifier key) {
                ACTOR_PROTOCOL_CHECK(update_head_atom, GroupMask, TypedIdentifier<HeadInfo>);
                mHeadKey = key;
            },
            [this](update_posture_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(update_posture_atom, TypedIdentifier<PostureData>);
                mIMUKey = key;
            }

        };
    }
};

HUB_REGISTER_CLASS(ArmorPredictor);
