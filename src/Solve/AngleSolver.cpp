#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "ExceptionProbe.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "PostureData.hpp"
#include "SelectedTarget.hpp"
#include "Utility.hpp"
#include <cmath>
#include <complex>

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <fmt/format.h>
#include <magic_enum.hpp>

#include "SuppressWarningEnd.hpp"

struct AngleSolverSettings final {
    double precision;
    double delay;
};

template <class Inspector>
bool inspect(Inspector& f, AngleSolverSettings& x) {
    return f.object(x).fields(f.field("precision", x.precision), f.field("delay", x.delay));
}

class AngleSolver final : public HubHelper<caf::event_based_actor, AngleSolverSettings, set_target_info_atom> {
    Identifier mKey, mIMUKey, mHeadKey;

public:
    AngleSolver(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}

    static std::complex<double> sqrtN(const std::complex<double>& x, double n) {
        if(auto r = std::hypot(x.real(), x.imag()); r > 0.0) {
            auto a = std::atan2(x.imag(), x.real());
            n = 1.0 / n;
            r = std::pow(r, n);
            a *= n;
            return std::polar(r, a);
        }
        return {};
    }

    static double ferrari(std::complex<double> a, std::complex<double> b, std::complex<double> c, std::complex<double> d,
                          std::complex<double> e) {
        std::complex<double> x[4];
        a = 1.0 / a;
        b *= a;
        c *= a;
        d *= a;
        e *= a;
        const auto p = (c * c + 12.0 * e - 3.0 * b * d) / 9.0;
        const auto q = (27.0 * d * d + 2.0 * c * c * c + 27.0 * b * b * e - 72.0 * c * e - 9.0 * b * c * d) / 54.0;
        const auto D = sqrtN(q * q - p * p * p, 2.0);
        std::complex<double> u = q + D;
        std::complex<double> v = q - D;
        if(v.real() * v.real() + v.imag() * v.imag() > u.real() * u.real() + u.imag() * u.imag()) {
            u = sqrtN(v, 3.0);
        } else {
            u = sqrtN(u, 3.0);
        }
        std::complex<double> y;
        if(u.real() * u.real() + u.imag() * u.imag() > 0.0) {
            v = p / u;
            const std::complex<double> o1(-0.5, +0.86602540378443864676372317075294);
            const std::complex<double> o2(-0.5, -0.86602540378443864676372317075294);
            std::complex<double>& yMax = x[0];
            double m2Max = 0.0;
            // int iMax = -1;
            for(int i = 0; i < 3; ++i) {
                y = u + v + c / 3.0;
                u *= o1;
                v *= o2;
                a = b * b + 4.0 * (y - c);
                if(const auto m2 = a.real() * a.real() + a.imag() * a.imag(); 0 == i || m2Max < m2) {
                    m2Max = m2;
                    yMax = y;
                    // iMax = i;
                }
            }
            y = yMax;
        } else {
            y = c / 3.0;
        }
        if(const auto m = sqrtN(b * b + 4.0 * (y - c), 2.0); m.real() * m.real() + m.imag() * m.imag() >= DBL_MIN) {
            const std::complex<double> n = (b * y - 2.0 * d) / m;

            a = sqrtN((b + m) * (b + m) - 8.0 * (y + n), 2.0);
            x[0] = (-(b + m) + a) / 4.0;
            x[1] = (-(b + m) - a) / 4.0;
            a = sqrtN((b - m) * (b - m) - 8.0 * (y - n), 2.0);
            x[2] = (-(b - m) + a) / 4.0;
            x[3] = (-(b - m) - a) / 4.0;
        } else {
            a = sqrtN(b * b - 8.0 * y, 2.0);
            x[0] = x[1] = (-b + a) / 4.0;
            x[2] = x[3] = (-b - a) / 4.0;
        }
        double ans = 1000;
        for(auto& i : x) {
            if(i.real() > 0 && std::fabs(i.imag()) < 1e7 && i.real() < ans)
                ans = i.real();
        }
        return ans;
    }

    caf::behavior make_behavior() override {
        return {
            [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
            [this](predict_success_atom, Identifier key) {
                ACTOR_PROTOCOL_CHECK(predict_success_atom, TypedIdentifier<SelectedTarget>);
                ACTOR_EXCEPTION_PROBE();

                auto data = BlackBoard::instance().get<SelectedTarget>(key);
                if(!(data.has_value() && data.value().selected.has_value()))
                    return;
                HubLogger::watch("armor type", magic_enum::enum_name(data.value().selected.value().type));

                const auto& globalSettings = GlobalSettings::get();
                const double g = globalSettings.gForce, bulletSpeed = globalSettings.bulletSpeed;

                constexpr auto square = [=](const double x) { return x * x; };

                const auto delayTime = mConfig.delay;

                Vector<UnitType::Distance, FrameOfRef::Robot> posRefRobot = data.value().position;
                Vector<UnitType::LinearVelocity, FrameOfRef::Robot> linearVel = data.value().selected.value().velocity;
                HubLogger::watch("z", posRefRobot.mVal.z);

                //(forward:+y,right:+x)
                //(forward:+y,right:+x)
                glm::dvec3 tfPos = {posRefRobot.mVal.x, -posRefRobot.mVal.z, posRefRobot.mVal.y} ;
                glm::dvec3 tfLinearVel = { linearVel.mVal.x, -linearVel.mVal.z, linearVel.mVal.y };

                // logInfo(fmt::format("Source Velocity {} {} {}", linearVelocity.raw().x, linearVelocity.raw().y,
                // linearVelocity.raw().z));
                tfPos = { tfPos.x + delayTime * tfLinearVel.x, tfPos.y + delayTime * tfLinearVel.y,
                          tfPos.z + delayTime * tfLinearVel.z };

                double airDuration =
                    ferrari(1, 0,
                            -(4 * g * tfPos.z + 4 * square(bulletSpeed) - 4 * square(tfLinearVel.x) - 4 * square(tfLinearVel.y)) /
                                square(g),
                            (8 * tfPos.x * tfLinearVel.x + 8 * tfPos.y * tfLinearVel.y) / square(g),
                            (4 * square(tfPos.x) + 4 * square(tfPos.y) + 4 * square(tfPos.z)) / square(g));
                double verticalSpeed = tfPos.z / airDuration - 0.5 * g * airDuration;
                double horizontalSpeedX = (tfPos.x + tfLinearVel.x * airDuration) / airDuration;
                double horizontalSpeedY = (tfPos.y + tfLinearVel.y * airDuration) / airDuration;

                double pitchAngle = std::asin(verticalSpeed / bulletSpeed);
                double yawAngle = std::atan2(horizontalSpeedY, horizontalSpeedX) - glm::half_pi<double>();

                bool isFire = true;
                // logInfo(fmt::format("x:{}, y:{}, z:{}", tfPos.x, tfPos.y, tfPos.z));
                sendAll(set_target_info_atom_v, mGroupMask, data.value().lastUpdate.time_since_epoch().count(), yawAngle,
                        pitchAngle, isFire);
            },

        };
    }
};

HUB_REGISTER_CLASS(AngleSolver);
