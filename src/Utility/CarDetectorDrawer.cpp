#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedCar.hpp"
#include "Hub.hpp"
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <caf/actor_ostream.hpp>
#include <caf/event_based_actor.hpp>

#include "SuppressWarningEnd.hpp"

class CarDetectorDrawer final : public HubHelper<caf::event_based_actor, void, image_frame_atom> {
private:
    Identifier mKey;

public:
    CarDetectorDrawer(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, mKey{ generateKey(this) } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [this](car_detect_available_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(car_detect_available_atom, TypedIdentifier<DetectedCarArray>);
                     auto res = BlackBoard::instance().get<DetectedCarArray>(key).value();
                     const cv::Scalar green{ 0.0, 255.0, 0.0 };
                     for(const auto& rect : res.cars) {
                         cv::rectangle(res.frame.frame, rect, green, 3);
                     }

                     sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res.frame)));
                 } };
    }
};

HUB_REGISTER_CLASS(CarDetectorDrawer);
