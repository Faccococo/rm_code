#pragma once
#include "SuppressWarningBegin.hpp"

#include <caf/allowed_unsafe_message_type.hpp>
#include <caf/type_id.hpp>

#include "SuppressWarningEnd.hpp"

struct ClassifiedNum final {
    int32_t num;
    double confidence;
};

CAF_BEGIN_TYPE_ID_BLOCK(ClassifiedNum, 300);

CAF_ADD_TYPE_ID(ClassifiedNum, (ClassifiedNum));

CAF_END_TYPE_ID_BLOCK(ClassifiedNum);

CAF_ALLOW_UNSAFE_MESSAGE_TYPE(ClassifiedNum);

ACTOR_PROTOCOL_DEFINE(num_classify_request_atom, TypedIdentifier<CameraFrame>);
