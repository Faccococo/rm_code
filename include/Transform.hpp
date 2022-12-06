#pragma once
#include <atomic>
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <glm/glm.hpp>

#include "SuppressWarningEnd.hpp"

enum class FrameOfRef : uint32_t { Ground, Robot, Gun, Camera, Armor };

enum class UnitType : uint32_t {
    Distance,
    Angle,
    Time,
    LinearVelocity,
    AngularVelocity,
    LinearAcceleration,
    AngularAcceleration,
    Undefined
};

template <UnitType Lhs, UnitType Rhs>
constexpr UnitType multiply = UnitType::Undefined;

template <>
constexpr UnitType multiply<UnitType::LinearVelocity, UnitType::Time> = UnitType::Distance;

template <>
constexpr UnitType multiply<UnitType::LinearAcceleration, UnitType::Time> = UnitType::LinearVelocity;

template <>
constexpr UnitType multiply<UnitType::AngularVelocity, UnitType::Time> = UnitType::Angle;

template <>
constexpr UnitType multiply<UnitType::AngularAcceleration, UnitType::Time> = UnitType::AngularVelocity;

template <UnitType Lhs, UnitType Rhs>
constexpr UnitType division = UnitType::Undefined;

template <>
constexpr UnitType division<UnitType::Distance, UnitType::Time> = UnitType::LinearVelocity;

template <>
constexpr UnitType division<UnitType::Angle, UnitType::Time> = UnitType::AngularVelocity;

template <>
constexpr UnitType division<UnitType::LinearVelocity, UnitType::Time> = UnitType::LinearAcceleration;

template <>
constexpr UnitType division<UnitType::AngularVelocity, UnitType::Time> = UnitType::AngularAcceleration;

template <UnitType Unit>
struct Scalar final {
    double mVal;

    Scalar() = default;
    Scalar(double val) : mVal(val) {}

    constexpr Scalar& operator=(double val) {
        mVal = val;
        return *this;
    }
    constexpr Scalar& operator=(const Scalar& rhs) {
        mVal = rhs.mVal;
        return *this;
    }

    Scalar operator+(const Scalar& rhs) const noexcept {
        return mVal + rhs.mVal;
    }
    Scalar& operator+=(const Scalar& rhs) noexcept {
        mVal += rhs.mVal;
        return (*this);
    }
    Scalar operator-(const Scalar& rhs) const noexcept {
        return mVal - rhs.mVal;
    }
    Scalar& operator-=(const Scalar& rhs) noexcept {
        mVal -= rhs.mVal;
        return (*this);
    }
    template <UnitType RhsUnit>
    Scalar<multiply<Unit, RhsUnit>> operator*(Scalar<RhsUnit> rhs) const noexcept {
        return Scalar<multiply<Unit, RhsUnit>>{ mVal * rhs.mVal };
    }
    template <UnitType RhsUnit>
    Scalar<division<Unit, RhsUnit>> operator/(Scalar<RhsUnit> rhs) const noexcept {
        return Scalar<division<Unit, RhsUnit>>{ mVal / rhs.mVal };
    }
    bool operator>(const Scalar& rhs) const noexcept {
        return mVal > rhs.mVal;
    }
    bool operator>=(const Scalar& rhs) const noexcept {
        return mVal >= rhs.mVal;
    }
    bool operator<(const Scalar& rhs) const noexcept {
        return mVal < rhs.mVal;
    }
    bool operator<=(const Scalar& rhs) const noexcept {
        return mVal <= rhs.mVal;
    }
    bool operator==(const Scalar& rhs) const noexcept {
        return mVal == rhs.mVal;
    }
    bool operator!=(const Scalar& rhs) const noexcept {
        return mVal != rhs.vmVall;
    }
};

template <UnitType Unit, FrameOfRef FoR>
class Vector final {
public:
    glm::dvec3 mVal;

    Vector() = default;
    Vector(double x, double y, double z) : mVal(x, y, z) {}
    Vector(const glm::dvec3& val) : mVal(val) {}
    Vector(glm::dvec3&& val) : mVal(std::move(val)) {}

    Vector& operator=(const glm::dvec3& rhsVal) {
        mVal = rhsVal;
        return *this;
    }

    Vector& operator=(glm::dvec3&& rhsVal) {
        mVal = std::move(rhsVal);
        return *this;
    }

    [[nodiscard]] auto& operator[](int i) noexcept {
        return mVal[i];
    }
    [[nodiscard]] const auto& operator[](int i) const noexcept {
        return mVal[i];
    }

    bool operator==(const Vector& rhs) const noexcept {
        return mVal == rhs.mVal;
    }
    bool operator!=(const Vector& rhs) const noexcept {
        return mVal != rhs.mVal;
    }

    Vector operator+(const Vector& rhs) const noexcept {
        return mVal + rhs.mVal;
    }
    Vector& operator+=(const Vector& rhs) noexcept {
        mVal += rhs.mVal;
        return *this;
    }

    Vector operator-(const Vector& rhs) const noexcept {
        return mVal - rhs.mVal;
    }
    Vector& operator-=(const Vector& rhs) noexcept {
        mVal -= rhs.mVal;
        return *this;
    }

    template <UnitType RhsUnit>
    auto operator*(const Scalar<RhsUnit>& rhs) const noexcept {
        return Vector<multiply<Unit, RhsUnit>, FoR>(mVal * rhs.mVal);
    }
    template <UnitType RhsUnit>
    auto operator/(const Scalar<RhsUnit>& rhs) const noexcept {
        return Vector<division<Unit, RhsUnit>, FoR>(mVal / rhs.mVal);
    }
    Vector operator-() const noexcept {
        return -mVal;
    }
};

template <UnitType Lhs, UnitType Rhs, FrameOfRef FoR>
auto operator*(const Scalar<Lhs>& lhs, const Vector<Rhs, FoR>& rhs) noexcept {
    return rhs * lhs;
}

template <UnitType Unit, FrameOfRef FoR>
auto length(const Vector<Unit, FoR>& val) noexcept {
    return Scalar<Unit>{ glm::length(val.mVal) };
}

template <UnitType Unit, FrameOfRef FoR>
auto lerp(const Vector<Unit, FoR>& a, const Vector<Unit, FoR>& b, double u) noexcept {
    return Vector<Unit, FoR>{ glm::mix(a.mVal, b.mVal, u) };
}

template <UnitType Unit, FrameOfRef FoR>
class Point final {
public:
    glm::dvec3 mVal;

    Point() = default;
    Point(double x, double y, double z) : mVal(x, y, z) {}
    Point(const glm::dvec3& val) : mVal(val) {}
    Point(glm::dvec3&& val) : mVal(std::move(val)) {}

    Point& operator=(const glm::dvec3& rhsVal) {
        mVal = rhsVal;
        return *this;
    }
    Point& operator=(glm::dvec3&& rhsVal) {
        mVal = std::move(rhsVal);
        return *this;
    }

    [[nodiscard]] auto& operator[](int i) noexcept {
        return mVal[i];
    }
    [[nodiscard]] const auto& operator[](int i) const noexcept {
        return mVal[i];
    }

    bool operator==(const Point& rhs) const noexcept {
        return mVal == rhs.mVal;
    }
    bool operator!=(const Point& rhs) const noexcept {
        return mVal != rhs.mVal;
    }

    Point operator+(const Vector<Unit, FoR>& rhs) const noexcept {
        return mVal + rhs.mVal;
    }
    Point& operator+=(const Vector<Unit, FoR>& rhs) noexcept {
        mVal += rhs.mVal;
        return *this;
    }

    Point operator-(const Vector<Unit, FoR>& rhs) const noexcept {
        return mVal - rhs.mVal;
    }
    Point& operator-=(const Vector<Unit, FoR>& rhs) noexcept {
        mVal -= rhs.mVal;
        return *this;
    }
    Vector<Unit, FoR> operator-(const Point& rhs) const noexcept {
        return mVal - rhs.mVal;
    }
};

template <UnitType Unit, FrameOfRef FoR>
auto lerp(const Point<Unit, FoR>& a, const Point<Unit, FoR>& b, double u) noexcept {
    return Point<Unit, FoR>{ glm::mix(a.mVal, b.mVal, u) };
}

template <UnitType Unit, FrameOfRef FoR>
auto distance(const Point<Unit, FoR>& a, const Point<Unit, FoR>& b) noexcept {
    return Scalar<Unit>{ glm::distance(a.mVal, b.mVal) };
}

struct Normalized final {};

template <FrameOfRef FoR>
class Normal final {
    glm::dvec3 mVal;

public:
    Normal(const glm::dvec3& val, Normalized) : mVal(val) {}
    Normal(const glm::dvec3& val) : mVal(glm::normalize(val)) {}

    template <UnitType Unit>
    explicit Normal(const Vector<Unit, FoR>& v, Normalized) : mVal{ v.mVal } {}
    template <UnitType Unit>
    explicit Normal(const Vector<Unit, FoR>& v) : mVal{ glm::normalize(v.mVal) } {}

    template <UnitType Unit>
    auto operator*(const Scalar<Unit> distance) const noexcept {
        return Vector<Unit, FoR>{ mVal * distance.mVal };
    }
    template <UnitType Unit>
    auto operator*(const Vector<Unit, FoR> rhs) const noexcept {
        return Scalar<Unit>{ mVal * rhs.mVal };
    }
    Normal operator-() const noexcept {
        return { -mVal, Normalized{} };
    }

    [[nodiscard]] const glm::dvec3& raw() const noexcept {
        return mVal;
    }
};

template <FrameOfRef FoR>
auto cross(const Normal<FoR>& a, const Normal<FoR>& b) noexcept {
    return Normal<FoR>{ glm::cross(a, b), Normalized{} };
}

template <FrameOfRef FoR>
auto dot(const Normal<FoR>& a, const Normal<FoR>& b) noexcept {
    return glm::dot(a.raw(), b.raw());
}

template <UnitType Unit, FrameOfRef FoR>
auto dot(const Vector<Unit, FoR>& a, const Normal<FoR>& b) noexcept {
    return glm::dot(a.mVal, b.raw());
}

template <UnitType Unit, FrameOfRef FoR>
auto dot(const Normal<FoR>& a, const Vector<Unit, FoR>& b) noexcept {
    return glm::dot(a.raw(), b.mVal);
}

template <UnitType Unit, FrameOfRef FoR>
auto dot(const Vector<Unit, FoR>& a, const Vector<Unit, FoR>& b) noexcept {
    return glm::dot(a.mVal, b.mVal);
}

template <UnitType Unit, FrameOfRef FoR>
auto normalize(Vector<Unit, FoR> v) {
    return Normal<FoR>{ v };
}

template <FrameOfRef A, FrameOfRef B, bool HasTranslate = false>
class Transform final {
    glm::dmat4 mTransform;  // transform A to B

public:
    Transform() = default;

    Transform(const glm::dmat4& transform) : mTransform(transform) {}
    Transform(glm::dmat4&& transform) : mTransform(std::move(transform)) {}

    Transform& operator=(const glm::dmat4& rhs) {
        mTransform = rhs;
        return *this;
    }
    Transform& operator=(glm::dmat4&& rhs) {
        mTransform = std::move(rhs);
        return *this;
    }

    template <UnitType Unit>
    std::enable_if_t<HasTranslate, Point<Unit, B>> operator()(const Point<Unit, A> rhs) const noexcept {
        return glm::dvec3{ mTransform * glm::dvec4{ rhs.mVal, 1.0 } };
    }
    template <UnitType Unit>
    Vector<Unit, B> operator()(const Vector<Unit, A> rhs) const noexcept {
        return glm::dvec3{ mTransform * glm::dvec4{ rhs.mVal, 0.0 } };
    }

    Normal<B> operator()(const Normal<A> rhs) const noexcept {
        return glm::dvec3{ mTransform * glm::dvec4{ rhs.raw(), 0.0 } };
    }

    operator Transform<A, B, false>() const noexcept {
        return Transform<A, B, false>(mTransform);
    }

    const glm::dmat4& raw() const noexcept {
        return mTransform;
    }

    auto invTransformMat() const noexcept {
        return glm::inverse(mTransform);
    }

    auto invTransformObj() const noexcept {
        return Transform<B, A, HasTranslate>(glm::inverse(mTransform));
    }

    template <UnitType Unit>
    std::enable_if_t<HasTranslate, Point<Unit, A>> invTransform(const Point<Unit, B> rhs) const noexcept {
        return glm::dvec3{ glm::inverse(mTransform) * glm::dvec4{ rhs.mVal, 1.0 } };
    }

    Normal<A> invTransform(const Normal<B> rhs) const noexcept {
        return glm::dvec3{ glm::inverse(mTransform) * glm::dvec4{ rhs.mVal, 0.0 } };
    }

    template <UnitType Unit>
    Vector<Unit, A> invTransform(const Vector<Unit, B> rhs) const noexcept {
        return glm::dvec3{ glm::inverse(mTransform) * glm::dvec4{ rhs.mVal, 0.0 } };
    }

    // translatePoint under B
    Point<UnitType::Distance, B> translatePoint() const {
        return Point<UnitType::Distance, B>(mTransform[3]);
    }
};

template <FrameOfRef A, FrameOfRef B, bool LhsHasTranslate, bool RhsHasTranslate>
auto combine(const Transform<A, B, LhsHasTranslate>& first, const Transform<A, B, RhsHasTranslate>& second) noexcept {
    return Transform < A, B, LhsHasTranslate && RhsHasTranslate > (second.raw() * first.raw());
}

template <FrameOfRef A, FrameOfRef B, FrameOfRef C, bool LhsHasTranslate, bool RhsHasTranslate>
auto combine(const Transform<A, B, LhsHasTranslate>& first, const Transform<B, C, RhsHasTranslate>& second) noexcept {
    return Transform < A, C, LhsHasTranslate && RhsHasTranslate > (second.raw() * first.raw());
}

template <FrameOfRef A, FrameOfRef B, FrameOfRef C, bool LhsHasTranslate, bool RhsHasTranslate>
auto combine(const Transform<B, C, LhsHasTranslate>& second, const Transform<A, B, RhsHasTranslate>& first) noexcept {
    return Transform < A, C, LhsHasTranslate && RhsHasTranslate > (second.raw() * first.raw());
}