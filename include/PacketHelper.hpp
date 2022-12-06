#pragma once

#include "Crc.hpp"
#include <cstdint>
#include <cstring>

template <int bufferSize>
class PacketReader {
    const std::array<uint8_t, bufferSize>& buffer;
    int now = 6;
    std::array<uint8_t, 8> rawData{};

public:
    explicit PacketReader(const std::array<uint8_t, bufferSize>& buffer) : buffer(buffer) {}

    template <typename T>
    T read() {
        memcpy(rawData.data(), buffer.data() + now, sizeof(T));
        now += sizeof(T);
        return *reinterpret_cast<T*>(rawData.data());
    }

    uint8_t read() {
        return buffer[now++];
    }

    float readCompressedFloat(float min, float precision) {
        uint32_t raw = read<uint16_t>();
        return static_cast<float>(raw) * precision + min;
    }
};

template <int bodySize, int id>
class PacketBuffer {
    // Header: magicNumber, dataLen, seq, seq, crc8, id

public:
    int now = 6;
    std::array<uint8_t, bodySize + 8> buffer{ 0xA5, bodySize, 0, 0, Crc::getHeaderCRC8(bodySize), id };

    template <typename T>
    void serialize(T data) {
        memcpy(buffer.data() + now, &data, sizeof(T));
        now += sizeof(T);
    }

    void serialize(uint8_t data) {
        buffer[now++] = data;
    }

    void serialize(float data, float min, float precision) {
        assert(data >= min);
        assert(data <= min + 65535 * precision);
        uint16_t fixed = static_cast<uint16_t>((data - min) / precision);
        serialize(fixed);
    }

    void serializeCrc16() {
        uint16_t crc16Result = Crc::Get_CRC16_Check_Sum(buffer.data(), bodySize + 6, Crc::CRC16_INIT);
        serialize<uint16_t>(crc16Result);
    }

    [[nodiscard]] constexpr int size() const {
        return bodySize + 8;
    }

    void copyToSendBuffer(void* dest) {
        memcpy(dest, buffer.data(), size());
    }
};
