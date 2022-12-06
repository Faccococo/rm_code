#pragma once
#include "DataDesc.hpp"
#include <any>
#include <optional>
#include <shared_mutex>
#include <unordered_map>
#include <utility>

class BlackBoard final {
    std::unordered_map<size_t, std::pair<std::shared_mutex, std::any>> mItems;
    std::shared_mutex mMutex;

    void insertImpl(size_t hashValue, std::any val);
    std::pair<std::shared_mutex, std::any>* getImpl(size_t hashValue);
public:
    template <typename T>
    std::optional<T> get(const Identifier key) {
        if(const auto ptr = getImpl(typeid(T).hash_code() ^ key.val)) {
            std::shared_lock guard{ ptr->first };
            return std::any_cast<T>(ptr->second);
        }
        return std::nullopt;
    }

    template <typename T>
    TypedIdentifier<T> updateSync(const Identifier key, T val) {
        const auto hashCode = typeid(T).hash_code() ^ key.val;
        if(const auto ptr = getImpl(hashCode)) {
            std::lock_guard guard{ ptr->first };
            ptr->second = std::move(val);
        } else
            insertImpl(hashCode, std::move(val));
        return { { key.val } };
    }

    static BlackBoard& instance();
};
