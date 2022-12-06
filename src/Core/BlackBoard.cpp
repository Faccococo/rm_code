#include "BlackBoard.hpp"

BlackBoard& BlackBoard::instance() {
    static BlackBoard instance;
    return instance;
}

std::pair<std::shared_mutex, std::any>* BlackBoard::getImpl(const size_t hashValue) {
    std::shared_lock guard{ mMutex };
    if(const auto iter = mItems.find(hashValue); iter != mItems.cend())
        return &iter->second;
    return nullptr;
}

void BlackBoard::insertImpl(const size_t hashValue, std::any val) {
    std::lock_guard guard{ mMutex };
    mItems[hashValue].second = std::move(val);
}
