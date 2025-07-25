//
// Created by Philipp Remy on 18.07.25.
//

#ifndef LRUCAMERACACHE_HPP
#define LRUCAMERACACHE_HPP

#include <AVDepthMap/Metal/LRUCache.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @struct CameraPair
 * @brief Support class for operating an LRU cache of downscaled cameras
 * @note The CameraPair (camera id, downscale) give a unique key for LRU cache.
 */
struct CameraPair : public std::pair<int, int>
{
    CameraPair()
      : std::pair<int, int>(0, 0)
    {}
    CameraPair(int i)
      : std::pair<int, int>(i, i)
    {}
    CameraPair(int i, int j)
      : std::pair<int, int>(i, j)
    {}

    CameraPair& operator=(int i)
    {
        this->first = this->second = i;
        return *this;
    }
};

inline bool operator==(const CameraPair& l, const CameraPair& r) { return (l.first == r.first && l.second == r.second); }

inline bool operator<(const CameraPair& l, const CameraPair& r) { return (l.first < r.first || (l.first == r.first && l.second < r.second)); }

using LRUCameraCache = LRUCache<CameraPair>;
using LRUCameraIdCache = LRUCache<int>;

}  // namespace depthMap
}  // namespace aliceVision

#endif //LRUCAMERACACHE_HPP
