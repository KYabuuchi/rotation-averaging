// https://qiita.com/ganariya/items/df35d253726269bda436
#include <Eigen/Dense>
#include <unordered_map>
#include <vector>


struct HashPair {
  template <class T1, class T2>
  size_t operator()(const std::pair<T1, T2>& p) const
  {
    auto hash1 = std::hash<T1>{}(p.first);
    auto hash2 = std::hash<T2>{}(p.second);

    size_t seed = 0;
    // https://stackoverflow.com/questions/4948780/magic-number-in-boosthash-combine
    seed ^= hash1 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hash2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

using Matrix3dVector = std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>;
using RelativeRotations = std::unordered_map<std::pair<size_t, size_t>, Eigen::Matrix3d, HashPair>;