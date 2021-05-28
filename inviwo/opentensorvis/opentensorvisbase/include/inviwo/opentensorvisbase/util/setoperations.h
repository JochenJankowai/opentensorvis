#pragma once

#include <vector>

namespace inviwo{

    namespace util{

        namespace set{
// Input: A collection of vectors
// Output: Intersection set of the entries of the input vectors
template <typename T>
std::vector<T> intersection(const std::vector<std::vector<T>>& vecs) {
    auto last_intersection = vecs[0];
    std::vector<T> curr_intersection;

    for (size_t i{1}; i < vecs.size(); ++i) {
        std::set_intersection(last_intersection.begin(), last_intersection.end(), vecs[i].begin(),
                              vecs[i].end(), std::back_inserter(curr_intersection));
        std::swap(last_intersection, curr_intersection);
        curr_intersection.clear();
    }
    return last_intersection;
}

// Input: A collection of vectors
// Output: Union set of the entries of the input vectors
template <typename T>
std::vector<T> union_set(const std::vector<std::vector<T>>& vecs) {
    auto last_union = vecs[0];
    std::vector<T> curr_union;

    for (size_t i{1}; i < vecs.size(); ++i) {
        std::set_union(last_union.begin(), last_union.end(), vecs[i].begin(), vecs[i].end(),
                       std::back_inserter(curr_union));
        std::swap(last_union, curr_union);
        curr_union.clear();
    }
    return last_union;
}

// Input: Two vectors
// Output: Intersection set of the entries of the input vectors
template <typename T>
std::vector<T> union_set(const std::vector<T>& vec1, const std::vector<T>& vec2) {
    std::vector<T> ret;
    std::set_union(vec1.begin(), vec1.end(), vec2.begin(), vec2.end(), std::back_inserter(ret));
    return ret;
}

template <typename Container, typename... Containers>
auto concatenate(Container first, Containers... vectorsToAppend) {
    (first.insert(std::end(first), std::begin(vectorsToAppend), std::end(vectorsToAppend)), ...);

    return std::move(first);
}

template <typename T>
std::vector<T> unique(const std::vector<T>& vector) {
    std::vector<T> ret(vector);
    auto last = std::unique(ret.begin(), ret.end());
    ret.erase(last, ret.end());
    return ret;
}

        }
    }
}



