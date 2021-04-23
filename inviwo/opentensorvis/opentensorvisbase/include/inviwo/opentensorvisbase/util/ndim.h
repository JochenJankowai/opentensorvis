#pragma once

#include <inviwo/core/common/inviwo.h>

namespace inviwo {
namespace util {
namespace ndim {
template <typename T, typename Operator>
std::vector<T> arithmeticOperation(const std::vector<T> &a, const T val,
                                   Operator o) {
  std::vector<T> result;
  result.reserve(a.size());

  std::transform(std::begin(a), std::end(a), std::back_inserter(result),
                 [&](const auto v) { return o(v, val); });

  return result;
}

template <typename T, typename Operator>
std::vector<T> arithmeticOperation(const std::vector<T> &a,
                                   const std::vector<T> &b, Operator o) {
  std::vector<T> result;
  result.reserve(a.size());

  std::transform(std::begin(a), std::end(a), std::begin(b),
                 std::back_inserter(result),
                 [&](const auto v1, const auto v2) { return o(v1, v2); });

  return result;
}

template <typename T>
std::vector<T> plus(const std::vector<T> &a, const T val) {
  return arithmeticOperation(a, val, std::plus<T>());
}

template <typename T>
std::vector<T> plus(const std::vector<T> &a, const std::vector<T> &b) {
  return arithmeticOperation(a, b, std::plus<T>());
}

template <typename T>
std::vector<T> operator+(const std::vector<T> &a, const T val) {
  return arithmeticOperation(a, val, std::plus<T>());
}

template <typename T>
std::vector<T> operator+(const std::vector<T> &a, const std::vector<T> &b) {
  return arithmeticOperation(a, b, std::plus<T>());
}

template <typename T>
std::vector<T> minus(const std::vector<T> &a, const T val) {
  return arithmeticOperation(a, val, std::minus<T>());
}

template <typename T>
std::vector<T> minus(const std::vector<T> &a, const std::vector<T> &b) {
  return arithmeticOperation(a, b, std::minus<T>());
}

template <typename T>
std::vector<T> operator-(const std::vector<T> &a, const T val) {
  return arithmeticOperation(a, val, std::minus<T>());
}

template <typename T>
std::vector<T> operator-(const std::vector<T> &a, const std::vector<T> &b) {
  return arithmeticOperation(a, b, std::minus<T>());
}

template <typename T>
std::vector<T> divide(const std::vector<T> &a, const T val) {
  return arithmeticOperation(a, val, std::divides<T>());
}

template <typename T>
std::vector<T> divide(const std::vector<T> &a, const std::vector<T> &b) {
  return arithmeticOperation(a, b, std::divides<T>());
}

template <typename T>
std::vector<T> operator/(const std::vector<T> &a, const T val) {
  return arithmeticOperation(a, val, std::divides<T>());
}

template <typename T>
std::vector<T> operator/(const std::vector<T> &a, const std::vector<T> &b) {
  return arithmeticOperation(a, b, std::divides<T>());
}

template <typename T>
std::vector<T> multiply(const std::vector<T> &a, const T val) {
  return arithmeticOperation(a, val, std::multiplies<T>());
}

template <typename T>
std::vector<T> multiply(const std::vector<T> &a, const std::vector<T> &b) {
  return arithmeticOperation(a, b, std::multiplies<T>());
}

template <typename T>
std::vector<T> operator*(const std::vector<T> &a, const T val) {
  return arithmeticOperation(a, val, std::multiplies<T>());
}

template <typename T>
std::vector<T> operator*(const std::vector<T> &a, const std::vector<T> &b) {
  return arithmeticOperation(a, b, std::multiplies<T>());
}

template <typename T>
inline auto dot(const std::vector<T> &a, const std::vector<T> &b) {
  return std::inner_product(std::begin(a), std::end(a), std::begin(b), T{0});
}
}  // namespace ndim
}  // namespace util
}  // namespace inviwo