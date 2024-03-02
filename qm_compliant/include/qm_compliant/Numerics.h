//
// Created by skywoodsz on 2023/7/13.
//

#ifndef SRC_NUMERICS_H
#define SRC_NUMERICS_H

#include <cmath>
#include <limits>
#include <type_traits>

namespace qm{

/**
* Almost equal which uses machine epsilon to compare floating-point values for equality.
* refer to: https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
*
* @tparam T1: data type of x.
* @tparam T2: data type of y.
* @tparam T3: data type of prec.
* @param [in] x: First floating-point number.
* @param [in] y: Second floating-point number.
* @param [in] prec: The comparison precision.
* @return bool: true if x=y.
*/
template <class T1, class T2, class T3>
bool almost_eq(T1&& x, T2&& y, T3&& prec) {
    static_assert(std::is_floating_point<typename std::remove_reference<T1>::type>::value, "First argument is not floating point!");
    static_assert(std::is_floating_point<typename std::remove_reference<T2>::type>::value, "Second argument is not floating point!");
    static_assert(std::is_floating_point<typename std::remove_reference<T3>::type>::value, "prec is not floating point!");
    // the machine epsilon has to be scaled to the magnitude of the values used
    // and multiplied by the desired precision unless the result is subnormal
    using Type = const std::remove_reference_t<T1>;
    const auto absDiff = std::abs(x - static_cast<Type>(y));
    const auto magnitude = std::min(std::abs(x), std::abs(static_cast<Type>(y)));
    return absDiff <= static_cast<Type>(prec) * magnitude || absDiff < std::numeric_limits<Type>::min();
}

/**
* Almost equal which uses machine epsilon to compare floating-point values for equality.
* refer to: https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
*
* @tparam T1: data type of x.
* @tparam T2: data type of y.
* @param [in] x: First floating-point number.
* @param [in] y: Second floating-point number.
* @return bool: true if x=y.
*/
template <class T1, class T2>
bool almost_eq(T1&& x, T2&& y) {
    const auto prec = std::numeric_limits<std::remove_reference_t<T1>>::epsilon();
    return almost_eq(x, y, prec);
}

/**
* Almost less-equal which uses machine epsilon to compare floating-point values for equality.
*
* @tparam T1: data type of x.
* @tparam T2: data type of y.
* @tparam T3: data type of prec.
* @param [in] x: First floating-point number.
* @param [in] y: Second floating-point number.
* @param [in] prec: The comparison precision.
* @return bool: true if x<=y.
*/
template <class T1, class T2, class T3>
bool almost_le(T1&& x, T2&& y, T3&& prec) {
    return x < y || almost_eq(x, y, prec);
}

/**
* Almost less-equal which uses machine epsilon to compare floating-point values for equality.
*
* @tparam T1: data type of x.
* @tparam T2: data type of y.
* @param [in] x: First floating-point number.
* @param [in] y: Second floating-point number.
* @return bool: true if x<=y.
*/
// template <class T1, class T2, class T3>
template <class T1, class T2>
bool almost_le(T1&& x, T2&& y) {
    return x < y || almost_eq(x, y);
}

/**
* Almost greater-equal which uses machine epsilon to compare floating-point values for equality.
*
* @tparam T1: data type of x.
* @tparam T2: data type of y.
* @tparam T3: data type of prec.
* @param [in] x: First floating-point number.
* @param [in] y: Second floating-point number.
* @param [in] prec: The comparison precision.
* @return bool: true if x>=y.
*/
template <class T1, class T2, class T3>
bool almost_ge(T1&& x, T2&& y, T3&& prec) {
    return x > y || almost_eq(x, y, prec);
}

/**
* Almost greater-equal which uses machine epsilon to compare floating-point values for equality.
*
* @tparam T1: data type of x.
* @tparam T2: data type of y.
* @param [in] x: First floating-point number.
* @param [in] y: Second floating-point number.
* @return bool: true if x>=y.
*/
template <class T1, class T2>
bool almost_ge(T1&& x, T2&& y) {
    return x > y || almost_eq(x, y);
}

}

#endif //SRC_NUMERICS_H
