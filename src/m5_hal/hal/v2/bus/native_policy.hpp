// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_NATIVE_POLICY_HPP_
#define M5_HAL_BUS_NATIVE_POLICY_HPP_

#include <tuple>
#include <type_traits>
#include <utility>

namespace m5::hal::v2::native {

/*! @brief Borrow a caller-owned native resource for one Bus binding. */
template <class Resource>
class Borrowed {
public:
    Resource& resource(void) const
    {
        return *_resource;
    }

private:
    explicit constexpr Borrowed(Resource& resource) noexcept : _resource{&resource}
    {
    }

    Resource* _resource;

    template <class T>
    friend constexpr Borrowed<T> borrowed(T& resource) noexcept;
};

/*! @brief Mark a native resource as caller-owned. Rvalues are rejected. */
template <class Resource>
constexpr Borrowed<Resource> borrowed(Resource& resource) noexcept
{
    return Borrowed<Resource>{resource};
}

/*! @brief Arguments from which the selected provider creates an owned native resource. */
template <class... Args>
class Managed {
public:
    const std::tuple<Args...>& arguments(void) const&
    {
        return _arguments;
    }

    std::tuple<Args...>&& arguments(void) &&
    {
        return std::move(_arguments);
    }

private:
    explicit constexpr Managed(Args... args) : _arguments{std::move(args)...}
    {
    }

    std::tuple<Args...> _arguments;

    template <class... T>
    friend constexpr Managed<std::decay_t<T>...> managed(T&&... args);
};

/*! @brief Ask the selected provider to create and own a native resource. */
template <class... Args>
constexpr Managed<std::decay_t<Args>...> managed(Args&&... args)
{
    return Managed<std::decay_t<Args>...>{std::forward<Args>(args)...};
}

template <class T>
struct is_borrowed : std::false_type {};

template <class Resource>
struct is_borrowed<Borrowed<Resource>> : std::true_type {};

template <class T>
struct is_managed : std::false_type {};

template <class... Args>
struct is_managed<Managed<Args...>> : std::true_type {};

}  // namespace m5::hal::v2::native

#endif  // M5_HAL_BUS_NATIVE_POLICY_HPP_
