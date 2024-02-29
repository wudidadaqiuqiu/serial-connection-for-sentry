#pragma once
#include "easy_robot_commands/shared_member/pkg_info.hpp"

namespace TransmiteInfo {
template <typename TupleMSG>
struct SubsTupleT {
    static constexpr std::size_t MSG_NUN = std::tuple_size<TupleMSG>::value;

    template <std::size_t... Index>
    struct seq_geT {
        using seq = std::index_sequence<Index...>;
    };

    template <typename... MSGs>
    struct SubsTuple {
        using type = std::tuple<typename rclcpp::Subscription<MSGs>::SharedPtr...>;
    };

    template <std::size_t... Indices>
    static constexpr auto makeSubsTuple(std::index_sequence<Indices...>) ->
        typename SubsTuple<std::tuple_element_t<Indices, TupleMSG>...>::type {
        return SubsTuple<std::tuple_element_t<Indices, TupleMSG>...>::type();
    }

    using Type = decltype(makeSubsTuple(std::make_index_sequence<MSG_NUN>{}));
};

}