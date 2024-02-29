#pragma once
#include <concepts>


namespace StructDef {

template<typename MSG, typename T>
concept ValidReceiveStruct = requires(T& a, MSG& c) {
    { T::sub_topic } -> std::convertible_to<std::string_view>;
    { T::ID } -> std::convertible_to<uint8_t>;
    { a.data };
    { a.transfer_to(c)};
};
}