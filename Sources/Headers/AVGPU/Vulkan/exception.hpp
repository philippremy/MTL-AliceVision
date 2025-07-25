// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <vulkan/vulkan.hpp>
#include <vulkan/vulkan_shared.hpp>

#include <exception>
#include <optional>
#include <sstream>

/**
 * A class for handling Vulkan exceptions
 *
 * Provides exception handling to be used in Vulkan contexts
 */
class VulkanException final : public std::exception {

public:
    /**
     * Turns a vk::Result and a message into a throwable exception instance
     *
     * @param[in] message The error message to provide with the exception.
     * @param[in] vkResult Optional: A vk::Result to provide with the exception, if available.
     * @note If no vkResult is provided, vk::Result::eErrorUnknown will be used.
     */
    explicit VulkanException(const std::string& message, const std::optional<vk::Result>& vkResult = {}) : m_message(message), m_vkResult(vkResult)
    {
        std::stringstream s;
        s << "A Vulkan Error occured: "
        << this->m_message
        << " Internal Vulkan Error Code: "
        << vk::to_string(this->m_vkResult.value_or(vk::Result::eErrorUnknown))
        << ".";
        this->m_what = s.str();
    }

    /**
     * Overwritten function to get the current exception string
     *
     * @return A const char pointer to the current exception description.
     */
    const char* what() const noexcept override
    {
        return this->m_what.c_str();
    }

private:
    std::optional<vk::Result> m_vkResult;   // Option holding the underlying vk::Result
    std::string m_message;                  // The error message
    std::string m_what;                     // The member holding the assembled exception description

};

/**
 * A macro used for throwing Vulkan-Hpp errors while assigning on success
 *
 * @param func The function call statement
 * @param err_msg The error message to supply to the exception on failure
 *
 * @throw VulkanException On vk::Result != eSuccess, throws the eception.
 *
 * @return The unwrapped result value on success
 */
#define ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(func, err_msg) \
    [&]() { \
        auto [result, target] = func; \
        if(result != vk::Result::eSuccess) { \
            throw VulkanException(err_msg, result); \
        } \
        return target; \
    }()

#define ALICEVISION_THROW_ON_VULKAN_HPP_RESULT_ERROR(func, err_msg) \
    [&]() { \
        auto result = func; \
        if(result != vk::Result::eSuccess) { \
            throw VulkanException(err_msg, result); \
        } \
    }()

template<typename... OutputTypes, typename Func, typename... InputArgs>
auto ALICEVISION_THROW_ON_VULKAN_ERROR(Func&& func, const char* error_msg, InputArgs&&... inputs)
{
    std::tuple<OutputTypes...> outputs;

    VkResult result = std::apply(
        [&](auto&... outs) {
            return func(std::forward<InputArgs>(inputs)..., &outs...);
        }, outputs);

    if (static_cast<vk::Result>(result) != vk::Result::eSuccess) {
        throw VulkanException(error_msg, static_cast<vk::Result>(result));
    }

    if constexpr (sizeof...(OutputTypes) == 1) {
        return std::get<0>(outputs);
    } else {
        return outputs;
    }
}
