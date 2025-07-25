// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <vulkan/vulkan.hpp>
#include <vulkan/vulkan_shared.hpp>

#include <vulkan/vulkan_core.h>
#include <vulkan/vk_enum_string_helper.h>

#include <sstream>
#include <iostream>

constexpr const char* USER_INFO = "This information is targeted at developers - if you see this as an end user, make sure you built the Release configuration! Your performance will be significantly reduced!";

/**
 * An implementation for any Vulkan message calls by a validation layer
 *
 * @param messageSeverity The message severity of the debug call
 * @param messageType The type (subsystem) for the debug call
 * @param pCallbackData The callback data, if any
 * @param pUserData User data, if any
 * @return A boolean telling Vulkan if the program should halt (abort)
 * execution
 */
static VKAPI_ATTR VkBool32 VKAPI_CALL validationMessageCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
    VkDebugUtilsMessageTypeFlagsEXT messageType,
    const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData,
    void *pUserData)
{
    std::stringstream s;
    switch (messageSeverity) {
        case VkDebugUtilsMessageSeverityFlagBitsEXT::VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT: {
            s << "[VULKAN_TRACE] ";
            break;
        }
        case VkDebugUtilsMessageSeverityFlagBitsEXT::VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT: {
            s << "[VULKAN_INFO] ";
            break;
        }
        case VkDebugUtilsMessageSeverityFlagBitsEXT::VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT: {
            s << "[VULKAN_WARN] ";
            break;
        }
        case VkDebugUtilsMessageSeverityFlagBitsEXT::VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT: {
            s << "[VULKAN_ERROR] ";
            break;
        }
        default: {
            s << "[VULKAN_UNKNOWN] ";
            break;
        };
    }
    switch (messageType) {
        case VkDebugUtilsMessageTypeFlagBitsEXT::VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT: {
            s << "General message";
            break;
        }
        case VkDebugUtilsMessageTypeFlagBitsEXT::VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT: {
            s << "Validation message";
            break;
        }
        case VkDebugUtilsMessageTypeFlagBitsEXT::VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT: {
            s << "Performance message";
            break;
        }
        case VkDebugUtilsMessageTypeFlagBitsEXT::VK_DEBUG_UTILS_MESSAGE_TYPE_DEVICE_ADDRESS_BINDING_BIT_EXT: {
            s << "Device Address Binding message";
            break;
        }
        default: {
            s << "Unknown message";
            break;
        }
    }
    if (pCallbackData != nullptr) {
        s << "\n    [MSG-ID]: ";
        s << pCallbackData->messageIdNumber;
        s << "\n    [MSG-ID-NAME]: ";
        if (pCallbackData->pMessageIdName != nullptr) {
            s << pCallbackData->pMessageIdName;
        } else {
            s << "N/A";
        }
        s << "\n    [MSG]: ";
        if (pCallbackData->pMessage != nullptr) {
            s << pCallbackData->pMessage;
        } else {
            s << "N/A";
        }
        s << "\n    [CMD-BUFFERS]: ";
        for (int i = 0; i < pCallbackData->cmdBufLabelCount; i++) {
            s << "\n        [CMD-BUFFER " << i << "]: ";
            if (pCallbackData->pCmdBufLabels + sizeof(VkDebugUtilsMessengerCallbackDataEXT) * i != nullptr) {
                s << pCallbackData->pCmdBufLabels[i].pLabelName;
            } else {
                s << "N/A";
            }
        }
        if (pCallbackData->cmdBufLabelCount == 0) {
            s << "No command buffers in use.";
        }
        s << "\n    [QUEUES]: ";
        for (int i = 0; i < pCallbackData->queueLabelCount; i++) {
            s << "\n        [QUEUE " << i << "]: ";
            if (pCallbackData->pQueueLabels + sizeof(VkDebugUtilsLabelEXT) * i != nullptr) {
                s << pCallbackData->pQueueLabels[i].pLabelName;
            } else {
                s << "N/A";
            }
        }
        if (pCallbackData->queueLabelCount == 0) {
            s << "No queues in use.";
        }
        s << "\n    [VULKAN-OBJECTS]: ";
        for (int i = 0; i < pCallbackData->objectCount; i++) {
            s << "\n        [OBJECT " << i << "]: ";
            if (pCallbackData->pObjects + sizeof(VkDebugUtilsLabelEXT) * i != nullptr) {
                s << "\n            [OBJECT-TYPE]: ";
                s << string_VkObjectType(pCallbackData->pObjects[i].objectType);
                s << "\n            [OBJECT-ADDRESS]: ";
                s << &pCallbackData->pObjects[i].objectHandle;
                s << "\n            [OBJECT-NAME]: ";
                if (pCallbackData->pObjects[i].pObjectName != nullptr) {
                    s << pCallbackData->pObjects[i].pObjectName;
                } else {
                    s << "N/A";
                }
            } else {
                s << "N/A";
            }
        }
        if (pCallbackData->objectCount == 0) {
            s << "No objects in use.";
        }
    }
    s << "\n    [USER-DATA]: ";
    if (pUserData != nullptr) {
        s << "Address is: " << pUserData << " (Stringified: " << static_cast<const char *>(pUserData) << ")";
    } else {
        s << "N/A";
    }
    std::cerr << s.str() << std::endl;
    if (messageSeverity == VkDebugUtilsMessageSeverityFlagBitsEXT::VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
        return VK_TRUE;
    }
    return VK_FALSE;
}
