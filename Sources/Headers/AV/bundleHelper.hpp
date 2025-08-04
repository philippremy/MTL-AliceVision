// This file is part of the AliceVision project.
// Copyright (c) 2025 - Present AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVSystem/Logger.hpp>

#if defined(__APPLE__) || defined(__MACH__)
    #include <CoreFoundation/CoreFoundation.h>
#endif

#include <string>
#include <optional>

namespace aliceVision {

enum class BundleResource {
    SENSOR_DB,
    OCIO_PROFILE
};

inline std::optional<CFBundleRef> getBundleWithIdentifier(const std::string& identifier)
{
#if defined(__APPLE__) || defined(__MACH__)
    const CFStringRef bundleName = CFStringCreateWithCString(kCFAllocatorDefault, identifier.c_str(), kCFStringEncodingUTF8);
    if(const CFBundleRef resolvedBundle = CFBundleGetBundleWithIdentifier(bundleName)) {
        CFRetain(resolvedBundle);
        CFRelease(bundleName);
        return resolvedBundle;
    }
    CFRelease(bundleName);
    ALICEVISION_LOG_TRACE("Resolved macOS Bundle with identifier: " << identifier);
#endif
    return std::nullopt;
}

inline std::optional<std::string> getResourceFromBundleInternal(CFBundleRef bundle, const std::string& resourceName, const std::string& subDir = "")
{
#if defined(__APPLE__) || defined(__MACH__)
    std::string resolvedResource;
    if(subDir.empty()) {
        const CFStringRef resourceNameCFString = CFStringCreateWithCString(kCFAllocatorDefault, resourceName.c_str(), kCFStringEncodingUTF8);
        const CFURLRef resourceURL = CFBundleCopyResourceURL(bundle, resourceNameCFString, nullptr, nullptr);
        if(!resourceURL) {
            CFRelease(resourceNameCFString);
            return std::nullopt;
        }
        CFStringRef resourcePath = CFURLCopyFileSystemPath(resourceURL, kCFURLPOSIXPathStyle);
        CFRelease(resourceURL);
        resolvedResource = std::string(CFStringGetCStringPtr(resourcePath, kCFStringEncodingUTF8));
        CFRelease(resourcePath);
        CFRelease(resourceNameCFString);
    } else {
        const CFStringRef subDirName = CFStringCreateWithCString(kCFAllocatorDefault, subDir.c_str(), kCFStringEncodingUTF8);
        const CFStringRef resourceNameCFString = CFStringCreateWithCString(kCFAllocatorDefault, resourceName.c_str(), kCFStringEncodingUTF8);
        const CFURLRef resourceURL = CFBundleCopyResourceURL(bundle, resourceNameCFString, nullptr, subDirName);
        if(!resourceURL) {
            CFRelease(resourceNameCFString);
            CFRelease(subDirName);
            return std::nullopt;
        }
        CFStringRef resourcePath = CFURLCopyFileSystemPath(resourceURL, kCFURLPOSIXPathStyle);
        CFRelease(resourceURL);
        resolvedResource = std::string(CFStringGetCStringPtr(resourcePath, kCFStringEncodingUTF8));
        CFRelease(resourcePath);
        CFRelease(resourceNameCFString);
        CFRelease(subDirName);
    }
    ALICEVISION_LOG_TRACE("Resolved resource from macOS Bundle: " << resolvedResource);
    return resolvedResource;
#endif
    return std::nullopt;
}

inline std::optional<std::string> getResourceFromBundle(const BundleResource& forResource)
{
#if defined(__APPLE__) || defined(__MACH__)
    // Switch cases
    switch (forResource) {
        case BundleResource::SENSOR_DB: {
            const auto bundle = getBundleWithIdentifier("org.aliceVision.AVSensorDB");
            if(!bundle)
                break;
            const auto resource = getResourceFromBundleInternal(bundle.value(), "cameraSensors.db");
            if(resource.has_value()) {
                CFRelease(bundle.value());
                return resource;
            }
            CFRelease(bundle.value());
        }

        case BundleResource::OCIO_PROFILE: {
            const auto bundle = getBundleWithIdentifier("org.aliceVision.AVImage");
            if(!bundle)
                break;
            const auto resource = getResourceFromBundleInternal(bundle.value(), "config.ocio");
            if(resource.has_value()) {
                CFRelease(bundle.value());
                return resource;
            }
            CFRelease(bundle.value());
        }
    }
#endif
    return std::nullopt;
}

}
