// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVSystem/Logger.hpp>

#if __APPLE__ || __MACH__

#include <CoreFoundation/CoreFoundation.h>

// On macOS, the Vulkan driver will be dynamically resolved relative to the
// current bundle. If no such bundle is found and the user did not specify
// "VK_ICD_FILENAMES" manually, Vulkan will try the standard paths (which can
// lead to incompatibility ot fail completely.
[[gnu::constructor]]
static void onLibraryLoad()
{
    ALICEVISION_LOG_INFO("Trying to initialize the Vulkan loader paths...");
    // Try to fetch the current bundle
    CFStringRef bundleID = CFStringCreateWithCString(kCFAllocatorDefault, "org.aliceVision.AVGPU", kCFStringEncodingUTF8);
    CFBundleRef frameworkBundle = CFBundleGetBundleWithIdentifier(bundleID);
    CFRelease(bundleID);
    if(frameworkBundle == nullptr) {
        return;
    }
    CFRetain(frameworkBundle);
    CFStringRef resourceName = CFStringCreateWithCString(kCFAllocatorDefault, "MoltenVK_icd", kCFStringEncodingUTF8);
    CFStringRef resourceExt = CFStringCreateWithCString(kCFAllocatorDefault, "json", kCFStringEncodingUTF8);
    CFStringRef subdir = CFStringCreateWithCString(kCFAllocatorDefault, "Vulkan", kCFStringEncodingUTF8);
    CFURLRef bundleURL = CFBundleCopyResourceURL(frameworkBundle, resourceName, resourceExt, subdir);
    if(bundleURL == nullptr) {
        CFRelease(resourceName);
        CFRelease(resourceExt);
        CFRelease(subdir);
        return;
    }
    unsigned char bundlePath[PATH_MAX];
    if(!CFURLGetFileSystemRepresentation(bundleURL, true, bundlePath, sizeof(bundlePath))) {
        CFRelease(bundleURL);
        CFRelease(resourceName);
        CFRelease(resourceExt);
        CFRelease(subdir);
        CFRelease(frameworkBundle);
        return;
    }
    setenv("VK_ICD_FILENAMES", reinterpret_cast<const char*>(bundlePath), false);
    ALICEVISION_LOG_INFO("Set Vulkan loader path to: " << std::string(reinterpret_cast<const char*>(bundlePath)));
    // Release resources
    CFRelease(bundleURL);
    CFRelease(resourceName);
    CFRelease(resourceExt);
    CFRelease(subdir);
    CFRelease(frameworkBundle);
}

#endif