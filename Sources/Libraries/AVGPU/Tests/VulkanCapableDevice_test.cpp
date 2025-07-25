// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AV/config.hpp>
#include <AVSystem/Logger.hpp>
#include <AVGPU/gpu.hpp>

#define BOOST_TEST_MODULE VulkanCapableDevice

#include <boost/test/unit_test.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    #include <AVGPU/Vulkan/memory.hpp>
#endif

using namespace aliceVision::gpu;

BOOST_AUTO_TEST_CASE(VulkanCapableDevice)
{
    ALICEVISION_LOG_INFO("Running test case VulkanCapableDevice");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    ALICEVISION_LOG_INFO(gpuInformationVulkan());
    BOOST_CHECK(gpuSupportVulkan(1024));
#else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Vulkan support!");
    BOOST_CHECK(false);
#endif
}

constexpr uint32_t DEVICEID = 22811;
constexpr uint32_t DEVICEID2 = 26607;

struct rgba
{
    float r; float g; float b; float a;
    constexpr rgba(float r_, float g_, float b_, float a_)
    : r(r_), g(g_), b(b_), a(a_) {}
};

constexpr rgba TESTDATA[5] = { rgba(255,255,255,255), rgba(0,0,0,0), rgba(255,255,255,255), rgba(0,0,0,0), rgba(255,255,255,255) };

BOOST_AUTO_TEST_CASE(VulkanAllocations)
{
    ALICEVISION_LOG_INFO("Running test case VulkanAllocations");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    const auto buf1 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID, 512, true, false);
    const auto buf2 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID, 512, true, true);
    const auto buf3 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID, 512, false, false);
    const auto buf4 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID, 512, false, true);
    buf1.copyFromHostToBuffer(TESTDATA, 5, 0, 0);
    buf2.copyFromHostToBuffer(TESTDATA, 5, 0, 0);
    buf3.copyFromHostToBuffer(TESTDATA, 5, 0, 0);
    buf4.copyFromHostToBuffer(TESTDATA, 5, 0, 0);
    auto data1 = buf1.getData();
    auto data2 = buf2.getData();
    auto data3 = buf3.getData();
    auto data4 = buf4.getData();
    BOOST_CHECK(data1[2].a == 255);
    BOOST_CHECK(data2[2].a == 255);
    BOOST_CHECK(data3[2].a == 255);
    BOOST_CHECK(data4[2].a == 255);
#else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Vulkan support!");
    BOOST_CHECK(false);
#endif
}

BOOST_AUTO_TEST_CASE(VulkanHostCopyOps)
{
    ALICEVISION_LOG_INFO("Running test case VulkanHostCopyOps");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    const auto img1 = VulkanMemoryBase<rgba>::create1DImage(DEVICEID, 512, vk::Format::eR32G32B32A32Sfloat, 2, true, false);
    const auto img2 = VulkanMemoryBase<rgba>::create1DImage(DEVICEID, 512, vk::Format::eR32G32B32A32Sfloat, 2, false, false);
    const auto img3 = VulkanMemoryBase<rgba>::create1DImage(DEVICEID, 512, vk::Format::eR32G32B32A32Sfloat, 2, true, true);
    const auto img4 = VulkanMemoryBase<rgba>::create1DImage(DEVICEID, 512, vk::Format::eR32G32B32A32Sfloat, 2, false, true);
    img1.copyFromHostToImage(TESTDATA, 5, 0, 0);
    img2.copyFromHostToImage(TESTDATA, 5, 0, 0);
    img3.copyFromHostToImage(TESTDATA, 5, 0, 0);
    img4.copyFromHostToImage(TESTDATA, 5, 0, 0);
    auto data5 = img1.getData();
    auto data6 = img2.getData();
    auto data7 = img3.getData();
    auto data8 = img4.getData();
    BOOST_CHECK(data5[2].a == 255);
    BOOST_CHECK(data6[2].a == 255);
    BOOST_CHECK(data7[2].a == 255);
    BOOST_CHECK(data8[2].a == 255);
#else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Vulkan support!");
    BOOST_CHECK(false);
#endif
}

BOOST_AUTO_TEST_CASE(VulkanCopyBufferToBufferSameDevice)
{
    ALICEVISION_LOG_INFO("Running test case VulkanCopyBufferToBufferSameDevice");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    const auto buf5 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID, 512, true, false);
    buf5.copyFromHostToBuffer(TESTDATA, 5, 0, 0);
    const auto buf6 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID, 512, true, false);
    buf6.copyFromAllocation(buf5, 0, 0);
    const auto data9 = buf6.getData();
    BOOST_CHECK(data9[2].a == 255);
#else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Vulkan support!");
    BOOST_CHECK(false);
#endif
}

BOOST_AUTO_TEST_CASE(VulkanCopyImageToBufferSameDevice)
{
    ALICEVISION_LOG_INFO("Running test case VulkanCopyImageToBufferSameDevice");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    const auto img5 = VulkanMemoryBase<rgba>::create1DImage(DEVICEID, 512, vk::Format::eR32G32B32A32Sfloat, 2, true, false);
    img5.copyFromHostToImage(TESTDATA, 5, 0, 0);
    const auto buf7 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID, 512, true, false);
    buf7.copyFromAllocation(img5, 0, 0);
    const auto data10 = buf7.getData();
    BOOST_CHECK(data10[2].a == 255);
#else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Vulkan support!");
    BOOST_CHECK(false);
#endif
}

BOOST_AUTO_TEST_CASE(VulkanCopyImageToImageSameDevice)
{
    ALICEVISION_LOG_INFO("Running test case VulkanCopyImageToImageSameDevice");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    const auto img6 = VulkanMemoryBase<rgba>::create1DImage(DEVICEID, 512, vk::Format::eR32G32B32A32Sfloat, 2, true, false);
    img6.copyFromHostToImage(TESTDATA, 5, 0, 0);
    const auto img7 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID, 512, true, false);
    img7.copyFromAllocation(img6, 0, 0);
    const auto data11 = img7.getData();
    BOOST_CHECK(data11[2].a == 255);
#else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Vulkan support!");
    BOOST_CHECK(false);
#endif
}

BOOST_AUTO_TEST_CASE(VulkanCopyBufferToImageSameDevice)
{
    ALICEVISION_LOG_INFO("Running test case VulkanCopyBufferToImageSameDevice");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    const auto buf8 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID, 512, true, false);
    buf8.copyFromHostToBuffer(TESTDATA, 5, 0, 0);
    const auto img8 = VulkanMemoryBase<rgba>::create1DImage(DEVICEID, 512, vk::Format::eR32G32B32A32Sfloat, 2, true, false);
    img8.copyImageFromBuffer(buf8, 0, 0);
    const auto data12 = img8.getData();
    BOOST_CHECK(data12[2].a == 255);
#else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Vulkan support!");
    BOOST_CHECK(false);
#endif
}

BOOST_AUTO_TEST_CASE(VulkanCopyBufferToBufferForeignDeviceL2R)
{
    ALICEVISION_LOG_INFO("Running test case VulkanCopyBufferToBufferForeignDeviceL2R");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    const auto buf9 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID, 512, true, false);
    buf9.copyFromHostToBuffer(TESTDATA, 5, 0, 0);
    const auto buf10 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID2, 512, true, false);
    buf10.copyFromAllocation(buf9, 0, 0);
    const auto data13 = buf10.getData();
    BOOST_CHECK(data13[2].a == 255);
#else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Vulkan support!");
    BOOST_CHECK(false);
#endif
}

BOOST_AUTO_TEST_CASE(VulkanCopyImageToBufferForeignDeviceL2R)
{
    ALICEVISION_LOG_INFO("Running test case VulkanCopyImageToBufferForeignDeviceL2R");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    const auto img9 = VulkanMemoryBase<rgba>::create1DImage(DEVICEID, 512, vk::Format::eR32G32B32A32Sfloat, 2, true, false);
    img9.copyFromHostToImage(TESTDATA, 5, 0, 0);
    const auto buf11 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID2, 512, true, false);
    buf11.copyFromAllocation(img9, 0, 0);
    const auto data14 = buf11.getData();
    BOOST_CHECK(data14[2].a == 255);
#else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Vulkan support!");
    BOOST_CHECK(false);
#endif
}

BOOST_AUTO_TEST_CASE(VulkanCopyImageToImageForeignDeviceL2R)
{
    ALICEVISION_LOG_INFO("Running test case VulkanCopyImageToImageForeignDeviceL2R");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    const auto img10 = VulkanMemoryBase<rgba>::create1DImage(DEVICEID, 512, vk::Format::eR32G32B32A32Sfloat, 2, true, false);
    img10.copyFromHostToImage(TESTDATA, 5, 0, 0);
    const auto img11 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID2, 512, true, false);
    img11.copyFromAllocation(img10, 0, 0);
    const auto data15 = img11.getData();
    BOOST_CHECK(data15[2].a == 255);
#else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Vulkan support!");
    BOOST_CHECK(false);
#endif
}

BOOST_AUTO_TEST_CASE(VulkanCopyBufferToImageForeignDeviceL2R)
{
    ALICEVISION_LOG_INFO("Running test case VulkanCopyBufferToImageForeignDeviceL2R");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    const auto buf12 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID, 512, true, false);
    buf12.copyFromHostToBuffer(TESTDATA, 5, 0, 0);
    const auto img12 = VulkanMemoryBase<rgba>::create1DImage(DEVICEID2, 512, vk::Format::eR32G32B32A32Sfloat, 2, true, false);
    img12.copyImageFromBuffer(buf12, 0, 0);
    const auto data16 = img12.getData();
    BOOST_CHECK(data16[2].a == 255);
#else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Vulkan support!");
    BOOST_CHECK(false);
#endif
}

BOOST_AUTO_TEST_CASE(VulkanCopyBufferToBufferForeignDeviceR2L)
{
    ALICEVISION_LOG_INFO("Running test case VulkanCopyBufferToBufferForeignDeviceR2L");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    const auto buf13 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID2, 512, true, true);
    buf13.copyFromHostToBuffer(TESTDATA, 5, 0, 0);
    const auto buf14 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID, 512, true, true);
    buf14.copyFromAllocation(buf13, 0, 0);
    const auto data17 = buf14.getData();
    BOOST_CHECK(data17[2].a == 255);
#else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Vulkan support!");
    BOOST_CHECK(false);
#endif
}

BOOST_AUTO_TEST_CASE(VulkanCopyImageToBufferForeignDeviceR2L)
{
    ALICEVISION_LOG_INFO("Running test case VulkanCopyImageToBufferForeignDeviceR2L");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    const auto img13 = VulkanMemoryBase<rgba>::create1DImage(DEVICEID2, 512, vk::Format::eR32G32B32A32Sfloat, 2, true, true);
    img13.copyFromHostToImage(TESTDATA, 5, 0, 0);
    const auto buf15 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID, 512, true, true);
    buf15.copyFromAllocation(img13, 0, 0);
    const auto data18 = buf15.getData();
    BOOST_CHECK(data18[2].a == 255);
#else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Vulkan support!");
    BOOST_CHECK(false);
#endif
}

BOOST_AUTO_TEST_CASE(VulkanCopyImageToImageForeignDeviceR2L)
{
    ALICEVISION_LOG_INFO("Running test case VulkanCopyImageToImageForeignDeviceR2L");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    const auto img14 = VulkanMemoryBase<rgba>::create1DImage(DEVICEID2, 512, vk::Format::eR32G32B32A32Sfloat, 2, true, true);
    img14.copyFromHostToImage(TESTDATA, 5, 0, 0);
    const auto img15 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID, 512, true, true);
    img15.copyFromAllocation(img14, 0, 0);
    const auto data19 = img15.getData();
    BOOST_CHECK(data19[2].a == 255);
#else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Vulkan support!");
    BOOST_CHECK(false);
#endif
}

BOOST_AUTO_TEST_CASE(VulkanCopyBufferToImageForeignDeviceR2L)
{
    ALICEVISION_LOG_INFO("Running test case VulkanCopyBufferToImageForeignDeviceR2L");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    const auto buf16 = VulkanMemoryBase<rgba>::createBuffer(DEVICEID2, 512, true, true);
    buf16.copyFromHostToBuffer(TESTDATA, 5, 0, 0);
    const auto img16 = VulkanMemoryBase<rgba>::create1DImage(DEVICEID, 512, vk::Format::eR32G32B32A32Sfloat, 2, true, true);
    img16.copyImageFromBuffer(buf16, 0, 0);
    const auto data20 = img16.getData();
    BOOST_CHECK(data20[2].a == 255);
#else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Vulkan support!");
    BOOST_CHECK(false);
#endif
}