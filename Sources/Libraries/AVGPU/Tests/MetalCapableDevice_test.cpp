//
// Created by Philipp Remy on 16.07.25.
//

#include <AV/config.hpp>
#include <AVSystem/Logger.hpp>
#include <AVGPU/gpu.hpp>

#define BOOST_TEST_MODULE MetalCapableDevice

#include "AVGPU/Metal/command.hpp"
#include "AVGPU/Metal/device.hpp"

#include <boost/test/unit_test.hpp>

#include <ranges>

using namespace aliceVision::gpu;

BOOST_AUTO_TEST_CASE(MetalCapableDevice)
{
    ALICEVISION_LOG_INFO("Running test case MetalCapableDevice");
    #if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)
    ALICEVISION_LOG_INFO(gpuInformationMTL());
    BOOST_CHECK(gpuSupportMTL(1024));
    #else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Metal support!");
    BOOST_CHECK(false);
    #endif
}

BOOST_AUTO_TEST_CASE(MetalAllocations)
{
    ALICEVISION_LOG_INFO("Running test case MetalAllocation");
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)
    const auto mtlManager = MTLDeviceManager::getInstance();
    const auto devices = mtlManager->getDevices();
    const MTLResourceManager* resman1 = nullptr;
    const MTLResourceManager* resman2 = nullptr;
    MTLCommandManager* cmdMng = nullptr;
    float testArray[1024];
    for (int k = 0; k < 1024; k++)
    {
        testArray[k] = k;
    }
    int i = 0;
    for (const auto [id, dev] : devices)
    {
        if (i == 0)
            resman1 = mtlManager->getResourceManager(id);
        else
            resman2 = mtlManager->getResourceManager(id);
        cmdMng = mtlManager->getCommandManager(id);
        i++;
    }
    cmdMng->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVGPU");
    for(int j=0; j < 1024; j++)
    {
        {
            auto alloc2 = resman1->createBuffer<float>(1024, true);
            auto alloc1 = resman1->createTexture2D<float>(MTL::PixelFormatR32Float, 512, 2, 1, true);
            // alloc1.copyFromHost(testArray, 1024, 0, 0);
            // const auto data1 = alloc1.getData();
            // alloc2.copyFromResource(alloc1, 0, 0);
            // const auto data2 = alloc2.getData();
            // float t = (*data2)[1023];
            // BOOST_CHECK((*data2)[1023] == 1023.f);
        }
        {
            auto alloc2 = resman2->createBuffer<float>(1024, true);
            auto alloc1 = resman2->createTexture2D<float>(MTL::PixelFormatR32Float, 512, 2, 1, true);
            // alloc1.copyFromHost(testArray, 1024, 0, 0);
            // const auto data1 = alloc1.getData();
            // alloc2.copyFromResource(alloc1, 0, 0);
            // const auto data2 = alloc2.getData();
            // float t = (*data2)[1023];
            // BOOST_CHECK((*data2)[1023] == 1023.f);
        }
        std::cout << "ITER: " << j << std::endl;
    }
    sleep(15);
    #else
    ALICEVISION_LOG_ERROR("AVGPU was compiled without Metal support!");
    BOOST_CHECK(false);
    #endif
}
