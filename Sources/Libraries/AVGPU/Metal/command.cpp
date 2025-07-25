//
// Created by Philipp Remy on 17.07.25.
//

#include <AV/bundleHelper.hpp>
#include <AV/omp.hpp>
#include <AVGPU/Metal/command.hpp>
#include <AVGPU/Metal/device.hpp>
#include <CoreFoundation/CoreFoundation.h>

#include <sstream>

namespace aliceVision {
namespace gpu {

MTLCommandManager::MTLCommandManager(NS::SharedPtr<MTL::Device> device)
  : m_device(device)
{}

MTLCommandManager* MTLCommandManager::loadLibrary(const std::string& withName, const std::string& fromBundle)
{
    if (this->m_metalLibraries.contains(withName))
        return this;
    if (fromBundle.empty())
    {
        // Get default bundle path
        CFBundleRef defaultBundle = CFBundleGetMainBundle();
        if (!defaultBundle)
        {
            CFRelease(defaultBundle);
            ALICEVISION_THROW_ERROR("Unable to get main bundle! Try passing an explicit bundle name!");
        }
        const CFStringRef libraryNameCFString = CFStringCreateWithCString(kCFAllocatorDefault, withName.c_str(), kCFStringEncodingUTF8);
        const CFURLRef libraryURL = CFBundleCopyResourceURL(defaultBundle, libraryNameCFString, CFSTR("metallib"), nullptr);
        if (!libraryURL)
        {
            CFRelease(libraryNameCFString);
            CFRelease(defaultBundle);
            ALICEVISION_THROW_ERROR("Unable to get Metal library " << withName << "from main bundle! Try passing an explicit bundle name!");
        }
        NS::Error* libraryLoadingError;
        NS::SharedPtr<MTL::Library> library = NS::TransferPtr(this->m_device->newLibrary(reinterpret_cast<const NS::URL*>(libraryURL), &libraryLoadingError));
        if (library.get() == nil)
        {
            CFRelease(libraryURL);
            CFRelease(libraryNameCFString);
            CFRelease(defaultBundle);
            NS::SharedPtr<NS::String> errDesc = NS::TransferPtr(libraryLoadingError->localizedDescription());
            ALICEVISION_THROW_ERROR("Unable to get default Metal library! Try passing an explicit bundle name! Error: " << std::string(errDesc->utf8String()));
        }
        this->m_metalLibraries.emplace(withName, library);
        CFRelease(libraryURL);
        CFRelease(libraryNameCFString);
        CFRelease(defaultBundle);
        return this;
    }
    const std::optional<CFBundleRef> bundleOpt = getBundleWithIdentifier(fromBundle);
    if (!bundleOpt.has_value())
        ALICEVISION_THROW_ERROR("Cannot get default Metal library! Requested bundle " << fromBundle << " could not be resolved!");
    const CFStringRef libraryNameCFString = CFStringCreateWithCString(kCFAllocatorDefault, withName.c_str(), kCFStringEncodingUTF8);
    const CFURLRef libraryURL = CFBundleCopyResourceURL(bundleOpt.value(), libraryNameCFString, CFSTR("metallib"), nullptr);
    if (!libraryURL)
    {
        CFRelease(libraryNameCFString);
        CFRelease(bundleOpt.value());
        ALICEVISION_THROW_ERROR("Unable to get Metal library " << withName << "from bundle " << fromBundle << "! The resource was not found!");
    }
    NS::Error* libraryLoadingError;
    NS::SharedPtr<MTL::Library> library = NS::TransferPtr(this->m_device->newLibrary(reinterpret_cast<const NS::URL*>(libraryURL), &libraryLoadingError));
    if (library.get() == nil)
    {
        CFRelease(libraryURL);
        CFRelease(libraryNameCFString);
        CFRelease(bundleOpt.value());
        NS::SharedPtr<NS::String> errorString = NS::RetainPtr(libraryLoadingError->localizedDescription());
        ALICEVISION_THROW_ERROR("Unable to get Metal library" << withName << " for bundle " << fromBundle
                                                                                  << ". Error: " << std::string(errorString->utf8String()));
    }
    CFStringRef libraryPath = CFURLCopyPath(libraryURL);
    ALICEVISION_LOG_TRACE("Resolved requested Metal Library: " << std::string(CFStringGetCStringPtr(libraryPath, kCFStringEncodingUTF8)));
    CFRelease(libraryPath);
    CFRelease(libraryURL);
    CFRelease(libraryNameCFString);
    CFRelease(bundleOpt.value());
    this->m_metalLibraries.emplace(withName, library);
    return this;
}

MTLCommandManager* MTLCommandManager::pipeline(const std::string& withName, const std::string& fromLibrary)
{
    if (this->m_pipelineStates.contains(withName))
        this->m_currentPipeline = this->m_pipelineStates.at(withName);
    else
    {
        if (!this->m_metalLibraries.contains(fromLibrary))
            ALICEVISION_THROW_ERROR("Unable to lookup Metal kernel function from library "
                                    << fromLibrary << ". The library was not loaded; please call loadLibrary() first!");
        const NS::SharedPtr<NS::String> functionName = NS::TransferPtr(NS::String::alloc()->init(withName.c_str(), NS::UTF8StringEncoding));
        NS::SharedPtr<MTL::Function> function = NS::TransferPtr(this->m_metalLibraries.at(fromLibrary)->newFunction(functionName.get()));
        if (function.get() == nil)
            ALICEVISION_THROW_ERROR("Requested Metal kernel function with name " << withName << " was not found in library " << fromLibrary);
        NS::Error* maybeError = nullptr;
        NS::SharedPtr<MTL::ComputePipelineState> pipelineState =
          NS::TransferPtr(this->m_device->newComputePipelineState(function.get(), &maybeError));
        if (maybeError != nil)
        {
            NS::SharedPtr<NS::String> errorString = NS::RetainPtr(maybeError->localizedDescription());
            ALICEVISION_THROW_ERROR("Failed to create ComputePipelineState from function " << withName
                                                                                           << ". Error: " << std::string(errorString->utf8String()));
        }
        this->m_pipelineStates.emplace(withName, pipelineState);
        this->m_currentPipeline = pipelineState;
    }
    return this;
}

MTLCommandManager* MTLCommandManager::commandBuffer(const std::source_location& ctx)
{
    // Create a UUID
    const CFUUIDRef uuid = CFUUIDCreate(kCFAllocatorDefault);
    const CFStringRef uuidStr = CFUUIDCreateString(kCFAllocatorDefault, uuid);
    std::stringstream ctxS;
    ctxS << "[" << std::string(CFStringGetCStringPtr(uuidStr, kCFStringEncodingUTF8)) << "] ";
    ctxS << ctx.file_name() << ", " << ctx.function_name() << ", " << ctx.line();
    this->m_commandBuffer = NS::TransferPtr(MTLDeviceManager::getInstance()->getQueue(this->m_device->registryID())->commandBuffer());
    this->m_commandBuffer->setLabel(NS::String::alloc()->init(ctxS.str().c_str(), NS::UTF8StringEncoding));
    CFRelease(uuidStr);
    CFRelease(uuid);
    return this;
}

MTLCommandManager* MTLCommandManager::commandEncoder()
{
    this->m_commandEncoder = NS::TransferPtr(this->m_commandBuffer->computeCommandEncoder(MTL::DispatchTypeConcurrent));
    this->m_commandEncoder->setComputePipelineState(this->m_currentPipeline.get());
    return this;
}

MTLCommandManager* MTLCommandManager::dispatchDimensions(const MTL::Size& totalThreads, const MTL::Size& threadsPerThreadgroup)
{
    // NOTE: This function should be available on any Mac GPU (see https://developer.apple.com/metal/Metal-Feature-Set-Tables.pdf)
    //       due to this being a requirement for the Mac2 GPU family, which all Metal devices support. On iOS, the support starts
    //       with the A11 Bionic, so everything >= iPhone 8 is suppoted.
    this->m_commandEncoder->dispatchThreads(totalThreads, threadsPerThreadgroup);
    return this;
}

MTLCommandManager* MTLCommandManager::endRecording()
{
    this->m_commandEncoder->endEncoding();
    return this;
}

MTLCommandManager* MTLCommandManager::commitCommands()
{
    this->m_commandBuffersInFlight.emplace_back(this->m_commandBuffer);
    this->m_commandBuffersInFlight.back()->addCompletedHandler([this](const MTL::CommandBuffer* commandBuffer) {
        if (commandBuffer->status() != MTL::CommandBufferStatusCompleted)
            this->m_cmdBufferErrors.emplace_back(std::string(commandBuffer->label()->utf8String()), NS::RetainPtr(commandBuffer->error()));
        this->m_cmdBufferLogs.emplace_back(commandBuffer->label()->utf8String(), NS::RetainPtr(commandBuffer->logs()));
    });
    this->m_commandBuffersInFlight.back()->commit();
    return this;
}

MTLCommandManager* MTLCommandManager::wait(uint64_t count)
{
    uint64_t processed = 0;

    while (!this->m_commandBuffersInFlight.empty() && processed < count)
    {
        auto& cmdBuffer = this->m_commandBuffersInFlight.front();

        if (cmdBuffer->status() != MTL::CommandBufferStatusCompleted)
        {
            cmdBuffer->waitUntilCompleted();
        }

        this->m_commandBuffersInFlight.pop_front();
        ++processed;
    }

    return this;
}

MTLCommandManager* MTLCommandManager::waitAll()
{
    for (const auto& cmdBuffer : this->m_commandBuffersInFlight)
    {
        if (cmdBuffer->status() != MTL::CommandBufferStatusCompleted)
        {
            cmdBuffer->waitUntilCompleted();
        }
    }
    this->m_commandBuffersInFlight.clear();
    return this;
}

MTLCommandManager* MTLCommandManager::reset()
{
    this->m_currentPipeline.reset();
    this->m_commandBuffer.reset();
    this->m_commandEncoder.reset();
    return this;
}

MTLCommandManager* MTLCommandManager::createSampler(const uint64_t idx)
{
    NS::SharedPtr<MTL::SamplerDescriptor> samplerDescriptor = NS::TransferPtr(MTL::SamplerDescriptor::alloc()->init());
    samplerDescriptor->setBorderColor(MTL::SamplerBorderColorTransparentBlack);
    samplerDescriptor->setMagFilter(MTL::SamplerMinMagFilterLinear);
    samplerDescriptor->setMinFilter(MTL::SamplerMinMagFilterLinear);
    samplerDescriptor->setNormalizedCoordinates(true);
    samplerDescriptor->setRAddressMode(MTL::SamplerAddressModeClampToEdge);
    samplerDescriptor->setTAddressMode(MTL::SamplerAddressModeClampToEdge);
    samplerDescriptor->setSAddressMode(MTL::SamplerAddressModeClampToEdge);
    samplerDescriptor->setMipFilter(MTL::SamplerMipFilterNearest);
    NS::SharedPtr<MTL::SamplerState> samplerState = NS::TransferPtr(this->m_device->newSamplerState(samplerDescriptor.get()));
    this->m_commandEncoder->setSamplerState(samplerState.get(), idx);
    return this;
}

MTLCommandManager* MTLCommandManager::createSampler(const bool normalizedCoords, const bool subpixelInterpolation, uint64_t idx)
{
    NS::SharedPtr<MTL::SamplerDescriptor> samplerDescriptor = NS::TransferPtr(MTL::SamplerDescriptor::alloc()->init());
    samplerDescriptor->setBorderColor(MTL::SamplerBorderColorTransparentBlack);
    samplerDescriptor->setMagFilter(subpixelInterpolation ? MTL::SamplerMinMagFilterLinear : MTL::SamplerMinMagFilterNearest);
    samplerDescriptor->setMinFilter(subpixelInterpolation ? MTL::SamplerMinMagFilterLinear : MTL::SamplerMinMagFilterNearest);
    samplerDescriptor->setNormalizedCoordinates(normalizedCoords);
    samplerDescriptor->setRAddressMode(MTL::SamplerAddressModeClampToEdge);
    samplerDescriptor->setTAddressMode(MTL::SamplerAddressModeClampToEdge);
    samplerDescriptor->setSAddressMode(MTL::SamplerAddressModeClampToEdge);
    samplerDescriptor->setMipFilter(normalizedCoords ? MTL::SamplerMipFilterNearest : MTL::SamplerMipFilterNotMipmapped);
    NS::SharedPtr<MTL::SamplerState> samplerState = NS::TransferPtr(this->m_device->newSamplerState(samplerDescriptor.get()));
    this->m_commandEncoder->setSamplerState(samplerState.get(), idx);
    return this;
}

std::unordered_map<std::string, std::string> MTLCommandManager::getErrors() const
{
    std::unordered_map<std::string, std::string> errors;
    #if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENMP)
    #pragma omp parallel for
    #endif
    for (uint64_t i=0; i < this->m_cmdBufferErrors.size(); i++)
    {
        NS::SharedPtr<NS::String> errNSString = NS::RetainPtr(this->m_cmdBufferErrors[i].second->localizedDescription());
        const std::string errString = std::string(errNSString->utf8String());
        #if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENMP)
        #pragma omp critical
        #endif
        {
            errors.emplace(this->m_cmdBufferErrors[i].first, errString);
        }
    }
    return errors;
}

std::unordered_map<std::string, std::string> MTLCommandManager::getLogs() const
{
    std::unordered_map<std::string, std::string> logs;
    #if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENMP)
    #pragma omp parallel for
    #endif
    for (uint64_t i=0; i < this->m_cmdBufferLogs.size(); i++)
    {
        NS::FastEnumerationState* enumerationState = nullptr;
        NS::Object* logArray[256];
        const uint64_t count = this->m_cmdBufferLogs[i].second->countByEnumerating(enumerationState, logArray, 256);
        std::stringstream logS;
        logS << "[" << this->m_cmdBufferLogs[i].first << "]:";
        for (uint64_t j = 0; j < std::min(count, 256ull); j++)
        {
            const NS::SharedPtr<NS::String> description = NS::RetainPtr(static_cast<MTL::FunctionLog*>(logArray[j])->description());
            const NS::SharedPtr<MTL::FunctionLogDebugLocation> debugLoc = NS::RetainPtr(static_cast<MTL::FunctionLog*>(logArray[j])->debugLocation());
            const NS::SharedPtr<NS::String> funcName =  NS::RetainPtr(debugLoc->functionName());
            const uint64_t line = debugLoc->line();
            const uint64_t col = debugLoc->column();
            logS << "\t[" << std::string(funcName->utf8String()) << ", Line " << line << ", Column " << col << "]:" << std::string(description->utf8String()) << "\n";
        }
        #if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENMP)
        #pragma omp critical
        #endif
        {
            logs.emplace(this->m_cmdBufferLogs[i].first, logS.str());
        }
    }
    return logs;
}

}  // namespace gpu
}  // namespace aliceVision