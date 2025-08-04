//
// Created by Philipp Remy on 17.07.25.
//

#ifndef COMMAND_HPP
#define COMMAND_HPP

#include <Metal/Metal.hpp>

#include <deque>
#include <string>
#include <source_location>

namespace aliceVision {
namespace depthMap {

/**
 * @brief A manager class for performing work on a Metal device
 *
 * This class eases and streamlines the process of doing work on Apple Metal devices.
 * It handles loading libraries, functions, creates command buffer and pipeline states,
 * records commands, commits work, waits for command buffers to finish and handles simple
 * error and logging infrastructure.
 *
 * The general process looks like this:
 * (1) Load a library with loadLibrary()
 * (2) Create a command buffer with commandBuffer()
 * (3) Create a compute pipeline with pipeline()
 * (4) Create a compute command encoder with commandEncoder()
 * (5) Bind resources with bind()
 * (6) Set the threads and threadgroups with dispatchDimensions()
 * (7) End the command encoder with endRecording()
 * (8) Commit the command buffer with commitCommands()
 * (9) Wait for a specific count of command buffer with wait(), or wait for all commands with waitAll()
 * (10) [OPTIONAL] Inspect errors and logs with getLogs() and getErrors()
 */
class MTLCommandManager
{
    // Make MTLDeviceManager a friend, so it can privately construct instances
    friend class DeviceManager;

  public:
    /**
     * @brief Loads a library into the internal cache, if it is not present
     *
     * This method loads a library for the given name and from the specified bundle.
     * If fromBundle is empty, it attempts to resolve the default library for the default
     * bundle.
     * A call to this function must precede any calls which might reference the library or
     * any of its functions.
     *
     * @param withName The name of the library it should be stored as (must be unique!)
     * @param fromBundle The bundle from which to load the library. Pass "" to load from the default bundle
     * @return A pointer to this (MTLCommandManager) for method chaining
     */
    MTLCommandManager* loadLibrary(const std::string& withName, const std::string& fromBundle = std::string(""));

    /**
     * @brief Creates or retrieves a pipeline with the given function name
     *
     * This method sets the current pipeline state. For this, it either
     * loads a pipeline state from its internal cache (if it was used before)
     * or creates a new one by trying to load a Metal function from the library
     * with the specified name.
     *
     * @note The library from which the function should be loaded must have been
     * added with loadLibrary(), otherwise the lookup will fail!
     *
     * @param withName The name of the function/pipeline to load
     * @param fromLibrary The name of the library to perform lookup in
     * @return A pointer to this (MTLCommandManager) for method chaining
     */
    MTLCommandManager* pipeline(const std::string& withName, const std::string& fromLibrary);

    /**
     * @brief Creates a new commandBuffer, identified by the context it was created in
     *
     * This method creates a new command buffer (by retrieving it from the associated
     * device queue) and sets it as the current command buffer.
     *
     * @warning Note that changing the ctx might cause unintentional overwrites of previous
     * error and logging state. Only set this parameter if you are absolutely sure about this.
     *
     * @param ctx The source context of the calling environment
     * @return A pointer to this (MTLCommandManager) for method chaining
     */
    MTLCommandManager* commandBuffer(const std::source_location& ctx = std::source_location::current());
    /**
     * @brief Creates a new compute command encoder and sets it as the current command encoder
     *
     * This method creates and sets a new compute command encoder. It also sets the current
     * pipeline state as its compute pipeline state.
     *
     * @note Make sure that before calling this method you have set the pipeline state accordingly
     * by calling pipeline().
     *
     * @warning It is required that any other command encoders have properly been finalized by
     * calling endRecording(), before a new one is created. The caller must ensure proper ordering
     * of method calls.
     *
     * @return A pointer to this (MTLCommandManager) for method chaining
     */
    MTLCommandManager* commandEncoder();
    /**
     * @brief Ends recording into the current command encoder
     *
     * This method ends recording into the current command encoder,
     * therefore "finalizing" it.
     *
     * @note Be sure that all resources, threads and threadgroups have been
     * set before calling this method.
     *
     * @return A pointer to this (MTLCommandManager) for method chaining
     */
    MTLCommandManager* endRecording();
    /**
     * @brief Commits the current command buffer to the Metal device
     *
     * This method commits (i.e., submits) the current command buffer
     * in its current configuration to be executed by the GPU. Internally,
     * it enqueues a spot in the execution queue and instructs the GPU to
     * execute the command.
     *
     * @return A pointer to this (MTLCommandManager) for method chaining
     */
    MTLCommandManager* commitCommands();
    /**
     * @brief Waits for a specified count of command buffers to finish execution
     *
     * This method waits for a maximum of _count_ command buffers to finish executing
     * on the GPU. If there are less command buffers still in flight, less are waited
     * for. If there is nothing to wait for, the method returns immediately.
     *
     * @note Waiting follows a FIFO pattern. The oldest command buffers are awaited first.
     *
     * @param count The count of command buffers to wait for
     * @return A pointer to this (MTLCommandManager) for method chaining
     */
    MTLCommandManager* wait(uint64_t count);
    /**
     * @brief Waits for all command buffers to finish execution
     *
     * This method waits for all command buffers to finish executing
     * on the GPU. If there is nothing to wait for, the method returns immediately.
     *
     * @note Waiting follows a FIFO pattern. The oldest command buffers are awaited first.
     *
     * @return A pointer to this (MTLCommandManager) for method chaining
     */
    MTLCommandManager* waitAll();
    /**
     * @brief Resets the internal state and allows for a fresh command chain
     *
     * This method clears the current internal states. I.e., it frees the
     * current command buffer, the current pipeline state, and the command
     * encoder.
     *
     * @note This method does not clear committed command buffers.
     *
     * @warning This method erases all states, so setting up a new command
     * must follow the pattern stated in the class description.
     *
     * @return A pointer to this (MTLCommandManager) for method chaining
     */
    MTLCommandManager* reset();
    /**
     * @brief Applies thread and threadgroup dimensions for the current command encoder
     *
     * This method sets the threads per grid and the threads per threadgroup for the current
     * command encoder.
     *
     * @warning If the underlying kernel function assumes that a minimum size is submitted,
     * make sure that the sizes cover the whole area. Otherwise, partial results are to be
     * expected. If the underlying kernel function does not perform bounds checking and the
     * speicified sizes overflow the working area, the behaviour can be undefined or cause
     * GPU faults.
     *
     * @param totalThreads The total count of threads required to execute (usually image width/height)
     * @param threadsPerThreadgroup The count of threads per threadgroup
     * @return A pointer to this (MTLCommandManager) for method chaining
     */
    MTLCommandManager* dispatchDimensions(const MTL::Size& totalThreads, const MTL::Size& threadsPerThreadgroup);

    /**
     * @brief Binds the given resource to the specified index in the current command encoder
     *
     * This method binds the given resource to the specified index in the command encoder.
     * Resource and buffer indices are differentiated internally, so it is the callers'
     * responsibility to keep track of and provide the correct binding index for the resource
     * type.
     *
     * @warning If Push Constants are bound, buffer index 30 is reserved for Push Constants.
     *
     * @tparam Resource A value matching the resource concept. Specifies at compile time whether this is a Buffer or a Texture
     * @tparam T The underlying data type of the resource
     * @param resource A reference to the shared resource wrapper instance
     * @param idx The index at which this resource should be bound to.
     * @return A pointer to this (MTLCommandManager) for method chaining
     */
    MTLCommandManager* bind(const MTL::Buffer* resource, uint64_t idx);

    /**
     * @brief Directly binds the provided push constants to buffer(0)
     *
     * This method binds "Push Constants" directly to GPU memory,
     * making them available at buffer binding 30 (last buffer index).
     *
     * @warning sizeof(T) must be smaller than 4KB, as Metal only
     * supports binding of data which is smaller than 4KB (see
     * https://developer.apple.com/documentation/metal/mtlcomputecommandencoder/setbytes(_:length:index:)?language=objc).
     *
     * @tparam T The type of PushConstants to bind
     * @param pushConstants A constant reference to an initialized T.
     * @return A pointer to this (MTLCommandManager) for method chaining
     */
    template<typename T>
    MTLCommandManager* pushConstants(const T& pushConstants);

    /**
     * @brief Directly binds the provided push constants to buffer(0)
     *
     * This method binds "Push Constants" directly to GPU memory,
     * making them available at buffer binding 30 (last buffer index).
     *
     * @warning sizeof(T) must be smaller than 4KB, as Metal only
     * supports binding of data which is smaller than 4KB (see
     * https://developer.apple.com/documentation/metal/mtlcomputecommandencoder/setbytes(_:length:index:)?language=objc).
     *
     * @tparam T The type of constant to bind
     * @param constant A constant reference to an initialized T.
     * @param withIndex The index to bind the parameter to
     * @return A pointer to this (MTLCommandManager) for method chaining
     */
    template<typename T>
    MTLCommandManager* constant(const T& constant, uint64_t idx);

    /**
     * @brief Returns errors which occured during command buffer execution
     *
     * This method returns all occured errors of command buffers which faulted
     * during execution. The map is empty, if no errors occured.
     *
     * @return A map holding command buffer identification labels as keys and strings containing error descriptions
     */
    std::unordered_map<std::string, std::string> getErrors() const;

    /**
     * @brief Returns logs which were recorded during command buffer execution
     *
     * This method returns all recorded logs of command buffers during
     * execution. The map might be empty, if no logs were recorded.
     *
     * @return A map holding command buffer identification labels as keys and strings containing logs
     */
    std::unordered_map<std::string, std::string> getLogs() const;


  private:
    explicit MTLCommandManager(const NS::SharedPtr<MTL::Device>& device);

    // Current Command State
    NS::SharedPtr<MTL::ComputePipelineState> m_currentPipeline;
    NS::SharedPtr<MTL::CommandBuffer> m_commandBuffer;
    NS::SharedPtr<MTL::ComputeCommandEncoder> m_commandEncoder;

    // Command Buffers in flight
    std::deque<NS::SharedPtr<MTL::CommandBuffer>> m_commandBuffersInFlight = {};

    // Logging
    std::vector<std::pair<std::string, NS::SharedPtr<NS::Error>>> m_cmdBufferErrors = {};
    std::vector<std::pair<std::string, NS::SharedPtr<MTL::LogContainer>>> m_cmdBufferLogs = {};

    // Internal Cache
    NS::SharedPtr<MTL::Device> m_device;
    std::unordered_map<std::string, NS::SharedPtr<MTL::Library>> m_metalLibraries = {};  // A map holding library names and libraries
    std::unordered_map<std::string, NS::SharedPtr<MTL::ComputePipelineState>> m_pipelineStates =
      {};  // A map holding pipeline names and pipeline states
};

}  // namespace gpu
}  // namespace aliceVision

#include <AVDepthMap/Metal/host/impl/DeviceCommandManager.inl>

#endif  // COMMAND_HPP
