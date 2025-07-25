### The AliceVision Binary Targets
###
### A collection of binaries providing functionality for various photogrammetry tasks
###
### Binaries will be added in order of dependency.
### That means, that only those dependencies are available, which are strictly required.
### Other binaries will be added step-by-step.
###
add_subdirectory(VectorGraphics VectorGraphics)
add_subdirectory(Pipeline)
add_subdirectory(Convert)
add_subdirectory(Export)
add_subdirectory(Utils)
