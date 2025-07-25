# Headers
set(sensorDB_files_headers
    Datasheet.hpp
    parseDatabase.hpp
)

# Sources
set(sensorDB_files_sources
    Datasheet.cpp
    parseDatabase.cpp
)

alicevision_add_library(AVSensorDB
    SOURCES ${sensorDB_files_sources}
    HEADERS ${sensorDB_files_headers}
    PRIVATE_LINKS
        Boost::system
        Boost::boost
    RESOURCES
        "./Resources/cameraSensors.db"

)

# Unit tests
alicevision_add_test(Tests/ParseDatabase_test.cpp NAME "SensorDB_ParseDatabase" LINKS AVSensorDB)
