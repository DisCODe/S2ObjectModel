# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Find required packages

find_package(PCL 1.6 REQUIRED)

# Create an executable file from sources:
ADD_LIBRARY(ISS3DKeypoints SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(ISS3DKeypoints ${DisCODe_LIBRARIES} ${PCL_LIBRARIES})

INSTALL_COMPONENT(ISS3DKeypoints)

