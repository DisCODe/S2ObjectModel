# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Find required packages


# Create an executable file from sources:
ADD_LIBRARY(SingleS20MMatcher SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(SingleS20MMatcher ${DisCODe_LIBRARIES} )

INSTALL_COMPONENT(SingleS20MMatcher)
