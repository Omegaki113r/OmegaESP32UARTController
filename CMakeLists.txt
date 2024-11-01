set(SOURCES src/OmegaESP32UARTController.c)
set(PUBLIC_HEADER_DIRS inc)
set(PRIVATE_HEADER_DIRS priv/inc)

file(
        DOWNLOAD https://raw.githubusercontent.com/Omegaki113r/OmegaUtilityDriver/refs/heads/main/inc/OmegaUtilityDriver.h 
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/OmegaUtilityDriver.h
    )

idf_component_register(
                        SRCS ${SOURCES}
                        INCLUDE_DIRS ${PUBLIC_HEADER_DIRS}
                        REQUIRES driver
                        )
target_include_directories(${COMPONENT_LIB} PRIVATE ${PRIVATE_HEADER_DIRS})

include(ExternalProject)
set(OmegaUARTController_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/OmegaUARTController_install)
ExternalProject_Add(OmegaUARTController
    GIT_REPOSITORY  https://github.com/Omegaki113r/OmegaUARTController.git
    GIT_TAG origin/main
    CMAKE_ARGS
        # Use the same CMake toolchain file as for the main project.
        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
        # Pass the install directory to the subproject.
        -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    INSTALL_DIR ${OmegaUARTController_INSTALL_DIR}
    BUILD_BYPRODUCTS "${OmegaUARTController_INSTALL_DIR}/lib/libOmegaUARTController.a"
    )
add_prebuilt_library(OmegaUARTController11 "${OmegaUARTController_INSTALL_DIR}/lib/libOmegaUARTController.a")
target_include_directories(OmegaUARTController11 INTERFACE "${OmegaUARTController_INSTALL_DIR}/include")
add_dependencies(OmegaUARTController11 OmegaUARTController1)
target_link_libraries(${COMPONENT_LIB} INTERFACE OmegaUARTController11)

target_include_directories(${COMPONENT_LIB} PUBLIC "${OmegaUARTController_INSTALL_DIR}/include")

# add_library(UARTController STATIC ${SOURCES})
# target_include_directories(UARTController PUBLIC ${HEADER_DIRS})
