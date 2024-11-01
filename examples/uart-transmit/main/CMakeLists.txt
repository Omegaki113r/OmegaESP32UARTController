set(SOURCES main.c)
set(HEADER_DIRS .)

idf_component_register(
                        SRCS ${SOURCES}
                        INCLUDE_DIRS ${HEADER_DIRS}
                        REQUIRES OmegaESP32UARTController
                        )
