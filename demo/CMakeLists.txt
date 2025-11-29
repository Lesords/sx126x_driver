cmake_minimum_required(VERSION 3.25)

project(sx126x_demo LANGUAGES C)

add_executable(sx126x_demo
    main.c
    sx126x_hal_linux.c
)

add_executable(spi_loopback
    spi_loopback.c
)

target_link_libraries(sx126x_demo PRIVATE sx126x_driver)

target_include_directories(sx126x_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)
