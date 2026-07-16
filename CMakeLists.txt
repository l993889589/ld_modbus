cmake_minimum_required(VERSION 3.16)

project(ld_modbus VERSION 0.2.0 LANGUAGES C)

option(LD_MODBUS_BUILD_TESTS "Build ld_modbus host tests" ON)

add_library(ld_modbus STATIC
    src/ld_modbus_client.c
    src/ld_modbus_core.c
    src/ld_modbus_rtu_framer.c
    src/ld_modbus_server.c
)

target_include_directories(ld_modbus PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(ld_modbus PUBLIC c_std_99)

if(MSVC)
    target_compile_options(ld_modbus PRIVATE /W4 /WX)
else()
    target_compile_options(ld_modbus PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

include(GNUInstallDirs)
install(TARGETS ld_modbus EXPORT ld_modbusTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT ld_modbusTargets
    FILE ld_modbusTargets.cmake
    NAMESPACE ld_modbus::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ld_modbus
)

if(LD_MODBUS_BUILD_TESTS)
    enable_testing()
    add_executable(ld_modbus_tests tests/test_ld_modbus.c)
    target_link_libraries(ld_modbus_tests PRIVATE ld_modbus)
    if(MSVC)
        target_compile_options(ld_modbus_tests PRIVATE /W4 /WX /UNDEBUG)
    else()
        target_compile_options(ld_modbus_tests PRIVATE
            -Wall -Wextra -Wpedantic -Werror -UNDEBUG
        )
    endif()
    add_test(NAME ld_modbus_all COMMAND ld_modbus_tests)

    add_executable(ld_modbus_rtu_framer_tests
        tests/test_ld_modbus_rtu_framer.c
    )
    target_link_libraries(ld_modbus_rtu_framer_tests PRIVATE ld_modbus)
    if(MSVC)
        target_compile_options(ld_modbus_rtu_framer_tests PRIVATE
            /W4 /WX /UNDEBUG
        )
    else()
        target_compile_options(ld_modbus_rtu_framer_tests PRIVATE
            -Wall -Wextra -Wpedantic -Werror -UNDEBUG
        )
    endif()
    add_test(NAME ld_modbus_rtu_framer_all COMMAND ld_modbus_rtu_framer_tests)

    add_executable(ld_modbus_rtu_receiver_tests
        tests/test_ld_modbus_rtu_receiver.c
    )
    target_link_libraries(ld_modbus_rtu_receiver_tests PRIVATE ld_modbus)
    if(MSVC)
        target_compile_options(ld_modbus_rtu_receiver_tests PRIVATE
            /W4 /WX /UNDEBUG
        )
    else()
        target_compile_options(ld_modbus_rtu_receiver_tests PRIVATE
            -Wall -Wextra -Wpedantic -Werror -UNDEBUG
        )
    endif()
    add_test(NAME ld_modbus_rtu_receiver_all
             COMMAND ld_modbus_rtu_receiver_tests)
endif()
