cmake_minimum_required(VERSION 3.20)
project(control_lab_c LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

function(enable_strict_warnings target_name)
    if(MSVC)
        target_compile_options(${target_name} PRIVATE /W4 /WX)
    else()
        target_compile_options(
            ${target_name}
            PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Werror
        )
    endif()
endfunction()

add_library(control_lab_maglev STATIC src/maglev_observer.c)
target_include_directories(control_lab_maglev PUBLIC include)
enable_strict_warnings(control_lab_maglev)

add_library(control_lab_two_tank STATIC src/two_tank.c)
target_include_directories(control_lab_two_tank PUBLIC include)
enable_strict_warnings(control_lab_two_tank)

if(NOT MSVC)
    target_link_libraries(control_lab_maglev PUBLIC m)
    target_link_libraries(control_lab_two_tank PUBLIC m)
endif()

add_executable(maglev_observer_demo apps/maglev_observer_demo.c)
target_link_libraries(maglev_observer_demo PRIVATE control_lab_maglev)
enable_strict_warnings(maglev_observer_demo)

add_executable(two_tank_demo apps/two_tank_demo.c)
target_link_libraries(two_tank_demo PRIVATE control_lab_two_tank)
enable_strict_warnings(two_tank_demo)

include(CTest)
if(BUILD_TESTING)
    add_executable(test_maglev_observer tests/test_maglev_observer.c)
    target_link_libraries(test_maglev_observer PRIVATE control_lab_maglev)
    enable_strict_warnings(test_maglev_observer)
    add_test(NAME c_maglev_observer COMMAND test_maglev_observer)

    add_executable(test_two_tank tests/test_two_tank.c)
    target_link_libraries(test_two_tank PRIVATE control_lab_two_tank)
    enable_strict_warnings(test_two_tank)
    add_test(NAME c_two_tank_anti_windup COMMAND test_two_tank)
endif()
