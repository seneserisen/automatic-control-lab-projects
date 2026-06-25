cmake_minimum_required(VERSION 3.20)
project(control_lab_c LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_library(control_lab_maglev STATIC
    src/maglev_observer.c
)
target_include_directories(control_lab_maglev PUBLIC include)

if(MSVC)
    target_compile_options(control_lab_maglev PRIVATE /W4 /WX)
else()
    target_compile_options(control_lab_maglev PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_link_libraries(control_lab_maglev PUBLIC m)
endif()

add_executable(maglev_observer_demo apps/maglev_observer_demo.c)
target_link_libraries(maglev_observer_demo PRIVATE control_lab_maglev)

include(CTest)
if(BUILD_TESTING)
    add_executable(test_maglev_observer tests/test_maglev_observer.c)
    target_link_libraries(test_maglev_observer PRIVATE control_lab_maglev)
    if(NOT MSVC)
        target_compile_options(test_maglev_observer PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME c_maglev_observer COMMAND test_maglev_observer)
endif()
