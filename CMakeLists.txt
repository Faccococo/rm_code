cmake_minimum_required(VERSION 3.16)

project(artinx_hub)
set(CMAKE_CXX_STANDARD 17)
enable_testing()

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    add_compile_definitions("UNICODE")
    add_compile_definitions("_UNICODE")
    add_compile_options("/utf-8")
    add_compile_options("/MP")
    add_compile_options("/Wall" "/WX" "/wd4514" "/wd4625" "/wd5026" "/wd4626" "/wd5027" "/wd4820" "/wd4365" "/wd4355" "/wd4582" "/wd4619" "/wd5031" "/wd4668" "/wd5039" "/wd5204" "/wd4868" "/wd5045" "/wd4946" "/wd4866" "/wd4371" "/wd4711" "/wd4710" "/wd5220" "/wd4324")
    add_compile_options("/permissive-")

    if (CMAKE_BUILD_TYPE MATCHES Debug)
        add_compile_options("/Od" "/Oy-")
    else ()
        add_compile_options("/O2" "/Ob2" "/Oi" "/Ot" "/Oy" "/GT" "/GL" "/Qpar")
        add_link_options("/LTCG:incremental" "/OPT:REF" "/OPT:ICF")
    endif ()
elseif (CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options("-Wall" "-Werror")

    if (CMAKE_BUILD_TYPE MATCHES Debug)
        add_compile_options("-O0" "-fno-omit-frame-pointer")
    else ()
        add_compile_options("-Ofast")
    endif ()
endif ()

if (CMAKE_BUILD_TYPE MATCHES Debug)
    add_compile_definitions("ARTINXHUB_DEBUG")
endif ()

include_directories(BEFORE include)

add_subdirectory(src)
add_subdirectory(tests)
