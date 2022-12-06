cmake_minimum_required(VERSION 3.16)

function(add_test_with_conf TEST_NAME TEST_FILE)
    add_test(NAME ${TEST_NAME} COMMAND $<TARGET_FILE:ArtinxHub> ${CMAKE_SOURCE_DIR}/tests/${TEST_FILE} WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endfunction()

add_subdirectory(Solve)
add_subdirectory(Utility)
add_subdirectory(ActorFramework)
