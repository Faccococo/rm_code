cmake_minimum_required(VERSION 3.16)

set(SOURCE 
"Core/BlackBoard.cpp"
"Core/ChaosMonkey.cpp"
"Core/Daemon.cpp"
"Core/Logger.cpp"
"Core/PerformanceMonitor.cpp"
"DataLink/HttpServer.cpp"
"DataLink/AsyncSerial/AsyncSerial.cpp"
"DataLink/AsyncSerial/BufferedAsyncSerial.cpp"
"DataLink/SerialPort.cpp"
"Detect/ArmorDetector.cpp"
"Detect/NNetArmorDetector.cpp"
"Detect/EnergyDetector.cpp"
"Detect/OreAlignment.cpp"
"Detect/OreExchangeRectifier.cpp"
"HubMain.cpp"
"Postprocess/ArmorPredictor.cpp"
"Preprocess/ColorCalibration.cpp"
"Preprocess/IMUFilter.cpp"
"Preprocess/Undistort.cpp"
"Sensor/DahengDriver.cpp"
"Sensor/ImageSequenceReplay.cpp"
"Sensor/VideoReplay.cpp"
"Solve/AngleSolver.cpp"
"Solve/ArmorLocator.cpp"
"Solve/RadarLocator.cpp"
"Strategy/Hero.cpp"
"Strategy/Infantry.cpp"
"Strategy/Sentry.cpp"
"Strategy/UAV.cpp"
"Test/ActorFramework/Actor.cpp"
"Test/ActorFramework/Availability.cpp"
"Test/ActorFramework/DaemonTester.cpp"
"Test/ActorFramework/FrameRateKeeperTester.cpp"
"Test/Detect/FakeSerialPort.cpp"
"Test/Sensor/FixedIMU.cpp"
"Test/Solve/ArmorLocatorTester.cpp"
"Test/Solve/FakeDetector.cpp"
"Test/Solve/FakeHead.cpp"
"Test/Solve/FakeIMU.cpp"
"Test/Solve/SimpleStrategy.cpp"
"Test/Solve/Simulator.cpp"
"Test/TestHelper.cpp"
"Utility/ArmorDetectorDrawer.cpp"
"Utility/CarDetectorDrawer.cpp"
"Utility/Crc.cpp"
"Utility/KM.cpp"
"Utility/OpenCVHelper.cpp"
"Utility/PID.cpp"
"Utility/Timer.cpp"
"Utility/VideoRecorder.cpp"
)

find_package(CAF REQUIRED COMPONENTS core io)
find_package(glm CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_path(CPP_HTTPLIB_INCLUDE_DIRS "httplib.h")
find_package(OpenCV CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)


if(${ARTINX_HUB_CAMERA} STREQUAL "USB3")
    set(DAHENG_SDK_PATH $ENV{DAHENG_SDK})
elseif(${ARTINX_HUB_CAMERA} STREQUAL "USB2")
    set(DAHENG_SDK_PATH $ENV{DAHENG_ROOT}/sdk)
else()
    message(FATAL "Invalid camera type")
endif()
message(${ARTINX_HUB_CAMERA} ":" ${DAHENG_SDK_PATH})

#find_package(Bullet CONFIG REQUIRED)
find_package(magic_enum CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(Boost REQUIRED system)
set(InferenceEngine_DIR $ENV{ONEAPI_ROOT}/openvino/openvino_2021/deployment_tools/inference_engine/share/)
find_package(InferenceEngine CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

add_executable(ArtinxHub ${SOURCE})
target_include_directories(ArtinxHub PRIVATE ${CAF_INCLUDE_DIRS})
target_link_libraries(ArtinxHub PRIVATE CAF::core)
target_link_libraries(ArtinxHub PRIVATE opencv_core opencv_videoio opencv_mcc opencv_calib3d opencv_barcode opencv_tracking)
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    target_link_libraries(ArtinxHub PRIVATE Iphlpapi)
endif ()
target_link_libraries(ArtinxHub PRIVATE glm::glm)
target_include_directories(ArtinxHub PRIVATE ${CPP_HTTPLIB_INCLUDE_DIRS})
if(${ARTINX_HUB_CAMERA} STREQUAL "USB2")
target_compile_definitions(ArtinxHub PRIVATE "ARTINX_DAHENG_USB2")
target_include_directories(ArtinxHub PRIVATE ${DAHENG_SDK_PATH}/include)
target_link_directories(ArtinxHub PRIVATE ${DAHENG_SDK_PATH}/lib/pub)
target_link_directories(ArtinxHub PRIVATE ${DAHENG_SDK_PATH}/genicam/bin/Linux64_x64)
else()
target_include_directories(ArtinxHub PRIVATE ${DAHENG_SDK_PATH}/inc)
target_link_directories(ArtinxHub PRIVATE ${DAHENG_SDK_PATH}/lib/x64)
endif()

target_link_libraries(ArtinxHub PRIVATE gxiapi)
#target_link_libraries(ArtinxHub PRIVATE LinearMath Bullet3Common BulletDynamics BulletCollision)
target_link_libraries(ArtinxHub PRIVATE magic_enum::magic_enum)
target_link_libraries(ArtinxHub PRIVATE fmt::fmt)
target_link_libraries(ArtinxHub PRIVATE ${InferenceEngine_LIBRARIES})
target_link_libraries(ArtinxHub PRIVATE nlohmann_json nlohmann_json::nlohmann_json)
target_link_libraries(ArtinxHub PRIVATE Eigen3::Eigen)

add_custom_command(TARGET ArtinxHub PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/pages $<TARGET_FILE_DIR:ArtinxHub>/pages)
add_custom_command(TARGET ArtinxHub PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/scripts $<TARGET_FILE_DIR:ArtinxHub>/scripts)
add_custom_command(TARGET ArtinxHub PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/data $<TARGET_FILE_DIR:ArtinxHub>/data)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    target_compile_options(ArtinxHub PRIVATE /W4 /WX-)
    add_custom_command(TARGET ArtinxHub PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${DAHENG_SDK_PATH}/../../APIDll/Win64/GxIAPI.dll $<TARGET_FILE_DIR:ArtinxHub>)
    set(INFERENCE_ENGINE_ROOT $ENV{ONEAPI_ROOT}/openvino/openvino_2021/deployment_tools/inference_engine)
    file(GLOB INFERENCE_ENGINE_DLLS ${INFERENCE_ENGINE_ROOT}/bin/intel64/${CMAKE_BUILD_TYPE}/*.dll)
    add_custom_command(TARGET ArtinxHub PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${INFERENCE_ENGINE_DLLS} $<TARGET_FILE_DIR:ArtinxHub>)
    file(GLOB_RECURSE INFERENCE_ENGINE_EXTERNAL_DLLS ${INFERENCE_ENGINE_ROOT}/external/*.dll)
    add_custom_command(TARGET ArtinxHub PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${INFERENCE_ENGINE_EXTERNAL_DLLS} $<TARGET_FILE_DIR:ArtinxHub>)
    file(GLOB NGRAPH_DLLS $ENV{ONEAPI_ROOT}/openvino/openvino_2021/deployment_tools/ngraph/lib/*.dll)
    remove(NGRAPH_DLLS $ENV{ONEAPI_ROOT}/openvino/openvino_2021/deployment_tools/ngraph/lib/libprotobuf.dll)
    remove(NGRAPH_DLLS $ENV{ONEAPI_ROOT}/openvino/openvino_2021/deployment_tools/ngraph/lib/libprotobufd.dll)
    add_custom_command(TARGET ArtinxHub PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${NGRAPH_DLLS} $<TARGET_FILE_DIR:ArtinxHub>)
    add_custom_command(TARGET ArtinxHub PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${INFERENCE_ENGINE_ROOT}/bin/intel64/${CMAKE_BUILD_TYPE}/plugins.xml $<TARGET_FILE_DIR:ArtinxHub>)
endif ()

add_custom_command(TARGET ArtinxHub PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/config_templates/caf-application.conf $<TARGET_FILE_DIR:ArtinxHub>)
