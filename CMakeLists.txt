cmake_minimum_required(VERSION 3.5.1)

project(zense_via_grpc)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O0")

#OpenCV
find_package(OpenCV 3.4.3 REQUIRED)

#PicoZense
find_package(PkgConfig REQUIRED)
pkg_check_modules(PicoZense libpicozense REQUIRED)
include_directories(${PicoZense_INCLUDE_DIRS})

#Protobuf
find_package(Protobuf CONFIG REQUIRED)
set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
set(_REFLECTION gRPC::grpc++_reflection)
set(_PROTOBUF_PROTOC $<TARGET_FILE:protobuf::protoc>)

find_package(gRPC CONFIG REQUIRED)
message(STATUS "Using gRPC ${gRPC_VERSION}")
set(_GRPC_GRPCPP gRPC::grpc++)
set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:gRPC::grpc_cpp_plugin>)

get_filename_component(hw_proto_zense "${CMAKE_CURRENT_BINARY_DIR}/../protos/zense.proto" ABSOLUTE)
get_filename_component(hw_proto_zense_path "${hw_proto_zense}" PATH)

set(hw_proto_zense_srcs "${CMAKE_CURRENT_BINARY_DIR}/../protos/zense.pb.cc")
set(hw_proto_zense_hdrs "${CMAKE_CURRENT_BINARY_DIR}/../protos/zense.pb.h")
set(hw_grpc_zense_srcs "${CMAKE_CURRENT_BINARY_DIR}/../protos/zense.grpc.pb.cc")
set(hw_grpc_zense_hdrs "${CMAKE_CURRENT_BINARY_DIR}/../protos/zense.grpc.pb.h")
add_custom_command(
      OUTPUT "${hw_proto_zense_srcs}" "${hw_proto_zense_hdrs}" "${hw_grpc_zense_srcs}" "${hw_grpc_zense_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}/../protos"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}/../protos"
        -I "${hw_proto_zense_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${hw_proto_zense}"
      DEPENDS "${hw_proto_zense}")

# Include generated *.pb.h files
include_directories(
      include
      protos
      "${CMAKE_CURRENT_BINARY_DIR}"
    )

set(SOURCE_CODE
  src/common.cpp
  src/parameter_manager.cpp
  src/pico_zense_manager.cpp
  src/pico_zense_simple_publisher.cpp
  src/pico_zense_undistorter.cpp
)


foreach(_target pico_zense_simple_publisher)
  add_executable(${_target} "src/${_target}.cpp"
    ${hw_proto_zense_srcs}
    ${hw_grpc_zense_srcs}
    ${SOURCE_CODE}
    )
  target_link_libraries(${_target}
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF}
    ${OpenCV_LIBRARIES}
    ${PicoZense_LIBRARIES}
    )
endforeach()