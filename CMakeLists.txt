cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ASIO_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/asio-1.22.1/include)

########################################
project(first LANGUAGES CXX)
add_executable(first 
    src/first.cpp
)
target_compile_options(first PUBLIC -DASIO_STANDALONE)
target_include_directories(first PUBLIC ${ASIO_INCLUDE_DIR})
target_link_libraries(first PUBLIC pthread)

########################################
project(udpc LANGUAGES CXX)
add_executable(udpc 
    src/udpc.cpp
)
target_include_directories(udpc PUBLIC ${ASIO_INCLUDE_DIR})
target_link_libraries(udpc PUBLIC pthread)

# project(udps LANGUAGES CXX)
# add_executable(udps 
#     src/udps.cpp
# )
# target_include_directories(udps PUBLIC ${ASIO_INCLUDE_DIR})
# target_link_libraries(udps PUBLIC pthread)

########################################
project(uart LANGUAGES CXX)
add_executable(uart 
    src/uart.cpp
)
target_include_directories(uart PUBLIC ${ASIO_INCLUDE_DIR})
target_link_libraries(uart PUBLIC pthread)

########################################
project(can LANGUAGES CXX)
add_executable(can 
    src/can.cpp
)
target_include_directories(can PUBLIC ${ASIO_INCLUDE_DIR})
target_link_libraries(can PUBLIC pthread)

########################################
project(multicast_rx LANGUAGES CXX)
add_executable(multicast_rx 
    src/multicast_rx.cpp
)
target_include_directories(multicast_rx PUBLIC ${ASIO_INCLUDE_DIR})
target_link_libraries(multicast_rx PUBLIC pthread)

project(multicast_tx LANGUAGES CXX)
add_executable(multicast_tx 
    src/multicast_tx.cpp
)
target_include_directories(multicast_tx PUBLIC ${ASIO_INCLUDE_DIR})
target_link_libraries(multicast_tx PUBLIC pthread)

########################################
project(unix_socket_tx LANGUAGES CXX)
add_executable(unix_socket_tx 
    src/unix_socket_tx.cpp
)
target_include_directories(unix_socket_tx PUBLIC ${ASIO_INCLUDE_DIR})
target_link_libraries(unix_socket_tx PUBLIC pthread)

project(unix_socket_rx LANGUAGES CXX)
add_executable(unix_socket_rx 
    src/unix_socket_rx.cpp
)
target_include_directories(unix_socket_rx PUBLIC ${ASIO_INCLUDE_DIR})
target_link_libraries(unix_socket_rx PUBLIC pthread)
