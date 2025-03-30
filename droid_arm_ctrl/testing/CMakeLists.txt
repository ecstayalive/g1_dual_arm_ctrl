add_executable(test_arm_api test_arm_api.cpp)
target_link_libraries(test_arm_api unitree_sdk2)

add_executable(read_arm_q read_arm_q.cpp)
target_link_libraries(read_arm_q unitree_sdk2)
