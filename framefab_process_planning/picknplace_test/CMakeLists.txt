
add_executable(picknplace_test src/picknplace_test.cpp)
target_link_libraries(picknplace_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS picknplace_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
