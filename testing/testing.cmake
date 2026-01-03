# install the navigation test cases
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/testing DESTINATION share/${PROJECT_NAME})

# install the test node
install(PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/testing/test_node.py

    DESTINATION
    lib/${PROJECT_NAME}
)
