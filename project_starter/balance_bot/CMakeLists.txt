set(BALANCE_BOT_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DBALANCE_BOT_FOLDER="${BALANCE_BOT_FOLDER}")

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/balance_bot)
ADD_EXECUTABLE (simviz_balance_bot simviz.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE(controller_balance_bot controller.cpp utils.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (simviz_balance_bot ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_balance_bot ${CS225A_COMMON_LIBRARIES})
