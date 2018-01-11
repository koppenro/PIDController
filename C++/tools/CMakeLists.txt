message(${PROJECT_SOURCE_DIR})

file(GLOB HEADERS "*.h")
file(GLOB SOURCES "*.cc")



add_library(LibTools SHARED ${SOURCES} ${HEADERS})


set(LIBS ${LIBS})
TARGET_LINK_LIBRARIES(LibTools ${LIBS} -pthread)
