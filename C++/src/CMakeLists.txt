

include_directories(${PROJECT_SOURCE_DIR}/tools)


link_directories(${PROJECT_SOURCE_DIR/lib})

set(LIBS ${LIBS} LibTools)

file(GLOB BINARIES RELATIVE ${PROJECT_SOURCE_DIR}/src "*.cc")

#Kurzform um alle Binaries mit ihrem File namen zu bauen, und linken mit der
# LIbrary LIBS
message("#### Building the following executables: ####")
foreach( sourcefile ${BINARIES} )
  string(REPLACE ".cc" "" name ${sourcefile})
  message(STATUS "    ${name}")
  add_executable(${name} ${sourcefile})
  target_link_libraries(${name} ${LIBS})
endforeach(sourcefile ${BINARIES})
message("#### End ####")
