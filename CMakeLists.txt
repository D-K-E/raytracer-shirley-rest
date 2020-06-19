cmake_minimum_required (VERSION 3.0.2)
project("Isin Izleme - Raytracing")

#find_package(OpenGL REQUIRED)
find_package (Threads REQUIRED)

set (CMAKE_CXX_FLAGS "-std=c++17")
set (CMAKE_BUILD_TYPE "Debug")
set (FLAGS "-ldl -ggdb -Wall -Wextra -pthread")

include_directories(
    # my headers
    "./include/"
    )

# ---------- Ortak -------------------

# ---------- Haftasonu ---------------
# ---------- Sonraki -----------------
# ---------- Nihai -------------------

#set( GLFW_SHARED_LIB
#    "${PROJECT_SOURCE_DIR}/libs/glfw/src/libglfw.so")

set (ASSIMP_SHARED_LIB
    "${PROJECT_SOURCE_DIR}/libs/assimp/libassimp.so.4")

set ( ALL_LIBS
    #${OpenGL}
    #${GLFW_SHARED_LIB}
    ${ASSIMP_SHARED_LIB}
    ${Threads}
    ${FLAGS}
    )


#add_executable(final.out "src/final.cpp")
#target_link_libraries(final.out ${ALL_LIBS})
#install(TARGETS final.out DESTINATION "${PROJECT_SOURCE_DIR}/bin/")

add_executable(nosampling.out "src/cornell.cpp")
target_link_libraries(nosampling.out ${ALL_LIBS})
install(TARGETS nosampling.out DESTINATION "${PROJECT_SOURCE_DIR}/bin/")
