cmake_minimum_required(VERSION 3.6)
project(mocapPlayer_starter)

#########################################################
# SET COMPILER
#########################################################
set (CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++11)

#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)

#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

#########################################################
# FIND FLTK
#########################################################
set(FLTK_DIR /Users/yiqizhong/code/fltk-1.3.4-1/biuld)
find_package(FLTK REQUIRED NO_MODULE)
if(NOT FLTK_FOUND)
    message(ERROR " FLTK not found!")
endif(NOT FLTK_FOUND)

#########################################################
# MOCAPPLAYER EXE FILES
#########################################################
SET(MOCAPPLAYER_SOURCE
        displaySkeleton.cpp
        interface.cpp
        motion.cpp
        posture.cpp
        skeleton.cpp
        transform.cpp
        vector.cpp
        mocapPlayer.cpp
        ppm.cpp
        pic.cpp
        performanceCounter.cpp)

SET(MOCAPPLAYER_HEADERS
        displaySkeleton.h
        interface.h
        motion.h
        posture.h
        skeleton.h
        transform.h
        vector.h
        mocapPlayer.h
        pic.h
        performanceCounter.h)


#########################################################
# INTERPOLATE EXE FILES
#########################################################
SET(INTERPOLATE_SOURCE
        motion.cpp
        posture.cpp
        skeleton.cpp
        transform.cpp
        vector.cpp
        interpolator.cpp
        quaternion.cpp
        interpolate.cpp
        )

SET(INTERPOLATE_HEADERS
        motion.h
        posture.h
        skeleton.h
        transform.h
        vector.h
        interpolator.h
        quaternion.h
        )


#########################################################
# ADD EXECUTABLES
#########################################################
add_executable(mocapPlayer ${MOCAPPLAYER_SOURCE} ${MOCAPPLAYER_HEADERS})
add_executable(interpolate ${INTERPOLATE_SOURCE} ${INTERPOLATE_HEADERS})


#########################################################
# LINK LIBRARIES
#########################################################
target_include_directories(mocapPlayer PUBLIC ${FLTK_INCLUDE_DIRS})
target_include_directories(interpolate PUBLIC ${FLTK_INCLUDE_DIRS})
target_link_libraries(mocapPlayer fltk fltk_gl ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
target_link_libraries(interpolate fltk fltk_gl ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

