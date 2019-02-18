#Specify the version being used as well as the language
cmake_minimum_required(VERSION 2.8.11)

#Name your project here
project(qcl-tdi)

#set the module directory
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

#find the pthreads package
find_package(Threads)

#find the X11 package
find_package(X11)

#find the STIM package
find_package(STIM REQUIRED)

#ask for the Aerotech A3200 library file
find_library(AEROTECH_LIB NAMES A3200C.lib PATHS lib32)

#ask for the Teledyne API libraries
find_library(TELEDYNE_ACQ_LIB NAMES ipsacq.lib PATHS lib32)
find_library(TELEDYNE_SAPERA_LIB NAMES ipssaperalt.lib PATHS lib32)
find_library(TELEDYNE_FRAMEGRAB_LIB NAMES sbfem.lib PATHS lib32)

#ask for the MIRcat libraries
find_library(MIRCAT_LIB NAMES MIRcatSDK.lib PATHS lib32)

#ask for the OpenGL libraries
#find_library(OPENGL_LIB NAMES OpenGL32.lib PATHS lib32)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

#build the executable in the binary directory on MS Visual Studio
if ( MSVC )
	SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${OUTPUT_DIRECTORY}")
	SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIRECTORY}")
endif ( MSVC )

include_directories(
					"include"
					${STIM_INCLUDE_DIR}
					${OPENGL_INCLUDE_DIRS}
					${GLUT_INCLUDE_DIR}
					)

#Assign source files to the appropriate variables
file(GLOB SRC_CPP "*.cpp")
file(GLOB SRC_H "*.h")

#create an executable file
add_executable(qcl-mean
				${SRC_H}
				calculate-mean.cpp
				)

#set the link libraries
target_link_libraries(qcl-mean
						${AEROTECH_LIB}
						${TELEDYNE_ACQ_LIB}
						${TELEDYNE_SAPERA_LIB}
						${TELEDYNE_FRAMEGRAB_LIB}
						${CMAKE_THREAD_LIBS_INIT}
						${MIRCAT_LIB}
		  )

add_executable(qcl-laser-tuning-discrete
				${SRC_H}
				laser-power-compensation_new.cpp
				)

#set the link libraries
target_link_libraries(qcl-laser-tuning-discrete
						${AEROTECH_LIB}
						${TELEDYNE_ACQ_LIB}
						${TELEDYNE_SAPERA_LIB}
						${TELEDYNE_FRAMEGRAB_LIB}
						${CMAKE_THREAD_LIBS_INIT}
						${MIRCAT_LIB}
		  )


add_executable(qcl-laser-tuning-constant
				${SRC_H}
				laser-power-compensation_new.cpp
				)

#set the link libraries
target_link_libraries(qcl-laser-tuning-constant
						${AEROTECH_LIB}
						${TELEDYNE_ACQ_LIB}
						${TELEDYNE_SAPERA_LIB}
						${TELEDYNE_FRAMEGRAB_LIB}
						${CMAKE_THREAD_LIBS_INIT}
						${MIRCAT_LIB}
		  )

add_executable(qcl-const-power
				${SRC_H}
				constant-power-imaging.cpp
				)

#set the link libraries
target_link_libraries(qcl-const-power
						${AEROTECH_LIB}
						${TELEDYNE_ACQ_LIB}
						${TELEDYNE_SAPERA_LIB}
						${TELEDYNE_FRAMEGRAB_LIB}
						${CMAKE_THREAD_LIBS_INIT}
						${MIRCAT_LIB}
		  )

add_executable(qcl-laser-scan
				${SRC_H}
				laser-scanning.cpp
				)

#set the link libraries
target_link_libraries(qcl-laser-scan
						${AEROTECH_LIB}
						${TELEDYNE_ACQ_LIB}
						${TELEDYNE_SAPERA_LIB}
						${TELEDYNE_FRAMEGRAB_LIB}
						${CMAKE_THREAD_LIBS_INIT}
						${MIRCAT_LIB}
		  )
		  
add_executable(qcl-capture-frames
				${SRC_H}
				capture-frames.cpp
				)

#set the link libraries
target_link_libraries(qcl-capture-frames
						${AEROTECH_LIB}
						${TELEDYNE_ACQ_LIB}
						${TELEDYNE_SAPERA_LIB}
						${TELEDYNE_FRAMEGRAB_LIB}
						${CMAKE_THREAD_LIBS_INIT}
						${MIRCAT_LIB}
		  )

add_executable(qcl-holo-cap
				${SRC_H}
				holo3pos.cpp
				)

#set the link libraries
target_link_libraries(qcl-holo-cap
						${AEROTECH_LIB}
						${TELEDYNE_ACQ_LIB}
						${TELEDYNE_SAPERA_LIB}
						${TELEDYNE_FRAMEGRAB_LIB}
						${CMAKE_THREAD_LIBS_INIT}
						${MIRCAT_LIB}
		  )

add_executable(showim
				${SRC_H}
				showim.cpp
				)

#set the link libraries
target_link_libraries(showim
						#${OPENGL_LIB}
						${OPENGL_gl_LIBRARY}
						${OPENGL_glu_LIBRARY}
						${GLUT_LIBRARIES}

		  )

add_executable(qcl-inte
				${SRC_H}
				inte.cpp
				)

#set the link libraries
target_link_libraries(qcl-inte
						${AEROTECH_LIB}
						${TELEDYNE_ACQ_LIB}
						${TELEDYNE_SAPERA_LIB}
						${TELEDYNE_FRAMEGRAB_LIB}
						${CMAKE_THREAD_LIBS_INIT}
						${MIRCAT_LIB}
		  )

add_executable(qcl-snap-gns
				${SRC_H}
				snap-gns.cpp
				)

#set the link libraries
target_link_libraries(qcl-snap-gns
						${AEROTECH_LIB}
						${TELEDYNE_ACQ_LIB}
						${TELEDYNE_SAPERA_LIB}
						${TELEDYNE_FRAMEGRAB_LIB}
						${CMAKE_THREAD_LIBS_INIT}
						${MIRCAT_LIB}
		  )

add_executable(qcl-snap-gw
				${SRC_H}
				snap-gw.cpp
				)

#set the link libraries
target_link_libraries(qcl-snap-gw
						${AEROTECH_LIB}
						${TELEDYNE_ACQ_LIB}
						${TELEDYNE_SAPERA_LIB}
						${TELEDYNE_FRAMEGRAB_LIB}
						${CMAKE_THREAD_LIBS_INIT}
						${MIRCAT_LIB}
		  )