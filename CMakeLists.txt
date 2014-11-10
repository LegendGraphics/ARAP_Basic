cmake_minimum_required( VERSION 2.8 )
project( Arap )

find_package( Qt4 REQUIRED )
find_package( VTK REQUIRED )
include( ${QT_USE_FILE} )
include( ${VTK_USE_FILE} )

FILE ( GLOB Project_SRCS "src/*.cpp" )
FILE ( GLOB SVD_SRCS "include/*.hpp" )
# SET( Project_SRCS src/main.cpp 
#  		    src/CranioViewer.cpp )

SET( Project_UIS src/Arap.ui )
SET( Project_MOC_HDRS include/Arap.h )
SET( Project_INCLUDE_DIR include/ )

QT4_WRAP_UI( Project_UIS_H ${Project_UIS} )

QT4_WRAP_CPP( Project_MOC_SRCS ${Project_MOC_HDRS} )

INCLUDE_DIRECTORIES( ${Project_INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${VTK_DIR} ../../eigen/ )

#SET( Project_RCCS MyProject.qrc )
#QT4_ADD_RESOURCES( Project_RCC_SRCS ${Project_RCCS} )

ADD_EXECUTABLE( ${PROJECT_NAME} ${Project_SRCS} ${Project_UIS_H} ${Project_MOC_SRCS} ${SVD_SRCS} )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${QT_LIBRARIES} ${VTK_LIBRARIES} QVTK)

MESSAGE( STATUS "${VTK_LIBRARIES}" )