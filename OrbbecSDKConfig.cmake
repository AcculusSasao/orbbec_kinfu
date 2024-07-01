# MIT License : Copyright (c) 2024 Yukiyoshi Sasao
find_path(OrbbecSDK_INCLUDE_DIR
	NAMES libobsensor/ObSensor.hpp
	PATHS ${CMAKE_CURRENT_LIST_DIR}/SDK /usr/include /usr/local/include
	PATH_SUFFIXES include
)
find_library(OrbbecSDK_LIBRARY
	OrbbecSDK
	PATHS ${CMAKE_CURRENT_LIST_DIR}/SDK/lib /usr/lib /usr/local/lib
	PATH_SUFFIXES lib
)
find_library(DEPTHENGINE_LIBRARY
	depthengine
	PATHS ${CMAKE_CURRENT_LIST_DIR}/SDK/lib /usr/lib /usr/local/lib
	PATH_SUFFIXES lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OrbbecSDK
	REQUIRED_VARS
		OrbbecSDK_INCLUDE_DIR
		OrbbecSDK_LIBRARY
		DEPTHENGINE_LIBRARY
)

if(OrbbecSDK_FOUND AND NOT TARGET OrbbecSDK::OrbbecSDK)
	add_library(OrbbecSDK::OrbbecSDK SHARED IMPORTED)
	set_target_properties(OrbbecSDK::OrbbecSDK PROPERTIES
	IMPORTED_LINK_INTERFACE_LANGUAGES ["C"|"CXX"]
	IMPORTED_LOCATION ${OrbbecSDK_LIBRARY}
	INTERFACE_INCLUDE_DIRECTORIES ${OrbbecSDK_INCLUDE_DIR}
	)
endif()
if(OrbbecSDK_FOUND AND NOT TARGET OrbbecSDK::DepthEngine)
	add_library(OrbbecSDK::DepthEngine SHARED IMPORTED)
	set_target_properties(OrbbecSDK::DepthEngine PROPERTIES
	IMPORTED_LINK_INTERFACE_LANGUAGES ["C"|"CXX"]
	IMPORTED_LOCATION ${DEPTHENGINE_LIBRARY}
	INTERFACE_INCLUDE_DIRECTORIES ${OrbbecSDK_INCLUDE_DIR}
	)
endif()
