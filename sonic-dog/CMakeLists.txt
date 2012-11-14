set( SUBSYS_NAME sonic_dog )
set( SUBSYS_DESC "Audio library for guide dog" )
set( LIB_NAME gd_${SUBSYS_NAME} )

find_path( OPENAL_INCLUDE_DIR
	NAMES AL/al.h AL/alc.h AL/alut.h
	PATHS /usr/include /usr/local/include
)

find_library( OPENAL_LIB
	NAMES openal
	PATH /usr/lib /usr/lib64
)

find_library( ALUT_LIB
	NAMES alut
	PATHS /usr/lib /usr/lib64
)

set( srcs
		 sonic_dog.cpp
	 )

set( incs
		 sonic_dog.h
	 )

PCL_ADD_LIBRARY( ${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} )
