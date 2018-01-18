#
# Try to find Gecode
# Once done this will define
#
# GECODE_FOUND           - system has GECODE
# GECODE_INCLUDE_DIRS    - the GECODE include directories
# GECODE_LIBRARIES       - Link these to use GECODE
#

# please refer to chapter 2.3.3 in <Modeling and Programming with Gecode>
# to find more about Gecode installation directory. Here we use the default one.
FIND_PATH(GECODE_INCLUDE_DIR gecode/kernel.hh PATHS /usr/local/include/)

FIND_LIBRARY(GECODE_SEARCH_LIBRARY
	NAMES gecodesearch
	PATHS ${GECODE_INCLUDE_DIR}/../lib)

FIND_LIBRARY(GECODE_KERNEL_LIBRARY
	NAMES gecodekernel
	PATHS ${GECODE_INCLUDE_DIR}/../lib)

FIND_LIBRARY(GECODE_INT_LIBRARY
	NAMES gecodeint
	PATHS ${GECODE_INCLUDE_DIR}/../lib)

FIND_LIBRARY(GECODE_SUPPORT_LIBRARY
	NAMES gecodesupport
	PATHS "${GECODE_INCLUDE_DIR}/../lib")

FIND_LIBRARY(GECODE_GIST_LIBRARY
	NAMES gecodegist
	PATHS "${GECODE_INCLUDE_DIR}/../lib")

FIND_LIBRARY(GECODE_DRIVER_LIBRARY
	NAMES gecodedriver
	PATHS "${GECODE_INCLUDE_DIR}/../lib")

FIND_LIBRARY(GECODE_MINIMODEL_LIBRARY
	NAMES gecodeminimodel
	PATHS "${GECODE_INCLUDE_DIR}/../lib")

SET(GECODE_LIBRARY
	${GECODE_SEARCH_LIBRARY}
	${GECODE_KERNEL_LIBRARY}
	${GECODE_INT_LIBRARY}
	${GECODE_SUPPORT_LIBRARY}
	${GECODE_GIST_LIBRARY}
	${GECODE_DRIVER_LIBRARY}
	${GECODE_MINIMODEL_LIBRARY}
	)

#	NAMES gecodeflatzinc gecodedriver gecodegist gecodesearch gecodeminimodel gecodeset gecodefloat gecodeint gecodekernel gecodesupport

if(GECODE_LIBRARY AND GECODE_INCLUDE_DIR)
	message(STATUS "Found GECODE: ${GECODE_INCLUDE_DIR}")
	message(STATUS "Found GECODE: ${GECODE_LIBRARY}")
	set(GECODE_FOUND TRUE)
endif(GECODE_LIBRARY AND GECODE_INCLUDE_DIR)

IF (GECODE_FOUND)
	SET(GECODE_INCLUDE_DIRS ${GECODE_INCLUDE_DIR})
	SET(GECODE_LIBRARIES ${GECODE_LIBRARY})
ELSE (GECODE_FOUND)
	message(WARNING "could NOT find GECODE")
ENDIF (GECODE_FOUND)
