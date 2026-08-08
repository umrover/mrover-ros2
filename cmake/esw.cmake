# esw can messaging library (for dbc library)
# NOTE: this needs to be upreved for every formal release of the esw can messaging library (lib/dbc/)

# ensure cmake is new enough for fetch content
if (${CMAKE_VERSION} VERSION_LESS "3.14")
    message(FATAL_ERROR "esw mrover_can fetch requires CMake >= 3.14 for FetchContent_MakeAvailable (found ${CMAKE_VERSION})")
endif ()
include(FetchContent)

# TODO: point back at the real ESW repo once tooling access to umrover/mrover-esw is approved.
set(MROVER_CAN_GIT_REPOSITORY "https://github.com/edurso/cmake-lib-test.git")
set(MROVER_CAN_VERSION "v0.0.1")

# persistent local clone cache: survives clean, is gitignored,
# and used as an offline fallback if the repo is unreachable
set(MROVER_CAN_CACHE_DIR "${CMAKE_SOURCE_DIR}/.cache/esw")
set(MROVER_CAN_SOURCE_DIR "${MROVER_CAN_CACHE_DIR}/mrover_can-src")
file(MAKE_DIRECTORY ${MROVER_CAN_CACHE_DIR})

# get tags
execute_process(
    COMMAND git ls-remote --tags --sort=-v:refname ${MROVER_CAN_GIT_REPOSITORY}
    OUTPUT_VARIABLE MROVER_CAN_REMOTE_TAGS
    RESULT_VARIABLE MROVER_CAN_LS_REMOTE_RESULT
    ERROR_VARIABLE MROVER_CAN_LS_REMOTE_ERROR
    TIMEOUT 10
)

if (NOT MROVER_CAN_LS_REMOTE_RESULT EQUAL 0)
    if (EXISTS "${MROVER_CAN_SOURCE_DIR}/.git")
        message(WARNING
            "failed to reach esw mrover_can online (${MROVER_CAN_GIT_REPOSITORY}); "
            "${MROVER_CAN_LS_REMOTE_ERROR}"
            "build is assumed to be offline, "
            "falling back to cached mrover_can at ${MROVER_CAN_SOURCE_DIR}; "
            "mrover_can may be stale"
        )
        set(FETCHCONTENT_UPDATES_DISCONNECTED_MROVER_CAN ON)
    else ()
        message(FATAL_ERROR
            "failed to reach esw mrover_can online (${MROVER_CAN_GIT_REPOSITORY}); "
            "${MROVER_CAN_LS_REMOTE_ERROR}"
            "build is assumed to be offline - "
            "no cached mrover_can found, cannot proceed with build; "
            "either connect to a network, or clone a copy of the repo at tag ${MROVER_CAN_VERSION}"
        )
    endif ()
else ()
    string(REPLACE "\n" ";" MROVER_CAN_TAG_LINES "${MROVER_CAN_REMOTE_TAGS}")
    set(MROVER_CAN_TAG_FOUND FALSE)
    set(MROVER_CAN_LATEST_TAG "")
    foreach (MROVER_CAN_TAG_LINE IN LISTS MROVER_CAN_TAG_LINES)
        if (MROVER_CAN_TAG_LINE MATCHES "refs/tags/([^\t]+)$")
            set(MROVER_CAN_TAG_NAME "${CMAKE_MATCH_1}")
            string(REGEX REPLACE "\\^\\{\\}$" "" MROVER_CAN_TAG_NAME "${MROVER_CAN_TAG_NAME}")
            if (NOT MROVER_CAN_LATEST_TAG)
                set(MROVER_CAN_LATEST_TAG "${MROVER_CAN_TAG_NAME}")
            endif ()
            if (MROVER_CAN_TAG_NAME STREQUAL MROVER_CAN_VERSION)
                set(MROVER_CAN_TAG_FOUND TRUE)
            endif ()
        endif ()
    endforeach ()

    if (NOT MROVER_CAN_TAG_FOUND)
        message(FATAL_ERROR "selected mrover_can tag (${MROVER_CAN_VERSION}) not found, select valid tag")
    endif ()

    if (NOT MROVER_CAN_LATEST_TAG STREQUAL MROVER_CAN_VERSION)
        message(WARNING "a newer mrover_can release is available (${MROVER_CAN_LATEST_TAG}, current ${MROVER_CAN_VERSION})")
    endif ()
endif ()

set(_MROVER_CAN_SAVED_BASE_DIR ${FETCHCONTENT_BASE_DIR})
set(FETCHCONTENT_BASE_DIR ${MROVER_CAN_CACHE_DIR} CACHE PATH "" FORCE)

FetchContent_Declare(
    mrover_can
    GIT_REPOSITORY ${MROVER_CAN_GIT_REPOSITORY}
    GIT_TAG ${MROVER_CAN_VERSION}
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(mrover_can)

set(FETCHCONTENT_BASE_DIR ${_MROVER_CAN_SAVED_BASE_DIR} CACHE PATH "" FORCE)
