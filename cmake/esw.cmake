# esw can messaging library (for dbc library)
# NOTE: this needs to be upreved for every formal release of the esw can messaging library (lib/dbc/)

# ensure cmake is new enough for fetch content
if (${CMAKE_VERSION} VERSION_LESS "3.14")
    message(FATAL_ERROR "esw mrover_can fetch requires CMake >= 3.14 for FetchContent_MakeAvailable (found ${CMAKE_VERSION})")
endif ()
include(FetchContent)

set(MROVER_CAN_GIT_REPOSITORY "https://github.com/umrover/mrover-esw.git")
set(MROVER_CAN_VERSION "v0.1.0")

# persistent local cache: survives a clean build, is gitignored,
# and used as an offline fallback if the repo is unreachable
set(MROVER_CAN_CACHE_DIR "${CMAKE_SOURCE_DIR}/.cache/esw")

# target release asset
string(REGEX REPLACE "\\.git$" "" MROVER_CAN_REPO_URL "${MROVER_CAN_GIT_REPOSITORY}")
set(MROVER_CAN_RELEASE_URL "${MROVER_CAN_REPO_URL}/releases/download/${MROVER_CAN_VERSION}/mrover_can.tar.gz")
set(MROVER_CAN_ARCHIVE "${MROVER_CAN_CACHE_DIR}/mrover_can-${MROVER_CAN_VERSION}.tar.gz")

file(MAKE_DIRECTORY ${MROVER_CAN_CACHE_DIR})

# get tags
execute_process(
    COMMAND git ls-remote --tags --sort=-v:refname ${MROVER_CAN_GIT_REPOSITORY}
    OUTPUT_VARIABLE MROVER_CAN_REMOTE_TAGS
    RESULT_VARIABLE MROVER_CAN_LS_REMOTE_RESULT
    ERROR_VARIABLE MROVER_CAN_LS_REMOTE_ERROR
    TIMEOUT 10
)

# if no tags returned, assume offline
if (NOT MROVER_CAN_LS_REMOTE_RESULT EQUAL 0)
    # if no tags returned, assume offline
    if (EXISTS "${MROVER_CAN_ARCHIVE}")
        # warn if archive fallback needed, cannot check version
        message(WARNING
            "failed to reach esw mrover_can online (${MROVER_CAN_GIT_REPOSITORY}); "
            "${MROVER_CAN_LS_REMOTE_ERROR}"
            "build is assumed to be offline, "
            "falling back to cached mrover_can archive at ${MROVER_CAN_ARCHIVE}; "
            "mrover_can may be stale"
        )
    else ()
        # hard error if offline and no mrover_can. can disable building ESW in CMakeLists.txt
        message(FATAL_ERROR
            "failed to reach esw mrover_can online (${MROVER_CAN_GIT_REPOSITORY}); "
            "${MROVER_CAN_LS_REMOTE_ERROR}"
            "build is assumed to be offline - "
            "no cached mrover_can found, cannot proceed with build; "
            "either connect to a network, or place a valid mrover_can.tar.gz for "
            "tag ${MROVER_CAN_VERSION} at ${MROVER_CAN_ARCHIVE}"
        )
    endif ()
else ()
    # if online, parse latest tag
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
        # error if tag is not available (release deleted, incorrect semver)
        message(FATAL_ERROR "selected mrover_can tag (${MROVER_CAN_VERSION}) not found, select valid tag")
    endif ()

    if (NOT MROVER_CAN_LATEST_TAG STREQUAL MROVER_CAN_VERSION)
        # warn that selected version uses outdated release
        message(WARNING "a newer mrover_can release is available (${MROVER_CAN_LATEST_TAG}, current ${MROVER_CAN_VERSION})")
    endif ()
endif ()

if (EXISTS "${MROVER_CAN_ARCHIVE}")
    # avoid downloading if latest version is already cached, can be cleared with clean.sh --esw
    message(STATUS "using cached esw mrover_can archive at ${MROVER_CAN_ARCHIVE}")
else ()
    # download if cache does not exist, save to cache so fetch only uses cache
    message(STATUS "downloading esw mrover_can from ${MROVER_CAN_RELEASE_URL}")
    file(DOWNLOAD ${MROVER_CAN_RELEASE_URL} ${MROVER_CAN_ARCHIVE}
        STATUS MROVER_CAN_DOWNLOAD_STATUS
        TLS_VERIFY ON
        TIMEOUT 30
    )
    list(GET MROVER_CAN_DOWNLOAD_STATUS 0 MROVER_CAN_DOWNLOAD_CODE)
    list(GET MROVER_CAN_DOWNLOAD_STATUS 1 MROVER_CAN_DOWNLOAD_MSG)
    if (NOT MROVER_CAN_DOWNLOAD_CODE EQUAL 0)
        file(REMOVE ${MROVER_CAN_ARCHIVE})
        message(FATAL_ERROR
            "failed to download esw mrover_can release asset from ${MROVER_CAN_RELEASE_URL}: "
            "${MROVER_CAN_DOWNLOAD_MSG}; "
            "the repo/tag exist but no mrover_can.tar.gz asset could be fetched for release ${MROVER_CAN_VERSION}"
        )
    endif ()
endif ()

# fetch from cache (downloaded above)
FetchContent_Declare(
    mrover_can
    URL "file://${MROVER_CAN_ARCHIVE}"
)
FetchContent_MakeAvailable(mrover_can)

# log version for sanity
message(STATUS "esw mrover_can version: ${MROVER_CAN_VERSION}")
