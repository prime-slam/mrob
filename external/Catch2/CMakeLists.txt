find_package(Catch2 QUIET)

if (NOT Catch2_FOUND)
    message("Installing Catch2 library")
    include(FetchContent)

    FetchContent_Declare(
            catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG v2.x
    )

    FetchContent_MakeAvailable(catch2)
endif()