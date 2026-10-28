cmake_minimum_required(VERSION 3.20)
project(SuperMarioWorldRecomp C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
# static C runtime: smw.exe only needs SDL2.dll next to it
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

option(SMW_BUILD_TESTS "Also build the headless runner and the CPU/PPU/APU test programs" OFF)

# ---- your ROM (not included) ----------------------------------------------
# The ROM is read at configure time to statically recompile the 65816 code;
# the game also loads it at run time for graphics, levels and music.
set(SMW_ROM "" CACHE FILEPATH "Path to your Super Mario World (USA) ROM (.sfc/.smc)")
if(NOT SMW_ROM)
  foreach(name smw.sfc smw.smc)
    if(EXISTS "${CMAKE_SOURCE_DIR}/${name}")
      set(SMW_ROM "${CMAKE_SOURCE_DIR}/${name}" CACHE FILEPATH "Path to your Super Mario World (USA) ROM (.sfc/.smc)" FORCE)
      break()
    endif()
  endforeach()
endif()
if(NOT SMW_ROM OR NOT EXISTS "${SMW_ROM}")
  message(FATAL_ERROR
    "Super Mario World ROM not found.\n"
    "Copy your own Super Mario World (USA) ROM to ${CMAKE_SOURCE_DIR}/smw.sfc "
    "or configure with -DSMW_ROM=path/to/your/rom.sfc")
endif()
get_filename_component(SMW_ROM_ABS "${SMW_ROM}" ABSOLUTE)

# ---- SDL2 -------------------------------------------------------------------
# Uses an installed SDL2 (SDL2_DIR / CMAKE_PREFIX_PATH / system package); with
# MSVC and none installed, the official development package is downloaded.
set(SMW_SDL2_VERSION 2.32.10)
find_package(SDL2 CONFIG QUIET)
if(NOT SDL2_FOUND AND MSVC)
  set(_sdl_root "${CMAKE_BINARY_DIR}/_deps/SDL2-${SMW_SDL2_VERSION}")
  if(NOT EXISTS "${_sdl_root}/cmake/sdl2-config.cmake")
    set(_sdl_zip "${CMAKE_BINARY_DIR}/_deps/SDL2-devel-${SMW_SDL2_VERSION}-VC.zip")
    message(STATUS "Downloading SDL2 ${SMW_SDL2_VERSION} (VC development package)")
    file(DOWNLOAD
      "https://github.com/libsdl-org/SDL/releases/download/release-${SMW_SDL2_VERSION}/SDL2-devel-${SMW_SDL2_VERSION}-VC.zip"
      "${_sdl_zip}"
      EXPECTED_HASH SHA256=af347939395a58b365846aaea27391e69f9ec9d4dd650d6ac40802159b418a6e
      TLS_VERIFY ON)
    file(ARCHIVE_EXTRACT INPUT "${_sdl_zip}" DESTINATION "${CMAKE_BINARY_DIR}/_deps")
  endif()
  find_package(SDL2 CONFIG REQUIRED PATHS "${_sdl_root}" NO_DEFAULT_PATH)
endif()
if(NOT SDL2_FOUND)
  message(FATAL_ERROR "SDL2 not found: install the SDL2 development package (e.g. libsdl2-dev) or set SDL2_DIR")
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter)

# ---- static recompilation (65816 -> C) ------------------------------------
# Output goes to the build tree: it is derived from the ROM and must not be
# committed or redistributed.
set(GEN_DIR "${CMAKE_BINARY_DIR}/generated")
set(RECOMP_SEEDS "${CMAKE_SOURCE_DIR}/tools/recomp/seeds.txt")
file(GLOB RECOMP_TOOLS "${CMAKE_SOURCE_DIR}/tools/recomp/*.py")
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tools/recomp/recomp.py" "${SMW_ROM_ABS}" "${GEN_DIR}" "${RECOMP_SEEDS}"
  RESULT_VARIABLE RECOMP_RESULT)
if(NOT RECOMP_RESULT EQUAL 0)
  message(FATAL_ERROR "Static recompilation failed (is SMW_ROM your Super Mario World (USA) ROM?)")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${RECOMP_TOOLS} "${RECOMP_SEEDS}" "${SMW_ROM_ABS}")
file(GLOB GENERATED_SOURCES "${GEN_DIR}/*.c")
# lets the game find the ROM it was built from without any configuration
file(CONFIGURE OUTPUT "${CMAKE_BINARY_DIR}/config/smw_build_config.h"
  CONTENT "#define SMW_BUILD_ROM_PATH \"@SMW_ROM_ABS@\"\n" @ONLY)

# ---- compiler settings ------------------------------------------------------
set(CORE_SOURCES src/snes.c src/ppu.c src/apu.c src/apu_spc700.c src/apu_dsp.c)
set(APU_SOURCES src/apu.c src/apu_spc700.c src/apu_dsp.c)
if(MSVC)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
  add_compile_options(/wd4244 /wd4146 /wd4090 /wd4996)
  set(SMW_WARN /W3)
  set(SMW_GEN_WARN /W1)
else()
  set(SMW_WARN -Wall -Wno-unused-function -Wno-unused-variable)
  set(SMW_GEN_WARN -w)
endif()
set_source_files_properties(${GENERATED_SOURCES} PROPERTIES COMPILE_OPTIONS "${SMW_GEN_WARN}")

# ---- game -------------------------------------------------------------------
add_library(smw_core OBJECT ${CORE_SOURCES} ${GENERATED_SOURCES})
target_include_directories(smw_core PUBLIC src)
set_source_files_properties(${CORE_SOURCES} src/main.c PROPERTIES COMPILE_OPTIONS "${SMW_WARN}")

add_executable(smw src/main.c)
target_include_directories(smw PRIVATE "${CMAKE_BINARY_DIR}/config")
target_link_libraries(smw PRIVATE smw_core SDL2::SDL2)
if(UNIX)
  target_link_libraries(smw PRIVATE m)
endif()
if(MSVC)
  # console window only in debug builds
  set_target_properties(smw PROPERTIES WIN32_EXECUTABLE $<CONFIG:Release>)
  target_link_options(smw PRIVATE $<$<CONFIG:Release>:/ENTRY:mainCRTStartup>)
endif()
if(WIN32)
  add_custom_command(TARGET smw POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:SDL2::SDL2>" "$<TARGET_FILE_DIR:smw>")
endif()

# ---- tests and tools (optional) -----------------------------------------------
if(SMW_BUILD_TESTS)
  set(TEST_BIN "${CMAKE_BINARY_DIR}/tests")
  add_executable(smw_headless tests/headless/headless.c)
  target_link_libraries(smw_headless PRIVATE smw_core)

  add_executable(cpu_sst tests/cpu_sst/sst.c "${GEN_DIR}/interp.c")
  target_compile_definitions(cpu_sst PRIVATE SST_FLAT_BUS)
  target_include_directories(cpu_sst PRIVATE src)

  add_executable(ppu_test tests/ppu_test/ppu_test.c src/ppu.c)
  add_executable(spc700_test tests/apu_test/spc700_test.c ${APU_SOURCES})
  add_executable(dsp_test tests/apu_test/dsp_test.c ${APU_SOURCES})
  add_executable(apu_unit_test tests/apu_test/apu_unit_test.c ${APU_SOURCES})

  set(_tests smw_headless cpu_sst ppu_test spc700_test dsp_test apu_unit_test)
  set_target_properties(${_tests} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${TEST_BIN}")
  if(UNIX)
    foreach(t ${_tests})
      target_link_libraries(${t} PRIVATE m)
    endforeach()
  endif()

  # self-contained tests (no downloaded data needed): ctest --test-dir build
  enable_testing()
  file(MAKE_DIRECTORY "${TEST_BIN}/out")
  add_test(NAME apu_unit COMMAND apu_unit_test)
  add_test(NAME apu_dsp COMMAND dsp_test "${TEST_BIN}/out")
  add_test(NAME ppu COMMAND ppu_test "${TEST_BIN}/out")
endif()
