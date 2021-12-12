# openchronos-ng-elf project
#
# Copyright (C) 2016 Benjamin Sølberg <benjamin.soelberg@gmail.com>
# Copyright (C) 2017 Alexandre Pretyman <alexandre.pretyman@gmail.com>
#
# http://github.com/BenjaminSoelberg/openchronos-ng-elf
#
# This file is part of openchronos-ng.
#
# openchronos-ng is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# openchronos-ng is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.2)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/toolchains/toolchain-msp430-gcc-ti.cmake)
endif()

project(openchronos-ng-elf C)

set(openchronos_binary_filename "openchronos.elf")
set(module_config_files
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h
    ${CMAKE_CURRENT_SOURCE_DIR}/modinit.c
)

set(source_files
    ${module_config_files}
    messagebus.c
    openchronos.c
    boot.c
    menu.c

    drivers/lpm.c
    drivers/battery.c
    drivers/vti_as.c
    drivers/infomem.c
    drivers/buzzer.c
    drivers/display.c
    drivers/temperature.c
    drivers/rtca.c
    drivers/rtc_dst.c
    drivers/ports.c
    drivers/dsp.c
    drivers/radio.c
    drivers/ps.c
    drivers/ps_lib.c
    drivers/bmp_ps.c
    drivers/vti_ps.c
    drivers/adc12.c
    drivers/timer.c
    drivers/pmm.c
    drivers/rf1a.c
    drivers/wdt.c

    modules/battery.c
    modules/alarm.c
    modules/music.c
    modules/reset.c
    modules/tide.c
    modules/temperature.c
    modules/clock.c
    modules/hashutils.c
    modules/otp.c
    modules/stopwatch.c
    modules/accelerometer.c
    modules/buzztest.c
    modules/flyback.c
    modules/githash.c
    modules/backlight.c
    modules/altitude.c
)

# use git to determine revision for githash module
find_package(Git)
if (Git_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    OUTPUT_VARIABLE GITHASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
endif()

# revision needs to be upper case for display functions
string(TOUPPER "${GITHASH}" GITHASH)

# fill to get 7 characters
string(CONCAT GITHASH "${GITHASH}" "-------")
string(SUBSTRING ${GITHASH} 0 7 GITHASH)

# pass revision to githash module
set_source_files_properties(modules/githash.c PROPERTIES COMPILE_FLAGS -DGITHASH=\\\"${GITHASH}\\\")

add_executable(${openchronos_binary_filename} ${source_files})
target_include_directories(${openchronos_binary_filename} PRIVATE .)

add_custom_command(TARGET ${openchronos_binary_filename} POST_BUILD
  COMMAND
    ${CMAKE_SIZE} "${openchronos_binary_filename}"
  DEPENDS
    ${openchronos_binary_filename}
  )

find_package(PythonInterp)
if(NOT PYTHONINTERP_FOUND)
  message(WARNING "Python interpreter not found - Python tools disabled")
else()
  # explicit config target
  add_custom_target(config
      COMMAND
        ${PYTHON_EXECUTABLE}
        ${CMAKE_CURRENT_LIST_DIR}/tools/config.py
      COMMAND
        ${PYTHON_EXECUTABLE}
        ${CMAKE_CURRENT_LIST_DIR}/tools/make_modinit.py
       WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
       USES_TERMINAL
  )


  # this will make config.py and make_modinit.py run when trying to compile
  # and there is no config.h and modinit.c
  add_custom_command(
      OUTPUT ${module_config_files}
      COMMAND
        ${PYTHON_EXECUTABLE}
        ${CMAKE_CURRENT_LIST_DIR}/tools/config.py
      COMMAND
        ${PYTHON_EXECUTABLE}
        ${CMAKE_CURRENT_LIST_DIR}/tools/make_modinit.py
       WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )

  # generate ihex with objcopy
  set(openchronos_hex_filename "openchronos.hex")
  add_custom_command(
      OUTPUT
        ${openchronos_hex_filename}
      COMMAND
        ${CMAKE_OBJCOPY} -O ihex
        ${openchronos_binary_filename}
        ${openchronos_hex_filename}
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
      DEPENDS
        ${openchronos_binary_filename}
  )

  # generate ti-txt with srec_cat
  find_program(srecord_executable "srec_cat")
  if(NOT srecord_executable)
    message(WARNING "srec_cat not found")
  else()
    set(openchronos_txt_filename "openchronos.txt")
    add_custom_command(
        OUTPUT
          ${openchronos_txt_filename}
        COMMAND
          ${srecord_executable}
          ${openchronos_hex_filename} -intel
          -Output ${openchronos_txt_filename} -ti_txt
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        DEPENDS
          ${openchronos_hex_filename}
    )
    add_custom_target(
        ti-txt
        DEPENDS
          ${openchronos_txt_filename}
    )
    add_custom_target(
        radio-install
        COMMAND
          ${PYTHON_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/contrib/ChronosTool.py
          rfbsl ${openchronos_txt_filename}
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        DEPENDS
          ${openchronos_txt_filename}
    )
  endif()
endif()

find_program(mspdebug_executable "mspdebug")
if(NOT mspdebug_executable)
  message(WARNING
      "mspdebug not found: targets usb-install and usb-debug disabled"
  )
else()
  add_custom_target(usb-install
      COMMAND
        ${mspdebug_executable} rf2500 "prog ${openchronos_binary_filename}"
      WORKING_DIRECTORY
        ${CMAKE_CURRENT_BINARY_DIR}
      DEPENDS
        ${openchronos_binary_filename}
  )

  add_custom_target(usb-debug
      COMMAND
        ${mspdebug_executable} rf2500 "gdb"
      WORKING_DIRECTORY
        ${CMAKE_CURRENT_BINARY_DIR}
      DEPENDS
        ${openchronos_binary_filename}
  )
endif()

find_package(Doxygen)
if(NOT DOXYGEN_FOUND)
  message(WARNING "Doxygen not found: target doc disabled")
else()
  add_custom_target(
      doc
      COMMAND
        ${DOXYGEN_EXECUTABLE} Doxyfile
      WORKING_DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()
