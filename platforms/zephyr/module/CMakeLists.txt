############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

if(NOT CONFIG_PX4)
	return()
endif()

set(px4_source_dir ${ZEPHYR_CURRENT_MODULE_DIR})

if(NOT CONFIG_PX4_BOARD_CONFIG STREQUAL "")
	set(px4_config ${CONFIG_PX4_BOARD_CONFIG})
elseif(EXISTS ${px4_source_dir}/boards/zephyr/${BOARD}/default.px4board)
	set(px4_config zephyr_${BOARD})
else()
	set(px4_config zephyr_generic)
endif()

set(CONFIG ${px4_config} CACHE STRING "PX4 board configuration" FORCE)
unset(PX4_CONFIG_FILE CACHE)

set(PX4_GIT_SUBMODULES
	src/lib/events/libevents
	src/lib/heatshrink/heatshrink
	src/modules/mavlink/mavlink
)

zephyr_include_directories(include)

add_subdirectory(${px4_source_dir} ${CMAKE_CURRENT_BINARY_DIR}/px4)

target_link_libraries(app PRIVATE px4)

# compile settings for application code using the PX4 API
get_directory_property(px4_include_dirs DIRECTORY ${px4_source_dir} INCLUDE_DIRECTORIES)
get_directory_property(px4_compile_definitions DIRECTORY ${px4_source_dir} COMPILE_DEFINITIONS)
add_library(px4_interface INTERFACE)
target_include_directories(px4_interface INTERFACE ${px4_include_dirs})
target_compile_definitions(px4_interface INTERFACE ${px4_compile_definitions})
target_compile_options(px4_interface INTERFACE
	$<$<COMPILE_LANGUAGE:C>:-std=gnu11>
	$<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>
	-include visibility.h
)
add_dependencies(px4_interface uorb_headers)

zephyr_library_named(px4_tty)
zephyr_library_sources(${px4_source_dir}/platforms/zephyr/src/px4/common/px4_tty.c)

if(CONFIG_PX4_SERIAL)
	# open() opens the serial ports, see px4_tty.c
	zephyr_ld_options(-Wl,--wrap=open)
endif()

if(CONFIG_PX4_FLEXSPI_PREFETCH_OFF)
	zephyr_library_named(px4_flexspi)
	zephyr_library_sources(${px4_source_dir}/platforms/zephyr/src/px4/common/px4_flexspi.c)
endif()

if(CONFIG_PX4_PC_PROFILE)
	zephyr_library_named(px4_pc_profile)
	zephyr_library_sources(${px4_source_dir}/platforms/zephyr/src/px4/common/px4_pc_profile.c)
endif()

zephyr_library_named(px4_pthread)
# px4_pthread_attr_destroy() keeps the stacks that threads run on, see px4_pthread.c
zephyr_library_sources(${px4_source_dir}/platforms/zephyr/src/px4/common/px4_pthread.c)

if(CONFIG_PX4_DSHOT_PULSE_IO)
	zephyr_library_named(px4_dshot_pulse_io)
	zephyr_library_sources(${px4_source_dir}/platforms/zephyr/src/px4/common/dshot/dshot_pulse_io.c)
endif()

if(CONFIG_PX4_CAN)
	zephyr_library_named(px4_can)
	zephyr_library_sources(${px4_source_dir}/platforms/zephyr/src/px4/common/px4_can.c)
endif()

if(CONFIG_PX4_COMPONENT_METADATA AND TARGET modules__mavlink)
	# the files of /etc/extras in the ROMFS of NuttX, see cmake/kconfig.cmake
	set(romfs_dir ${CMAKE_CURRENT_BINARY_DIR}/romfs)
	set(romfs_data_string)
	set(romfs_entries_string "\t{\"extras\", NULL, 0},\n")
	set(romfs_inc_files)

	foreach(file ${config_romfs_extra_files})
		get_filename_component(name ${file} NAME)
		string(MAKE_C_IDENTIFIER ${name} identifier)
		generate_inc_file(${file} ${romfs_dir}/${name}.inc)
		list(APPEND romfs_inc_files ${romfs_dir}/${name}.inc)
		string(APPEND romfs_data_string "static const uint8_t ${identifier}[] = {\n#include \"${name}.inc\"\n};\n\n")
		string(APPEND romfs_entries_string "\t{\"extras/${name}\", ${identifier}, sizeof(${identifier})},\n")
	endforeach()

	configure_file(px4_romfs_files.c.in ${romfs_dir}/px4_romfs_files.c)

	zephyr_library_named(px4_romfs)
	zephyr_library_sources(
		${px4_source_dir}/platforms/zephyr/src/px4/common/px4_romfs.c
		${romfs_dir}/px4_romfs_files.c
		${romfs_inc_files}
	)
	add_dependencies(px4_romfs ${config_romfs_extra_dependencies})
endif()

if(CONFIG_PX4_ITCM)
	# the code of the ranked list that fits in the ITCM of the board
	dt_chosen(itcm_node PROPERTY "zephyr,itcm")
	dt_reg_size(itcm_size PATH ${itcm_node})
	math(EXPR itcm_budget "(${itcm_size} - ${CONFIG_PX4_ITCM_RESERVE}) * ${CONFIG_PX4_ITCM_FILL} / 100")
	set(itcm_ranked ${CMAKE_CURRENT_LIST_DIR}/itcm_functions.ld)
	set(itcm_fitted ${CMAKE_CURRENT_BINARY_DIR}/itcm_functions.ld)
	execute_process(
		COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/itcm_fit.py ${itcm_ranked} ${itcm_budget} ${itcm_fitted}
		OUTPUT_VARIABLE itcm_fit_output
		OUTPUT_STRIP_TRAILING_WHITESPACE
		COMMAND_ERROR_IS_FATAL ANY
	)
	message(STATUS "PX4 ITCM: ${itcm_fit_output}")
	set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${itcm_ranked} ${CMAKE_CURRENT_LIST_DIR}/itcm_fit.py)
	zephyr_linker_sources(ITCM_SECTION ${itcm_fitted})
endif()

if(CONFIG_PX4_ITCM_LIBRARIES)
	zephyr_linker_sources(ITCM_SECTION ${CMAKE_CURRENT_LIST_DIR}/itcm.ld)

	# the kernel, context switches and interrupts, but not the startup code, which runs before
	# the ITCM is loaded
	foreach(lib kernel arch__arm__core__mpu drivers__timer)
		zephyr_code_relocate(LIBRARY ${lib} LOCATION ITCM_TEXT NOKEEP)
	endforeach()

	zephyr_code_relocate(FILES
		${ZEPHYR_BASE}/arch/arm/core/cortex_m/exc_exit.c
		${ZEPHYR_BASE}/arch/arm/core/cortex_m/isr_wrapper.c
		${ZEPHYR_BASE}/arch/arm/core/cortex_m/swap_helper.S
		LOCATION ITCM_TEXT NOKEEP
	)

	# the libraries that run at the rate of the IMU, when they are built
	foreach(lib
		uORB cdev work_queue px4_work_queue perf
		modules__sensors vehicle_imu vehicle_angular_velocity sensor_calibration data_validator
		drivers_accelerometer drivers_gyroscope conversion
		modules__mc_rate_control modules__mc_att_control AttitudeControl
		modules__control_allocator mixer_module output_predictor
		lat_lon_alt modules__simulation__simulator_sih
	)
		if(TARGET ${lib})
			zephyr_code_relocate(LIBRARY ${lib} LOCATION ITCM_TEXT NOKEEP)
		endif()
	endforeach()

	# the USB device stack, which sends MAVLink to a ground station on USB: each message passes
	# through the CDC ACM class, the buffers and the USB controller driver
	if(CONFIG_USBD_CDC_ACM_CLASS)
		set(usb_files
			${ZEPHYR_BASE}/subsys/usb/device_next/class/usbd_cdc_acm.c
			${ZEPHYR_BASE}/subsys/usb/device_next/usbd_class.c
			${ZEPHYR_BASE}/subsys/usb/device_next/usbd_core.c
			${ZEPHYR_BASE}/drivers/usb/udc/udc_common.c
			${ZEPHYR_BASE}/lib/net_buf/buf.c
			${ZEPHYR_BASE}/lib/net_buf/buf_simple.c
		)

		if(CONFIG_UDC_NXP_EHCI)
			list(APPEND usb_files
				${ZEPHYR_BASE}/drivers/usb/udc/udc_mcux_ehci.c
				${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk-ng/middleware/usb/device/usb_device_ehci.c
				${ZEPHYR_HAL_NXP_MODULE_DIR}/mcux/mcux-sdk-ng/components/osa/fsl_os_abstraction_zephyr.c
			)
		endif()

		zephyr_code_relocate(FILES ${usb_files} LOCATION ITCM_TEXT NOKEEP)
	endif()
endif()

if(CONFIG_PX4_SHELL)
	# the PX4 commands as commands of the Zephyr shell, as in nsh, besides "px4 <command>"
	get_property(px4_builtins GLOBAL PROPERTY PX4_ZEPHYR_BUILTINS)
	set(shell_commands_string)

	foreach(builtin ${px4_builtins})
		string(APPEND shell_commands_string
			"SHELL_CMD_ARG_REGISTER(${builtin}, NULL, NULL, px4_shell_run_command, 1, SHELL_OPT_ARG_MAX);\n")
	endforeach()

	configure_file(px4_shell_commands.c.in ${CMAKE_CURRENT_BINARY_DIR}/px4_shell_commands.c)

	zephyr_library_named(px4_shell)
	zephyr_library_sources(
		${px4_source_dir}/platforms/zephyr/src/px4/common/px4_shell.c
		${CMAKE_CURRENT_BINARY_DIR}/px4_shell_commands.c
	)
endif()

if(CONFIG_PX4_MAVLINK_SHELL AND TARGET modules__mavlink)
	zephyr_library_named(px4_mavlink_shell)
	zephyr_library_sources(${px4_source_dir}/platforms/zephyr/src/px4/common/px4_mavlink_shell.c)
endif()
