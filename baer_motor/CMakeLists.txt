cmake_minimum_required(VERSION 3.15)

project(baer_motor LANGUAGES C CXX ASM)

find_bsp(ID com.sysprogs.project_importers.stm32.cubemx SOURCE_PROJECT baer_motor.ioc)

add_bsp_based_executable(
	NAME baer_motor
	GENERATE_BIN
	GENERATE_MAP
	SOURCES
		${BSP_ROOT}/lan9252/lan9252.c
		${BSP_ROOT}/lan9252/lan9252drv.c
		${BSP_ROOT}/lan9252/lan9252.h
		${BSP_ROOT}/lan9252/lan9252drv.h)

