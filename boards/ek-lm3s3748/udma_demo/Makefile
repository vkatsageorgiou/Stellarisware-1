#******************************************************************************
#
# Makefile - Rules for building the uDMA example.
#
# Copyright (c) 2008-2009 Luminary Micro, Inc.  All rights reserved.
# Software License Agreement
# 
# Luminary Micro, Inc. (LMI) is supplying this software for use solely and
# exclusively on LMI's microcontroller products.
# 
# The software is owned by LMI and/or its suppliers, and is protected under
# applicable copyright laws.  All rights are reserved.  You may not combine
# this software with "viral" open-source software in order to form a larger
# program.  Any use in violation of the foregoing restrictions may subject
# the user to criminal sanctions under applicable laws, as well as to civil
# liability for the breach of the terms and conditions of this license.
# 
# THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
# OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
# LMI SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
# CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
# 
# This is part of revision 4781 of the EK-LM3S3748 Firmware Package.
#
#******************************************************************************

#
# Defines the part type that this project uses.
#
PART=LM3S3748

#
# The base directory for DriverLib.
#
ROOT=../../..

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find source files that do not live in this directory.
#
VPATH=../drivers
VPATH+=../../../utils

#
# Where to find header files that do not live in the source directory.
#
IPATH=..
IPATH+=../../..

#
# The default rule, which causes the uDMA example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/udma_demo.axf

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir ${COMPILER}

#
# Rules for building the uDMA example.
#
${COMPILER}/udma_demo.axf: ${COMPILER}/cpu_usage.o
${COMPILER}/udma_demo.axf: ${COMPILER}/formike128x128x16.o
${COMPILER}/udma_demo.axf: ${COMPILER}/startup_${COMPILER}.o
${COMPILER}/udma_demo.axf: ${COMPILER}/udma_demo.o
${COMPILER}/udma_demo.axf: ${COMPILER}/ustdlib.o
${COMPILER}/udma_demo.axf: ${ROOT}/grlib/${COMPILER}/libgr.a
${COMPILER}/udma_demo.axf: ${ROOT}/driverlib/${COMPILER}/libdriver.a
ifeq (${COMPILER}, gcc)
${COMPILER}/udma_demo.axf: udma_demo.ld
endif
SCATTERgcc_udma_demo=udma_demo.ld
ifeq (${COMPILER}, sourcerygxx)
${COMPILER}/udma_demo.axf: udma_demo_sourcerygxx.ld
endif
SCATTERsourcerygxx_udma_demo=lm3s3748-rom.ld -T udma_demo_sourcerygxx.ld
ENTRY_udma_demo=ResetISR
CFLAGSgcc=-DTARGET_IS_DUSTDEVIL_RA0
CFLAGSsourcerygxx=-DTARGET_IS_DUSTDEVIL_RA0

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
