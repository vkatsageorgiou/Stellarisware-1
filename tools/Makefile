#******************************************************************************
#
# Makefile - Rules for building the utilities.
#
# Copyright (c) 2009 Luminary Micro, Inc.  All rights reserved.
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
# This is part of revision 4781 of the Stellaris Firmware Development Package.
#
#******************************************************************************

DIRS=aes_gen_key \
     converter   \
     dfuwrap     \
     eflash      \
     finder      \
     ftrasterize \
     makefsfile  \
     pnmtoc      \
     sflash

all:
	@for i in ${DIRS};                  \
	 do                                 \
	     if [ -d $${i} ];               \
	     then                           \
	         make -C $${i} || exit $$?; \
	     fi;                            \
	 done

clean:
	@for i in ${DIRS};              \
	 do                             \
	     if [ -d $${i} ];           \
	     then                       \
	         make -C $${i} clean;   \
	     fi;                        \
	 done;                          \
	 rm -f ${wildcard *~} __dummy__
