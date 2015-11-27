#******************************************************************************
#
# Makefile - Rules for building the font rasterization utility.
#
# Copyright (c) 2007-2009 Luminary Micro, Inc.  All rights reserved.
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

#
# The name of this application.
#
APP:=ftrasterize

#
# The object files that comprise this application.
#
OBJS:=ftrasterize.o                           \
      freetype-2.3.5/objs/.libs/libfreetype.a

#
# Include the generic rules.
#
include ../toolsdefs

#
# Add a -I for freetype to the CFLAGS.
#
CFLAGS:=${CFLAGS} -I freetype-2.3.5/include

#
# Fetch the tarball that contains FreeType.
#
freetype-2.3.5.tar.bz2:
	@URL=http://download.savannah.gnu.org/releases/freetype; \
	 URL=$${URL}/freetype-2.3.5.tar.bz2;                     \
	 if [ x${VERBOSE} = x ];                                 \
	 then                                                    \
	     echo "  WGET  `basename $${URL}`";                  \
	 else                                                    \
	     echo wget $${URL};                                  \
	 fi;                                                     \
	 wget $${URL}

#
# Unpack and build FreeType.
#
freetype-2.3.5/objs/.libs/libfreetype.a: freetype-2.3.5.tar.bz2
	@if [ ! -d freetype-2.3.5 ];                                             \
	 then                                                                    \
	     tar -xjf freetype-2.3.5.tar.bz2;                                    \
	 fi;                                                                     \
	 cd freetype-2.3.5;                                                      \
	 if [ ! -f builds/unix/ftconfig.h ];                                     \
	 then                                                                    \
	     ./configure CFLAGS="${CFLAGS}" LDFLAGS="${LDFLAGS}" --with-zlib=no; \
	 fi;                                                                     \
	 make

#
# Fetch the tarball that contains the Computer Modern fonts.
#
cmps-pc.zip:
	@URL=ftp://ftp.ams.org/pub/tex/psfonts/cm/cmps-pc.zip; \
	 if [ x${VERBOSE} = x ];                               \
	 then                                                  \
	     echo "  WGET  `basename $${URL}`";                \
	 else                                                  \
	     echo "wget $${URL}";                              \
	 fi;                                                   \
	 wget $${URL}

#
# Unpack the Computer Modern fonts.
#
cmpsfont: cmps-pc.zip
	@unzip -q cmps-pc.zip

#
# The rule to generate the Computer Modern compressed bitmap fonts.
#
fonts: ftrasterize${EXT} cmpsfont
	@for i in 12 14 16 18 20 22 24 26 28 30 32 34 36 38 40 42 44 46 48;   \
	 do                                                                   \
	     echo "  GEN   $${i} point";                                      \
	     ./ftrasterize -f cm -s $${i} cmpsfont/fonts/cmr17.pfb;           \
	     ./ftrasterize -f cm -s $${i} -b cmpsfont/fonts/cmb10.pfb;        \
	     ./ftrasterize -f cm -s $${i} -i cmpsfont/fonts/cmti12.pfb;       \
	     ./ftrasterize -f cmss -s $${i} cmpsfont/fonts/cmss17.pfb;        \
	     ./ftrasterize -f cmss -s $${i} -b cmpsfont/fonts/cmssbx10.pfb;   \
	     ./ftrasterize -f cmss -s $${i} -i cmpsfont/fonts/cmssi17.pfb;    \
	     ./ftrasterize -f cmsc -s $${i} cmpsfont/fonts/cmcsc10.pfb;       \
	 done

#
# Clean out additional build products.
#
clean::
	@rm -rf freetype-2.3.5 cmpsfont ${wildcard font*.c}

#
# A dependency to ensure that freetype is built before the ftrasterize
# application.
#
ftrasterize.o: freetype-2.3.5/objs/.libs/libfreetype.a
