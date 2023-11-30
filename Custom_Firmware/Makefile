#******************************************************************************
#
# Makefile - Rules for building the libraries, examples and docs.
#
# Copyright (c) 2023, Ambiq Micro, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# Third party software included in this distribution is subject to the
# additional license terms as defined in the /docs/licenses directory.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# This is part of revision release_sdk_4_4_1-7498c7b770 of the AmbiqSuite Development Package.
#
#******************************************************************************

#******************************************************************************
#
# Target directories.
#
#******************************************************************************

SUBDIRS := $(filter-out tools/ third_party/,$(dir $(wildcard */Makefile)))

HAL := $(dir $(wildcard mcu/*/hal/Makefile))
BSP := $(dir $(wildcard boards/*/bsp/Makefile))
EXAMDIRS := $(dir $(wildcard boards/[!c]*/examples/Makefile))

#******************************************************************************
#
# Build everything.
#
#******************************************************************************
# Use Foreach for job control load average limits to work.
all subdirs: projects
	$(foreach subdir,$(SUBDIRS),$(MAKE) -C $(subdir);)
clean: $(SUBDIRS)
$(SUBDIRS):
	$(MAKE) -C $@ $(MAKECMDGOALS)

#******************************************************************************
#
# Build projects for all boards.
#
#******************************************************************************
boards:
	$(MAKE) -C boards


#******************************************************************************
#
# Build examples (fast with job control).
#
#******************************************************************************
examdirs: $(EXAMDIRS)
examples:
	@$(MAKE) examdirs
$(EXAMDIRS):
	$(MAKE) -C $@ 

#******************************************************************************
#
# Build all BSP libraries.
#
#******************************************************************************
bsp: hal $(BSP)
$(BSP):
	$(MAKE) -C $@

#******************************************************************************
#
# Build the HAL.
#
#******************************************************************************
hal: $(HAL)
$(HAL):
	$(MAKE) -C $@

#******************************************************************************
#
# Build the doxygen documentation.
#
#******************************************************************************
doxygen doxy:
	$(MAKE) -C doxygen quick
doxy-clean:
	$(MAKE) -C doxygen clean


#******************************************************************************
#
# Remove build output files.
#
#******************************************************************************
clean: $(SUBDIRS)

.PHONY: all subdirs clean boards
.PHONY: doxygen doxy doxy-clean
.PHONY: svd svd-clean
.PHONY: hal bsp
.PHONY: examples examdirs
.PHONY: ext-build checkout style-check lib-brd-dox failed-build-check 
.PHONY: $(SUBDIRS) $(HAL) $(BSP) $(EXAMDIRS)
