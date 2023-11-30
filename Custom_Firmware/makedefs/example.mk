#******************************************************************************
#
# example.mk - Rules for building example projects.
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

FAMILY_CONFIG:=$(FAMILY)-system-config.yaml
DEFAULT_CONFIG:=$(SWROOT)/tools/config/$(FAMILY)-system-config.yaml

all: link_scripts
.PHONY: link_scripts
link_scripts:
	@if [ -f "mem_map.yaml" ]; then \
		python3 $(SWROOT)/tools/linker_config/linker_config.py -p $(FAMILY) "mem_map.yaml"; \
	fi
	@if [ -n "$(findstring apollo4, $(FAMILY))" ]; then \
		if [ -f "system-config.yaml" ]; then \
			python3 $(SWROOT)/tools/apollo4/generate_link_script.py "system-config.yaml"; \
		elif [ -f $(FAMILY_CONFIG) ]; then \
			echo "Using $(FAMILY_CONFIG) for linker config."; \
			python3 $(SWROOT)/tools/apollo4/generate_link_script.py $(FAMILY_CONFIG); \
		elif [ -f $(DEFAULT_CONFIG) ]; then \
			echo "Using $(DEFAULT_CONFIG) for linker config."; \
			python3 $(SWROOT)/tools/apollo4/generate_link_script.py $(DEFAULT_CONFIG); \
		else \
			echo "Couldn't find default linker config at: $(DEFAULT_CONFIG)"; \
		fi; \
	fi

SWROOT?=..
include $(SWROOT)/makedefs/subdirs.mk
