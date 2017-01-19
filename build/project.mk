# Copyright (c) 2015, Intel Corporation. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
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

# This Makefile MUST be included at the bottom of each project Makefile
#
# It expects the following project specific variables to be set:
#
# PROJECT      : the project name
# BOARD        : a board name, for which a $(BOARD).mk must be available
# BUILDVARIANT : debug|release
#
# By including this file, the project Makefile gains access to the SDK
# basic commands (setup, help, ...).
#
# When running the special setup command, a new Makefile is created in the
# output directory. This Makefile gives access to the specific build commands
# for the specified board (image, package, ...).
#

THIS_DIR   := $(shell dirname $(abspath $(lastword $(MAKEFILE_LIST))))
T          := $(abspath $(THIS_DIR)/..)
PARENT_DIR := $(shell dirname $(abspath $(firstword $(MAKEFILE_LIST))))

ifndef PROJECT
$(info $(ANSI_RED)No project short name (PROJECT) specified$(ANSI_OFF))
$(error )
endif

ifndef BUILDVARIANT
$(info $(ANSI_RED)No build variant (BUILDVARIANT) specified$(ANSI_OFF))
$(error )
endif

ifndef BOARD
$(info $(ANSI_RED)No board (BOARD) specified$(ANSI_OFF))
$(error )
endif

# By default, we expect to be called by a make command issued directly on the
# project Makefile at the root of the project directory.
PROJECT_PATH ?= $(PARENT_DIR)

ifndef OUT
# If the user didn't specify explicitly the output directory, it is set to a
# default value.
ifeq ($(CURDIR),$(PROJECT_PATH))
# We are being called from the project directory
ifneq ($(wildcard $(T)/../.repo),)
# This is the expected repo-based setup: use the Android convention
OUT := $(abspath $(T)/../out)
endif
else
# We are called from another directory: assume it is the output directory,
# following the convention used by other build systems (autotools, cmake, ...)
OUT := $(abspath $(CURDIR))
endif
else
# Make sure OUT is an absolute path
override OUT := $(abspath $(OUT))
endif

#
# Convenience proxy for 'make setup xx yy' commands
#
# All goals passed after setup that are not directly handled by this file are
# forwarded to the generated Makefile.
#
ifneq ($(filter setup,$(MAKECMDGOALS)),)

# We must prevent parallel processing to make sure setup runs first
.NOTPARALLEL:

# The default rule catches all targets for which no explicit rule is defined
.DEFAULT:
	$(AT)$(MAKE) -C $(OUT) $@

endif

#
# Versioning, setup and build info
#

# Use a default build tag when none is set by the caller
NOWDATE     := $(shell date +"%Y%m%d%H%M%S")
BUILD_TAG   ?= custom_build_$(USER)@$(HOSTNAME)$(NOWDATE)
PROJECT_SHORT_CODE ?= ATP1

# Version of the project, set to 1.0.0 when PV is reached
# These values should be overriden in the project's Makefile rather than
# modified here
VERSION_MAJOR  ?= 1
VERSION_MINOR  ?= 0
VERSION_PATCH  ?= 0

# Get build number from environment or generate from MMSS
ifeq ($(BUILD_NUMBER),)
BUILD_NUMBER_PADDED := $(shell date +"%M%S")
else
BUILD_NUMBER_PADDED := $(shell printf "%04d" $(BUILD_NUMBER))
endif
BUILD_NUMBER_TRUNCATED = $(shell echo $(BUILD_NUMBER_PADDED) | tail -c 5)

# If BUILD_TYPE is defined, use its first letter
ifneq ($(BUILD_TYPE),)
BUILD_LETTER = $(shell printf "%c" $(BUILD_TYPE) | tr a-z A-Z)
else
# Custom Build
BUILD_LETTER = C
BUILD_TYPE = custom
endif

# By default use the following:
# Year: %g, Workweek: %V (+1 to be aligned with the Intel WW calendar), Type: C (=Custom build), BuildNumber: %M%S
ifeq ($(BUILD_LETTER),W)
VERSION_STRING_SUFFIX ?= $(shell date +"%g")$(shell printf "%.*d" 2 $(shell expr `date +"%W"` + 1))$(BUILD_LETTER)$(shell printf "%04d" $(VERSION_PATCH))
else
VERSION_STRING_SUFFIX ?= $(shell date +"%g")$(shell printf "%.*d" 2 $(shell expr `date +"%V"` + 1))$(BUILD_LETTER)$(BUILD_NUMBER_TRUNCATED)
endif

ifeq ($(ARTIFACTORY_BUILD_URL),)
DOWNLOAD_DIR_URL = file://$(abspath $(PUB))
else
DOWNLOAD_DIR_URL := $(ARTIFACTORY_BUILD_URL)
endif

# These variables defined during setup must be remembered and made available
# when other make goals are invoked
SETUP_VARS += \
    PROJECT_PATH              \
    PROJECT                   \
    BOARD                     \
    BUILDVARIANT              \
    T                         \
    OUT                       \
    BUILD_TAG                 \
    BUILD_TYPE                \
    WORKWEEK                  \
    BUILDNUMBER               \
    VERSION_MAJOR             \
    VERSION_MINOR             \
    VERSION_PATCH             \
    VERSION_STRING_SUFFIX     \
    PROJECT_SHORT_CODE

# Also remember all variables related to the defconfig files
SETUP_VARS += $(filter %_DEFCONFIG,$(.VARIABLES))
SETUP_VARS += $(filter %_DEPENDS,$(.VARIABLES))

EXPANDED_SETUP_VARS = $(foreach var,$(SETUP_VARS),$(var)=$($(var)))

.PHONY: setup

# We look for a board Makefile first in the project, then in well-known places
ifneq ($(wildcard $(PROJECT_PATH)/boards/$(BOARD).mk),)
BOARD_MAKEFILE ?= $(PROJECT_PATH)/boards/$(BOARD).mk
else
ifneq ($(wildcard $(T)/build/boards/$(BOARD).mk),)
BOARD_MAKEFILE ?= $(T)/build/boards/$(BOARD).mk
endif
endif

OPTION_MAKEFILE ?= $(T)/projects/options.mk

# This make goal is the first that must be called: it creates the build
# configuration for the project based on the specified parameters
setup:
ifeq ($(OUT),)
	@echo $(ANSI_RED)You are using a custom installation.$(ANSI_OFF)
	@echo $(ANSI_RED)Please specify the output directory using OUT.$(ANSI_OFF)
	@exit 1
endif
	@echo Output directory: $(OUT)
ifneq ($(findstring $(OUT),$(T)),)
	@echo $(ANSI_RED)The output directory cannot contain the SDK$(ANSI_OFF)
	@exit 1
endif
ifneq ($(findstring $(T),$(OUT)),)
	@echo $(ANSI_RED)The output directory cannot be in the SDK source tree$(ANSI_OFF)
	@exit 1
endif
ifneq ($(BOARD_MAKEFILE),)
	$(AT)mkdir -p $(OUT)
ifneq ($(wildcard $(OUT)/Makefile),)
	@echo $(ANSI_CYAN)"Clearing existing setup"$(ANSI_OFF)
	$(AT)-$(MAKE) -C $(OUT) mrproper
	$(AT)rm -rf $(OUT)/Makefile   \
		$(OUT)/firmware           \
		$(OUT)/package_prefix.txt \
		$(OUT)/build_setup.json
	@echo $(ANSI_CYAN)"Done clearing existing setup"$(ANSI_OFF)
	@echo
endif
	@echo $(ANSI_CYAN)"Installing new setup"$(ANSI_OFF)
	@echo "   PROJECT       : $(PROJECT)"
	@echo "   VERSION       : $(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_PATCH)"
	@echo "   BOARD         : $(BOARD)"
	@echo "   BUILDVARIANT  : $(BUILDVARIANT)"
	@echo "   BUILD_TAG     : $(BUILD_TAG)"
	@echo "   BUILD_TYPE    : $(BUILD_TYPE)"
	@echo "   WORKWEEK      : $(WORKWEEK)"
	@echo "   BUILDNUMBER   : $(BUILDNUMBER)"
	@echo
	@echo "$(PROJECT)-$(BOARD)-$(BUILDVARIANT)-$(BUILD_TAG)" > $(OUT)/package_prefix.txt
	@echo "{\"PROJECT\": \"$(PROJECT)\", \"BOARD\": \"$(BOARD)\", \"BUILDVARIANT\": \"$(BUILDVARIANT)\", \"BUILD_TAG\": \"$(BUILD_TAG)\", \"BUILD_TYPE\": \"$(BUILD_TYPE)\", \"DOWNLOAD_DIR_URL\": \"$(DOWNLOAD_DIR_URL)\"}" > $(OUT)/build_setup.json
	$(AT)mkdir -p $(OUT)/firmware
	$(AT)echo $(ANSI_RED)"[tMK]"$(ANSI_OFF) "Creating $(OUT)/Makefile"
	@for var in $(EXPANDED_SETUP_VARS); \
	do \
	echo "override $$var" >> $(OUT)/Makefile; \
	done;
	@echo "include $(OPTION_MAKEFILE)" >> $(OUT)/Makefile
	@echo "-include $(PROJECT_PATH)/options.mk" >> $(OUT)/Makefile
	@echo "include ${BOARD_MAKEFILE}" >> $(OUT)/Makefile
else
	@echo $(ANSI_RED)"Board $(BOARD) is not officially supported."$(ANSI_OFF)
	@echo $(ANSI_RED)"Please provide a $(BOARD).mk."$(ANSI_OFF)
	@exit 1
endif
	@echo $(ANSI_CYAN)"Done setup"$(ANSI_OFF)
	@echo

# Common targets that DO NOT depend on setup below

# Help

include $(T)/build/help.mk

help::
	@echo ' setup            - setup the build environment. Use BOARD and BUILDVARIANT' \
		'parameters to specify the config.'
	@echo ' one_time_setup   - performs initial setup of host machine' \
		'(requires root privileges).'
	@echo ' check_host_setup - check that host has all necessary dependencies' \
		'installed for building'
	@echo

# one_time_setup
include $(T)/build/host/check.mk

# Declare global variables for pretty output
include $(T)/build/verbosity.mk
