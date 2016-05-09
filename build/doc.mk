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

ifndef T
$(error No top level source directory T specified)
endif

ifndef OUT
$(error No output directory (OUT) specified)
endif

# Project name, used in the documentation
PROJECT_NAME   ?= Unnamed

# Project version, used in the documentation
PROJECT_VERSION ?= $(BUILD_TAG)

# Project logo, used in the documentation
PROJECT_LOGO    ?= $(T)/doc/images/logo.png

# Path to image files, used in the documentation
IMAGE_PATH    ?= $(T)/doc/images

# A list of extra directories to scan for documentation
EXTRA_DOC_INPUT ?=

-include $(T)/packages/*/*/doc.mk
-include $(T)/packages/*/doc.mk
-include $(T)/projects/*/doc.mk
-include $(T)/internal/projects/*/doc.mk

# A list of extra directories to scan for documentation images
EXTRA_IMAGES_INPUT ?=

.PHONY: doc doc_package doc_view

doc:
	@mkdir -p $(OUT)/doc
	@cp $(T)/doc/Doxyfile.in $(OUT)/doc/Doxyfile
	@sed -i 's|@ROOT_DIR@|$(T)|g' $(OUT)/doc/Doxyfile
	@sed -i 's|@OUT_DIR@|$(OUT)/doc|g' $(OUT)/doc/Doxyfile
	@sed -i 's|@PROJECT_NAME@|$(PROJECT_NAME)|g' $(OUT)/doc/Doxyfile
	@sed -i 's|@PROJECT_VERSION@|$(PROJECT_VERSION)|g' $(OUT)/doc/Doxyfile
	@sed -i 's|@PROJECT_LOGO@|$(PROJECT_LOGO)|g' $(OUT)/doc/Doxyfile
	@sed -i 's|@IMAGE_PATH@|$(IMAGE_PATH)|g' $(OUT)/doc/Doxyfile
	@sed -i 's|@EXTRA_DOC_INPUT@|$(EXTRA_DOC_INPUT)|g' $(OUT)/doc/Doxyfile
	@sed -i 's|@EXTRA_IMAGES_INPUT@|$(EXTRA_IMAGES_INPUT)|g' $(OUT)/doc/Doxyfile
	doxygen $(OUT)/doc/Doxyfile 2>&1 | ( ! grep . )
	@echo "Documentation generated in $(OUT)/doc/"
	@echo "Type \"make doc_view\" to view it in your web browser"

doc_package: doc
	@mkdir -p $(PUB)
	@cd $(OUT)/doc/html && tar cvfz $(PUB)/documentation.tgz .
	@echo "Documentation archived as $(PUB)/documentation.tgz"

doc_view: doc
	@xdg-open $(OUT)/doc/html/index.html

help::
	@echo 'Documentation:'
	@echo ' doc		- generate the doxygen documentation'
	@echo ' doc_package	- generate a .zip in pub/ containing the doxygen documentation'
	@echo ' doc_view	- display the generated doxygen documentation in your browser'
	@echo

