# NX-MIMOSA Documentation Makefile
# Sphinx documentation build system

SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = .
BUILDDIR      = _build

# Colors
BLUE=\033[0;34m
GREEN=\033[0;32m
NC=\033[0m

.PHONY: help clean html dirhtml singlehtml pdf epub latexpdf linkcheck doctest coverage

help:
	@echo "$(BLUE)NX-MIMOSA Documentation Build$(NC)"
	@echo ""
	@echo "Available targets:"
	@echo "  $(GREEN)html$(NC)       - Build HTML documentation"
	@echo "  $(GREEN)dirhtml$(NC)    - Build HTML documentation with directory structure"
	@echo "  $(GREEN)singlehtml$(NC) - Build single-page HTML documentation"
	@echo "  $(GREEN)pdf$(NC)        - Build PDF documentation (requires LaTeX)"
	@echo "  $(GREEN)epub$(NC)       - Build EPUB documentation"
	@echo "  $(GREEN)linkcheck$(NC)  - Check external links"
	@echo "  $(GREEN)doctest$(NC)    - Run doctests"
	@echo "  $(GREEN)coverage$(NC)   - Check documentation coverage"
	@echo "  $(GREEN)clean$(NC)      - Remove build artifacts"
	@echo ""
	@echo "Usage: make <target>"

clean:
	rm -rf $(BUILDDIR)/*
	@echo "$(GREEN)Build directory cleaned.$(NC)"

html:
	@echo "$(BLUE)Building HTML documentation...$(NC)"
	$(SPHINXBUILD) -b html $(SOURCEDIR) $(BUILDDIR)/html $(SPHINXOPTS)
	@echo ""
	@echo "$(GREEN)Build finished. Documentation is in $(BUILDDIR)/html/$(NC)"

dirhtml:
	$(SPHINXBUILD) -b dirhtml $(SOURCEDIR) $(BUILDDIR)/dirhtml $(SPHINXOPTS)

singlehtml:
	$(SPHINXBUILD) -b singlehtml $(SOURCEDIR) $(BUILDDIR)/singlehtml $(SPHINXOPTS)

pdf: latexpdf

latexpdf:
	@echo "$(BLUE)Building PDF documentation...$(NC)"
	$(SPHINXBUILD) -b latex $(SOURCEDIR) $(BUILDDIR)/latex $(SPHINXOPTS)
	cd $(BUILDDIR)/latex && make
	@echo ""
	@echo "$(GREEN)PDF built: $(BUILDDIR)/latex/*.pdf$(NC)"

epub:
	$(SPHINXBUILD) -b epub $(SOURCEDIR) $(BUILDDIR)/epub $(SPHINXOPTS)

linkcheck:
	@echo "$(BLUE)Checking links...$(NC)"
	$(SPHINXBUILD) -b linkcheck $(SOURCEDIR) $(BUILDDIR)/linkcheck $(SPHINXOPTS)
	@echo ""
	@echo "$(GREEN)Link check complete. Results in $(BUILDDIR)/linkcheck/$(NC)"

doctest:
	$(SPHINXBUILD) -b doctest $(SOURCEDIR) $(BUILDDIR)/doctest $(SPHINXOPTS)

coverage:
	@echo "$(BLUE)Checking documentation coverage...$(NC)"
	$(SPHINXBUILD) -b coverage $(SOURCEDIR) $(BUILDDIR)/coverage $(SPHINXOPTS)
	@echo ""
	@cat $(BUILDDIR)/coverage/python.txt 2>/dev/null || echo "No coverage report found"

# Live reload for development
livehtml:
	sphinx-autobuild $(SOURCEDIR) $(BUILDDIR)/html $(SPHINXOPTS)

# Catch-all target
%:
	$(SPHINXBUILD) -M $@ $(SOURCEDIR) $(BUILDDIR) $(SPHINXOPTS)
