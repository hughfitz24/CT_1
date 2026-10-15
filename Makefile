# pdfTeX engine with the LaTeX format, required by template.tex.
.DEFAULT_GOAL := all
PDFTEX ?= pdftex
TEXFLAGS ?= -fmt=pdflatex -interaction=nonstopmode -halt-on-error -file-line-error

.PHONY: all clean
all:
	$(PDFTEX) $(TEXFLAGS) main.tex
	$(PDFTEX) $(TEXFLAGS) main.tex

# Remove auxiliary files; keep template.pdf.
clean:
	rm -f template.aux template.log template.out template.toc template.lof template.lot
