# Makefile for compiling the manuscript or packaging for Overleaf

MAIN = main
BUILD_DIR = build
OVERLEAF_ZIP = v_lidar_manuscript_overleaf.zip

.PHONY: all clean overleaf

all:
	@which pdflatex > /dev/null || (echo "Error: pdflatex is not installed. You can install texlive or upload $(OVERLEAF_ZIP) to Overleaf." && exit 1)
	pdflatex $(MAIN)
	bibtex $(MAIN)
	pdflatex $(MAIN)
	pdflatex $(MAIN)

overleaf:
	@echo "Creating Overleaf zip archive..."
	zip -r ../$(OVERLEAF_ZIP) main.tex references.bib sections/ figures/
	@echo "Archive created at: ../$(OVERLEAF_ZIP)"

clean:
	rm -f *.aux *.log *.out *.bbl *.blg *.toc *.synctex.gz
