# Put user-specific changes in your own Makefile.user.
# Make will silently continue if that file does not exist.
-include Makefile.user

# Mac OS X users may need to add to Makefile.user:
#  LATEX = /usr/texbin/latex
#  PDFLATEX = /usr/texbin/pdflatex
#  BIBTEX = /usr/texbin/bibtex
LATEX ?= latex
PDFLATEX ?= pdflatex
BIBTEX ?= bibtex

MAKE ?= make

# To regenerate list of files, do one of the following:
#  latex-process-inputs --makefilelist thesis.tex
#  latex-process-inputs --makefilelist 2012-ftfjp.tex
TEX_FILES = \
guide-dog.tex


NAME	:= guide-dog
# EPSFIGS := bst.eps
# FIGS	:= $(DOTFIGS) $(PNGFIGS) $(EPSFIGS)
#export BIBINPUTS ?= .:bib

all: $(NAME).pdf guide-dog.pdf
#all: figures-all $(NAME).pdf 2012-ftfjp.pdf

clean:
	rm -f $(NAME).bbl $(NAME).dvi $(NAME).ps $(NAME).pdf $(NAME).aux $(NAME).log $(NAME).blg $(NAME)-gs.pdf $(NAME)-dir.pdf $(NAME).out

%.ps: %.dot
	dot -Tps $< > $@

%.ps: %.png
	pngtopnm $< | pnmtops -noturn > $@

.PRECIOUS: %.dvi
%.dvi: %.tex $(FIGS) %.bbl
	${LATEX} $<
	${LATEX} $<
	${LATEX} $<

%.bbl: %.tex bib-update
	${LATEX} $<
	${BIBTEX} $(basename $<)

%.ps: %.dvi
	dvips -P cmz -t letter -o $@ $<

%.pdf: %.tex $(FIGS) %.bbl
	${PDFLATEX} '$(basename $<).tex'
	while grep "Rerun to get" $(basename $<).log; do ${PDFLATEX} '$(basename $<).tex'; done

%.eps: %.gnuplot
	gnuplot $<

xdvi:
	xdvi ${NAME}.dvi > /dev/null &

ispell: spell

spell:
	for file in ${TEX_FILES}; do ispell $$file; done

bib:
	hg clone https://plume-bib.googlecode.com/hg/ bib
.PHONY: bib-update
bib-update: bib
	cd bib && hg pull -u && make

showvars:
	echo LATEX = ${LATEX}
