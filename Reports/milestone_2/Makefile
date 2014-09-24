TGT=report
DEPS= 

LATEX=latex

all: $(TGT)
pdf: $(TGT).pdf

$(TGT): $(TGT).pdf

$(TGT).pdf: $(TGT).ps
	ps2pdf $(TGT).ps $(TGT).pdf


$(TGT).ps: $(TGT).dvi
	dvips -o $(TGT).ps $(TGT)

$(TGT).dvi: $(TGT).tex $(DEPS)
	$(LATEX) $(TGT) &&  $(LATEX) $(TGT) && $(LATEX) $(TGT)

.PHONY: clean clobber fresh
clean: 
	-$(RM) $(TGT).dvi $(TGT).aux $(TGT).log
	-$(RM) $(TGT).toc $(TGT).idx $(TGT).bbl
	-$(RM) $(TGT).toc $(TGT).ind $(TGT).blg 
	-$(RM) $(TGT).lot $(TGT).lof $(TGT).blg 
	-$(RM) $(TGT).ind
	-$(RM) $(TGT).out
	-$(RM) $(TGT).ps
	-$(RM) *.aux
	-$(RM) missfont.log *~

clobber: clean
	-$(RM) $(TGT).ps $(TGT).pdf

fresh: clobber all
