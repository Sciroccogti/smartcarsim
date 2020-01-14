PROJ=SCS
VERSION=1.0.1
FILE=$(PROJ)-$(VERSION).tex
LAFLAGS=-interaction=nonstopmode
pdf:
	xelatex -no-pdf $(LAFLAGS) $(FILE)
	xelatex -no-pdf $(LAFLAGS) $(FILE)
	xelatex -no-pdf $(LAFLAGS) $(FILE)
	xelatex -no-pdf $(LAFLAGS) $(FILE)
	xelatex $(LAFLAGS) $(FILE)
clean:
	rm -rf *.aux *.xdv *.idx *.log *.out *.toc $(PROJ)-$(VERSION).pdf
