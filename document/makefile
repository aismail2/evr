.PHONY:	all clean

vpath	%.dot	source
vpath	%.tex	source

DOTS		=	

TEX			=	evr.tex\

OBJECTS		=	$(addprefix build/,$(subst .dot,.jpg,$(DOTS)))		\
				$(addprefix build/,$(subst .tex,.pdf,$(TEX)))

all: $(OBJECTS)

clean:
	rm -f build/*.jpg
	rm -f build/*.aux
	rm -f build/*.log
	rm -f build/*.out
	rm -f build/*.toc
	rm -f build/*.pdf

build/%.jpg:%.dot
	dot -Tjpg $< -o $@
	feh $@ >/dev/null 2>/dev/null &
	
build/%.pdf:%.tex
	pdflatex --output-directory=build $<
	pdflatex --output-directory=build $<
	[ ! -f $@.lock ] && touch $@.lock && mupdf $@ >/dev/null 2>/dev/null && rm $@.lock&
