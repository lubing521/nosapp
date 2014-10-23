#CFLAGS=-I../../include -I./
#LDFLAGS=-L../../lib -lnorouter cgicomm.o
#CGILDFLAGS=$(LDFLAGS) -lcgi
#CC=mipsel-unknown-linux-uclibc-gcc

all: 
	make -C src
	[ -d "GuogeeSmartHome/bin" ] || mkdir GuogeeSmartHome/bin
	cp src/ismartus GuogeeSmartHome/bin/ismartus
	../../utility/package_app.sh GuogeeSmartHome
	cp GuogeeSmartHome.opk ~/tftp

clean:
	make clean -C src
	rm -f *.opk*


