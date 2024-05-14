ifeq ($(PLATFORM),EMBEDDED)
ENDIANNESS = BIG
include ../../../../../arch/ARM/ixp42x/module.mk
endif

LIB = libmercuryapi.a

OBJS += serial_transport_posix.o
OBJS += serial_transport_llrp.o
OBJS += tmr_strerror.o
OBJS += tmr_param.o
OBJS += hex_bytes.o
OBJS += tm_reader.o
OBJS += tm_reader_async.o
OBJS += serial_reader.o
OBJS += serial_reader_l3.o
OBJS += tmr_utils.o

OBJS += osdep_posix.o

HEADERS += serial_reader_imp.h
HEADERS += tm_config.h
HEADERS += tm_reader.h
HEADERS += tmr_filter.h
HEADERS += tmr_gen2.h
HEADERS += tmr_gpio.h
HEADERS += tmr_ipx.h
HEADERS += tmr_iso180006b.h
HEADERS += tmr_params.h
HEADERS += tmr_read_plan.h
HEADERS += tmr_region.h
HEADERS += tmr_serial_reader.h
HEADERS += tmr_serial_transport.h
HEADERS += tmr_status.h
HEADERS += tmr_tag_auth.h
HEADERS += tmr_tag_data.h
HEADERS += tmr_tag_lock_action.h
HEADERS += tmr_tag_protocol.h
HEADERS += tmr_tagop.h
HEADERS += tmr_types.h
HEADERS += tmr_utils.h

DBG ?= -g
CWARN = -Wall
CFLAGS += -I. $(DBG) $(CWARN) -lcurl
LDFLAGS += -lcurl

ifneq ($(PLATFORM),EMBEDDED)
PROGS += demo
endif
PROGS += filter
PROGS += locktag
PROGS += multireadasync
PROGS += read
PROGS += readasync
PROGS += readintoarray
PROGS += blockpermalock
PROGS += blockwrite
PROGS += embeddedreadtid
PROGS += licensekey
PROGS += multiprotocolread
PROGS += savedconfig
PROGS += writetag
PROGS += readasynctrack
PROGS += readasyncfilter

all: $(LIB) $(PROGS)

$(OBJS): $(HEADERS)

$(LIB): $(OBJS)
	ar -rc $@ $^

../samples/read.o: $(HEADERS)
read: ../samples/read.o $(LIB)
	$(CC) -o $@ $^ $(CFLAGS)

../samples/blockpermalock.o: $(HEADERS)
blockpermalock: ../samples/blockpermalock.o $(LIB)
	$(CC) $(CFLAGS) -o $@ $^

../samples/blockwrite.o: $(HEADERS)
blockwrite: ../samples/blockwrite.o $(LIB)
	$(CC) $(CFLAGS) -o $@ $^

../samples/embeddedreadtid.o: $(HEADERS)
embeddedreadtid: ../samples/embeddedreadtid.o $(LIB)
	$(CC) $(CFLAGS) -o $@ $^

../samples/licensekey.o: $(HEADERS)
licensekey: ../samples/licensekey.o $(LIB)
	$(CC) $(CFLAGS) -o $@ $^

../samples/multiprotocolread.o: $(HEADERS)
multiprotocolread: ../samples/multiprotocolread.o $(LIB)
	$(CC) $(CFLAGS) -o $@ $^

../samples/savedconfig.o: $(HEADERS)
savedconfig: ../samples/savedconfig.o $(LIB)
	$(CC) $(CFLAGS) -o $@ $^

../samples/writetag.o: $(HEADERS)
writetag: ../samples/writetag.o $(LIB)
	$(CC) $(CFLAGS) -o $@ $^

../samples/readintoarray.o: $(HEADERS)
readintoarray: ../samples/readintoarray.o $(LIB)
	$(CC) $(CFLAGS) -o $@ $^

../samples/readasync.o: $(HEADERS)
readasync: ../samples/readasync.o $(LIB)
	$(CC) $(CFLAGS) -o $@ $^ -lpthread

../samples/multireadasync.o: $(HEADERS)
multireadasync: ../samples/multireadasync.o $(LIB)
	$(CC) $(CFLAGS) -o $@ $^ -lpthread

../samples/demo.o: $(HEADERS)
demo: ../samples/demo.o $(LIB)
	$(CC) $(CFLAGS) -o $@ $^ -lreadline

../samples/filter.o: $(HEADERS)
filter: ../samples/filter.o $(LIB)
	$(CC) $(CFLAGS) -o $@ $^

../samples/locktag.o: $(HEADERS)
locktag: ../samples/locktag.o $(LIB)
	$(CC) $(CFLAGS) -o $@ $^

../samples/readasynctrack.o: $(HEADERS)
readasynctrack: ../samples/readasynctrack.o $(LIB)
	$(CC) $(CFLAGS) -o $@ $^ -lpthread

../samples/readasyncfilter.o: $(HEADERS)
readasyncfilter: ../samples/readasyncfilter.o $(LIB)
	$(CC) $(CFLAGS) -o $@ $^ -lpthread

.PHONY: clean
clean:
	rm -f $(LIB) $(PROGS) *.o ../samples/*.o core tests/*.output

.PHONY: test
TESTSCRIPTS ?=
test: demo
	tests/runtests.sh $(TESTSCRIPTS)

longtest: demo
	while [ 1 ]; do echo Iteration: `date`; make test; done

test-sleeprecovery: demo
	./demo -v $(URI) <tests/test-sleeprecovery.prelim-script
# @todo Turn sleeprecovery into a real script when we know what to put in the key file

# What serial ports exist?
list-linux-ports:
	echo /dev/tty{ACM,USB,S}*

## Measure library size
libsize: $(LIB) $(LIB).stripped
	svn di tm_config.h
	echo DBG: $(DBG)
	ls -l $<*
	echo

%.stripped: %
	cp -p $< $@
	strip $@

## Test of minimum library size
empty.c:
	echo >$@

libempty.a: empty.o
	ar -rc $@ $^

libemptysize: libempty.a libempty.a.stripped
	ls -l $<*
	echo
