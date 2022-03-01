#PLDA_PATH = ../plda_api
#PLDA_LIB_ACCESS = ../api

CFLAGS = -DLINUX -g $(DEBFLAGS)
CFLAGS += -fPIC -pedantic -std=c99 -fopenmp

LFLAGS += -lpthread -lrt
#PLDA_LIBS = $(PLDA_PATH)/plda_api.o $(PLDA_LIB_ACCESS)/plda_lib_access.o
XPCI_LIBS = xpci_interface.o xpci_time.o xpci_registers.o xpci_imxpad.o xpci_calib_imxpad.o xpci_asyncLib.o

EXE  = xpci_registers

#all : libxpci_lib xpci_registers
all : libxpci_lib

#plda_api.o : $(PLDA_PATH)/plda_api.c $(PLDA_PATH)/plda_api.h
#	$(CC) -c $(CFLAGS) -o $@ $< 

#plda_lib_access.o : $(PLDA_LIB_ACCESS)/plda_lib_access.c $(PLDA_LIB_ACCESS)/plda_lib_access.h
#	$(CC) -c $(CFLAGS) -o $@ $< 

xpci_interface.o : xpci_interface.c xpci_interface.h  xpci_interface_expert.h
	$(CC) -c $(CFLAGS) -o $@ $< 

xpci_time.o : xpci_time.c xpci_time.h
	$(CC) -c $(CFLAGS) -o $@ $< 

xpci_registers.o : xpci_registers.c xpci_registers.h
	$(CC) -c $(CFLAGS) -o $@ $< 

xpci_registers : xpci_registers.c xpci_registers.h
	$(CC) -DTEST $(CFLAGS) -o $@ $< 

xpci_asyncLib.o : xpci_asyncLib.c
	$(CC) -c $(CFLAGS) -o $@ $< 

xpci_imxpad.o : xpci_imxpad.c xpci_imxpad.h
	$(CC) -c $(CFLAGS) -o $@ $<
	
xpci_calib_imxpad.o : xpci_calib_imxpad.c xpci_calib_imxpad.h
	$(CC) -c $(CFLAGS) -o $@ $<

#libxpci_lib : $(XPCI_LIBS) $(PLDA_LIBS)
libxpci_lib : $(XPCI_LIBS)
	#ar -cqv $@.a  $(XPCI_LIBS) $(PLDA_LIBS)
	#$(CC) -shared -Wl,-soname,$@.so.1 -o $@.so.1.0 $(XPCI_LIBS) $(PLDA_LIBS)
	ar -cqv $@.a  $(XPCI_LIBS)
	$(CC) -shared -Wl,-soname,$@.so.1 -o $@.so.1.0 $(XPCI_LIBS)
	ln -sf $@.so.1.0 $@.so.1
	ln -sf $@.so.1.0 $@.so
	rm $(XPCI_LIBS)

test: $(EXE) 

clean :
	rm -f *.o *.a *.so* xpci_registers 
