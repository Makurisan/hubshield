CFLAGS_ALL=-I../../libusbgx/build/include -L../../libusbgx/build/lib -lz -lpthread 

pi3: CFLAGS+=-static $(CFLAGS_ALL) -lusbgx -lconfig 
pi3: pi3.c gadget-hid.c
	$(CC)  $^ $(CFLAGS) -o $@


pi400test: CFLAGS+=-static $(CFLAGS_ALL) -lusbgx -lconfig
pi400test: pi3.c gadget-hid.c
	$(CC)  $^ $(CFLAGS) -o $@


clean:
	-rm pi3
