all: router

router: ospfnode.o common.o driver.o
	g++ ospfnode.cc common.cc driver.cc -o router

clean:
	rm *.o router
