obj-m+=encoder.o

all:
	make -C /lib/modules/3.8.13-bone71/build/ M=$(PWD) modules
clean:
	make -C /lib/modules/3.8.13-bone71/build/ M=$(PWD) clean
