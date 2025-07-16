obj-m += LD2450.o
#obj-m += LD2450_gpt.o

all: dt app module 

module:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

dt: serdev_overlay.dts
	dtc -@ -I dts -O dtb -o serdev_overlay.dtbo serdev_overlay.dts

app: 
	gcc -o LD2450_app LD2450_app.c -lm

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -rf serdev_overlay.dtbo
	rm -rf LD2450_app
