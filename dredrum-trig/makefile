PRG            = dredrum
OBJ            = dredrum.o 
INCLUDE_PATH   = -I.././dredrum

COMPILE_TARGET = atmega48p
UPLOAD_TARGET  = atmega48p
PROGRAMMER     = usbtiny
LFUSE          = 0xff
HFUSE          = 0xd4

a: $(PRG).elf lst hex setfuses upload
	
upload: $(PRG).hex 
	avrdude -c $(PROGRAMMER) -p $(UPLOAD_TARGET) -U flash:w:$(PRG).hex

$(PRG).elf: $(OBJ)
	avr-gcc -g -Wall -mmcu=$(COMPILE_TARGET) -o $@ $^ 

%.o: %.c
	avr-gcc -g -Wall -O2 -mmcu=$(COMPILE_TARGET) -fno-exceptions -ffunction-sections -fdata-sections $(INCLUDE_PATH) -c $< 

lst:  $(PRG).lst
%.lst: %.elf
	avr-objdump -h -S $< > $@

hex:  $(PRG).hex
%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

clean:
	rm -rf *.o $(PRG).elf *.lst *.hex

setfuses:
	avrdude -c $(PROGRAMMER) -p $(UPLOAD_TARGET) -U lfuse:w:$(LFUSE):m
	avrdude -c $(PROGRAMMER) -p $(UPLOAD_TARGET) -U hfuse:w:$(HFUSE):m


