://github.com/rm-hull/spidev-test

- with this module we can test the spi communication
  
* prerequisites
 - load the spidev module
   e.g. config.txt spi=on

* unload v-hub and unload the overlay
  - sudo dtoverlay -r 0 # check index
  - sudo rmmod v-hub

* build the tool
	gcc spidev_test.c -o spi_test
	spi_test -v # to test the transmit

The spidev_test module can be used as a base for copying the project file
to the boards via SPI on spi0.0


