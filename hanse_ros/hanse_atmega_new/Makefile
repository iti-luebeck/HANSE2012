.PHONY: all upload clean

all: build/Makefile
	$(MAKE) -C build

upload: build/Makefile
	$(MAKE) -C build upload_hanse_atmega

build/Makefile:
	@mkdir -p build
	@(cd build && cmake ..)

clean:
	@rm -rf build
