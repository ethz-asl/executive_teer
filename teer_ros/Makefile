all: installed

GIT_DIR = src
GIT_URL = git://github.com/ethz-asl/teer.git
include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR)
	touch installed

clean:
	rm -rf installed src

wipe: clean
