SRC = src
OBJ = build/obj
DIST = dist

PLUGINS = VH_Flocking

ifeq ($(OSTYPE), linux-gnu)
NDK = /usr/local/Nuke6.3v1
PLUGIN_EXT = so
else
NDK = /Applications/Nuke6.3v1/Nuke6.3v1.app/Contents/MacOS
PLUGIN_EXT = dylib
endif

ALL = $(foreach a,$(PLUGINS),$(DIST)/$(a).$(PLUGIN_EXT))
LINKS = $(foreach a,$(PLUGINS),$(HOME)/.nuke/$(a).$(PLUGIN_EXT))

CXX = g++
CXXFLAGS = -g -c -DUSE_GLEW -I$(NDK)/include -fPIC
LD = g++
LDFLAGS = -shared -L$(NDK) -lDDImage


all: $(ALL)


install: $(LINKS)


.PHONY: dirs
dirs:
	mkdir -p $(OBJ)
	mkdir -p $(DIST)


$(HOME)/.nuke/%.$(PLUGIN_EXT): $(DIST)/%.$(PLUGIN_EXT)
	ln -s $(abspath $<) $@


.PRECIOUS : $(OBJ)/%.o
$(OBJ)/%.o: $(SRC)/%.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<


$(DIST)/%.$(PLUGIN_EXT): $(OBJ)/%.o
	$(LD) $(LDFLAGS) -o $@ $<


.PHONY: uninstall
uninstall:
	rm -f $(LINKS)


clean: uninstall
	rm -f $(OBJ)/* $(DIST)/*

