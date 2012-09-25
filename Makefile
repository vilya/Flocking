# The version of Nuke we're compiling for. Change these to match your version.
NUKE_VERSION_MAJOR = 6
NUKE_VERSION_MINOR = 3

SRC = src
OBJ = build/obj
DIST = dist

PLUGINS = VH_Flocking

ifeq ($(OSTYPE), linux-gnu)
NDK = /usr/local/Nuke6.3v8
PLUGIN_EXT = so
INSTALL = $(HOME)/.nuke/$(NUKE_VERSION_MAJOR).$(NUKE_VERSION_MINOR)/plugins
else
NDK = /Applications/Nuke6.3v1/Nuke6.3v1.app/Contents/MacOS
PLUGIN_EXT = dylib
INSTALL = $(HOME)/.nuke/$(NUKE_VERSION_MAJOR).$(NUKE_VERSION_MINOR)/plugins
endif

ALL = $(foreach a,$(PLUGINS),$(DIST)/$(a).$(PLUGIN_EXT))
LINKS = $(foreach a,$(PLUGINS),$(INSTALL)/$(a).$(PLUGIN_EXT))

CXX = g++
CXXFLAGS = \
  -g \
  -c \
  -DUSE_GLEW \
  -DNUKE_VERSION_MAJOR=$(NUKE_VERSION_MAJOR) \
  -DNUKE_VERSION_MINOR=$(NUKE_VERSION_MINOR) \
  -I$(NDK)/include \
  -fPIC
LD = g++
LDFLAGS = -shared -L$(NDK) -lDDImage


all: $(ALL)


install: $(LINKS)


.PHONY: dirs
dirs:
	mkdir -p $(OBJ)
	mkdir -p $(DIST)


$(INSTALL)/%.$(PLUGIN_EXT): $(DIST)/%.$(PLUGIN_EXT)
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

