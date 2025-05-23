BUILDDIR	:=	./build
SRC_BASE	:=	.
TARGET		:= loupe-flightdeck

SOURCES = \
	flightdeck.cpp

LIBS = 

INCLUDES = \
	-I$(SRC_BASE)/xplane-sdk/CHeaders/XPLM \
	-I$(SRC_BASE)/xplane-sdk/CHeaders/Widgets

DEFINES = -DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DAPL=0 -DIBM=0 -DLIN=1

############################################################################


VPATH = $(SRC_BASE)

CSOURCES	:= $(filter %.c, $(SOURCES))
CXXSOURCES	:= $(filter %.cpp, $(SOURCES))

CDEPS64			:= $(patsubst %.c, $(BUILDDIR)/obj64/%.cdep, $(CSOURCES))
CXXDEPS64		:= $(patsubst %.cpp, $(BUILDDIR)/obj64/%.cppdep, $(CXXSOURCES))
COBJECTS64		:= $(patsubst %.c, $(BUILDDIR)/obj64/%.o, $(CSOURCES))
CXXOBJECTS64	:= $(patsubst %.cpp, $(BUILDDIR)/obj64/%.o, $(CXXSOURCES))
ALL_DEPS64		:= $(sort $(CDEPS64) $(CXXDEPS64))
ALL_OBJECTS64	:= $(sort $(COBJECTS64) $(CXXOBJECTS64))

CFLAGS := $(DEFINES) $(INCLUDES) -fPIC -fvisibility=hidden -Wall -g -O2


# Phony directive tells make that these are "virtual" targets, even if a file named "clean" exists.
.PHONY: all clean $(TARGET)
# Secondary tells make that the .o files are to be kept - they are secondary derivatives, not just
# temporary build products.
.SECONDARY: $(ALL_OBJECTS) $(ALL_OBJECTS64) $(ALL_DEPS)



# Target rules - these just induce the right .xpl files.

$(TARGET): $(BUILDDIR)/$(TARGET)/64/lin.xpl
	

$(BUILDDIR)/$(TARGET)/64/lin.xpl: $(ALL_OBJECTS64)
	@echo Linking $@
	mkdir -p $(dir $@)
	g++ -m64 -static-libgcc -shared -Wl,--version-script=exports.txt -o $@ $(ALL_OBJECTS64) $(LIBS)

# Compiler rules

# What does this do?  It creates a dependency file where the affected
# files are BOTH the .o itself and the cdep we will output.  The result
# goes in the cdep.  Thus:
# - if the .c itself is touched, we remake the .o and the cdep, as expected.
# - If any header file listed in the cdep turd is changed, rebuild the .o.
$(BUILDDIR)/obj64/%.o : %.c
	mkdir -p $(dir $@)
	g++ $(CFLAGS) -m64 -c $< -o $@
	g++ $(CFLAGS) -MM -MT $@ -o $(@:.o=.cdep) $<

$(BUILDDIR)/obj64/%.o : %.cpp
	mkdir -p $(dir $@)
	g++ $(CFLAGS) -m64 -c $< -o $@
	g++ $(CFLAGS) -MM -MT $@ -o $(@:.o=.cppdep) $<

clean:
	@echo Cleaning out everything.
	rm -rf $(BUILDDIR)

# Include any dependency turds, but don't error out if they don't exist.
# On the first build, every .c is dirty anyway.  On future builds, if the
# .c changes, it is rebuilt (as is its dep) so who cares if dependencies
# are stale.  If the .c is the same but a header has changed, this 
# declares the header to be changed.  If a primary header includes a 
# utility header and the primary header is changed, the dependency
# needs a rebuild because EVERY header is included.  And if the secondary
# header is changed, the primary header had it before (and is unchanged)
# so that is in the dependency file too.
-include $(ALL_DEPS64)


