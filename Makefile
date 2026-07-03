#
# 'make'        build executable file 'main'
# 'make clean'  removes all .o and executable files
#

EIGENDIR = externals/eigen-3.4.0
VTU11DIR = externals/vtu11

OMPINC = $(HOME)/opt/openmp/include
OMPLIB = $(HOME)/opt/openmp/lib


# define the Cpp compiler to use
CXX = g++-15

# define any compile-time flags
CXXFLAGS := -std=c++20 -Wall -Wextra -g -Xpreprocessor -fopenmp


# define library paths in addition to /usr/lib
#   if I wanted to include libraries not in /usr/lib I'd specify
#   their path using -Lpath, something like:
LDFLAGS= -lomp

# define output directory
OUTPUT	:= output

# define source directory
SRC		:= src

# define include directory
INCLUDE	:= include 

# define lib directory
LIB		:= lib ${VTKLIBDIR}

MAIN	:= main
SOURCEDIRS	:= $(shell find $(SRC) -type d)
INCLUDEDIRS	:= $(shell find $(INCLUDE) -type d)
LIBDIRS		:= $(shell find $(LIB) -type d)
FIXPATH = $1
RM = rm -f
MD	:= mkdir -p

# define any directories containing header files other than /usr/include
INCLUDES	:= $(patsubst %,-I%, $(INCLUDEDIRS:%/=%)) -I${EIGENDIR} -I${VTU11DIR} -I${OMPINC}

# define the C libs
LIBS		:= $(patsubst %,-L%, $(LIBDIRS:%/=%)) -L${OMPLIB} -lomp

# define the C source files
SOURCES		:= $(wildcard $(patsubst %,%/*.cpp, $(SOURCEDIRS)))

# define the C object files by replacing src/ with output/ and .cpp with .o
OBJECTS		:= $(patsubst $(SRC)/%.cpp,$(OUTPUT)/%.o,$(SOURCES))

#
# The following part of the makefile is generic; it can be used to 
# build any executable just by changing the definitions above and by
# deleting dependencies appended to the file from 'make depend'
#

OUTPUTMAIN	:= $(call FIXPATH,$(OUTPUT)/$(MAIN))

all: $(OUTPUTMAIN)
	@echo Executing 'all' complete!

$(OUTPUTMAIN): $(OBJECTS) 
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(OUTPUTMAIN) $(OBJECTS) $(LFLAGS) $(LIBS)

# modern pattern rule for building .o from .cpp
$(OUTPUT)/%.o: $(SRC)/%.cpp
	@$(MD) $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $<  -o $@

.PHONY: clean
clean:
	$(RM) $(OUTPUTMAIN)
	$(RM) -r $(OUTPUT)/*.o $(OUTPUT)/*/*.o $(OUTPUT)/*/*/*.o
	@echo Cleanup complete!

run: all
	./$(OUTPUTMAIN)
	@echo Executing 'run: all' complete!