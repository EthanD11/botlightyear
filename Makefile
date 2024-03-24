# -----------------------------------------------------------------------------------
#Variables
# Arguments for the compilation
CXX = g++
FLAGS:=-Wall -O3
LIBS:=-llgpio -lm -lpthread -ldxl_sbc_c -lsl_lidar_sdk -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_aruco -lopencv_videoio -lopencv_imgcodecs
# Directories
HEADERS_DIR:=headers
SOURCES_DIR:=src
TESTS_DIR:=tests
OBJ_DIR:=bin
# List of cpp files
SOURCES = $(wildcard $(SOURCES_DIR)/*.cpp)
TESTS = $(wildcard $(TESTS_DIR)/*.cpp)

# List of .o files
SOURCES_OBJ = $(addprefix $(OBJ_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
SOURCES_OBJ := $(filter-out sthg_to_filter_out, $(SOURCES_OBJ))

# -----------------------------------------------------------------------------------
# Rules
all: main.cpp $(SOURCES_OBJ)
	@$(CXX) -I$(HEADERS_DIR) $(FLAGS) $^ -o exe_botlightyear $(LIBS)

run: all
	@./exe_botlightyear

#Create the OBJ_DIR directory
$(OBJ_DIR):
	@mkdir -p $(OBJ_DIR)

# Compiling a binary file from a source file
$(OBJ_DIR)/%.o: $(SOURCES_DIR)/%.cpp | $(OBJ_DIR)
	@$(CXX) -I$(HEADERS_DIR) $(CFLAGS) -o $@ -c $< $(LIBS)

# Compiling all the sources
compile: $(SOURCES_OBJ)

#Compiling a random file that use SOURCES file
%.o: %.cpp $(SOURCES_OBJ)
	@$(CXX) -I$(HEADERS_DIR) $(FLAGS) $^ -o $@ $(LIBS)


#Do an individual test
test_%: $(TESTS_DIR)/test_%.o $(SOURCES_OBJ)
	@./$<
	@rm $<

#Run all the tests
tests: $(TESTS:.cpp=.o)
	@$(foreach test, $^, ./$(test);)

# Run Camera program
camera: camera_program
	@./bin/camera
	@rm ./bin/camera

camera_program:
	@g++ ./src/cameraTag.cpp -o ./bin/camera -lpthread -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_aruco -lopencv_videoio -lopencv_imgcodecs -I$(HEADERS_DIR)
#@g++ ./src/cameraTag.cpp -o ./bin/camera.o -lpthread -I/usr/include/opencv4 -I/path/to/raspicam/include -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_aruco -lopencv_videoio -L/usr/local/lib -L/path/to/raspicam/lib -lraspicam -lraspicam_cv



#Run valgrind on the project
#valgrind : fec
#	@valgrind --leak-check=yes --show-leak-kinds=all ./fec input_binary -f output.txt

#Clean the project of all object file.
clean:
	@rm -f $(SOURCES_DIR)/*.o
	@rm -f $(TESTS_DIR)/*.o
	@rm -f *.o
	@rm -f fec
	@rm -f bin/*.o

#Clean the OBJ_DIR directory
clean_obj:
	@rm -f $(OBJ_DIR)/*

.PHONY: clean clean_obj tests test_%
