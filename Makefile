# -----------------------------------------------------------------------------------
#Variables
# Arguments for the compilation
CXX = g++
FLAGS:=-Wall -O3 -g
LIBS:=-llgpio -lm -lpthread -ldxl_sbc_c -lsl_lidar_sdk -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_videoio -lopencv_imgcodecs -lopencv_aruco

# Directories 
HEADERS_DIR:=headers 
SOURCES_DIR:=src
TESTS_DIR:=tests
OBJ_DIR:=bin

# List of cpp files
SOURCES = $(wildcard $(SOURCES_DIR)/*.cpp)
TESTS = $(wildcard $(TESTS_DIR)/*.cpp)

# List of .o files
SOURCES_OBJ = $(addprefix $(OBJ_DIR)/,$(notdir $(SOURCES:.cpp=.o)))
SOURCES_OBJ_NO_SHARED = $(SOURCES_OBJ:$(OBJ_DIR)/action%=)
SOURCES_OBJ_NO_SHARED := $(SOURCES_OBJ_NO_SHARED:$(OBJ_DIR)/shared_variables.o=)
SOURCES_OBJ_NO_SHARED := $(SOURCES_OBJ_NO_SHARED:$(OBJ_DIR)/decision.o=)
SOURCES_OBJ_NO_SHARED := $(SOURCES_OBJ_NO_SHARED:$(OBJ_DIR)/lidarTop.o=)

# -----------------------------------------------------------------------------------
# Rules
all: main.cpp $(SOURCES_OBJ)
	@$(CXX) -I$(HEADERS_DIR) $(FLAGS) $^ -o exe_botlightyear $(LIBS)

run: all
	@./exe_botlightyear

valgrind: all
	@valgrind --leak-check=yes --track-origins=yes ./exe_botlightyear

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


#Do an individual test, without shared variables
test_%: $(TESTS_DIR)/test_%.cpp $(SOURCES_OBJ_NO_SHARED)
	@$(CXX) -I$(HEADERS_DIR) $(FLAGS) $^ -o $@ $(LIBS)
	@./$@
	@rm $@

#Do an individual test with shared variables
test_action_%: $(TESTS_DIR)/test_action_%.o $(SOURCES_OBJ)
	@./$<
	@rm $<

#Do an individual test
valtest_%: $(TESTS_DIR)/test_%.o $(SOURCES_OBJ)
	@valgrind --leak-check=yes  --track-origins=yes ./$< 
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

lidarTop: lidarTop_program 
	@./bin/lidarTop 
	@rm ./bin/lidarTop 

val_lidarTop: lidarTop_program 
	@valgrind --leak-check=yes --track-origins=yes ./bin/lidarTop 
	@rm ./bin/lidarTop 
 
# Run the lidar program 
lidarTop_program: 
	@g++ ./tests/test_lidarTop.cpp ./src/lidarTop.cpp ./src/lidar.cpp -o ./bin/lidarTop -lsl_lidar_sdk -lpthread -L./rplidar/output/Linux/Release -I./rplidar/sdk/include -I./rplidar/sdk/src -I$(HEADERS_DIR)

#Run valgrind on the project
#valgrind : fec
#	@valgrind --leak-check=yes --show-leak-kinds=all ./fec input_binary -f output.txt

#Clean the project of all object file.
clean:
	@rm -f $(SOURCES_DIR)/*.o
	@rm -f $(TESTS_DIR)/*.o
	@rm -f *.o
	@rm -f exe_botlightyear
	@rm -f bin/*.o

#Clean the OBJ_DIR directory
clean_obj:
	@rm -f $(OBJ_DIR)/*

.PHONY: clean clean_obj tests test_%
