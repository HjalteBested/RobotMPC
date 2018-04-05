# When clock skew appears uncomment this line
# $(shell find . -exec touch {} \;)

TARGET_EXEC := robotMPC.out

BUILD_DIR := ./build
SRC_DIRS  := ./src

CC = g++
#CFLAGS = -c -Wall -O1 $(shell pkg-config --cflags opencv gsl eigen3)
CFLAGS = -c -Wall -O1 $(shell pkg-config --cflags eigen3)

LDFLAGS = -L /usr/local/lib 
#LIBS = -lpthread $(shell pkg-config --libs opencv gsl eigen3)
LIBS = -lpthread $(shell pkg-config --libs eigen3)

SRCS := $(shell	find $(SRC_DIRS)	-name	*.cpp	-or	-name	*.c	-or	-name	*.s)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

#MAV_FLAG := -I mavlink/include/mavlink/v2.0
#CPPFLAGS := $(MAV_FLAG) -MMD -MP

$(TARGET_EXEC): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS) $(LIBS)
	$(MKDIR_P) ./simData
# assembly
$(BUILD_DIR)/%.s.o: %.s
	$(MKDIR_P) $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@ 

# c source
$(BUILD_DIR)/%.c.o: %.c
	$(MKDIR_P) $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

# c++ source
$(BUILD_DIR)/%.cpp.o: %.cpp
	$(MKDIR_P) $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@



.PHONY: clean doc c mrc

c :
	make -C ./src_c_connector

mrc :
	make -C ./src_c_connector mrc

clean:
	$(RM) -r $(BUILD_DIR) $(TARGET_EXEC) ./Documentation ./simData
	make -C ./src_c_connector clean
doc:
	doxygen Doxyfile

-include $(DEPS)

MKDIR_P := mkdir -p