# The first target in a makefile is always the default target.
# If you run "make" without any arguments, it will execute the default target.
# In this case, the default target named "all" has one dependency named "hello".
all: hello

# The "hello" target depends on a single file, hello.cpp.
# If hello.cpp changed since the last time "make" was called, it will run the
# command on the following line.
# This target compiles the code and outputs an executable with the same name.
# You can run this by typing "make hello".
hello: hello.cpp
	g++ -Wall -o hello hello.cpp -lwiringPi

# A third target named "clean" was added for easily removing build files from
# the folder.
# This optional target has no dependencies and runs the remove command.
# You can run this by typing "make clean".
clean:
	rm -f *.o hello

#See http://mrbook.org/blog/tutorials/make/ for more information.