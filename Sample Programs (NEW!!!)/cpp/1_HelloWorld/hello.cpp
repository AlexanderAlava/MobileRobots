// This program prints hello world to the console every 500 milliseconds.
// Open the folder that contains this file. 
// Then click Tools > Open current folder in terminal.
// Enter the command "make" to build the executable and then enter "./hello" to execute it.
// You can use the app named Geany to modify C++ files, 
// but we recommend using the above commands to compile and execute the program.

#include <iostream>
#include <wiringPi.h>

using namespace std;

int main()
{
	cout << "Press Ctrl+C to exit." << endl;
	
	while (true)
	{
		cout << "Hello world!" << endl;
		delay(500);
	}
}
