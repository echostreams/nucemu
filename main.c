#include <stdio.h>
#include <unistd.h>

int main(int argc, char** argv)
{
	int i = 0x800;
	printf("Hello World! %08x\n", i);
	return 0;
}