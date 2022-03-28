#include <stdio.h>

int main(int argc, char* argv[])
{
	int array[2];
	printf("Calculating Program...\n");
	
	printf("Calcuating %s and %s...\n\n", argv[1], argv[2]);
	
	array[0] = (int)*argv[1]-'0';
	array[1] = (int)*argv[2]-'0';
	
	printf("%d + %d = %d\n", array[0], array[1], array[0] + array[1]);
	printf("%d - %d = %d\n", array[0], array[1], array[0] - array[1]);
	printf("%d * %d = %d\n", array[0], array[1], array[0] * array[1]);
	printf("%d / %d = %d\n", array[0], array[1], array[0] / array[1]);
 
	
 
	return 0;
 
}
