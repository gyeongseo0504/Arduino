#include <stdio.h>
/*
void swap(int* x, int* y)
{
	int temp = *x;
	*x = *y;
	*y = temp;
}*/

void sub(int b[], int n);
int main()
{

	/*
	char* pc;
	int* pi;
	double* pd;

	pc = (char*)10000;
	pi = (int*)10000;
	pd = (double*)10000;
	printf("pc=%u, pc+1=%u, pc+2= %u\n", pc, pc + 1, pc + 2);
	printf("pi=%u, pi+1=%u, pi+2= %u\n", pi, pi + 1, pi + 2);
	printf("pd=%u, pd+1=%u, pd+2= %u\n", pd, pd + 1, pd + 2);
	
	return 0;

	int i = 10;
	int* pi = &i;

	printf("i=%d, pi=%p\n", i, pi);

	(*pi)++;
	printf("i=%d, pi=%p\n", i, pi);

	*pi++;
	printf("i=%d, pi=%p\n", i, pi);
	return 0;
	

	int data = 0x0A0B0C0D;
	char* pc;
	int i;

	pc = (char*)&data;
	for (i = 0; i < 4; i++)
		printf("*(pc + %d) = %02X\n", i, *(pc + i));
	return 0;

	

		int a = 100, b = 200;
		printf("a = %d, b = %d\n", a, b);
		swap(&a, &b);
		printf("a = %d, b = %d\n", a, b);
		
	int a[] = { 10 , 20 , 30, 40, 50 };

	printf("a = %u\n", a);
	printf("a + 1 = %u\n", a + 1);
	printf("a = %d\n", a);
	printf("(a+1) = %d\n", (a+1));



	printf("&a[0] = %u\n",&a[0]);
	printf("&a[1] = %u\n",&a[1]);
	printf("&a[2] = %u\n",&a[2]);
	return 0;

	int a[] = { 10 , 20 , 30, 40, 50 };
	int* p;

	p = a;
	printf("a[0] = %d a[1] = %d a[2] = %d\n", a[0], a[2], a[3]);
	printf("p[0] = %d p[1] = %d p[2] = %d\n", p[0], p[2], p[3]);

	p[0] = 60;
	p[1] = 70;
	p[2] = 80;

	printf("a[0] = %d a[1] = %d a[2] = %d\n", a[0], a[2], a[3]);
	printf("p[0] = %d p[1] = %d p[2] = %d\n", p[0], p[2], p[3]);
	return 0;
	*/

	int a[3] = { 1,2,3 };

	printf("%d %d %d\n", a[0], a[1], a[2]);
	sub(a, 3);
	printf("%d %d %d\n", a[0], a[1], a[2]);

	return 0;
}
void sub(int b[], int n)
{
	b[0] = 4;
	b[1] = 5;
	b[2] = 6;
}