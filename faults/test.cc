#include "gemu_faults.hh"

int main(){	
	int x=24,y=8,z;
	gemu_add_to_sub();
	z=x+y;
	
	printf("1 z: %d\n", z);
	gemu_disable_add_to_sub();
	z=x+y;	
	
	printf("2 z: %d\n", z);
	gemu_sub_to_add();
	z=x-y;
	
	printf("3 z: %d\n", z);
	gemu_disable_sub_to_add();
	z=x-y;
	
	printf("4 z: %d\n", z);
	gemu_mult_to_add();
	z=x*y;
	
	printf("5 z: %d\n", z);
	gemu_disable_mult_to_add();
	z=x*y;
	printf("6 z: %d\n", z);
	gemu_add_to_mult();
	z=x+y;
	printf("7 z: %d\n", z);
	gemu_disable_add_to_mult();
	z=x+y;
	printf("8 z: %d\n", z);
	
	//gemu_div_to_add();
	z=x/y;
	printf("z: %d\n", z);

	gemu_add_to_div();
	z=x+y;
	printf("9 z: %d\n", z);
	gemu_disable_add_to_div();
	z=x+y;
	printf("10 z: %d\n", z);
	gemu_mult_to_sub();
	z=x*y;
	printf("11 z: %d\n", z);
	gemu_disable_mult_to_sub();
	z=x*y;
	printf("12 z: %d\n", z);
	gemu_mult_to_div();
	z=x*y;
	printf("13 z: %d\n", z);
	gemu_disable_mult_to_div();
	z=x*y;
	printf("14 z: %d\n", z);
	gemu_sub_to_mult();
	z=x-y;
	printf("15 z: %d\n", z);
	gemu_disable_sub_to_mult();
	z=x-y;
	printf("16 z: %d\n", z);
	
	//gemu_div_to_sub();
	z=x/y;
	printf("z: %d\n", z);

	gemu_sub_to_div();
	z=x-y;	
	printf("17 z: %d\n", z);
	gemu_disable_sub_to_div();
	z=x-y;	
	printf("18 z: %d\n", z);

	gemu_mult_to_sub();
	gemu_disable_all_faults();
	printf("19 z: %d\n", z);
	z=x*y;
	
	gemu_sub_to_mult();
	gemu_mult_to_div();
	gemu_add_to_div();
	z=x-y;
	printf("20 z: %d\n", z);
	z=x*y;
	printf("21 z: %d\n", z);
	z=x+y;
	printf("22 z: %d\n", z);
	
	gemu_disable_all_faults();
	
	gemu_disable_sub_to_div();
	z=x-y;
	printf("23 z: %d\n", z);
	fclose(pFile);
	
}

