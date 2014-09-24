#include "gemu_faults.hh"

void gemu_mult_to_div(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"1\n", 10);
	close(fp);
}

void gemu_disable_mult_to_div(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"513\n", 10);
	close(fp);
}


void gemu_add_to_sub(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"2\n", 10);
	close(fp);
}

void gemu_disable_add_to_sub(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"514\n", 10);
	close(fp);
}

void gemu_sub_to_add(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"4\n", 10);
	close(fp);
}

void gemu_disable_sub_to_add(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"516\n", 10);
	close(fp);
}

void gemu_mult_to_add(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"8\n", 10);
	close(fp);
}

void gemu_disable_mult_to_add(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"520\n", 10);
	close(fp);
}

void gemu_add_to_mult(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"16\n", 10);
	close(fp);
}

void gemu_disable_add_to_mult(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"528\n", 10);
	close(fp);
}

void gemu_disable_add_to_div(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"544\n", 10);
	close(fp);
}

void gemu_mult_to_sub(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"64\n", 10);
	close(fp);
}

void gemu_disable_mult_to_sub(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"576\n", 10);
	close(fp);
}


void gemu_sub_to_mult(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"128\n", 10);
	close(fp);
}

void gemu_disable_sub_to_mult(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"640\n", 10);
	close(fp);
}

void gemu_sub_to_div(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"256\n", 10);
	close(fp);
}

void gemu_disable_sub_to_div(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"768\n", 10);
	close(fp);
}


void gemu_disable_all_faults(){
	int fp;
	fp = open("/dev/chdev", O_WRONLY);
	write(fp,"0\n", 10);
	close(fp);
}
