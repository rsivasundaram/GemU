#ifndef __GEMU_FAULTS_HH__
#define __GEMU_FAULTS_HH__

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

void gemu_mult_to_div();
void gemu_disable_mult_to_div();
void gemu_add_to_sub();
void gemu_disable_add_to_sub();
void gemu_sub_to_add();
void gemu_disable_sub_to_add();
void gemu_mult_to_add();
void gemu_disable_mult_to_add();
void gemu_add_to_mult();
void gemu_disable_add_to_mult();
void gemu_add_to_div();
void gemu_disable_add_to_div();
void gemu_mult_to_sub();
void gemu_disable_mult_to_sub();
void gemu_sub_to_mult();
void gemu_disable_sub_to_mult();
void gemu_sub_to_div();
void gemu_disable_sub_to_div();
void gemu_disable_all_faults();


#endif  // __GEMU_FAULTS_HH__
