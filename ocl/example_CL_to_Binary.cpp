#include<stdio.h>
#include<pcl/ocl/utils/ocl_manager.h>
#include<CL/cl.hpp>
using namespace cl;
#include <utility>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main(int argc, char ** argv)
{

OCLManager test,*test1;
test1 = test.getInstance();

Context context = test1->getContext();
CommandQueue queue = test1->getQueue();

// Read source file
std::string sourceFile("median_filter.cl");
std::string sourceFile1("median_filter.clbin");
Program program = test1->buildProgramFromSource(sourceFile);
test1->saveBinary(&program, sourceFile1);
return 0;
}
