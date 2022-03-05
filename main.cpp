#include "Calibration.h"

int main()
{
    string root_path = "../Image" ;
    string output_file = "../output.txt";
    tool t;
    t.calibration(root_path,output_file);
}