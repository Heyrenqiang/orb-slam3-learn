#ifndef __DATAIO_H__
#define __DATAIO_H__
#include <istream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
using namespace std;
class DataIO{
public:
    DataIO();
    void Read_image(vector<string> &img_names,vector<double> &timestamps);
    void Set_image_dir(string dir);
    void Set_image_timestamp_dir(string dir);

private:
    string ms_image_dir;
    string ms_image_timestamp_dir;
};
#endif // __DATAIO_H__