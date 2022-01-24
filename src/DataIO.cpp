#include "../include/DataIO.hpp"

// using namespace std;
DataIO::DataIO()
{
}
void DataIO::Read_image(vector<string> &img_names,vector<double> &timestamps)
{   
    ifstream fTimes;
    fTimes.open(ms_image_timestamp_dir.c_str());
    img_names.reserve(5000);
    timestamps.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            img_names.push_back(ms_image_dir + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            timestamps.push_back(t/1e9);

        }
    }  
}
void DataIO::Set_image_dir(string dir)
{
    ms_image_dir = dir;
}
void DataIO::Set_image_timestamp_dir(string dir) {
    ms_image_timestamp_dir = dir;
}