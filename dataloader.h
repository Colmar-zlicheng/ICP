#include <iostream>
#include <string>
#include <fstream>
#include <vector>
using namespace std;

struct PointXYZ
{
    double x, y, z;
};

// Read 3D points in .pcd file
PointXYZ readPCD(vector<PointXYZ> &cloud, const string filename, const int header_lines, bool Decentroided)
{
    ifstream file(filename);
    if (file.bad())
    {
        cout << "Failed to open file: " << filename << endl;
        exit(1);
    }

    char Header[header_lines][1024]; // Save header in pcd file(first 11 lines)
    int Points_num;                  // Number of points
    string data_type;                // Stored type: ascii or binary
    string data_columns_type;        // Colmues: x y z

    cout << "Reading header of " << filename << " <<< ";
    for (int i = 0; i < header_lines; i++)
    {
        file.getline(Header[i], 1024);
        // cout << Header[i] << endl;

        // FIELDS x y z rgb
        if (i == header_lines - 9)
        {
            string col_t = Header[i];
            size_t pos = col_t.find("FIELDS");
            size_t size = col_t.size();
            data_columns_type = col_t.substr(pos + 7, 5);
            // cout << "data_columns_type:" << data_columns_type << endl;
        }

        // Points num
        if (i == header_lines - 2)
        {
            string P_n = Header[i], Points_Str;
            size_t pos = P_n.find("POINTS");
            size_t size = P_n.size();
            Points_Str = P_n.substr(pos + 7, size);
            Points_num = atoi(Points_Str.c_str());
            cout << "Points:" << Points_num << endl;
        }

        // Data type: ascii or binary
        if (i == header_lines - 1)
        {
            string d_t = Header[i], DATA_SIZE;
            size_t pos = d_t.find("DATA");
            size_t size = d_t.size();
            data_type = d_t.substr(pos + 5, size);
            // std::cout << "data_type:" << data_type << std::endl;
        }
    }

    cout << "Reading points of " << filename << endl;
    double tmp;
    PointXYZ p;
    PointXYZ center;
    center.x = center.y = center.z = 0;

    if ((data_columns_type == "x y z") && (data_type == "ascii"))
    {
        int i = 0;
        // while (!file.eof())
        while (i < Points_num)
        {
            file >> p.x >> p.y >> p.z >> tmp;
            i++;
            // compute center point(sum)
            center.x += p.x;
            center.y += p.y;
            center.z += p.z;
            // if (file.peek() == EOF)
            // {
            //     break;
            // }
            cloud.push_back(p);
        }
    }
    else
    {
        cout << "Don't support data_columns_type: " << data_columns_type << " or data_type: " << data_type << endl;
        exit(1);
    }

    // compute center point(average)
    center.x = center.x / double(Points_num);
    center.y = center.y / double(Points_num);
    center.z = center.z / double(Points_num);

    // Compute Decentroided Point Clouds
    if (Decentroided)
    {
        cout << "Compute Decentroided Point Clouds" << endl;
        for (int i = 0; i < Points_num; i++)
        {
            cloud[i].x -= center.x;
            cloud[i].y -= center.y;
            cloud[i].z -= center.z;
        }
    }
    cout << endl;
    return center;
}

// double *save_center(double x[])
// {
//     // center[6] = before[0], before[1], before[2], after[0], after[1], after[2]
//     double *center = new double[6];
//     center[0] = x[0];
//     center[1] = x[1];
//     center[2] = x[2];
//     center[3] = x[3];
//     center[4] = x[4];
//     center[5] = x[5];
//     return center;
// }

// double *Centroid(string data_before, string data_after)
// {
//     double *out;
//     double center[6] = {0};
//     center[0] = 0.111;
//     center[1] = 0.222;
//     center[2] = 0.333;
//     center[3] = 0.444;
//     center[4] = 0.555;
//     center[5] = 0.666;

//     out = save_center(center);
//     return out;
// }
