#include <iostream>
#include <string>
#include <time.h>
#include "dataloader.h"
using namespace std;

int main()
{
    // get start time
    clock_t start_time = clock();

    string data_before, data_after;
    data_before = "data/points_before.pcd";
    data_after = "data/points_after.pcd";

    vector<PointXYZ> points_before;
    vector<PointXYZ> points_after;
    // read points from pcd file and pretreat data
    readPCD(points_before, data_before, 11);
    readPCD(points_after, data_after, 13);

    // double center_before[3] = {0}, center_after[3] = {0};
    // double *x = Centroid(data_before, data_after);
    // for (int i = 0; i < 3; i++)
    // {
    //     center_before[i] = x[i];
    //     center_after[i] = x[i + 3];
    // }
    // delete[] x;
    // cout << center_before[0] << " " << center_before[1] << " " << center_before[2] << endl;
    // cout << center_after[0] << " " << center_after[1] << " " << center_after[2] << endl;

    // compute running time
    clock_t end_time = clock();
    cout << "Running Time: " << static_cast<double>(end_time - start_time) / CLOCKS_PER_SEC << "s" << endl;

    return 0;
}
