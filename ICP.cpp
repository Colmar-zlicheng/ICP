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

    PointXYZ center_before;
    PointXYZ center_after;
    vector<PointXYZ> points_before;
    vector<PointXYZ> points_after;
    // read points from pcd file and pretreat data
    center_before = readPCD(points_before, data_before, 11);
    center_after = readPCD(points_after, data_after, 13);

    // compute running time
    clock_t end_time = clock();
    cout << "Running Time: " << static_cast<double>(end_time - start_time) / CLOCKS_PER_SEC << "s" << endl;

    return 0;
}
