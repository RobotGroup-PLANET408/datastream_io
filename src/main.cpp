/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     main.cpp: This source file is used to test functions
 * @note
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "datastream.h"
#include "data_in.h"
#include "data_out.h"
#include "data_conv.h"

int main(int argc, char **argv)
{

    // 1. Open the txt file
    std::string filepath_in = "/home/leiwh/Research/Data/HubeiJD/20250612/03/ground-truth/proj_MEMS.txt";
    FILE *infile = fopen(filepath_in.c_str(), "rt");

    std::string filepath_out = "/home/leiwh/Research/Data/HubeiJD/20250612/03/ground-truth/proj_MEMS.ref";
    FILE *outfile = fopen(filepath_out.c_str(), "wt");

    // 2. Skip info lines
    char buffer[1024] = {'\0'};
    for (int i = 0; i < 0; i++)
        fgets(buffer, sizeof(buffer), infile);

    // 3. Extract GNSS solution data
    while (!feof(infile))
    {
        // clear old data and read new data
        memset(buffer, '\0', sizeof(buffer));
        fgets(buffer, sizeof(buffer), infile);

        // check the length of data
        int charnum = strlen(buffer);
        if (charnum <= 0)
            continue;

        double GPSWeek = 0;
        double GPSSecond = 0.0, LLH[3] = {0.0}, VENU[3] = {0.0}, Att[3] = {0.0};
        sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &GPSWeek, &GPSSecond, &LLH[0], &LLH[1], &LLH[2], &VENU[0], &VENU[1], &VENU[2], &Att[0], &Att[1], &Att[2]);

        if (GPSSecond - int(GPSSecond) != 0.0)
            continue;

        double XYZ[3] = {0.0}, VXYZ[3] = {0.0}, Azi[3] = {0.0};
        LLH[0] *= D2R, LLH[1] *= D2R;
        gnss_common::LLH2XYZ(LLH, XYZ);

        Eigen::Matrix3d R_nToe = gnss_common::ComputeRotMat_ENU2ECEF(LLH[0], LLH[1]);
        Eigen::Vector3d VENU_vec = Eigen::Vector3d(VENU);
        Eigen::Vector3d VXYZ_vec = R_nToe * VENU_vec;

        Att[0] *= D2R, Att[1] *= D2R, Att[2] *= D2R;
        if (Att[2] < 0.0)
        {
            Att[2] = -Att[2];
        }
        else
        {
            Att[2] = IPS_PI2 - Att[2];
        }
        Att[2] = IPS_PI2 - Att[2];

        fprintf(outfile, "%d %12.6f %15.9f %15.9f %15.9f %d %s %s %s %15.3f %15.3f %15.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %12.9f %12.9f %12.9f\n",
                int(GPSWeek), GPSSecond, LLH[0] * R2D, LLH[1] * R2D, LLH[2], 1, "Fixed", "2023/09/25", "3:01:59.00", XYZ[0], XYZ[1], XYZ[2],
                0.0, 0.0, 0.0, VXYZ_vec[0], VXYZ_vec[1], VXYZ_vec[2], VENU_vec[0], VENU_vec[1], VENU_vec[2], Att[2] * R2D, Att[1] * R2D, Att[0] * R2D);
    }

    fclose(infile);
    fclose(outfile);

    return 0;
}