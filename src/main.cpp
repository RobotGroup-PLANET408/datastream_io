
/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     main.cpp: This source file is used to test functions
 * @note
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "datastream.h"
#include "data_in.h"
#include "data_out.h"
#include "data_conv.h"

int main()
{
    // step 1: extract data
    std::string filename = "/home/leiwh/Research/Document/Paper02/Version-02/Data/2023-09-25-03-01-32_maximal/result/ForPaper02/GICILIB/RTK/GPS+GAL/04_500_1.0/gici_result.txt";
    std::list<dataio_common::Solution_INS> sol_datas;
    dataio_common::Extract_INSSolution_TXTFile_MAIN(filename.c_str(), sol_datas, dataio_common::dataformat::GICILIB_Format, dataio_common::timesystem::Linux_time, 0);

    // step 2: interpolate
    std::list<dataio_common::Solution_INS> sol_datas_interpolated;
    dataio_common::Interpolate_INSSolutionData(sol_datas, sol_datas_interpolated);

    // step 3: write data
    filename = "/home/leiwh/Research/Document/Paper02/Version-02/Data/2023-09-25-03-01-32_maximal/result/ForPaper02/GICILIB/RTK/GPS+GAL/04_500_1.0/gici_result.ftf";
    dataio_common::Write_INSolution_TXTFile_MAIN(filename.c_str(), sol_datas_interpolated, dataio_common::dataformat::RobotGVINS_Format);

    return 0;
}