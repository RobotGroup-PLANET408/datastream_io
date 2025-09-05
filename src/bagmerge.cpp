
/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     bagmerge.cpp: This source file is used to merge multiple bag files to one bag file
 * @note
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "datastream.h"
#include "data_in.h"

int main(int argc, char **argv)
{
    ROS_INFO("Launch the node to merge bagfiles.");

    ///< 1. Check input arguments
    if (argc < 2)
    {
        ROS_ERROR("The number of input arguments is wrong. Please check.");
        return 0;
    }

    ///< 2. Read configutation file
    ROS_INFO("Start to read configuration file: %s.", argv[1]);
    dataio_common::Configutation config;
    if (dataio_common::Read_ConfigFile(argv[1], config) == false)
    {
        ROS_ERROR("The number of input arguments is wrong. Please check.");
        return 0;
    }

    ///< 3. Open the bag to write
    std::string bag_outfilepath = config.output_filepath + "/mergedata.bag";
    rosbag::Bag bag_out(bag_outfilepath, rosbag::bagmode::Write);
    if (!bag_out.isOpen())
    {
        ROS_ERROR("Fail to open the output bag file: %s", bag_outfilepath.c_str());
        return 0;
    }

    ///< 4. Merge each rosbag
    for (int i = 0; i < config.rosbag_filename.size(); i++)
    {
        // get filepath
        std::string bag_infilepath = config.rosbag_filepath + "/" + config.rosbag_filename.at(i);

        ROS_INFO("Start to process the rosbag: %s.", bag_infilepath.c_str());

        try
        {
            // open the bag file to extract data
            rosbag::Bag bag_in(bag_infilepath, rosbag::bagmode::Read);
            rosbag::View view(bag_in);

            // write the message to output bag file
            for (const rosbag::MessageInstance &m : view)
            {
                bag_out.write(m.getTopic(), m.getTime(), m);
            }

            bag_in.close();
        }
        catch (const rosbag::BagException &e)
        {
            ROS_ERROR("Fail to open bag file: %s to read.", e.what());
            continue;
        }
    }

    // Remember to close bag file
    bag_out.close();

    ROS_INFO("The merge node has completed execution.");

    return 1;
}
