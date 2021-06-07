#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sstream>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <boost/program_options.hpp>

#include <iostream>
#include <algorithm>
#include <iterator>
#include "sensor_msgs/PointCloud2.h"

using namespace boost;
namespace po = boost::program_options;
using namespace std;

template<class T>
ostream& operator<<(ostream& os, const vector<T>& v)
{
    copy(v.begin(), v.end(), ostream_iterator<T>(os, " "));
    return os;
}


class converter{
public:
    typedef std::tuple<sensor_msgs::PointCloud2, ros::Time, std::string> message;
    std::map<int,message> sequenced_pointcloud;

    // Load all messages in sencept bag into a map structure that can be indexed later for synchronization
    void LoadSencept(const std::string& path, const int sencept_seq_offset){
        int seq = 0;

        rosbag::Bag rosbag(path, rosbag::bagmode::Read);

        rosbag::View view(rosbag);


        foreach(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::PointCloud2ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
            if (s != NULL)
                sequenced_pointcloud[sencept_seq_offset+(seq++)] = std::make_tuple(*s, m.getTime(), m.getTopic());
        }
        cout<<"Found: "<<seq<<" sencept point clouds"<<endl;
        rosbag.close();
    }

    converter(const std::string& dir, const std::vector<std::string>& bags, const std::string& sencept_bag, const std::string& output_bag, const int sencept_seq_offset){

        LoadSencept(dir+sencept_bag, sencept_seq_offset);
        rosbag::Bag output_rosbag(dir+output_bag, rosbag::bagmode::Write);
        int seq = 0;


        for(auto bag = bags.begin() ; bag!=bags.end() ; bag++){
            int i = std::distance(bags.begin(), bag);
            std::string bag_file = dir + (*bag);
            rosbag::Bag rosbag;
            rosbag.open(bag_file, rosbag::bagmode::Read);
            rosbag::View view(rosbag);

            foreach(rosbag::MessageInstance const m, view)
            {


                //std_msgs::String::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
                if(m.getTopic()=="/radar_frames"){
                    cout<<"seq: "<<seq<<endl;
                    auto synced_pointcloud = sequenced_pointcloud.find(seq++);
                    if(synced_pointcloud!=sequenced_pointcloud.end())
                        output_rosbag.write(std::get<2>(synced_pointcloud->second), m.getTime(), std::get<0>(synced_pointcloud->second)); //make use of timestamp here
                }
                else
                    output_rosbag.write(m.getTopic(), m.getTime() , m);
            }

            rosbag.close();

        }
        output_rosbag.close();
    }


};
int main(int argc, char **argv)
{

    std::vector<std::string> bag_names;
    std::string dir, output_bag, sencept_bag;
    int sencept_to_oru_offset;
    ros::init(argc, argv, "converter");
    ros::NodeHandle n;
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")
            ("dir", po::value<std::string>(&dir)->default_value(""))
            ("sencept_bag", po::value<std::string>(&sencept_bag)->default_value(""))
            ("output_bag", po::value<std::string>(&output_bag)->default_value(""))
            ("sencept_to_oru_offset", po::value<int>(&sencept_to_oru_offset)->default_value(0))
            ("files", po::value<std::vector<std::string>>()->multitoken(), "input file");



    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }
    std::vector<std::string> bag_files;
    if (!vm["files"].empty() && (bag_files = vm["files"].as<vector<std::string> >()).size() >0 ) {
        cout<<"Loading: "<<bag_files<<endl;
    }
    else {
        cout<<"No bag files provided"<<endl;
        exit(0);
    }
    if(sencept_bag==""){
        cout<<"No sencept bag provided"<<endl;
        exit(0);
    }
    if(output_bag==""){
        cout<<"No output bag provided"<<endl;
        exit(0);
    }


    cout<<"From directory: "<<dir<<endl;


    converter c(dir, bag_files, sencept_bag, output_bag, sencept_to_oru_offset);




    return 0;
}
