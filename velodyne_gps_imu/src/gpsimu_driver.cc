#include "gpsimu_driver.h"

#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/Temperature.h>

GpsImuDriver::GpsImuDriver():nh_("~")
{
  // Initialize
  udp_socket_ = 0;
  is_connected_ = false;

  // Advertise topics, read parameters
  std::cout << "advertising topics" << std::endl;
  nmea_pub = nh_.advertise<std_msgs::String>("nmea_sentence",10);
  imu_pub = nh_.advertise<geometry_msgs::TwistStamped>("imu_data",100);
  gpstime_pub = nh_.advertise<std_msgs::Time>("gpstime",10);
  temperature_pub = nh_.advertise<sensor_msgs::Temperature>("temperature",10);
  nh_.param("device_ip",devip_,std::string(""));
  nh_.param("udpport",udpport_,8308);

  // Start listening at UDP port
  bind(udpport_);
}

//-----------------------------------------------------------------------------
bool GpsImuDriver::bind(const int udp_port)
{
  if( isConnected() )
    disconnect();

  try
  {
    // Bind socket
      udp_socket_ = new boost::asio::ip::udp::socket(io_service_, boost::asio::ip::udp::v4());
      udp_socket_->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 8308));
      // Start async reading
      asyncReceiveFrom();
      io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
      is_connected_ = true;
  }
  catch (std::exception& e)
  {
      std::cerr << "Exception: " <<  e.what() << std::endl;
      return false;
  }
  std::cout << "Receiving Velodyne IMU data at local UDP port " << udp_port << "." << std::endl;

  return true;
}

//-----------------------------------------------------------------------------
void GpsImuDriver::asyncReceiveFrom()
{
  udp_socket_->async_receive_from(boost::asio::buffer(&udp_buffer_[0],udp_buffer_.size()), udp_endpoint_,
                                  boost::bind(&GpsImuDriver::handleSocketRead, this,
                                              boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

//-----------------------------------------------------------------------------
void GpsImuDriver::handleSocketRead(const boost::system::error_code &error, std::size_t bytes_transferred)
{
  if (!error )
  {
    if( !devip_.empty() && udp_endpoint_.address().to_string() != devip_ )
    {
      asyncReceiveFrom();
      return;
    }

    // Handle packet
    velodyne_packet_structs::VelodynePositioningPacketRaw vppr;
    std::memcpy(&vppr,&udp_buffer_[0],sizeof(vppr));
    handlePacket(vppr);
    asyncReceiveFrom();
  }
  else
  {
      if( error.value() != 995 )
      {
        std::cerr << "ERROR: " << "data connection error: " << error.message() << "(" << error.value() << ")" << std::endl;
        std::cout << "Trying to rebind socket" << std::endl;
        if( bind(udpport_) )
          std::cout << "Rebind sucessful" << std::endl;
        else
          std::cerr << "" << std::endl;
       }
  }
  last_data_time_ = std::time(0);
}

//-----------------------------------------------------------------------------
bool GpsImuDriver::handlePacket(velodyne_packet_structs::VelodynePositioningPacketRaw &vppr)
{
  auto convert12bit2int16 = [] (u_int16_t v) -> int16_t
  {
      v = v & 0x0fff;
      int16_t r=v;
      if(r>2047)
          r=-((~(r--)) & 0x0fff);
      return r;
  };

  auto explode = [] (const std::string& text, const std::string& separators) -> std::vector<std::string>
  {
    std::vector<std::string> words;
    size_t n     = text.length ();
    size_t start = 0;
    size_t stop = text.find_first_of (separators);
    if (stop > n) stop = n;

    while (start <= n)
    {
            words.push_back (text.substr (start, stop-start));
            start = stop+1;
            stop = text.find_first_of (separators, start);
            if (stop > n) stop = n;
    }
    return words;
  };


  auto nmeaToUnixTime = [explode] (const std::string& nmea_sentence) -> u_int32_t
  {
      std::vector< std::string > words = explode(nmea_sentence,",");
      if(words.size() < 10 )
          return -1;
      uint32_t hour = atoi(words[1].substr(0,2).c_str());
      uint32_t min = atoi(words[1].substr(2,2).c_str());
      uint32_t sec = atoi(words[1].substr(4,2).c_str());

      uint32_t day = atoi(words[9].substr(0,2).c_str());
      uint32_t mon = atoi(words[9].substr(2,2).c_str());
      uint32_t year = atoi(words[9].substr(4,2).c_str());

      time_t rawtime;
      struct tm * timeinfo;
      time ( &rawtime );
      timeinfo = localtime ( &rawtime );
      timeinfo->tm_year = year + 100;
      timeinfo->tm_mon = mon - 1;
      timeinfo->tm_mday = day;
      mktime ( timeinfo );
      int yday = timeinfo->tm_yday;

      year += 100;
      uint32_t unix_time = sec + min*60 + hour*3600 + yday*86400 + (year-70)*31536000 + ((year-69)/4)*86400 - ((year-1)/100)*86400 + ((year+299)/400)*86400;
      return unix_time;
  };

  velodyne_packet_structs::VelodynePositioningPacket vpp;

  for( int i=0; i<3; i++ )
  {
      vpp.gyro_temp_accel[i].gyro = convert12bit2int16(vppr.gyro_temp_accel[i].gyro) * 0.09766;
      vpp.gyro_temp_accel[i].temp = convert12bit2int16(vppr.gyro_temp_accel[i].temp) * 0.1453+25;
      vpp.gyro_temp_accel[i].accel_x = convert12bit2int16(vppr.gyro_temp_accel[i].accel_x) * 0.001221;
      vpp.gyro_temp_accel[i].accel_y = convert12bit2int16(vppr.gyro_temp_accel[i].accel_y) * 0.001221;
  }

  const float earth_gravity = 9.80665;

  vpp.gps_timestamp = vppr.gps_timestamp/1000000.0;
  vpp.gyro_temp_accel_xyz[0].gyro = vpp.gyro_temp_accel[1].gyro;
  vpp.gyro_temp_accel_xyz[0].temp = vpp.gyro_temp_accel[1].temp;
  vpp.gyro_temp_accel_xyz[0].accel_x = -vpp.gyro_temp_accel[0].accel_y * earth_gravity;
  vpp.gyro_temp_accel_xyz[0].accel_y = vpp.gyro_temp_accel[2].accel_x * earth_gravity;

  vpp.gyro_temp_accel_xyz[1].gyro = vpp.gyro_temp_accel[0].gyro;
  vpp.gyro_temp_accel_xyz[1].temp = vpp.gyro_temp_accel[0].temp;
  vpp.gyro_temp_accel_xyz[1].accel_x = -vpp.gyro_temp_accel[1].accel_y * earth_gravity;
  vpp.gyro_temp_accel_xyz[1].accel_y = -vpp.gyro_temp_accel[2].accel_y * earth_gravity;

  vpp.gyro_temp_accel_xyz[2].gyro = -vpp.gyro_temp_accel[2].gyro;
  vpp.gyro_temp_accel_xyz[2].temp = vpp.gyro_temp_accel[2].temp;
  vpp.gyro_temp_accel_xyz[2].accel_x = vpp.gyro_temp_accel[0].accel_x * earth_gravity;
  vpp.gyro_temp_accel_xyz[2].accel_y = vpp.gyro_temp_accel[1].accel_x * earth_gravity;

  std_msgs::String nmea_sentence_msg;
  std::string nmea_sentence(vppr.nmea_sentence);

  nmea_sentence = nmea_sentence.substr(0,nmea_sentence.length()-2);
  nmea_sentence_msg.data = nmea_sentence;
  nmea_pub.publish(nmea_sentence_msg);

  double nmeaUnixTime = nmeaToUnixTime(nmea_sentence);

  nmeaUnixTime += (vppr.gps_timestamp%1000000)/1000000.0;
  std_msgs::Time gpstime_msg;
  gpstime_msg.data = ros::Time(nmeaUnixTime);
  gpstime_pub.publish(gpstime_msg);

  geometry_msgs::TwistStamped imumsg;
  imumsg.header.frame_id="/velodyne:"+devip_;
  imumsg.header.stamp = ros::Time::now();

  imumsg.twist.linear.x = (vpp.gyro_temp_accel_xyz[0].accel_x+vpp.gyro_temp_accel_xyz[0].accel_y)/2.0;
  imumsg.twist.linear.y = (vpp.gyro_temp_accel_xyz[1].accel_x+vpp.gyro_temp_accel_xyz[1].accel_y)/2.0;
  imumsg.twist.linear.z = (vpp.gyro_temp_accel_xyz[2].accel_x+vpp.gyro_temp_accel_xyz[2].accel_y)/2.0;
  imumsg.twist.angular.x = vpp.gyro_temp_accel_xyz[0].gyro;
  imumsg.twist.angular.y = vpp.gyro_temp_accel_xyz[1].gyro;
  imumsg.twist.angular.z = vpp.gyro_temp_accel_xyz[2].gyro;
  imu_pub.publish(imumsg);

  for( std::size_t i=0; i<3; i++ )
  {
    sensor_msgs::Temperature tmpmsg;
    std::stringstream ss;
    ss << "/velodyne:" << devip_ << i;
    tmpmsg.header.frame_id = ss.str();
    tmpmsg.header.stamp = ros::Time::now();
    tmpmsg.temperature = vpp.gyro_temp_accel_xyz[i].temp;
    temperature_pub.publish(tmpmsg);
  }
}

//-----------------------------------------------------------------------------
void GpsImuDriver::disconnect()
{
    is_connected_ = false;
    try
    {
        if( udp_socket_ )
            udp_socket_->close();
        io_service_.stop();
        if( boost::this_thread::get_id() != io_service_thread_.get_id() )
            io_service_thread_.join();
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " <<  e.what() << std::endl;
    }
}

//-----------------------------------------------------------------------------
GpsImuDriver::~GpsImuDriver()
{
  disconnect();
  delete udp_socket_;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_gpsimu_driver", ros::init_options::AnonymousName);
    GpsImuDriver* gid = new GpsImuDriver();
    ros::spin();
    delete gid;
    return 0;
}
