#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <boost/program_options.hpp>
#include "libio/serial_protocol.h"
#include "libio/serial_com.h"
#include <unistd.h>
#include <ros/ros.h>

namespace po = boost::program_options;
serial_protocol::SPThrowHandler th;

po::options_description options("options");
void usage(char* argv[])
{
  std::cout << "usage: " << argv[0] << " [options]" << std::endl << options << std::endl;
}

bool handleCommandline(std::string& device, bool& verbose, bool& ignore_timeout, std::string& device_filename,
                       std::string& sensor_filename, std::string& sensor_args, int argc, char* argv[])
{
  // default input device
  device = "/dev/ttyACM0";
  sensor_args = "";
  verbose = false;

  th.throw_at_timeout = true;
  th.throw_at_header_error = false;
  th.throw_at_did_error = false;
  th.throw_at_checksum_error = false;
  th.throw_at_size_error = false;
  th.throw_at_request_error = false;
  th.throw_at_failed_request = false;
  th.throw_at_send_error = true;

  // define processed options
  po::options_description inputs("input options");
  inputs.add_options()
    ("serial,s", po::value<std::string>(&device)->implicit_value(device), "serial input device")
    ("sensor_file", po::value<std::string>(&sensor_filename)->implicit_value(sensor_filename),"sensor type definition filename")
    ("device_file", po::value<std::string>(&device_filename)->implicit_value(device_filename), "device type definition filename");
  options.add_options()
    ("verbose,v", "activate verbosity")
    ("ignore_timeout", "do not quit at data timeout")
    ("quit_at_datagram_errors", "")
    ("quit_at_request_errors", "")
    ("sensor_args", po::value<std::string>(&sensor_args)->implicit_value(sensor_args), "extra args for the sensor parser ex: arg1=val1;arg2=val2")
    ("help,h", "Display this help message.");
  options.add(inputs);

  po::variables_map map;
  po::store(po::command_line_parser(argc, argv).options(options).run(), map);

  if (map.count("verbose"))
    verbose = true;
  if (map.count("ignore_timeout"))
    th.throw_at_timeout = false;
  if (map.count("quit_at_datagram_errors"))
  {
    th.throw_at_header_error = true;
    th.throw_at_did_error = true;
    th.throw_at_checksum_error = true;
    th.throw_at_size_error = true;
  }
  if (map.count("quit_at_request_errors"))
  {
    th.throw_at_request_error = true;
    th.throw_at_failed_request = true;
  }
  if (map.count("help"))
    return true;
  po::notify(map);
  return false;
}

bool bRun = true;
void mySigIntHandler(int sig)
{
  bRun = false;
}

int main(int argc, char** argv)
{
  std::string sSerial;
  std::string sDeviceFilename;
  std::string sSensorFilename;
  std::string sSensorArgs;
  bool bVerbose;
  bool bIgnoreTimeout;

  serial_protocol::SerialCom s;

  try
  {
    if (handleCommandline(sSerial, bVerbose, bIgnoreTimeout, sDeviceFilename, sSensorFilename, sSensorArgs, argc, argv))
    {
      usage(argv);
      return EXIT_SUCCESS;
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    usage(argv);
    return EXIT_FAILURE;
  }

  ros::init(argc, argv, "agni_serial_protocol_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("");

  // register Ctrl-C handler
  signal(SIGINT, mySigIntHandler);
  try
  {
    std::cout << "connecting to " << sSerial << "\n";
    s.connect(sSerial);
    s.setTimeOut(1000);
    s.setVerbose(bVerbose);
    std::cout << "connected\n";
    std::cout << "creating serial protocol with device_filename " << sDeviceFilename
              << " and sensor_filename " << sSensorFilename;
    if (sSensorArgs.length())
      std::cout << " passing following args: " << sSensorArgs << "\n";
    else
      std::cout << std::endl;
    serial_protocol::SerialProtocolBase p(&s, sDeviceFilename, sSensorFilename, sSensorArgs);
    p.verbose = bVerbose;
    p.set_throw_handler(th);
    std::cout << "initializing serial protocol\n";
    std::cout << "initializing ros\n";
    p.init_ros(nh);  // must be first to get diagnostic messages over ROS when initializing communication with the
                     // device
    std::cout << "initialized ros\n";
    if (p.init())
    {
      std::cout << "initialized\n";
      std::cout << "start streaming\n";
      p.start_streaming();
      std::cout << "streaming (press ctrl-c to quit)\n";
      while (bRun && ros::ok())  // loop until Ctrl-C
      {
        try
        {
          p.update();
          p.publish();
          // std::cout << ".";
        }
        catch (const std::exception& e)
        {
          std::cerr << e.what() << std::endl;
          break;
        }
      }
      p.stop_streaming();
    }
    else
      std::cout << "initialization failed\n";
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  std::cout << "disconnecting\n";
  s.disconnect();
  std::cout << "disconnected\n";
  return 0;
}
