#include "serial_protocol.h"
#include <agni_serial_protocol/Topology.h>
#include <agni_serial_protocol/TopologyECD.h>
#include "registered_sensors_impl.h"
#include "yaml-cpp/yaml.h"
#include <exception>
#include <stdexcept>
#include <cstring>
#include <string>
#include <sstream>
#include <iostream>
#include <utility>
#include <map>

#ifdef HAVE_ROS
#define sp_diag_msg(args, level)                                                                                       \
  do                                                                                                                   \
  {                                                                                                                    \
    std::stringstream __sstr;                                                                                          \
    uint8_t __level = level;                                                                                           \
    if (__level == diagnostic_msgs::DiagnosticStatus::ERROR)                                                           \
      __sstr << "ERROR: ";                                                                                             \
    if (__level == diagnostic_msgs::DiagnosticStatus::WARN)                                                            \
      __sstr << "WARNING: ";                                                                                           \
    __sstr << args;                                                                                                    \
    std::cerr << __sstr.str() << std::endl;                                                                            \
    update_diagnostic(level, __sstr.str());                                                                            \
  } while (0)
#define sp_error(args) sp_diag_msg(args, diagnostic_msgs::DiagnosticStatus::ERROR)
#define sp_warn(args) sp_diag_msg(args, diagnostic_msgs::DiagnosticStatus::WARN)
#else
#define sp_diag_msg(args, level)                                                                                       \
  do                                                                                                                   \
  {                                                                                                                    \
    std::stringstream __sstr;                                                                                          \
    __sstr << args;                                                                                                    \
    std::cerr << level << __sstr.str();                                                                                \
  } while (0)
#define sp_error(args) sp_diag_msg(args, "ERROR: ")
#define sp_warn(args) sp_diag_msg(args, "WARNING: ")
#endif

namespace serial_protocol
{
uint8_t SensorBase::base_sensor_count{ 0 };

SensorBase::SensorBase(const uint16_t sen_len, const SensorType sensor_type)
  : sensor_type(sensor_type), len(0), dataptr(0), timestamp(0), previous_timestamp(0)
{
  // check data len consistency to registered len
  if (this->sensor_type.data_length > 0)
  {
    if (sen_len != this->sensor_type.data_length)
      throw SPInvalidSizeException("sensor: non-matching data length");
  }
  len = sen_len;
  base_sensor_id = ++base_sensor_count;
}

SensorBase::~SensorBase()
{
  if (dataptr)
    delete (uint8_t*)dataptr;
  base_sensor_count--;
}

bool SensorBase::init()
{
  if (len)
  {
    if (dataptr != 0)
      delete (uint8_t*)dataptr;
    dataptr = (void*)new uint8_t[len];
    if (dataptr != 0)
      return true;
  }
  return false;
}

void SensorBase::process_args()
{
  if (args.length())
  {
    args_map_str.clear();
    args_map_float.clear();

    char* token;
    // way around the const char issue of s_args.c_str()
    std::vector<char> args_vect(args.begin(), args.end() + 1);
    // Do we have multiple args ?
    bool multi_args = false;
    if (args.find(";") != std::string::npos)
    {
      // split at ";"
      token = std::strtok(args_vect.data(), ";");
      multi_args = true;
    }
    else
    {
      token = args_vect.data();
    }
    while (token != NULL)
    {
      // find the key
      std::string s(token);
      size_t pos = s.find("=");
      // if no key, warn and pass
      if (pos == std::string::npos)
      {
        std::cerr << "sp: sensor parse arg failed, no value found, ignoring entry. syntax is :var_name=var_value;"
                  << std::endl;
      }
      else
      {
        std::string key = s.substr(0, pos);
        std::string value_str = s.substr(pos + 1, std::string::npos);
        // try convert value to a float
        try
        {
          float f = std::stof(value_str);
          if (args_map_float.find(key) == args_map_float.end())
          {
            args_map_float[key] = f;
          }
          else
          {
            std::cerr << "sp: sensor parse arg duplicate entry found: " << key << std::endl;
          }
        }
        catch (const std::exception& e)
        {
          if (args_map_str.find(key) == args_map_str.end())
          {
            args_map_str[key] = value_str;
          }
          else
          {
            std::cerr << "sp: sensor parse arg duplicate entry found: " << key << std::endl;
          }
        }
      }
      if (multi_args)
        token = std::strtok(NULL, ";");
      else
        break;
    }
  }
}

bool SensorBase::unpack(uint8_t* buf)
{
  // store the raw value, no processing at all
  if (!dataptr)
    return false;
  extract_timestamp(buf);
  std::memcpy((uint8_t*)dataptr, buf + SP_TIMESTAMP_LEN, len);
  return true;
}

void* SensorBase::get_data()
{
  return dataptr;
}

uint32_t SensorBase::get_timestamp()
{
  return timestamp;
}

void SensorBase::extract_timestamp(uint8_t* buf)
{
  std::memcpy((uint8_t*)&timestamp, buf, SP_TIMESTAMP_LEN);
}

/* **********************
 *
 * based on Factory Pattern in C++
 * Cale Dunlap, 15 Sep 2012
 *
 */

SensorFactory::SensorFactory()
{
  Register("undefined", &Sensor_Default::Create);
  Register("MID_tactile_fingertip_teensy", &Sensor_MID_tactile_fingertip_teensy::Create);
  Register("BNO055", &Sensor_BNO055::Create);
  Register("BNO08X", &Sensor_BNO08X::Create);
  Register("BMA255", &Sensor_BMA255::Create);
  Register("MPU9250", &Sensor_MPU9250::Create);
  Register("MPU9250_accelerometer", &Sensor_IMU_MPU9250_Acc::Create);
  Register("mpl115a2", &Sensor_MPL115A2::Create);
  Register("AS5013_y_position", &Sensor_AS5013y::Create);
  Register("AS5013", &Sensor_AS5013::Create);
  Register("iobject_myrmex", &Sensor_iobject_myrmex::Create);
  Register("tactile_module_16x16_v2", &Sensor_myrmex_v2::Create);
  Register("tactile_glove_teensy", &Sensor_tactile_glove_teensy::Create);
  Register("tactile_glove_teensy_bend", &Sensor_tactile_glove_teensy_bend::Create);
  Register("BMP388modified_pressure_array", &Sensor_BMP388modified_pressure_array::Create);
  Register("BMP388_testboard_4sensors", &Sensor_BMP388modified_pressure_array::Create);
  Register("Generic_position_sensors_float", &Sensor_generic_position_float::Create);
}

void SensorFactory::Register(const std::string& sensor_name, CreateSensorFn fnCreate)
{
  factory_map[sensor_name] = fnCreate;
}

SensorBase* SensorFactory::CreateSensor(const uint16_t sen_len, const SensorType sensor_type)
{
  SensorFactoryMap::iterator it = factory_map.find(sensor_type.name);
  if (it != factory_map.end())
    return it->second(sen_len, sensor_type);
  return NULL;
}

/* *********************** */

Device::Device()
{
  device.id = 0;
  topology_type = 0;
  alive = false;
}

void Device::init(const DeviceType dev_type)
{
  device = dev_type;
}

Device::~Device()
{
  std::vector<std::pair<SensorBase*, bool>>::iterator s;
  for (s = sensors.begin(); s != sensors.end(); s++)
  {
    delete (*s).first;
  }
}

#ifdef HAVE_ROS
void Device::init_ros(ros::NodeHandle& nh)
{
  std::vector<std::pair<SensorBase*, bool>>::iterator s;
  for (s = sensors.begin(); s != sensors.end(); s++)
  {
    SensorBase* sensor = (*s).first;
    sensor->init_ros(nh);
  }
}
#endif

std::string Device::get_serial()
{
  return serialnum;
}

void Device::set_serial(std::string serial_number)
{
  serialnum = serial_number;
}

uint8_t Device::get_topology_type()
{
  return topology_type;
}

std::vector<topoECD> Device::get_topology_matrix(uint8_t& rows, uint8_t& cols)
{
  rows = topology_rows;
  cols = topology_cols;
  return topology_ecds;
}

void Device::set_topology_matrix(const std::vector<topoECD>& topology, uint8_t rows, uint8_t cols)
{
  topology_type = SP_TOP_TYPE_MATRIX;
  topology_rows = rows;
  topology_cols = cols;
  topology_ecds = topology;
}

std::pair<SensorBase*, bool>* Device::get_sensor_by_idx(const uint8_t idx)
{
  int sensor_idx = idx - 1;
  if (sensor_idx >= 0 && sensor_idx < (int)sensors.size())
    return &sensors[sensor_idx];
  else
    return NULL;
}

bool Device::exists_sensor(const uint8_t idx)
{
  if (idx == 0)
    return 0;
  else
    return idx <= sensors.size();
}

void Device::add_sensor(const uint16_t data_len, const SensorType sensor_type, const std::string args)
{
  try
  {
    SensorBase* sensor = SensorFactory::Get()->CreateSensor(data_len, sensor_type);

    if (sensor)
    {
      sensor->args = args;
      sensor->process_args();
      if (sensor->init())
      {
        sensors.push_back(std::make_pair(sensor, true));
      }
      else
      {
        delete sensor;
        throw std::runtime_error("sp: config answer from non-master not supported");
      }
    }
    else
    {
      std::cerr << "no sensor library for this sensor " << sensor_type.name << ", using default one" << std::endl;
      SensorType st;
      st.id = 0x0000;
      st.name = "undefined";
      st.data_length = data_len;
      sensor = SensorFactory::Get()->CreateSensor(data_len, st);
      if (sensor)
      {
        if (sensor->init())
        {
          sensors.push_back(std::make_pair(sensor, true));
        }
        else
        {
          delete sensor;
          throw std::runtime_error("sp: config answer from non-master not supported");
        }
        // throw std::runtime_error("sp: could not create a sensor");
      }
      else
      {
        throw std::runtime_error("sp: could not create a sensor");
      }
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "sp: could not add a sensor" << e.what() << std::endl;
    throw std::runtime_error("sp: could not add a sensor");
  }
}

void Device::publish_all()
{
  std::vector<std::pair<SensorBase*, bool>>::iterator it;
  for (it = sensors.begin(); it != sensors.end(); it++)
  {
    std::pair<SensorBase*, bool> item = *it;
    if (item.second)
    {
      item.first->publish();
    }
  }
}

/* *********************** */

SerialProtocolBase::SerialProtocolBase(SerialCom* serial_com, const std::string device_filename,
                                       const std::string sensor_filename, const std::string sensor_args)
  : verbose(false)
  , node_prefix("")
  , streaming(false)
  , s(serial_com)
  , d_filename(device_filename)
  , s_filename(sensor_filename)
  , s_args(sensor_args)
  , last_cmd(SP_CMD_NONE)
{
  throw_handler.throw_at_timeout = true;
}

SerialProtocolBase::~SerialProtocolBase()
{
}

bool SerialProtocolBase::init()
{
  try
  {
    stop_streaming();
    config();
    req_serialnum();
    req_topology();
#ifdef HAVE_ROS
    // initialize ros for devices here because the device are now created
    dev.init_ros(nh);
#endif
    return true;
  }
  catch (const std::exception& e)
  {
    sp_error("sp: init failed:" << e.what() << std::endl);
    return false;
  }
}

// https://stackoverflow.com/a/38813146
void SerialProtocolBase::parse_sensor_args()
{
  // parser for "TactileModule:name=toto;TactileModule:channel=foo;IMU:name=tutu"
  if (s_args.length())
  {
    std::map<std::string, std::string> sensor_args_map;
    char* token;
    // way around the const char issue of s_args.c_str()
    std::vector<char> s_args_vect(s_args.begin(), s_args.end() + 1);
    // Do we have multiple args ?
    bool multi_args = false;
    if (s_args.find(";") != std::string::npos)
    {
      // split at ";"
      token = std::strtok(s_args_vect.data(), ";");
      multi_args = true;
    }
    else
    {
      token = s_args_vect.data();
    }
    while (token != NULL)
    {
      if (verbose)
        std::cout << "sp: parsing extra_args" << std::endl;
      // find the key
      std::string s(token);
      size_t pos = s.find(":");
      // if no key, warn and pass
      if (pos == std::string::npos)
      {
        sp_warn(
            "sp: parse sensor arg failed, no sensor type found, ignoring entry. syntax is "
            ":sensor_type:var_name=var_value");
      }
      else
      {
        std::string key = s.substr(0, pos);
        if (sensor_args_map.find(key) == sensor_args_map.end())
        {
          sensor_args_map[key] = s.substr(pos + 1, std::string::npos);
        }
        else
        {
          sensor_args_map[key] = sensor_args_map[key] + ";" + s.substr(pos + 1, std::string::npos);
          s.substr(pos + 1, std::string::npos);
        }
      }
      if (multi_args)
        token = std::strtok(NULL, ";");
      else
        break;
    }

    // convert the map to one using hex sensor types
    args_dict.clear();
    for (auto const& p : sensor_args_map)
    {
      uint16_t sen_driver_id;
      // std::cout << "searching for " << p.first << std::endl;
      // find the sensor_driver_id by name
      if (get_sensor_driver_id(sen_driver_id, p.first))
      {
        args_dict[sen_driver_id] = p.second;
        // std::cout << "    found  " << sen_driver_id << std::endl;
      }
    }
  }
  // for debug only
  if (verbose)
  {
    std::cout << " sensor_args dictionary" << std::endl;
    for (auto const& p : args_dict)
      std::cout << " {" << p.first << " => " << p.second << "}" << '\n';
  }
}

#ifdef HAVE_ROS
void SerialProtocolBase::init_ros(ros::NodeHandle& nh)
{
  this->nh = nh;
  node_prefix = ros::this_node::getName() + "/";
  diagnostic_state.level = diagnostic_msgs::DiagnosticStatus::OK;
  // initialize publishers
  diag_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>(node_prefix + "/state", 10, true);  // latched
  // initialize services
  service_set_period =
      nh.advertiseService(node_prefix + "set_period", &SerialProtocolBase::service_set_period_cb, this);
  service_send_cmd = nh.advertiseService(node_prefix + "send_cmd", &SerialProtocolBase::service_send_cmd_cb, this);
  service_get_serialnumber =
      nh.advertiseService(node_prefix + "get_serialnumber", &SerialProtocolBase::service_get_serialnum_cb, this);
  service_get_topology =
      nh.advertiseService(node_prefix + "get_topology", &SerialProtocolBase::service_get_topology_cb, this);
  service_get_devicemap =
      nh.advertiseService(node_prefix + "get_devicemap", &SerialProtocolBase::service_get_devicemap_cb, this);
  service_clear_warnings = nh.advertiseService(node_prefix + "clear_warnings",
                                               &SerialProtocolBase::service_clear_diagnostic_warnings_cb, this);
}

bool SerialProtocolBase::service_set_period_cb(agni_serial_protocol::SetPeriod::Request& req,
                                               agni_serial_protocol::SetPeriod::Response& res)
{
  res.result = agni_serial_protocol::SetPeriod::Response::FAILED;
  size_t size = req.period_map.sensor_ids.size();
  if (size)
  {
    if (size == req.period_map.periods.size())  // valid data with same size
    {
      if (size == 1)  // one logical sensor
      {
        unsigned int period = req.period_map.periods[0] * 10.0;
        if (period >= 1 && period < 65536)  // accept value higher than one times 100 us
        {
          try
          {
            set_period(req.period_map.sensor_ids[0], (uint16_t)period);
            res.result = agni_serial_protocol::SetPeriod::Response::SUCCESS;
          }
          catch (const std::exception& e)
          {
            std::stringstream sstr;
            sstr << "Failed to set period: " << e.what();
            res.message = sstr.str();
          }
        }
        else
        {
          res.message = "period must be >= 0.1 and < 6553.5 ms";
        }
      }
      else
      {
        std::map<uint8_t, uint16_t> period_map;
        unsigned int invalid_period = 0;
        for (unsigned int i = 0; i < req.period_map.sensor_ids.size(); ++i)
        {
          unsigned int period = req.period_map.periods[i] * 10.0;
          uint8_t sen_id = req.period_map.sensor_ids[i];
          if (period >= 1 && period < 65536 && exists_sensor(sen_id))  // accept value higher than one times 100 us
          {
            period_map[req.period_map.sensor_ids[i]] = (uint16_t)period;
          }
          else
          {
            invalid_period++;
          }
        }
        try
        {
          set_periods(period_map);

          if (invalid_period)
          {
            std::stringstream sstr;
            sstr << invalid_period << " out of range periods were skipped";
            res.message = sstr.str();
            res.result = agni_serial_protocol::SetPeriod::Response::INCOMPLETE;
          }
          else
            res.result = agni_serial_protocol::SetPeriod::Response::SUCCESS;
        }
        catch (const std::exception& e)
        {
          std::stringstream sstr;
          sstr << "Failed to set period: " << e.what();
          res.message = sstr.str();
        }
      }
    }
    else
    {
      res.message = "Mismatching sensor_ids/periods size";
    }
  }
  else
  {
    res.message = "Set period cannot be empty";
  }
  return true;
}

bool SerialProtocolBase::service_send_cmd_cb(agni_serial_protocol::SendCmd::Request& req,
                                             agni_serial_protocol::SendCmd::Response& res)
{
  res.result = agni_serial_protocol::SendCmd::Response::FAILED;
  // TODO write mutex

  switch (req.command)
  {
    case SP_CMD_STOP_STREAM: {
      try
      {
        stop_streaming();
        res.result = agni_serial_protocol::SendCmd::Response::SUCCESS;
      }
      catch (const std::exception& e)
      {
        std::stringstream sstr;
        sstr << "Failed to stop streaming: " << e.what();
        res.message = sstr.str();
      }
      break;
    }
    case SP_CMD_START_STREAM_CONT_ALL: {
      try
      {
        start_streaming(SP_CMD_START_STREAM_CONT_ALL);
        res.result = agni_serial_protocol::SendCmd::Response::SUCCESS;
      }
      catch (const std::exception& e)
      {
        std::stringstream sstr;
        sstr << "Failed to start streaming: " << e.what();
        res.message = sstr.str();
      }
      break;
    }
    case SP_CMD_ALIVE: {
      // just use the alive variable
      res.result =
          dev.alive ? agni_serial_protocol::SendCmd::Response::TRUE : agni_serial_protocol::SendCmd::Response::FALSE;
      break;
    }

    default: {
      std::stringstream sstr;
      sstr << "unsupported command 0x" << std::hex << (int)req.command;
      res.message = sstr.str();
    }
  }
  return true;
}

bool SerialProtocolBase::service_get_serialnum_cb(agni_serial_protocol::GetSerialNumber::Request& req,
                                                  agni_serial_protocol::GetSerialNumber::Response& res)
{
  res.serial_number = dev.get_serial();
  return true;
}

bool SerialProtocolBase::service_get_topology_cb(agni_serial_protocol::GetTopology::Request& req,
                                                 agni_serial_protocol::GetTopology::Response& res)
{
  res.topology.type = dev.get_topology_type();
  if (res.topology.type == agni_serial_protocol::Topology::TYPE_MATRIX)
  {
    std::vector<topoECD> ecds = dev.get_topology_matrix(res.topology.rows, res.topology.cols);
    for (auto ecd : ecds)
    {
      agni_serial_protocol::TopologyECD ecd_msg;
      ecd_msg.sensor_ids = ecd;
      res.topology.ecds.push_back(ecd_msg);
    }
  }
  return true;
}

bool SerialProtocolBase::service_get_devicemap_cb(agni_serial_protocol::GetDeviceMap::Request& req,
                                                  agni_serial_protocol::GetDeviceMap::Response& res)
{
  const std::vector<std::pair<SensorBase*, bool>> sensors = dev.get_sensors();
  for (unsigned int i = 0; i < sensors.size(); ++i)
  {
    res.device_map.sensor_ids.push_back(i + 1);
    res.device_map.sensor_names.push_back(sensors[i].first->get_type().name);
    res.device_map.driver_ids.push_back(sensors[i].first->get_type().id);
  }
  return true;
}

bool SerialProtocolBase::service_clear_diagnostic_warnings_cb(std_srvs::Empty::Request& req,
                                                              std_srvs::Empty::Response& res)
{
  if (diagnostic_state.level == diagnostic_msgs::DiagnosticStatus::WARN)
  {
    update_diagnostic(diagnostic_msgs::DiagnosticStatus::OK);
  }
  return true;
}
#endif

void SerialProtocolBase::config()
{
  uint8_t send_buf[SP_MAX_BUF_SIZE];
  // request config
  uint32_t send_buf_len = gen_master_config_req(send_buf);
  try
  {
    // printf("sp: fake write\n");
    s->write(send_buf, (size_t)send_buf_len);
  }
  catch (const std::exception& e)
  {
    sp_error("sp: config req send error " << e.what() << std::endl);
    // do not make it a SPFailedRequestException as it should always be fatal
    throw std::runtime_error("sp: failed to config");
  }
  try
  {
    read();
  }
  catch (const SPBaseException& e)
  {
    sp_error(" " << std::string(e.what()));
    // do not make it a SPFailedRequestException as it should always be fatal
    throw std::runtime_error("sp: failed to config");
  }
  catch (const std::exception& e)
  {
    sp_error(" " << e.what());
    if (1)  // errors should be handled and print debug info all the time
    {
      print_buffer();
    }
    throw;
  }
}

void SerialProtocolBase::ping()
{
  uint8_t send_buf[20];
  // request config
  uint32_t send_buf_len = gen_master_ping_req(send_buf);
  try
  {
    s->write(send_buf, (size_t)send_buf_len);
  }
  catch (const std::exception& e)
  {
    sp_error("sp: ping request send error: " << e.what() << std::endl);
    throw SPFailedRequestException("failed to send ping request");
  }
  // no need to read here as ping will occur during normal update
  // read(true);
  ping_requested = true;
}

bool SerialProtocolBase::init_device_from_config(uint8_t* buf, const uint8_t config_num)
{
  for (uint8_t i = 0; i < config_num; i++)
  {
    uint8_t* config_buf = buf + i * SP_SDSC_SIZE * sizeof(uint8_t);
    uint16_t sen_type = config_buf[0] + config_buf[1] * 256;
    uint16_t sen_len = config_buf[2] + config_buf[3] * 256;
    if (verbose)
      printf("sp: for sensor %d, found sen_type %04X and sen_len %u\n", i, sen_type, sen_len);
    if (exists_sensor_driver(sen_type))
    {
      try
      {
        // find args for this sensor_type
        std::string args = "";
        if (args_dict.find(sen_type) != args_dict.end())
        {
          args = args_dict[sen_type];
        }
        // add the sensor to the device (implicitly instantiates a dedicted parser)
        dev.add_sensor(sen_len, sensor_types[sen_type], args);
      }
      catch (const std::exception& e)
      {
        sp_error(e.what() << std::endl);
        return false;
      }
    }
    else
    {
      printf("sp: no sensor driver found for logical sensor %d with type %04X, check registred_devices.yaml\n", i,
             sen_type);
      return false;
    }
  }
  return true;
}

void SerialProtocolBase::req_serialnum()
{
  uint8_t send_buf[SP_HEADER_LEN + SP_DID_LEN + SP_CMD_LEN + SP_CHKSUM_LEN];
  // request serial
  uint32_t send_buf_len = gen_serialnum_req(send_buf);
  try
  {
    s->write(send_buf, (size_t)send_buf_len);
  }
  catch (const std::exception& e)
  {
    sp_error("sp: req serialnum send error: " << e.what());
    throw SPFailedRequestException("failed to req serial");
  }
  read_maybe_non_impl_cmd("serialnum");
}

void SerialProtocolBase::req_topology()
{
  uint8_t send_buf[SP_HEADER_LEN + SP_DID_LEN + SP_CMD_LEN + SP_CHKSUM_LEN];
  // request topology
  uint32_t send_buf_len = gen_topology_req(send_buf);
  try
  {
    s->write(send_buf, (size_t)send_buf_len);
  }
  catch (const std::exception& e)
  {
    sp_error("sp: req topology send error: " << e.what());
    throw SPFailedRequestException("failed to req topology");
  }
  read_maybe_non_impl_cmd("topology");
}

void SerialProtocolBase::read_maybe_non_impl_cmd(const std::string req_fn_name)
{
  try
  {
    read();
  }
  catch (const SPReadTimeOutException& e)
  {
    // function might not be implemented in the device, so ignore if no response
    if (verbose)
    {
      sp_warn("sp:the device did not answer to a " + req_fn_name + " request");
    }
  }
  catch (const SPDatagramException& e)
  {
    print_buffer();
    if (throw_handler.throw_at_size_error)
      throw;
    else
    {
      sp_error(" " << e.what());
      return;
    }
  }
  catch (const SPBaseException& e)
  {
    if (throw_handler.throw_at_did_error)
      throw;
    else
    {
      sp_error(" " << e.what());
      return;
    }
  }
  catch (const std::exception& e)
  {
    // sp_error(" " << e.what());
    if (1)  // errors should be handled and print debug info all the time
    {
      print_buffer();
    }
    throw;
  }
}

bool SerialProtocolBase::set_device(const uint8_t dev_id)
{
  if (exists_device(dev_id))
  {
    dev.init(device_types[dev_id]);
    return true;
  }
  else
    return false;
}

void SerialProtocolBase::set_period(const uint8_t sen_id, const uint16_t period)
{
  uint8_t send_buf[SP_HEADER_LEN + SP_DID_LEN + SP_CMD_LEN + SP_CMD_DATA_SZ_LEN + SP_PERIOD_SENSOR_DATA_LEN +
                   SP_CHKSUM_LEN];
  uint32_t send_buf_len = 0;
  if (sen_id == 0)
  {
    throw SPInvalidRequestException("set_period: sen_id cannot be 0 for set period");
  }
  else
  {
    // check if sensor exists
    if (exists_sensor(sen_id))
    {
      send_buf_len = gen_period_sensor_req(send_buf, sen_id, period);
    }
    else
    {
      throw SPInvalidRequestException("set_period: sen_id does not exist");
    }
  }
  if (send_buf_len)
  {
    try
    {
      send(send_buf, send_buf_len);
    }
    catch (const SPSendException& e)
    {
      sp_error("sp: failed to set period " << e.what());
      throw SPFailedRequestException("failed to set period");
    }
  }
  else
  {
    throw SPInvalidRequestException("invalid set period request");
  }
}

void SerialProtocolBase::set_periods(const std::map<uint8_t, uint16_t> period_map)
{
  if (period_map.size())
  {
    uint8_t send_buf[SP_HEADER_LEN + SP_DID_LEN + SP_CMD_LEN + SP_CMD_DATA_SZ_LEN +
                     SP_PERIOD_DATA_LEN * period_map.size() + SP_CHKSUM_LEN];
    uint32_t send_buf_len = 0;
    send_buf_len = gen_period_master_req(send_buf, period_map);
    if (send_buf_len)
    {
      try
      {
        send(send_buf, send_buf_len);
      }
      catch (const SPSendException& e)
      {
        sp_error("sp: failed to set periods " << e.what());
        throw SPFailedRequestException("failed to set periods");
      }
    }
    else
    {
      throw SPInvalidRequestException("invalid set periods request");
    }
  }
}

bool SerialProtocolBase::valid_data(const uint8_t* buf, const uint32_t buf_len)
{
  return valid_header(buf) && valid_checksum(buf, buf_len);
}

bool SerialProtocolBase::valid_header(const uint8_t* buf)
{
  return (uint16_t)(buf[0] * 256 + buf[1]) == SP_HEADER;
}

bool SerialProtocolBase::valid_checksum(const uint8_t* buf, const uint32_t len)
{
  return compute_checksum(buf, len - 1) == buf[len - 1];
}

uint8_t SerialProtocolBase::compute_checksum(const uint8_t* buf, const uint32_t len)
{
  uint8_t xorsum = 0;
  for (uint32_t i = 0; i < len; i++)  // do xor for each data byte
  {
    xorsum = xorsum ^ buf[i];
  }
  return xorsum;
}

void SerialProtocolBase::read_device_types(const uint8_t v)
{
  // read params for given version
  if (d_filename != "")
  {
    if (verbose)
      std::cout << "sp: reading device_types from " << d_filename << std::endl;
    // bool success = false;
    try
    {
      YAML::Node root = YAML::LoadFile(d_filename);
      if (root["agni_serial_protocol"])
      {
        YAML::Node agni_serial_protocol = root["agni_serial_protocol"];
        for (YAML::iterator it = agni_serial_protocol.begin(); it != agni_serial_protocol.end(); ++it)
        {
          // std::string protocol_version_str = it->first.as<std::string>();
          // std::string protocol_version_str =
          uint8_t protocol_version = (uint8_t)it->first.as<int>();
          if (verbose)
            std::cout << "sp: found specs for version " << protocol_version << "\n";
          const YAML::Node& version_node = it->second;
          if (verbose)
            std::cout << "sp: extract version\n";
          if (version_node["device_types"])
          {
            if (v == protocol_version)
            {
              YAML::Node device_types_node = version_node["device_types"];
              if (verbose)
                std::cout << "sp: found specs for device_types \n";
              for (YAML::const_iterator devit = device_types_node.begin(); devit != device_types_node.end(); ++devit)
              {
                YAML::Node device_types_item = *devit;  //->second;
                if (verbose)
                  std::cout << "sp: extract device_type item ";
                DeviceType dt;
                std::stringstream sstr("");
                sstr << std::hex << device_types_item["type"].as<std::string>();
                int did;
                sstr >> did;
                dt.id = (uint8_t)did;
                dt.name = device_types_item["name"].as<std::string>();
                if (verbose)
                  std::cout << (int)dt.id << " named: " << dt.name << "\n";
                dt.description = device_types_item["description"].as<std::string>();
                device_types[dt.id] = dt;
              }
              // success = true;
              return;
            }
            else
            {
              std::cerr << "sp: yaml does not contain specs for version " << (int)v << "\n";
            }
          }
          else
          {
            std::cerr << "sp: yaml does not contain device_types \n";
          }
        }
      }
      else
      {
        std::cerr << "sp: yaml does not contain agni_serial_protocol \n";
      }
    }
    catch (const std::exception& e)
    {
      std::cerr << "sp: read device_types error: " << e.what() << std::endl;
    }
  }
  std::cout << "sp: using default device_types\n";

  // fallback

  DeviceType dt;
  dt.id = 0x00;
  dt.name = "undefined";
  dt.description = "undefined";
  device_types[dt.id] = dt;

  dt.id = 0x01;
  dt.name = "prototype";
  dt.description = "experimental hardware";
  device_types[dt.id] = dt;
}

void SerialProtocolBase::read_sensor_types(const uint8_t v)
{
  // read params for given version
  if (s_filename != "")
  {
    if (verbose)
      std::cout << "sp: reading sensor_types from " << s_filename << std::endl;
    // bool success = false;
    try
    {
      YAML::Node root = YAML::LoadFile(s_filename);
      if (root["agni_serial_protocol"])
      {
        YAML::Node agni_serial_protocol = root["agni_serial_protocol"];
        for (YAML::iterator it = agni_serial_protocol.begin(); it != agni_serial_protocol.end(); ++it)
        {
          uint8_t protocol_version = (uint8_t)it->first.as<int>();
          if (verbose)
            std::cout << "sp: found specs for version " << (int)protocol_version << "\n";
          const YAML::Node& version_node = it->second;
          if (verbose)
            std::cout << "sp: extract version\n";
          if (version_node["registered_devices"])
          {
            if (v == protocol_version)
            {
              YAML::Node sensor_types_node = version_node["registered_devices"];
              if (verbose)
                std::cout << "sp: found specs for registered_devices \n";
              for (YAML::const_iterator senit = sensor_types_node.begin(); senit != sensor_types_node.end(); ++senit)
              {
                YAML::Node sensor_types_item = *senit;  //->second;
                if (verbose)
                  std::cout << "sp: extract sensor_type item ";
                SensorType st;
                std::stringstream sstr("");
                sstr << std::hex << sensor_types_item["uid"].as<std::string>();
                int type_id;
                sstr >> type_id;
                st.id = (uint16_t)type_id;
                st.name = sensor_types_item["name"].as<std::string>();
                st.manufacturer = sensor_types_item["manufacturer"].as<std::string>();
                st.description = sensor_types_item["description"].as<std::string>();
                st.parser_library = sensor_types_item["parser_library"].as<std::string>();
                st.data_length = (uint16_t)sensor_types_item["data_length"].as<int>();
                if (verbose)
                  std::cout << (int)st.id << " named: " << st.name << " with length " << st.data_length << "\n";
                sensor_types[st.id] = st;
              }
              // success = true;
              return;
            }
            else
            {
              std::cerr << "sp: yaml does not contain specs for version " << (int)v << "\n";
            }
          }
          else
          {
            std::cerr << "sp: yaml does not contain sensor_types \n";
          }
        }
      }
      else
      {
        std::cerr << "sp: yaml does not contain agni_serial_protocol \n";
      }
    }
    catch (const std::exception& e)
    {
      std::cerr << "sp: read sensor_types error: " << e.what() << std::endl;
    }
  }
  std::cout << "sp: using default sensor_types\n";

  // hardcoded for now
  SensorType st;
  st.id = 0x0000;
  st.name = "undefined";
  st.manufacturer = "undefined";
  st.description = "undefined";
  st.parser_library = "dummy::dummy";
  st.data_length = 0;
  sensor_types[st.id] = st;

  st.id = 0xDC10;
  st.name = "MPU9250";
  st.manufacturer = "Drotek";
  st.description = "MPU9250 Drotek Inertial Measurement Unit";
  st.parser_library = "imu::mp9250";
  st.data_length = 2;
  sensor_types[st.id] = st;
}

// check if the dev_id exists in the database of device_types
bool SerialProtocolBase::exists_device(const uint8_t dev_id)
{
  return device_types.find(dev_id) != device_types.end();
}

// check if the sen_driver_id exists in the database of registered_devices
bool SerialProtocolBase::exists_sensor_driver(const uint16_t sen_driver_id)
{
  return sensor_types.find(sen_driver_id) != sensor_types.end();
}

// get the sen_driver_id by name in the database of registered_devices
bool SerialProtocolBase::get_sensor_driver_id(uint16_t& sen_driver_id, const std::string sen_driver_name)
{
  for (auto const& s : sensor_types)
  {
    if (s.second.name == sen_driver_name)
    {
      sen_driver_id = s.first;
      return true;
    }
  }
  return false;
}

bool SerialProtocolBase::exists_sensor(const uint8_t sen_id)
{
  return dev.exists_sensor(sen_id);
}

DeviceType SerialProtocolBase::get_device()
{
  return dev.device;
}

void SerialProtocolBase::trigger(const uint8_t mode, const uint8_t sen_id)
{
  if (!streaming)
  {
    uint8_t send_buf[5];
    uint32_t send_buf_len = 0;
    if (sen_id == 0)
    {
      send_buf_len = gen_master_trigger_req(send_buf);
    }
    else
    {
      // check if sensor exists
      if (exists_sensor(sen_id))
      {
        send_buf_len = gen_sensor_trigger_req(send_buf, sen_id);
      }
    }
    if (send_buf_len)
    {
      try
      {
        send(send_buf, send_buf_len);
      }
      catch (const SPSendException& e)
      {
        sp_warn("sp: failed to trigger " << e.what());
        throw SPFailedRequestException("failed to trigger");
      }
    }
    else
    {
      throw SPInvalidRequestException(" invalid trigger request");
    }
  }
  else
  {
    sp_warn("sp: cannot trigger while streaming, stop streaming first");
  }
}

void SerialProtocolBase::update()
{
  // handle exceptions, depending on settings
  try
  {
    read();
  }
  catch (const SPReadTimeOutException& e)
  {
    if (throw_handler.throw_at_timeout)
      throw;
    else
    {
      sp_error(" " << e.what());
      return;
    }
  }
  catch (const SPDatagramException& e)
  {
    print_buffer();
    if (throw_handler.throw_at_datagram_error())
      throw;
    else
    {
      sp_error(" " << e.what());
      return;
    }
  }
  catch (const SPInvalidRequestException& e)
  {
    if (throw_handler.throw_at_request_error)
      throw;
    else
    {
      sp_error(" " << e.what());
      return;
    }
  }
  catch (const SPFailedRequestException& e)
  {
    if (throw_handler.throw_at_failed_request)
      throw;
    else
    {
      sp_error(" " << e.what());
      return;
    }
  }
  catch (const SPSendException& e)
  {
    if (throw_handler.throw_at_send_error)
      throw;
    else
    {
      sp_error(" " << e.what());
      return;
    }
  }
  catch (const std::exception& e)
  {
    // sp_error(" " << e.what());
    if (1)  // errors should be handled and print debug info all the time
    {
      print_buffer();
    }
    throw;
  }

#ifdef HAVE_ROS
  if (!dev.alive)
    update_diagnostic(diagnostic_msgs::DiagnosticStatus::STALE);
  else
    update_diagnostic(diagnostic_msgs::DiagnosticStatus::OK);
#endif
}

void SerialProtocolBase::publish()
{
  dev.publish_all();
#ifdef HAVE_ROS
  ros::spinOnce();
#endif
}

#ifdef HAVE_ROS
void SerialProtocolBase::update_diagnostic(const uint8_t& level, const std::string& message)
{
  if (diagnostic_state.level == level && diagnostic_state.message == message)
    return;
  else
  {
    diagnostic_state.level = level;
    diagnostic_state.message = message;
    diag_pub.publish(diagnostic_state);
  }
}
#endif

void SerialProtocolBase::read_config(uint8_t* buf)
{
  // leave all exception a runtime_error to be fatal all the time

  // read additional config header
  size_t buf_len = 0;
  buf_len = s->read(buf + SP_VERSION_OFFSET, SP_VERSION_LEN + SP_DEVID_LEN + SP_SDSC_SZ_LEN);
  if (buf_len < SP_VERSION_LEN + SP_DEVID_LEN + SP_SDSC_SZ_LEN)
  {
    sp_error("sp: incorrect config header size: " << buf_len << " < "
                                                  << SP_VERSION_LEN + SP_DEVID_LEN + SP_SDSC_SZ_LEN);

    throw std::runtime_error("sp: device did not answer enough data");
  }
  // read versions
  uint8_t version = (uint8_t)buf[SP_VERSION_OFFSET];
  if (version > 0 and version <= SP_MAX_KNOWN_VERSION)
  {
    read_device_types(version);
    read_sensor_types(version);
  }
  else
  {
    sp_error("sp: found unknown version ("
             << "0x" << std::uppercase << std::hex << version << ")");
    throw std::runtime_error("sp: unsupported version");
  }

  // validate master is answering
  if (buf[SP_DID_OFFSET] != SP_DID_MASTER)
  {
    throw std::runtime_error("sp: config answer from non-master not supported");
  }

  // create a device
  if (!set_device(buf[SP_DEVID_OFFSET]))
  {
    sp_error("sp:device with id " << (int)buf[SP_DEVID_OFFSET]
                                  << " is not a known device type (check device_types.yaml)");
    throw std::runtime_error("sp: device type not found");
  }

  // get how much more config comes
  uint8_t config_num = (uint8_t)buf[SP_SDSC_OFFSET];
  // read next part of the config message
  size_t config_len = 0;
  config_len = s->read(buf + SP_SDSC_DATA_OFFSET, config_num * SP_SDSC_SIZE + SP_CHKSUM_LEN);

  // check data
  if (config_len != (size_t)(config_num * SP_SDSC_SIZE + SP_CHKSUM_LEN))
  {
    sp_error("sp: read " << config_len << " bytes of config_len instead of "
                         << config_num * SP_SDSC_SIZE + SP_CHKSUM_LEN);
    throw std::runtime_error("sp: incomplete sensor description datagram");
  }
  // validate the full frame
  if (valid_data(buf, SP_SDSC_DATA_OFFSET + config_len))
  {
    // parse sensor_args here because the sensor_type is now initialized
    parse_sensor_args();
    if (!init_device_from_config(buf + SP_SDSC_DATA_OFFSET, config_num))
    {
      throw std::runtime_error("sp: error initializing the device");
    }
  }
  else
  {
    // error, stop streaming, flush and retry
    throw std::runtime_error("sp: checksum or header invalid");
  }
}

void SerialProtocolBase::read_no_data(uint8_t* buf)
{
  // read one additional bytes to finish the buffer
  size_t buf_len = 0;
  buf_len = s->read(buf + SP_DATA_OFFSET, SP_CHKSUM_LEN);
  if (buf_len < SP_CHKSUM_LEN)
  {
    sp_error("sp: incorrect size nodata read: " << buf_len << " < " << SP_CHKSUM_LEN);
    throw SPInvalidSizeException(" device did not answer enough bytes for a no data read");
  }
}

void SerialProtocolBase::read_serialnum(uint8_t* buf)
{
  // read one additional byte to know the length
  size_t buf_len = 0;
  buf_len = s->read(buf + SP_SER_SZ_OFFSET, SP_SER_SZ_LEN);
  if (buf_len < SP_SER_SZ_LEN)
  {
    sp_error("sp: incorrect serialnum header size: " << buf_len << " < " << SP_SER_SZ_LEN);
    throw SPInvalidSizeException(" device did not answer enough data for serialnum");
  }
  // read serial data
  uint8_t sernum_awaitedlen = (uint8_t)buf[SP_SER_SZ_OFFSET];

  // read next part of the config message
  size_t sernum_len = 0;
  sernum_len = s->read(buf + SP_SER_NUM_OFFSET, sernum_awaitedlen + SP_CHKSUM_LEN);

  // check data
  if (sernum_len != (size_t)(sernum_awaitedlen + SP_CHKSUM_LEN))
  {
    sp_error("sp: read " << sernum_len << " bytes of sernum_len instead of " << sernum_awaitedlen + SP_CHKSUM_LEN);
    throw SPInvalidSizeException(" incomplete serialnum datagram");
  }
  // validate the full frame
  if (valid_data(buf, SP_SER_NUM_OFFSET + sernum_len))
  {
    // store the serial number
    dev.set_serial(std::string((char*)buf + SP_SER_NUM_OFFSET, sernum_awaitedlen));
  }
  else
  {
    // error
    throw SPInvalidChecksumException(" checksum or header invalid for serialnum");
  }
}

void SerialProtocolBase::read_topology(uint8_t* buf)
{
  // read one additional byte to know the type of topology
  size_t buf_len = 0;
  buf_len = s->read(buf + SP_TOP_TYPE_OFFSET, SP_TOP_TYPE_LEN);
  if (buf_len < SP_TOP_TYPE_LEN)
  {
    sp_error("sp: incorrect topo header size: " << buf_len << " < " << SP_TOP_TYPE_LEN);
    throw SPInvalidSizeException(" device did not answer enough data for topology request");
  }
  // save type
  uint8_t topo_type = (uint8_t)buf[SP_TOP_TYPE_OFFSET];
  // check type
  if (topo_type >= SP_TOP_TYPE_MAT_START)
  {
    // handling matrix topology
    uint8_t rows = (topo_type & 0xF0) >> 4;
    uint8_t cols = (topo_type & 0x0F);
    if (verbose)
      std::cout << "sp: topology found matrix type with size (" << (int)rows << ", " << (int)cols << ")" << std::endl;
    uint8_t num_ecds = rows * cols;  // max 15x15 = 225
    std::vector<topoECD> topology_ecds(num_ecds);

    // read all ecds
    uint8_t ecd_current_offset = 0;
    for (unsigned int i = 0; i < num_ecds; ++i)
    {
      // read size of the ecd message
      size_t ecd_len_byteread = 0;
      ecd_len_byteread = s->read(buf + SP_TOP_ECD_OFFSET + ecd_current_offset, SP_TOP_ECD_SZ_LEN);

      // check size
      if (ecd_len_byteread != (size_t)(SP_TOP_ECD_SZ_LEN))
      {
        sp_error("sp: topology ECD size missing");
        throw SPInvalidSizeException(" incomplete topology ECD datagram");
      }
      else
      {
        uint8_t ecd_len = (uint8_t)buf[SP_TOP_ECD_OFFSET + ecd_current_offset];
        // read data part of the ecd message
        size_t ecd_data_byteread = 0;
        ecd_data_byteread = s->read(buf + SP_TOP_ECD_OFFSET + ecd_current_offset + SP_TOP_ECD_SZ_LEN, ecd_len);

        // check data size
        if (ecd_data_byteread != (size_t)(ecd_len))
        {
          sp_error("sp: topology read " << ecd_data_byteread << " ECD data instead of " << ecd_len << " awaited");
          throw SPInvalidSizeException("sp: incomplete topology ECD datagram");
        }
        else
        {
          // fill an ecd message
          topoECD ecd;
          ecd.clear();
          // add logical ids to ecd
          if (verbose)
            std::cout << "sp: topology adding " << (int)ecd_len << " sensor ids at idx " << i << std::endl;
          for (unsigned int j = 0; j < ecd_len; ++j)
          {
            ecd.push_back((uint8_t)buf[SP_TOP_ECD_OFFSET + ecd_current_offset + SP_TOP_ECD_SZ_LEN + j]);
          }
          if (ecd_len > 0)
            topology_ecds[i] = ecd;
          ecd_current_offset += ecd_len + SP_TOP_ECD_SZ_LEN;
        }
      }
    }  // for ecds

    // read Checksum
    size_t checksum_len = 0;
    checksum_len = s->read(buf + SP_TOP_ECD_OFFSET + ecd_current_offset, SP_CHKSUM_LEN);

    // check checksum is there
    if (checksum_len != (size_t)(SP_CHKSUM_LEN))
    {
      sp_error("sp: topology has no checksum");
      throw SPInvalidSizeException(" topology checksum");
    }

    // validate the full frame
    if (valid_data(buf, SP_TOP_ECD_OFFSET + ecd_current_offset + SP_CHKSUM_LEN))
    {
      // store the topology
      dev.set_topology_matrix(topology_ecds, rows, cols);
    }
    else
    {
      // error
      throw SPInvalidChecksumException(" checksum or header invalid for topology");
    }
  }
  else
  {
    switch (topo_type)
    {
      default:
        sp_error("sp: topo different than matrix type are not yet supported");
        throw SPInvalidRequestException(" only topology in matrix type are supported for now");
    }
  }
}

void SerialProtocolBase::read_data(uint8_t* buf, const uint8_t did)
{
  // check sensor existence to compute awaited len
  std::pair<SensorBase*, bool>* sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    if (sensor->second)  // active
    {
      size_t data_len = 0;
      size_t expected_len = SP_TIMESTAMP_LEN + sensor->first->get_len() + SP_CHKSUM_LEN;
      // check buffer size
      if (expected_len + SP_DATA_OFFSET < SP_MAX_BUF_SIZE)
      {
        // read additional data (timestamp + sensor len + checksum)
        data_len = s->read(buf + SP_DATA_OFFSET, expected_len);
      }
      else
        throw std::overflow_error("sp:expects too much data for buffer max size");
      // check received data
      if (data_len == expected_len)
      {
        // validate the whole message
        if (valid_checksum(buf, SP_DATA_OFFSET + expected_len))
        {
          // unpack the data
          if (!sensor->first->unpack(buf + SP_DATA_OFFSET))
          {
            // "sp:sensor data storage not initialized"
            throw std::bad_alloc();
          }
          // parse the data (sensor specific)
          if (verbose)
            std::cout << "sp: parsing answer" << std::endl;
          sensor->first->parse();
          if (verbose)
            std::cout << "sp: data parsed" << std::endl;
          save_buffer(SP_DATA_OFFSET + expected_len);
        }
        else
        {
          std::cerr << "sp:data with invalid checksum" << std::endl;
          print_buffer(SP_DATA_OFFSET + expected_len);
          throw SPInvalidChecksumException("data with invalid checksum");
        }
      }
      else
        throw SPInvalidSizeException("invalid data size");
    }
    else
    {
      sp_error("sp:received data of an inactive sensor id " << (int)did);
      throw SPUnknownDIDException((int)did);
    }
  }
  else
  {
    // TODO we should maybe flush until we see a header ?
    sp_error("sp:found unknown sensor id " << (int)did);
    throw SPUnknownDIDException((int)did);
  }
}

void SerialProtocolBase::read_error(uint8_t* buf)
{
  size_t error_len = 0;
  error_len = s->read(buf + SP_ERR_TYP_OFFSET, SP_ERR_TYP_LEN + SP_CHKSUM_LEN);
  std::stringstream sstr;
  sstr << "sp: device sent an error:";
  if (error_len == SP_ERR_TYP_LEN + SP_CHKSUM_LEN)
  {
    uint8_t error_code = buf[SP_ERR_TYP_OFFSET];
    switch (error_code)
    {
      case SP_ERR_CHKSUM:
        if (!valid_checksum(buf, 5))
        {
          // TODO should we throw for a checksum error ?
          throw SPInvalidChecksumException("sp:err, chksum invalid checksum");
        }
        sstr << " device says checksum error ";
        break;
      case SP_ERR_UNKNCMD:
        if (!valid_checksum(buf, 5))
          throw SPInvalidChecksumException("sp:err, cmd invalid checksum");
        sstr << " device says unknown command ";
        break;
      case SP_ERR_UNKNOWN:
        if (!valid_checksum(buf, 5))
          throw SPInvalidChecksumException("sp:err, unk invalid checksum");
        sstr << " device encountered an unknown error";
        break;
      case SP_ERR_UNKNSID:
        if (!valid_checksum(buf, 5))
          throw SPInvalidChecksumException("sp:err, sid invalid checksum");
        sstr << " device encountered an unknown sensor id";
        break;
      default:
        if (error_code > 0 && error_code < SP_ERR_STRMAX)
        {
          // read the error string
          char strbuf[error_code + 1];  // include null terminating char
          // strbuf[0] = (char)buf[ERROR_OFFSET+2];  // first string was already read
          // read str_len because one character was already read but checksum is still to be read
          uint8_t str_len = error_code;  // include the checksum
          s->read(buf + SP_ERR_TYP_OFFSET + SP_ERR_TYP_LEN + SP_CHKSUM_LEN,
                  str_len);  // read the reminder of the string
          if (!valid_checksum(buf, SP_ERR_TYP_OFFSET + SP_ERR_TYP_LEN + str_len + SP_CHKSUM_LEN))
            throw SPInvalidChecksumException("sp:err, string invalid checksum");
          memcpy(strbuf, buf + SP_ERR_TYP_OFFSET + SP_ERR_TYP_LEN, str_len);
          strbuf[str_len] = '\0';  // end of string
          sstr << " device reports error: " << strbuf;
        }
        else
        {
          sstr << " device encountered an undocumented error";
        }
    }
    // TODO should those errors be warnings only (they are device reported errors not driver errors)
    sp_warn(sstr.str());
    return;
  }

  // HANDLE bug in an old firmware version (prior to 23/08/2021) that don't send a checksum for SP_ERR_UNKNCMD
  if (error_len == SP_ERR_TYP_LEN && buf[SP_ERR_TYP_OFFSET] == SP_ERR_UNKNCMD)
  {
    // ignore checksum, and fake that the read was successful
    sp_warn("device says unknown command, but didn't send a checksum. Consider updating the firmware to fix this bug.");
    return;
  }
  throw SPInvalidRequestException("sp:err, invalid error message");
}

void SerialProtocolBase::read()
{
  // init the first bytes to null to not see spurious values when debugging
  std::memset(read_buf, 0, 7);
  size_t read_buf_len = 0;
  // read 2 or 3 bytes for header or header  shifted by one.
  read_buf_len = s->read(read_buf, SP_HEADER_LEN);
  if (read_buf_len == SP_HEADER_LEN)
  {
    if (verbose)
      std::cout << "sp: decoding answer" << std::endl;
    // check header
    if (!valid_header(read_buf))
    {
      // invalid, so check if the header is shifted by one
      // read one more byte
      read_buf_len = s->read(read_buf + 2, 1);
      if (read_buf_len < 1)
      {
        if (verbose)
          std::cerr << "sp: cannot read more of the header" << std::endl;
        throw SPInvalidHeaderException();
      }
      else
      {
        if (!valid_header(read_buf + 1))
        {
          if (verbose)
            std::cerr << "sp: invalid header in the 3 bytes read" << std::endl;
          print_previous_buffer();
          // if flushed while device still streaming, it will fill the buffer and quit
          throw SPInvalidHeaderException();
        }
        else
        {
          // fix the buffer start
          read_buf[0] = SP_HDR1;
          read_buf[1] = SP_HDR2;
          // still warn the user
          // if (verbose)
          sp_warn("sp: header was shifted by one");
        }
      }
      // the buffer is synchronized again, continue
    }

    // read the DID
    read_buf_len = s->read(read_buf + SP_DID_OFFSET, SP_DID_LEN);
    if (read_buf_len == SP_DID_LEN)
    {
      // device is alive
      dev.alive = true;
      // decode datagram ID
      uint8_t did = read_buf[SP_DID_OFFSET];
      // read additional data depending on the datagram id
      switch (did)
      {
        case SP_DID_DEVCNF:  // process master response
          if (verbose)
            std::cout << "sp: processing config answer" << std::endl;
          read_config(read_buf);
          last_cmd = SP_CMD_NONE;
          break;
        case SP_DID_TOPOL:
          if (verbose)
            std::cout << "sp: processing topolgy answer" << std::endl;
          read_topology(read_buf);
          last_cmd = SP_CMD_NONE;
          break;
        case SP_DID_SERIAL:
          if (verbose)
            std::cout << "sp: processing serialnum answer" << std::endl;
          read_serialnum(read_buf);
          last_cmd = SP_CMD_NONE;
          break;
        case SP_DID_CALCNF:
          if (verbose)
            std::cerr << "sp: calib config answer decoding not yet implemented" << std::endl;
          read_no_data(read_buf);
          last_cmd = SP_CMD_NONE;
          break;
        case SP_DID_CALDSC:
          if (verbose)
            std::cerr << "sp: calib description answer decoding not yet implemented" << std::endl;
          read_no_data(read_buf);
          last_cmd = SP_CMD_NONE;
          break;
        case SP_DID_ERR:  // process error
          if (verbose)
            std::cerr << "sp: error code sent" << std::endl;
          read_error(read_buf);
          last_cmd = SP_CMD_NONE;
          break;
        case SP_DID_PING:
          if (verbose)
            std::cout << "sp: ping answer received" << std::endl;
          read_no_data(read_buf);
          // clear the flag
          ping_requested = false;
          last_cmd = SP_CMD_NONE;
          break;
        case SP_DID_BCAST:
          read_no_data(read_buf);
          last_cmd = SP_CMD_NONE;
          break;
        default:
          if (did >= SP_DID_LID1 && did <= SP_DID_LIDMAX)  // process data
          {
            if (verbose)
              std::cout << "sp: processing data answer of sensor id " << (int)did << std::endl;
            read_data(read_buf, did);

            last_cmd = SP_CMD_NONE;
          }
          else  // process invalid datagram id
          {
            sp_error("sp:found datagram id 0x" << std::hex << (int)did);
            read_no_data(read_buf);
            last_cmd = SP_CMD_NONE;
            throw SPUnknownDIDException((int)did);
          }
      }
    }
    else
    {
      // this is a timeout, after we got a valid header, there should always be more
      throw SPReadTimeOutException("missing datagram id");
    }
  }
  else
  {
    if (read_buf_len == 0)
    {
      // react depending on last command
      switch (last_cmd)
      {
        case SP_CMD_NONE: {
          // if last cmd is empty, device is either stopped (normal to not receive anything)
          // or streaming (not normal to not receive anything)

          // proceed to check if device is alive
          if (ping_requested)
          {
            // ping was already requested but not answered
            dev.alive = false;
            ping_requested = false;
            // nothing to await from sensor
            if (throw_handler.throw_at_timeout)
            {
              sp_error("sp: device does not answer to ping");
            }
            else
            {
              sp_warn("sp: device does not answer to ping");
            }
            throw SPReadTimeOutException("device does not answer to ping");
          }
          else
          {
            // no ping request yet, request one
            ping();
          }
          break;
        }
        // all commands awaiting an answer
        case SP_CMD_CONFRQ:
        case SP_CMD_TOPORQ:
        case SP_CMD_SERIAL:
        case SP_CMD_CALCNF:
        case SP_CMD_CALDSC: {
          std::stringstream sstr;
          sstr << "sp: no answer received from the device to command " << std::hex << (int)last_cmd;
          if (throw_handler.throw_at_timeout)
          {
            // sp_error(sstr.str());
            // nothing to await from sensor
            throw std::runtime_error(sstr.str());
          }
          else
          {
            sp_warn(sstr.str());
            last_cmd = SP_CMD_NONE;
          }
          break;
        }
        case SP_CMD_ALIVE:
        // we got no response to our ping but wait for another timeout
        // (flag ping_requested is still set)
        default:  // other commands not awaiting an answer
          last_cmd = SP_CMD_NONE;
      }
    }
    else
    {
      // received less than 2 bytes but more than 0
      // maybe the device was too slow, so for now, display a message
      std::cerr << "sp: read got only " << read_buf_len << " instead of 2 bytes : 0x" << std::hex << (int)read_buf[0]
                << " 0x" << std::hex << (int)read_buf[1] << std::endl;
      // TODO: ideally one wants to wait for 2 bytes and not lose the previous one in case the device is really slow
    }
  }
}

void SerialProtocolBase::print_buffer(size_t len)
{
  std::cerr << "sp: header 0x" << std::hex << (int)read_buf[0] << " 0x" << std::hex << (int)read_buf[1] << std::endl;
  std::cerr << "sp: data ";
  for (unsigned int i = 0; i < len - SP_HEADER_LEN; ++i)
  {
    std::cerr << " 0x" << std::hex << (int)read_buf[i + SP_HEADER_LEN];
  }
  std::cerr << std::endl;
}

void SerialProtocolBase::print_previous_buffer()
{
  if (previous_len > SP_HEADER_LEN)
  {
    std::cerr << "sp: past buffer header 0x" << std::hex << (int)previous_buffer[0] << " 0x" << std::hex
              << (int)previous_buffer[1] << std::endl;
    std::cerr << "sp: past buffer data ";
    for (unsigned int i = 0; i < previous_len - SP_HEADER_LEN; ++i)
    {
      std::cerr << " 0x" << std::hex << (int)previous_buffer[i + SP_HEADER_LEN];
    }
    std::cerr << std::endl;
    // avoid re-prints of past buffer
    previous_len = 0;
  }
}

void SerialProtocolBase::save_buffer(size_t len)
{
  memcpy(previous_buffer, read_buf, len);
  previous_len = len;
}

bool SerialProtocolBase::get_data_as_float(float& val, const uint8_t did)
{
  // check sensor existence
  std::pair<SensorBase*, bool>* sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    if (sensor->first->get_len() >= 4)
    {
      val = *((float*)sensor->first->get_data());
      return true;
    }
  }
  return false;
}

bool SerialProtocolBase::get_data_as_3_float(float& x, float& y, float& z, const uint8_t did)
{
  // check sensor existence
  std::pair<SensorBase*, bool>* sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    if (sensor->first->get_len() >= 12)
    {
      uint8_t* buf = (uint8_t*)sensor->first->get_data();
      if (buf)
      {
        memcpy(&x, buf, sizeof(float));
        memcpy(&y, buf + 4, sizeof(float));
        memcpy(&z, buf + 8, sizeof(float));
        return true;
      }
    }
  }
  return false;
}

bool SerialProtocolBase::get_data_as_short(short& val, const uint8_t did)
{
  // check sensor existence
  std::pair<SensorBase*, bool>* sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    val = *((short*)sensor->first->get_data());
    return true;
  }
  return false;
}

bool SerialProtocolBase::get_data_as_unsigned_short(unsigned short& val, const uint8_t did)
{
  // check sensor existence
  std::pair<SensorBase*, bool>* sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    val = *((unsigned short*)sensor->first->get_data());
    return true;
  }
  return false;
}

uint32_t SerialProtocolBase::get_timestamp(const uint8_t did)
{
  // check sensor existence
  uint32_t t = 0;
  std::pair<SensorBase*, bool>* sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    t = sensor->first->get_timestamp();
  }
  return t;
}

/*
// REDO THIS
void SerialProtocolBase::process(uint8_t *buf, size_t buf_len)
{
  const std::vector<SensorBase*> sensors = get_sensors();
  // for each incoming message
  size_t index = 0;
  while (index < buf_len)
  {
    // red the datagram id
    size_t sen_id = (size_t)buf[index];
    dev.get_sensor_by_idx(sen_id);
  for (auto it=sensors.begin();
       it != sensors.end();
       it++)
  {

     for (auto it=sensors.begin(); it != sensors.end(); it++)
    {
      if (it.second.second)
        cached_max_stream_size += it.second.first.len;
    }

    size_t sen_len = it.len
    // read the correct amount of bytes
    try
    {
      buf_len = s->read(buf, HEADER_LEN + 1 + sen_len);
    }
    catch (const std::exception &e) {
      std::cerr << " " <<  e.what() << std::endl;
      if (retry ==5)
        throw std::runtime_error("sp: device failed to answer");
    }
    // check if no error in the return value

    it.parse(buf)

  }

  unpack_data(buf, len)
  }
}

void SerialProtocolBase::unpack_data(uint8_t *buf, size_t len)
{
  valid_header(buf);
  //valid_checksum();
}*/

void SerialProtocolBase::start_streaming(const uint8_t mode)
{
  uint8_t buf[5];
  uint32_t buf_len = 0;
  bool planned_streaming = false;
  switch (mode)
  {
    case SP_CMD_START_STREAM_CONT_ALL:
      planned_streaming = true;
      buf_len = gen_command(buf, SP_DID_MASTER, mode, 0);
      break;
    case SP_CMD_START_STREAM_CONT_SEL:
      planned_streaming = false;
    default:
      break;
  }
  if (buf_len)
  {
    try
    {
      send(buf, buf_len);
      streaming = planned_streaming;
    }
    catch (const std::exception& e)
    {
      sp_error("sp: failed to start streaming " << e.what());
      throw std::runtime_error("sp: failed to start streaming");
    }
  }
}

void SerialProtocolBase::stop_streaming()
{
  uint8_t buf[5];
  uint32_t buf_len;
  buf_len = gen_command(buf, SP_DID_MASTER, SP_CMD_STOP_STREAM, 0);
  try
  {
    send(buf, buf_len);
    streaming = false;
    s->flush();
  }
  catch (const std::exception& e)
  {
    sp_error("sp: failed to stop streaming " << e.what());
    throw std::runtime_error("sp: failed to stop streaming");
  }
}

void SerialProtocolBase::send(const uint8_t* buf, const uint32_t len)
{
  try
  {
    s->write(buf, (size_t)len);
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    throw SPSendException();
  }
}

uint32_t SerialProtocolBase::gen_command(uint8_t* buf, const uint8_t destination, const uint8_t command,
                                         const uint32_t size, const uint16_t stride, const uint8_t* data)
{
  last_cmd = command;
  // memset(buf, HEADER, HEADER_LEN * sizeof(uint8_t)); // cannot be done due to
  // little endianess
  memset(buf, SP_HDR1, sizeof(uint8_t));
  memset(buf + 1, SP_HDR2, sizeof(uint8_t));
  memset(buf + SP_DID_OFFSET, destination, sizeof(uint8_t));
  memset(buf + SP_CMD_OFFSET, command, sizeof(uint8_t));

  if (size && stride && data != NULL)
  {
    // stride is not sent, as it is implicit in the command
    memset(buf + SP_CMD_DATA_SZ_OFFSET, Lowbyte((uint16_t)size), sizeof(uint8_t));
    memset(buf + SP_CMD_DATA_SZ_OFFSET + 1, Highbyte((uint16_t)size), sizeof(uint8_t));
    // stride serves to copy the full data vector
    for (uint32_t i = 0; i < size * stride && i + SP_CMD_DATA_OFFSET < SP_MAX_BUF_SIZE; i++)
    {
      buf[i + SP_CMD_DATA_OFFSET] = data[i];
    }
  }
  uint32_t extra_size = (size > 0 ? SP_CMD_DATA_SZ_LEN : 0) + size * stride;
  buf[SP_CMD_DATA_SZ_OFFSET + extra_size] = compute_checksum(buf, SP_CMD_DATA_SZ_OFFSET + extra_size);
  return SP_CMD_DATA_SZ_OFFSET + extra_size + SP_CHKSUM_LEN;
}

uint32_t SerialProtocolBase::gen_master_config_req(uint8_t* buf)
{
  return gen_command(buf, SP_DID_MASTER, SP_CMD_CONFRQ, 0);
}

uint32_t SerialProtocolBase::gen_master_ping_req(uint8_t* buf)
{
  return gen_command(buf, SP_DID_MASTER, SP_CMD_ALIVE, 0);
}

uint32_t SerialProtocolBase::gen_master_trigger_req(uint8_t* buf)
{
  return gen_command(buf, SP_DID_MASTER, SP_CMD_START_STREAM_TRIG_ALL, 0);
}

uint32_t SerialProtocolBase::gen_sensor_trigger_req(uint8_t* buf, uint8_t sen_id)
{
  return gen_command(buf, sen_id, SP_CMD_START_STREAM_TRIG_SEL, 0);
}

uint32_t SerialProtocolBase::gen_serialnum_req(uint8_t* buf)
{
  return gen_command(buf, SP_DID_MASTER, SP_CMD_SERIAL, 0);
}

uint32_t SerialProtocolBase::gen_topology_req(uint8_t* buf)
{
  return gen_command(buf, SP_DID_MASTER, SP_CMD_TOPORQ, 0);
}

uint32_t SerialProtocolBase::gen_period_master_req(uint8_t* buf, const std::map<uint8_t, uint16_t>& period_map)
{
  /*for (std::map<int, MyClass>::iterator it = Map.begin(); it != Map.end();
  ++it)
  {
    it->second.Method();
  }*/
  uint16_t size = period_map.size();
  if (size)
  {
    uint8_t data[size * SP_PERIOD_DATA_LEN];
    uint16_t i = 0;
    for (const auto& item : period_map)
    {
      data[i * SP_PERIOD_DATA_LEN] = item.first;
      data[i * SP_PERIOD_DATA_LEN + 1] = Lowbyte(item.second);
      data[i * SP_PERIOD_DATA_LEN + 2] = Highbyte(item.second);
      i++;
    }
    return gen_command(buf, SP_DID_MASTER, SP_CMD_PERIOD, size, SP_PERIOD_DATA_LEN, data);
  }
  else
    return 0;
}

uint32_t SerialProtocolBase::gen_period_sensor_req(uint8_t* buf, const uint8_t sen_id, const uint16_t period)
{
  uint8_t data[SP_PERIOD_SENSOR_DATA_LEN];
  data[0] = Lowbyte(period);
  data[1] = Highbyte(period);
  return gen_command(buf, sen_id, SP_CMD_PERIOD, SP_PERIOD_SENSOR_DATA_LEN, 1, data);
}

/*
  uint8_t stopbuf[5] = {0xF0,0xC4, 0x00, 0x00, 0x34};
  //uint8_t stopbuf[5] = {0xF0,0xC4, 0x00, 0xC0, 0xF4};
  //uint8_t stopbuf[5] = {0xF0,0xC4, 0x00, 0xF1, 0xC5};
  //for (int i=0 ; i<20; ++i)
    s.write(stopbuf, 5);

  printf("sc: stopped all transmissions\n");
  try{
  read_len = s.read(buf, 256); // read possibly incomplete frame
    }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }
  printf("sc: reflushed %lu chars\n", read_len);

*/
}  // namespace serial_protocol
