/*
	RFT Force Torque sensor. - serial communication version
    
	Simple stand-alone ROS node that takes data from RFT sensor 
	and Publishes it to a ROS topic	
	
	website: www.robotous.com
	e-mail: support@robotous.com
	
	Modified by Yash Vyas, University of Padua
	  - Ubuntu 24.04 LTS
	  - ROS2 Jazzy
*/

/*
	ver. 0.0.0, 2017.11.29 (Robotous)
	Ver. 0.0.1, 2017.12.18 (Robotous)
  ver. 1.0.1, 2024.06.17 (Yash)
*/
#define ROS_RFT_SERIAL_SW_VER	"VER 1.0.1(Read Only)"

#include <chrono>
#include <cstdio>
#include <stdlib.h>
#include <unistd.h>
#include <memory>
#include <string>
#include <thread>
#include <mutex>

#include "RFT_UART_SAMPLE.h"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>

using namespace std::chrono_literals;

/*
	Definitions
*/
#define RFT_SERVICE_OK  			(0)
#define RFT_SERVICE_RQST_TIMEOUT 	(0xF0)

/*
	Global Variables
*/
CRT_RFT_UART RFT_SENSOR;

/*
	Functions
*/
/*
	The main function of RFT Serial Node.....
	- you can send the operation command using rqt -> service caller
	- you can see the force/torque data using rqt_plot
	  * rqt_plot /RFT_FORCE/wrench/force
	  * rqt_plot /RFT_FORCE/wrench/torque
*/
class RFTSensorSerial : public rclcpp::Node {
  public:
    RFTSensorSerial() : Node("rft_sensor_serial") {
      // initial data
      read_fqs_ = {{200,0},{20,1},{40,2},{100,3},
        {200,4},{333,4},{350,6},{500,7},{1000,8}}; // map of output frequencies
      filter_fqs_ = {{500,1},{300,2},{200,3},{150,4},
        {100,5},{100,6},{50,6},{40,7},{30,8},{20,9},{10,10},{5,11},
        {3,12},{2,13},{1,14}};
      baud_params_ = {{115200,B115200},{921600,B921600},{460800,B460800},{230400,B230400},{57600,B57600}};

      RCLCPP_INFO(this->get_logger(), "Initializing ROBOTOUS sensor");
	    
      this->declare_parameter<std::string>("DEV_NAME", "/dev/ttyUSB0");
      this->declare_parameter<int>("PORT", 0);
      this->declare_parameter<int>("BAUD", 115200);
      this->declare_parameter<float>("FORCE_DIVIDER", 50.0f);
      this->declare_parameter<float>("TORQUE_DIVIDER", 2000.0f);
      this->declare_parameter<int>("FREQUENCY",200);
      this->declare_parameter<bool>("SET_CUTOFF_FILTER", false);
      this->declare_parameter<int>("FILTER_FREQUENCY",500);
      this->declare_parameter<float>("BIAS_FX",0.0f);
      this->declare_parameter<float>("BIAS_FY",0.0f);
      this->declare_parameter<float>("BIAS_FZ",0.0f);
      this->declare_parameter<float>("BIAS_TX",0.0f);
      this->declare_parameter<float>("BIAS_TY",0.0f);
      this->declare_parameter<float>("BIAS_TZ",0.0f);

      this->get_parameter("DEV_NAME", dev_name_);
      this->get_parameter("PORT", port_);
      this->get_parameter("BAUD", baud_rate_);
      this->get_parameter("FORCE_DIVIDER", force_divider_);
      this->get_parameter("TORQUE_DIVIDER", torque_divider_);
      this->get_parameter("FREQUENCY", read_freq_);
      this->get_parameter("SET_CUTOFF_FILTER", cutoff_filter_);
      this->get_parameter("FILTER_FREQUENCY", filter_freq_);
      this->get_parameter("BIAS_FX",bias_Fx);
      this->get_parameter("BIAS_FY",bias_Fy);
      this->get_parameter("BIAS_FZ",bias_Fz);
      this->get_parameter("BIAS_TX",bias_Tx);
      this->get_parameter("BIAS_TY",bias_Ty);
      this->get_parameter("BIAS_TZ",bias_Tz);

      RCLCPP_INFO(this->get_logger(), "RFT Serial device name and port: %s%d", dev_name_.c_str(), port_);
      RCLCPP_INFO(this->get_logger(), "RFT Serial port baud-rate: %d", baud_rate_);
      RCLCPP_INFO(this->get_logger(), "Force Divider of RFT sensor: %f", force_divider_);
      RCLCPP_INFO(this->get_logger(), "Torque Divider of RFT sensor: %f", torque_divider_);
      RCLCPP_INFO(this->get_logger(), "Read/Publish Frequency %d", read_freq_);
      RCLCPP_INFO(this->get_logger(), "Set Cutoff filter: %s", cutoff_filter_ ? "True" : "False");
      RCLCPP_INFO(this->get_logger(), "Cutoff filter frequency: %d", filter_freq_);
      RCLCPP_INFO(this->get_logger(), "Fx Bias set to %f.",bias_Fx);
      RCLCPP_INFO(this->get_logger(), "Fy Bias set to %f.",bias_Fy);
      RCLCPP_INFO(this->get_logger(), "Fz Bias set to %f.",bias_Fz);
      RCLCPP_INFO(this->get_logger(), "Tx Bias set to %f.",bias_Tx);
      RCLCPP_INFO(this->get_logger(), "Ty Bias set to %f.",bias_Ty);
      RCLCPP_INFO(this->get_logger(), "Tz Bias set to %f.",bias_Tz);

      bias_status_ = false;

      check_params(); // does check to see if params are correct


      bool success = init_sensor();
      if (success) {
        RCLCPP_INFO(this->get_logger(), "RFT Force/Torque Sensor <Serial> is ready!");
      } else {
        RCLCPP_ERROR(this->get_logger(), "RFT Force/Torque Sensor has hit an error during startup.");
        rclcpp::shutdown();
      }
      
      publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("RFT_FORCE", 10);
      bias_service_ = this->create_service<std_srvs::srv::SetBool>(
          "set_bias",
          std::bind(&RFTSensorSerial::bias_service_callback_, this, std::placeholders::_1, std::placeholders::_2));
      
      set_output_off_service_ = this->create_service<std_srvs::srv::Trigger>(
        "set_output_off",
        std::bind(&RFTSensorSerial::set_output_off_callback_, this, std::placeholders::_1, std::placeholders::_2));

      set_output_on_service_ = this->create_service<std_srvs::srv::Trigger>(
        "set_output_on",
        std::bind(&RFTSensorSerial::set_output_on_callback_, this, std::placeholders::_1, std::placeholders::_2));

      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(1), // set higher than read frequency
          std::bind(&RFTSensorSerial::timer_callback, this));

      
    };

    void execute_node() {
      rclcpp::Rate rate(100000);
      bool isSensorOk = false;
      while(rclcpp::ok()) {
        std::unique_lock<std::mutex> lock(com_port_mutex_);
        isSensorOk = RFT_SENSOR.readWorker();
        lock.unlock();
        
        if( (RFT_SENSOR.m_nCurrMode == CMD_FT_CONT) && isSensorOk )
        {
          RCLCPP_INFO(this->get_logger(), "published");
          auto wrench_msg = geometry_msgs::msg::WrenchStamped();
          wrench_msg.header.stamp = this->now();
          wrench_msg.wrench.force.x = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[0]+bias_Fx;
          wrench_msg.wrench.force.y = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[1]+bias_Fy;
          wrench_msg.wrench.force.z = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[2]+bias_Fz;
          wrench_msg.wrench.torque.x = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[3]+bias_Tx;
          wrench_msg.wrench.torque.y = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[4]+bias_Ty;
          wrench_msg.wrench.torque.z = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[5]+bias_Tz;
          // RCLCPP_INFO(this->get_logger(), "Published force/torque data: "
          //                               "Force (%.2f, %.2f, %.2f) N, Torque (%.2f, %.2f, %.2f) N.m",
          //                               wrench_msg.wrench.force.x, wrench_msg.wrench.force.y, wrench_msg.wrench.force.z,
          //                               wrench_msg.wrench.torque.x, wrench_msg.wrench.torque.y, wrench_msg.wrench.torque.z);
          publisher_->publish(wrench_msg);
        } else {
          rate.sleep();
        }
      }
    }

  private:
    bool init_sensor() {
      DWORD baud = (DWORD)baud_reg_;
      DWORD byte_size = CS8;
      BYTE port = (BYTE)port_;

      if (!RFT_SENSOR.openPort((char*)dev_name_.c_str(), port, baud, byte_size)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to open interface: %s port %d", dev_name_.c_str(), port_);
          return false;
      }
      
      // Set force/torque output frequency
      RCLCPP_INFO(this->get_logger(), "read_freq_param_ %d", read_freq_param_);

      // set frequency with wait
      // set_filter();

      RFT_SENSOR.m_RFT_IF_PACKET.setDivider(force_divider_, torque_divider_);
      // Start continuous mode output
      if (!RFT_SENSOR.rqst_FT_Continuous()) {
          RCLCPP_ERROR(this->get_logger(), "Failed to start continuous output.");
      }

      RCLCPP_INFO(this->get_logger(), "Started continuous output.");

      return true;
    }

    void check_params() {
      // check baud rate correct, and frequency / baud rate combination is correct
      // right now only check read_frequency is implemented

      if (baud_params_.find(baud_rate_) != baud_params_.end()) {
        baud_reg_ = baud_params_.at(baud_rate_);
      } else {
        baud_rate_ = B115200;
        RCLCPP_INFO(this->get_logger(), "Baud rate not within set of permissible values! Set to default of %d", 
          baud_rate_);
      }

      if (read_fqs_.find(read_freq_) != read_fqs_.end()) {
        read_freq_param_ = read_fqs_.at(read_freq_);
        if (((read_freq_ > 500) && (baud_rate_ < 921600)) ||
            ((read_freq_ > 333) && (baud_rate_ < 230400)) ||
            ((read_freq_ > 200) && (baud_rate_ < 115200)))
        {
          read_freq_ = 200;
          read_freq_param_ = 0;
          RCLCPP_INFO(this->get_logger(), "Read Frequency not feasible with baud rate of %d! Set to default of %d Hz", 
          baud_rate_, read_freq_);
        }
      } else {
        read_freq_ = 200;
        read_freq_param_ = 0;
        RCLCPP_INFO(this->get_logger(), "Read Frequency not within set of permissible values! Set to default of %d Hz", 
          read_freq_);
      }

      if (filter_fqs_.find(filter_freq_) != filter_fqs_.end()) {
        filter_param_ = filter_fqs_.at(filter_freq_);
        RCLCPP_INFO(this->get_logger(), "Filter frequency: %d Hz", filter_freq_);
      } else {
        filter_freq_ = 500;
        filter_param_ = 0;
        RCLCPP_INFO(this->get_logger(), "Filter Frequency not within set of permissible values! Set to default of %d,"
          " only applicable if the filter is ON.", filter_freq_);
      }
    }

    void bias_service_callback_(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        bool request_status = request->data;
        bool success = true;
        std::string message;

        std::unique_lock<std::mutex> lock(com_port_mutex_);
        
        if (request_status) {
          if (RFT_SENSOR.set_FT_Bias(1)) {
                    bias_status_ = true;
                    message = "Bias set to ON.";
                } else {
                    success = false;
                    message = "Failed to set bias to ON.";
                }
        } else {
          if (RFT_SENSOR.set_FT_Bias(0)) {
                    bias_status_ = true;
                    message = "Bias set to OFF.";
                } else {
                    success = false;
                    message = "Failed to set bias to OFF.";
                }
        }
        lock.unlock();

        // Set response for the service call
        response->success = success;
        response->message = message;

        RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    }

    void set_output_off_callback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        bool success;
        RCLCPP_INFO(this->get_logger(), "Set output OFF service called");

        std::unique_lock<std::mutex> lock(com_port_mutex_);
        if (RFT_SENSOR.rqst_FT_Stop()) {
            RCLCPP_INFO(this->get_logger(), "Stopped continuous output.");
            success = true;
          } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to stop continuous output.");
            success = false;
          }
        
        // if (request_status) {
        //   RCLCPP_INFO(this->get_logger(), "Request status ON");
        //   if (RFT_SENSOR.rqst_FT_Continuous()) {
        //     RCLCPP_INFO(this->get_logger(), "Started continuous output.");
        //   } else {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to start continuous output.");
        //     success = false;
        //   }
        // } else {
        //   if (RFT_SENSOR.rqst_FT_Stop()) {
        //     RCLCPP_INFO(this->get_logger(), "Stopped continuous output.");
        //   } else {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to stop continuous output.");
        //     success = false;
        //   }
        // }
        lock.unlock();

        // Set response for the service call
        response->success = success;
        response->message = success ? "Service call successful." : "Service call failed";
    }

    void set_output_on_callback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        bool success = true;
        RCLCPP_INFO(this->get_logger(), "Set output ON service called");

        std::unique_lock<std::mutex> lock(com_port_mutex_);
        RFT_SENSOR.rqst_FT_Continuous();
        // if (RFT_SENSOR.rqst_FT_Continuous()) {
        //     RCLCPP_INFO(this->get_logger(), "Started continuous output.");
        //     success = true;
        //   } else {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to start continuous output.");
        //     success = false;
        //   }
        lock.unlock();

        // Set response for the service call
        response->success = success;
        response->message = success ? "Service call successful." : "Service call failed";
    }

    void timer_callback() {
      rclcpp::Rate rate(1000);
      bool isSensorOk = false;
      while(rclcpp::ok()) {
        std::unique_lock<std::mutex> lock(com_port_mutex_);
        isSensorOk = RFT_SENSOR.readWorker();
        lock.unlock();
        
        // RCLCPP_INFO(this->get_logger)
        if( (RFT_SENSOR.m_nCurrMode == CMD_FT_CONT) && isSensorOk )
        {
          auto wrench_msg = geometry_msgs::msg::WrenchStamped();
          wrench_msg.header.stamp = this->now();
          wrench_msg.wrench.force.x = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[0]+bias_Fx;
          wrench_msg.wrench.force.y = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[1]+bias_Fy;
          wrench_msg.wrench.force.z = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[2]+bias_Fz;
          wrench_msg.wrench.torque.x = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[3]+bias_Tx;
          wrench_msg.wrench.torque.y = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[4]+bias_Ty;
          wrench_msg.wrench.torque.z = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[5]+bias_Tz;
          // RCLCPP_INFO(this->get_logger(), "Published force/torque data: "
          //                               "Force (%.2f, %.2f, %.2f) N, Torque (%.2f, %.2f, %.2f) N.m",
          //                               wrench_msg.wrench.force.x, wrench_msg.wrench.force.y, wrench_msg.wrench.force.z,
          //                               wrench_msg.wrench.torque.x, wrench_msg.wrench.torque.y, wrench_msg.wrench.torque.z);
          publisher_->publish(wrench_msg);
          break;
        } else {
          rate.sleep();
        }
      }
    }

    void set_filter(){
      RFT_SENSOR.set_FT_Filter_Type(1, 3);
      rclcpp::Rate rate(100);

      int waitTimeOutCnt = 0;

      while(RFT_SENSOR.m_bIsRcvd_Response_Pkt == false) {
        RCLCPP_INFO(this->get_logger(),"%d\n", waitTimeOutCnt);
        if (waitTimeOutCnt >= 500) break;
        rate.sleep();
        waitTimeOutCnt++;
      }

      // do{
      //   rate.sleep();
      //   waitTimeOutCnt++;
      //   RCLCPP_INFO(this->get_logger(),"%d\n", waitTimeOutCnt);
      //   if (waitTimeOutCnt >= 50)
      //     break;
      // } while (RFT_SENSOR.m_bIsRcvd_Response_Pkt == false);

      if (RFT_SENSOR.m_bIsRcvd_Response_Pkt)
      {
        if (RFT_SENSOR.m_RFT_IF_PACKET.m_response_result != 1)
        {
          RCLCPP_INFO(this->get_logger(),"Failed to set filter");
        }
        else
        {
          RCLCPP_INFO(this->get_logger(),"Successfully set filter");
        }
      }
      else
      {
        RCLCPP_INFO(this->get_logger(),"Time Out");
      }
    }

    uint8_t rft_response_wait(uint8_t opType){
      int result = RFT_SERVICE_OK;
      int waitTimeOut = 0;
      
      bool isRcvd = false;
      do
      {
        if( waitTimeOut >= 50 )
        {
          RCLCPP_WARN(this->get_logger(),"RCVD SERVICE TIMEOUT");
          isRcvd = true;
          result = RFT_SERVICE_RQST_TIMEOUT;
        }
        
        if( RFT_SENSOR.m_bIsRcvd_Response_Pkt)
        {
          isRcvd = true;
          rft_response_display(opType);
        }
        
        waitTimeOut++;
        usleep(10000);
        
      }while( isRcvd == false );
      
      return result;
    }

    uint8_t rft_response_display(uint8_t opType){
      uint8_t result = RFT_SERVICE_OK;
      switch ( opType )
      {
      case CMD_GET_PRODUCT_NAME:
        RCLCPP_INFO(this->get_logger(),"%s", RFT_SENSOR.m_RFT_IF_PACKET.m_rcvd_product_name);
        break;
      case CMD_GET_SERIAL_NUMBER:
        RCLCPP_INFO(this->get_logger(),"%s", RFT_SENSOR.m_RFT_IF_PACKET.m_rcvd_serial_number);
        break;
      case CMD_GET_FIRMWARE_VER:
        RCLCPP_INFO(this->get_logger(),"%s", RFT_SENSOR.m_RFT_IF_PACKET.m_rcvd_firmware_version);
        break;
      case CMD_SET_ID:
        result = NOT_SUPPORTED_CMD;
        break;
      case CMD_GET_ID:
        result = NOT_SUPPORTED_CMD;
        break;
      case CMD_SET_COMM_BAUDRATE:
        RCLCPP_INFO(this->get_logger(),"Cmd Type: %d, Result: %d, Err. Code: %d", RFT_SENSOR.m_RFT_IF_PACKET.m_response_cmd, RFT_SENSOR.m_RFT_IF_PACKET.m_response_result, RFT_SENSOR.m_RFT_IF_PACKET.m_response_errcode );
        result = RFT_SENSOR.m_RFT_IF_PACKET.m_response_errcode;
        break;
      case CMD_GET_COMM_BAUDRATE:
        RCLCPP_INFO(this->get_logger(),"Baud: %d, new Baud: %d", RFT_SENSOR.m_RFT_IF_PACKET.m_rcvd_curr_comm_baudrate, RFT_SENSOR.m_RFT_IF_PACKET.m_rcvd_set_comm_baudrate);
        break;
      case CMD_SET_FT_FILTER:
        RCLCPP_INFO(this->get_logger(),"Cmd Type: %d, Result: %d, Err. Code: %d", RFT_SENSOR.m_RFT_IF_PACKET.m_response_cmd, RFT_SENSOR.m_RFT_IF_PACKET.m_response_result, RFT_SENSOR.m_RFT_IF_PACKET.m_response_errcode );
        result = RFT_SENSOR.m_RFT_IF_PACKET.m_response_errcode;
        break;
      case CMD_GET_FT_FILTER:
        RCLCPP_INFO(this->get_logger(),"filter type: %d, sub. setting: %d", RFT_SENSOR.m_RFT_IF_PACKET.m_rcvd_filter_type, RFT_SENSOR.m_RFT_IF_PACKET.m_rcvd_filter_setting_value);
        break;
      case CMD_FT_ONCE:
        RCLCPP_INFO(this->get_logger(),"%0.3f, %.03f, %.03f, %0.3f, %.03f, %.03f",
        RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[0], RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[1], RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[2],
        RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[3], RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[4], RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[5] );	
        break;
      case CMD_FT_CONT:
        // This response packet is published.. see the while() of main() function.
        RCLCPP_INFO(this->get_logger(),"RFT Force Toque topic will be published...");
        break;
      case CMD_FT_CONT_STOP:
        // THERE IS NO - RESPONSE PACKET
        break;
      case CMD_RESERVED_1:
        result = NOT_SUPPORTED_CMD;
        break;
      case CMD_RESERVED_2:
        result = NOT_SUPPORTED_CMD;
        break;
      case CMD_SET_CONT_OUT_FRQ:
        RCLCPP_INFO(this->get_logger(),"Cmd Type: %d, Result: %d, Err. Code: %d", RFT_SENSOR.m_RFT_IF_PACKET.m_response_cmd, RFT_SENSOR.m_RFT_IF_PACKET.m_response_result, RFT_SENSOR.m_RFT_IF_PACKET.m_response_errcode );
        result = RFT_SENSOR.m_RFT_IF_PACKET.m_response_errcode;
        break;
      case CMD_GET_CONT_OUT_FRQ:
        RCLCPP_INFO(this->get_logger(),"output frq: %d", RFT_SENSOR.m_RFT_IF_PACKET.m_rcvd_tx_frq);
        break;
      case CMD_SET_BIAS:
        // THERE IS NO - RESPONSE PACKET
        break;
      case CMD_GET_OVERLOAD_COUNT:
        RCLCPP_INFO(this->get_logger(),"overload ount: %d, %d, %d, %d, %d, %d", 
        RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdOverloadCnt[0], RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdOverloadCnt[1], RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdOverloadCnt[2],
        RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdOverloadCnt[3], RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdOverloadCnt[4], RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdOverloadCnt[5] );
        break;

      default:
        result = NOT_SUPPORTED_CMD;
        break;
      }	
      
      return result;
    }

    std::string dev_name_;
    int port_;
    int baud_rate_;
    DWORD baud_rate_dw_;
    int baud_reg_;
    float force_divider_;
    float torque_divider_;
    int read_freq_;
    int read_freq_param_;
    bool cutoff_filter_;
    int filter_freq_;
    int filter_param_;
    bool bias_status_;
    unsigned long seq_;

    float bias_Fx;
    float bias_Fy;
    float bias_Fz;
    float bias_Tx;
    float bias_Ty;
    float bias_Tz;

    std::map<int, int> read_fqs_;
    std::map<int, int> filter_fqs_;
    std::map<int, int> baud_params_;

    std::mutex com_port_mutex_;
    CRT_RFT_UART RFT_SENSOR;

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr bias_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_output_off_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_output_on_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RFTSensorSerial>();
  // node->execute_node();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

