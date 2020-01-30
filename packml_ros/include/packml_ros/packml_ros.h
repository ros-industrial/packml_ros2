/**
 * @license Apache 2.0
 *
 * @copyright Copyright (c) 2017 Shaun Edwards
 * @copyright Copyright (c) 2019 ROS-Industrial Consortium Asia Pacific (ROS 2 compatibility)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef PACKML_ROS_H_
#define PACKML_ROS_H_

#include <chrono>
#include <thread>
#include <sstream>
#include <QtCore>
#include "packml_sm/state_machine.h"
#include "rclcpp/rclcpp.hpp"
#include "packml_msgs/srv/transition.hpp"
#include "packml_msgs/srv/all_status.hpp"


// Global variables for the node topics and services
/**
* @brief Variable that defines the number of state to transition to
*/
class SMNode{
  
    int command_int = 0;


    /**
    * @brief State machine object
    */
    std::shared_ptr<packml_sm::StateMachine> sm;


    /**
    * @brief Pointer for transition service server
    */
    rclcpp::Service<packml_msgs::srv::Transition>::SharedPtr trans_server_;


    /**
    * @brief Pointer for state and elapsed time status update service server
    */
    rclcpp::Service<packml_msgs::srv::AllStatus>::SharedPtr status_server_;


    // Global variables to keep track of time the SM has been in a state
    /**
    * @brief Counter for elapsed time in the stopped state
    */
    float stopped_state_t = 0.0;


    /**
    * @brief Counter for elapsed time in the idle state
    */
    float idle_state_t = 0.0;


    /**
    * @brief Counter for elapsed time in the starting state
    */
    float starting_state_t = 0.0;


    /**
    * @brief Counter for elapsed time in the execute state
    */
    float execute_state_t = 0.0;


    /**
    * @brief Counter for elapsed time in the completing state
    */
    float completing_state_t = 0.0;


    /**
    * @brief Counter for elapsed time in the complete state
    */
    float complete_state_t = 0.0;


    /**
    * @brief Counter for elapsed time in the clearing state
    */
    float clearing_state_t = 0.0;


    /**
    * @brief Counter for elapsed time in the suspended state
    */
    float suspended_state_t = 0.0;


    /**
    * @brief Counter for elapsed time in the aborting state
    */
    float aborting_state_t = 0.0;


    /**
    * @brief Counter for elapsed time in the aborted state
    */
    float aborted_state_t = 0.0;


    /**
    * @brief Counter for elapsed time in the holding state
    */
    float holding_state_t = 0.0;


    /**
    * @brief Counter for elapsed time in the held state
    */
    float held_state_t = 0.0;


    /**
    * @brief Counter for elapsed time in the unholding state
    */
    float unholding_state_t = 0.0;


    /**
    * @brief Counter for elapsed time in the suspending state
    */
    float suspending_state_t = 0.0;


    /**
    * @brief Counter for elapsed time in the unsuspending state
    */
    float unsuspending_state_t = 0.0;


    /**
    * @brief Counter for elapsed time in the resetting state
    */
    float resetting_state_t = 0.0;


    /**
    * @brief Counter for elapsed time in the stopping state
    */
    float stopping_state_t = 0.0;

public:
    /**
    * @brief The class constructor
    */
    SMNode(rclcpp::Node::SharedPtr node){
      // Create SM and connect to Qt components
      // Execute method runs forever until stopped
      // sm = packml_sm::StateMachine::continuousCycleSM();  
      sm = packml_sm::StateMachine::singleCyleSM();  // Execute method runs once
      // Needs to be calibrated with the time of the PLC
      sm->setExecute(std::bind(myExecuteMethod)); 
      sm->activate();
      printf("SM created\n");


      /**
      * @brief Callback function upon transition request by a client
      * @param req - data coming from the client
      * @param res - response to the client
      */
      auto transRequest =
      [this](const std::shared_ptr<packml_msgs::srv::Transition::Request> req,
      std::shared_ptr<packml_msgs::srv::Transition::Response> res) -> void {
        bool command_rtn = false;
        bool command_valid = true;
        command_int= static_cast<int>(req->command);
        std::stringstream ss;
        std::cout << "Evaluating transition request command: " << command_int << std::endl;
        switch(command_int) {
        case 5:
          command_rtn = sm->abort();
          break;
        case 7:
          command_rtn = sm->stop();
          break;
        case 1:
          command_rtn = sm->clear();
          break;
        case 4:
          command_rtn = sm->hold();
          break;
        case 6:
          command_rtn = sm->reset();
          break;
        case 2:
          command_rtn = sm->start();
          break;
        case 3:
          command_rtn = sm->stop();
          break;
        case 100:
          command_rtn = sm->suspend();
          break;
        case 102:
          command_rtn = sm->unhold();
          break;
        case 101:
          command_rtn = sm->unsuspend();
          break;
        default:
          command_valid = false;
          break;
        }
        if(command_valid)
        {
          if(command_rtn)
          {
            ss << "Successful transition request command: " << command_int;
            res->success = true;
            res->error_code = res->SUCCESS;
            res->message = ss.str();
          } else{
            ss << "Invalid transition request command: " << command_int;
            res->success = false;
            res->error_code = res->INVALID_TRANSITION_REQUEST;
            res->message = ss.str();
          }
        }else{
          ss << "Unrecognized transition request command: " << command_int;
          res->success = false;
          res->error_code = res->UNRECGONIZED_REQUEST;
          res->message = ss.str();
        }
      };



      /**
      * @brief Callback function upon status update request by a client
      * @param req - data coming from the client
      * @param res - response to the client
      */
      auto statusRequest =
      [this](const std::shared_ptr<packml_msgs::srv::AllStatus::Request> req,
      std::shared_ptr<packml_msgs::srv::AllStatus::Response> res) -> void {
        (void) req;
        int curr_state_ = getCurrentState();
        res->stopped_state = false;
        res->idle_state = false;
        res->starting_state = false;
        res->execute_state = false;
        res->completing_state = false;
        res->complete_state = false;
        res->clearing_state = false;
        res->suspended_state = false;
        res->aborting_state = false;
        res->aborted_state = false;
        res->holding_state = false;
        res->held_state = false;
        res->unholding_state = false;
        res->suspending_state = false;
        res->unsuspending_state = false;
        res->resetting_state = false;
        res->stopping_state = false;
        switch (curr_state_){
          case 2:
            res->stopped_state = true;
            stopped_state_t = stopped_state_t + 0.2;
            break;
          case 3: 
            res->starting_state = true;
            starting_state_t = starting_state_t + 0.2;
            break;
          case 4:
            res->idle_state = true;
            idle_state_t = idle_state_t + 0.2;
            break;
          case 5:
            res->suspended_state = true;
            suspended_state_t = suspended_state_t + 0.2;
            break;
          case 6:
            res->execute_state = true;
            execute_state_t = execute_state_t + 0.2;
            break;
          case 7:
            res->stopping_state = true;
            stopping_state_t = stopping_state_t  + 0.2;
            break;
          case 8:
            res->aborting_state = true;
            aborting_state_t = aborting_state_t + 0.2;
            break;
          case 9:
            res->aborted_state = true;
            aborted_state_t = aborted_state_t + 0.2;
            break;
          case 10:
            res->holding_state = true;
            holding_state_t = holding_state_t + 0.2;
            break;
          case 11:
            res->held_state = true;
            held_state_t = held_state_t + 0.2;
            break;
          case 100:
            res->resetting_state = true;
            resetting_state_t = resetting_state_t + 0.2;
            break;
          case 101:
            res->suspending_state = true;
            suspending_state_t = suspending_state_t + 0.2;
            break;
          case 102:
            res->unsuspending_state = true;
            unsuspending_state_t = unsuspending_state_t + 0.2;
            break;
          case 103:
            res->clearing_state = true;
            clearing_state_t = clearing_state_t + 0.2;
            break;
          case 104:
            res->unholding_state = true;
            unholding_state_t = unholding_state_t + 0.2;
            break;
          case 105:
            res->completing_state = true;
            completing_state_t = completing_state_t + 0.2;
            break;
          case 106:
            res->complete_state = true;
            complete_state_t = complete_state_t + 0.2;
            break;
          default:
            break;
        }
        res->t_stopped_state = stopped_state_t;
        res->t_starting_state = starting_state_t;
        res->t_idle_state = idle_state_t;
        res->t_suspended_state = suspended_state_t;
        res->t_execute_state = execute_state_t;
        res->t_stopping_state = stopping_state_t;
        res->t_aborting_state = aborting_state_t;
        res->t_aborted_state = aborted_state_t; 
        res->t_holding_state = holding_state_t;
        res->t_held_state = held_state_t;
        res->t_resetting_state = resetting_state_t;
        res->t_suspending_state = suspending_state_t;
        res->t_unsuspending_state = unsuspending_state_t;
        res->t_clearing_state = clearing_state_t;
        res->t_unholding_state = unholding_state_t;
        res->t_completing_state = completing_state_t;
        res->t_complete_state = complete_state_t;
      };
      // Create service to control the execution of the SM from RViz GUI
      trans_server_ = node->create_service<packml_msgs::srv::Transition>("transition", 
        transRequest);
      status_server_ = node->create_service<packml_msgs::srv::AllStatus>("allStatus", 
        statusRequest);
    }

    /**
    * @brief The class constructor
    */
    virtual ~SMNode(){ }


    /**
    * @brief Function to bind to the Execute state for the state machine, waiting for a set amount
    * of time
    * @return 0 - when completed
    */
    static int myExecuteMethod()
    {
      printf("This is my execute method (begin)\n");
      std::this_thread::sleep_for(std::chrono::seconds(10));
      printf("This is my execute method (end)\n");
      return 0;  // returning zero indicates non-failure
    }


    /**
    * @brief Function to query for the current state machine state.
    * @return state int
    */
    virtual int getCurrentState(){
      return sm->getCurrentState();
    }
};

/**
* @brief Function to be run in a thread to execute a QT object for a state machine
*/
void qtWorker(int argc, char * argv[])
{
  QCoreApplication a(argc, argv);
  a.exec();
  printf("Thread ready\n");
}

#endif  // PACKML_ROS_H_
