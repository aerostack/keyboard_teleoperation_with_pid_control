/*!*******************************************************************************************
 * \brief     Keyboard teleoperation with pid control implementation file.
 * \authors   Alberto Rodelgo
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include "../include/keyboard_teleoperation_with_pid_control_main.h"

void spinnerThread(){
  ros::spin();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "KEYBOARD TELEOPERATION WITH PID CONTROL");
  ros::NodeHandle n("~");
  n.param<std::string>("drone_id_namespace", drone_id_namespace, "drone1");
  std::thread thr(&spinnerThread);

  ros::param::get("~flight_action_topic_name", flight_action_topic_name);
  ros::param::get("~speed_ref_topic_name", speed_ref_topic_name);
  ros::param::get("~pose_ref_topic_name", pose_ref_topic_name);
  ros::param::get("~self_pose_topic_name", self_pose_topic_name);
  ros::param::get("~command_pitch_roll_topic_name", command_pitch_roll_topic_name);
  ros::param::get("~self_speed_topic_name", self_speed_topic_name);
  ros::param::get("~set_control_mode_service_name", set_control_mode_service_name);
  ros::param::get("~assumed_control_mode_topic_name", assumed_control_mode_topic_name);

  // ncurses initialization
  setlocale(LC_ALL, "");
  std::setlocale(LC_NUMERIC, "C");
  initscr();
  start_color();
  use_default_colors();  
  curs_set(0);
  noecho();
  nodelay(stdscr, TRUE);
  erase();
  refresh();
  init_pair(1, COLOR_BLUE, -1);
  init_pair(2, COLOR_GREEN, -1);
  init_pair(3, COLOR_CYAN, -1);
  init_pair(4, COLOR_RED, -1);
  init_pair(5, COLOR_YELLOW, -1);


  //Input variable
  char command = 0;

  //Publishers
  command_publ = n.advertise<aerostack_msgs::FlightActionCommand>("/" + drone_id_namespace + "/"+ flight_action_topic_name, 1, true);
  speed_reference_publ = n.advertise<geometry_msgs::TwistStamped>("/"+drone_id_namespace+"/"+speed_ref_topic_name, 1, true);
  pose_reference_publ = n.advertise<geometry_msgs::PoseStamped>("/"+drone_id_namespace+"/"+pose_ref_topic_name, 1, true);
  attitude_publ = n.advertise<std_msgs::Int8>("/"+drone_id_namespace+"/keyboard_teleoperation_with_pid_control/attitude_control", 1, true);

  //Subscribers
  self_pose_sub = n.subscribe("/"+drone_id_namespace+"/"+self_pose_topic_name, 1, selfLocalizationPoseCallback);
  control_mode = n.subscribe("/" + drone_id_namespace + "/" + assumed_control_mode_topic_name, 1, controlModeCallback);
  speed_reference_sub = n.subscribe("/"+drone_id_namespace+"/"+speed_ref_topic_name, 1, speedReferenceCallback);
  attitude_sub = n.subscribe("/"+drone_id_namespace+"/keyboard_teleoperation_with_pid_control/attitude_control", 1, attitudeCallback);
  ground_speed_sub = n.subscribe("/"+drone_id_namespace+"/"+self_speed_topic_name, 1, selfSpeedCallback);
  //Wait 3sec for initialization
  sleep(3);

  setControlModeClientSrv = n.serviceClient<aerostack_msgs::SetControlMode>("/" + drone_id_namespace + "/"+set_control_mode_service_name);
  
  //Teleoperation control mode
  current_mode = GROUND_SPEED;

  move(0,0);clrtoeol();
  printw("                - KEYBOARD TELEOPERATION WITH PID CONTROL -");
  move(3,0);clrtoeol();
  printw("--------------------------------------------------------------------------------");
  //Print controls
  printoutGroundSpeedControls();
  //LOOP
  ros::Rate loop_rate(FREQ_INTERFACE);

  while (ros::ok()){
    // Read messages
    ros::spinOnce();

    //Print control mode
    switch(current_mode){
          case POSE:
            move(2, 0);
            attron(COLOR_PAIR(2));printw("                        Teleoperation mode: Pose      "); attroff(COLOR_PAIR(2));
            clrtoeol(); refresh();  
          break;
          case ATTITUDE:
            move(2, 0);
            attron(COLOR_PAIR(3));printw("                        Teleoperation mode: Attitude      "); attroff(COLOR_PAIR(3));
            clrtoeol(); refresh();  
          break;
          case GROUND_SPEED:
            move(2, 0);
            attron(COLOR_PAIR(4));printw("                        Teleoperation mode: Ground speed         "); attroff(COLOR_PAIR(4));
            clrtoeol(); refresh();  
          break;
          default:
            move(2, 0);
            printw("                        Teleoperation mode: Unknown          ");
            clrtoeol(); refresh();   
          break;      
    }

    move(16,0);
    printw("                        Last key pressed: ");
    //Read command
    command = getch();
    switch (command){
      case 't':  // Take off
        takeOff();
        printw("t       ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Take off           ");clrtoeol();
        sleep(3);
        motion_reference_pose_msg.pose.position.z = self_localization_pose_msg.pose.position.z;
        pose_reference_publ.publish(motion_reference_pose_msg);
        //startQuadrotorControllerClientSrv=n.serviceClient<std_srvs::Empty>("/"+drone_id_namespace+"/quadrotor_pid_controller_process/start");
        //startQuadrotorControllerClientSrv.call(req);
        switch(current_mode){
          case POSE:
              setControlMode(aerostack_msgs::MotionControlMode::POSE);
              printoutPoseControls(); 
          break;
          case ATTITUDE:
              setControlMode(aerostack_msgs::MotionControlMode::ATTITUDE);
              printoutAttitudeControls();
          break;
          case GROUND_SPEED:
              setControlMode(aerostack_msgs::MotionControlMode::GROUND_SPEED);
              printoutGroundSpeedControls();         
          break;
        }
        move();
        break;
      case 'y':  // Land
        hover();
        land();
        //startQuadrotorControllerClientSrv=n.serviceClient<std_srvs::Empty>("/"+drone_id_namespace+"/quadrotor_pid_controller_process/stop");
        //startQuadrotorControllerClientSrv.call(req);
        printw("y        ");clrtoeol(); 
        move(17, 0); 
        printw("                        Last command:     Land             ");clrtoeol();            
        break;
      case ' ':  // Emergency stop 
        emergencyStop();
        printw("space     ");clrtoeol(); 
        move(17, 0); 
        printw("                        Last command:     Emergency stop            ");clrtoeol();
        break;
      case 'h':  // Hover   
      {
        hover();
        printw("h      ");clrtoeol();
        move(17, 0); printw("                        Last command:     Keep hovering             ");clrtoeol();refresh();

          if (setControlMode(aerostack_msgs::MotionControlMode::SPEED)){
              clearSpeedReferences();
              publishSpeedReference();
              boost::this_thread::sleep(boost::posix_time::milliseconds(2000)); //2 seconds
              while(abs(self_speed_msg.twist.linear.x) >= 0.01 || abs(self_speed_msg.twist.linear.y) >= 0.01){
                //Waiting
              }   
              if (current_mode == POSE){
                setControlMode(aerostack_msgs::MotionControlMode::POSE);
              }      
              if (current_mode == ATTITUDE){
                setControlMode(aerostack_msgs::MotionControlMode::ATTITUDE);
              }      
              if (current_mode == GROUND_SPEED){
                setControlMode(aerostack_msgs::MotionControlMode::GROUND_SPEED);
              }   
              motion_reference_pose_msg.pose = self_localization_pose_msg.pose;                                             
              pose_reference_publ.publish(motion_reference_pose_msg);        
        }
        break;
      }
      case 'q':  // Move upwards
        move();
        motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z + CTE_ALTITUDE;
        motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
        pose_reference_publ.publish(motion_reference_pose_msg);
        printw("q      ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Increase altitude         ");clrtoeol();
        break;
      case 'a':  //Move downwards
        move();
          motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z - CTE_ALTITUDE;
          motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
          pose_reference_publ.publish(motion_reference_pose_msg);
        printw("a        ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Decrease altitude         ");clrtoeol(); 
        break;         
      case 'z':  //(yaw) turn counter-clockwise
      {        
        move();       
        double yaw,r,p;
        toEulerianAngle(motion_reference_pose_msg, &r,&p,&yaw);
        q_rot.setRPY(r, p, yaw + CTE_YAW);
        current_commands_yaw = yaw + CTE_YAW;
        motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
        motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
        motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
        motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
        motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
        pose_reference_publ.publish(motion_reference_pose_msg);
        printw("z       ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Turn counter-clockwise        ");clrtoeol();  
        break;    
      }      
      case 'x':  // (yaw) turn clockwise
      {      
        move();       
        double yaw,r,p;
        toEulerianAngle(motion_reference_pose_msg, &r,&p,&yaw);
        q_rot.setRPY(r, p, yaw - CTE_YAW);
        current_commands_yaw = yaw - CTE_YAW;
        motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
        motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
        motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
        motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
        motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
        pose_reference_publ.publish(motion_reference_pose_msg);
        printw("x      ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Turn clockwise          ");clrtoeol();
      }
        break;                 
      case ASCII_KEY_RIGHT:
        move();
        if (current_mode == GROUND_SPEED){
          speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x;
          speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y - CTE_SPEED;                       
          publishSpeedReference();
        }
        if (current_mode == POSE){
          motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
          motion_reference_pose_msg.pose.position.x = motion_reference_pose_msg.pose.position.x;
          motion_reference_pose_msg.pose.position.y = motion_reference_pose_msg.pose.position.y - CTE_POSE;
          motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }
        if (current_mode == ATTITUDE){
          current_commands_roll = CTE_COMMANDS;
          current_commands_pitch = current_commands_pitch;
          q_rot.setRPY(current_commands_roll, current_commands_pitch, current_commands_yaw);
          motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
          motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
          motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
          motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
          motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
          pose_reference_publ.publish(motion_reference_pose_msg);
          std_msgs::Int8 msg_int;
          msg_int.data = RIGHT;
          attitude_publ.publish(msg_int);
        }   
        printw("\u2192            ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Increase movement to the right        ");clrtoeol();  
        break;               
      case ASCII_KEY_LEFT:
        move();
        if (current_mode == GROUND_SPEED){
          speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x;
          speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y + CTE_SPEED;                       
          publishSpeedReference();
        }
        if (current_mode == POSE){
          motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
          motion_reference_pose_msg.pose.position.x = motion_reference_pose_msg.pose.position.x;
          motion_reference_pose_msg.pose.position.y = motion_reference_pose_msg.pose.position.y + CTE_POSE;
          motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }
        if (current_mode == ATTITUDE){
          current_commands_roll = -CTE_COMMANDS;
          current_commands_pitch = current_commands_pitch;
          q_rot.setRPY(current_commands_roll, current_commands_pitch, current_commands_yaw);
          motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
          motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
          motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
          motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
          motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
          pose_reference_publ.publish(motion_reference_pose_msg);
          std_msgs::Int8 msg_int;
          msg_int.data = LEFT;
          attitude_publ.publish(msg_int);          
        }         
        printw("\u2190            ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Increase movement to the left         ");clrtoeol();
        break;       
      case ASCII_KEY_DOWN:
        move();
        if (current_mode == GROUND_SPEED){
          speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x - CTE_SPEED;
          speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y;                       
          publishSpeedReference();
        }
        if (current_mode == POSE){
          motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
          motion_reference_pose_msg.pose.position.x = motion_reference_pose_msg.pose.position.x - CTE_POSE;
          motion_reference_pose_msg.pose.position.y = motion_reference_pose_msg.pose.position.y;
          motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }
        if (current_mode == ATTITUDE){
          current_commands_roll = current_commands_roll;
          current_commands_pitch = CTE_COMMANDS;
          q_rot.setRPY(current_commands_roll, current_commands_pitch, current_commands_yaw);
          motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
          motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
          motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
          motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
          motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
          pose_reference_publ.publish(motion_reference_pose_msg);
          std_msgs::Int8 msg_int;
          msg_int.data = DOWN;
          attitude_publ.publish(msg_int);
        }             
        printw("\u2193     ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Increase backward      ");clrtoeol();
        break;        
      case ASCII_KEY_UP:
        move();
        if (current_mode == GROUND_SPEED){
          speed_reference_msg.twist.linear.x = current_speed_ref.twist.linear.x + CTE_SPEED;
          speed_reference_msg.twist.linear.y = current_speed_ref.twist.linear.y;                       
          publishSpeedReference();
        }
        if (current_mode == POSE){
          motion_reference_pose_msg.pose.orientation = motion_reference_pose_msg.pose.orientation;
          motion_reference_pose_msg.pose.position.x = motion_reference_pose_msg.pose.position.x + CTE_POSE;
          motion_reference_pose_msg.pose.position.y = motion_reference_pose_msg.pose.position.y;
          motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z;
          pose_reference_publ.publish(motion_reference_pose_msg);
        }
        if (current_mode == ATTITUDE){
          current_commands_roll = current_commands_roll;
          current_commands_pitch = -CTE_COMMANDS;
          q_rot.setRPY(current_commands_roll, current_commands_pitch, current_commands_yaw);
          motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
          motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
          motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
          motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
          motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
          pose_reference_publ.publish(motion_reference_pose_msg);
          std_msgs::Int8 msg_int;
          msg_int.data = UP;
          attitude_publ.publish(msg_int);
        }        
        printw("\u2191    ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Increase forward            ");clrtoeol();
        break;         
    case 'r':
    {
      printw("r        ");clrtoeol();
      move(17, 0); 
      printw("                        Last command:     Reset orientation           ");clrtoeol(); 
        current_commands_yaw = 0;
        motion_reference_pose_msg.pose.orientation.w = 0;
        motion_reference_pose_msg.pose.orientation.x = 0;
        motion_reference_pose_msg.pose.orientation.y = 0;
        motion_reference_pose_msg.pose.orientation.z = 0;
        motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
        pose_reference_publ.publish(motion_reference_pose_msg);     
    }
    break;
    case '1':
    {
      printw("1        ");clrtoeol(); 
      move(17, 0);   
      printw("                        Last command:     Ground speed mode               ");clrtoeol();    

      if (setControlMode(aerostack_msgs::MotionControlMode::GROUND_SPEED)){
          hover();
          clearSpeedReferences();
          publishSpeedReference();        
          motion_reference_pose_msg.pose = self_localization_pose_msg.pose;
          pose_reference_publ.publish(motion_reference_pose_msg);         
          current_mode = GROUND_SPEED;
          printoutGroundSpeedControls();
        }
    }
    break;    
    case '2':
    {
      printw("2        ");clrtoeol();
      move(17, 0); 
      printw("                        Last command:     Pose mode         ");clrtoeol();        
      if (setControlMode(aerostack_msgs::MotionControlMode::POSE)){
        hover();
        clearSpeedReferences();
        publishSpeedReference();
        motion_reference_pose_msg.pose = self_localization_pose_msg.pose;
        pose_reference_publ.publish(motion_reference_pose_msg);             
        printoutPoseControls();   
        current_mode = POSE;    
      }
    }
    break;  
    case '3':
    {
      printw("3        ");clrtoeol();
      move(17, 0); 
      printw("                        Last command:     Attitude mode          ");clrtoeol();    
      if (setControlMode(aerostack_msgs::MotionControlMode::ATTITUDE)){
        hover();
        clearSpeedReferences();
        publishSpeedReference();
        motion_reference_pose_msg.pose.position = self_localization_pose_msg.pose.position;
        double yaw,r,p;
        toEulerianAngle(self_localization_pose_msg, &r,&p,&yaw);
        q_rot.setRPY(0, 0, yaw);
        motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
        motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
        motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
        motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();        
        pose_reference_publ.publish(motion_reference_pose_msg);  
        current_mode = ATTITUDE;
        printoutAttitudeControls();
      }
    }
    break;                   
    }
    refresh();
    loop_rate.sleep();
  }

  endwin();
  return 0;
}

//Pose mode controls
void printoutPoseControls(){
  move(4,0);clrtoeol();
  printw(" BASIC MOTIONS                     POSE CONTROL");
  move(5,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   t");attroff(COLOR_PAIR(5)); printw("      Take off              ");  
  attron(COLOR_PAIR(5));printw("    \u2191");attroff(COLOR_PAIR(5));printw("  Increase forward position %.2f m  ",CTE_POSE);
  
  move(6,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   y");attroff(COLOR_PAIR(5)); printw("      Land                  ");
  attron(COLOR_PAIR(5));printw("    \u2193");attroff(COLOR_PAIR(5));printw("  Increase backward position %.2f m  ",CTE_POSE);
  
  move(7,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   h");attroff(COLOR_PAIR(5)); printw("      Keep hovering         ");
  attron(COLOR_PAIR(5));printw("    \u2192");attroff(COLOR_PAIR(5));printw("  Increase position to the right %.2f m  ",CTE_POSE);  

  move(8,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   space ");attroff(COLOR_PAIR(5));printw(" Emergency stop        ");
  attron(COLOR_PAIR(5));printw("    \u2190");attroff(COLOR_PAIR(5));printw("  Increase position to the left %.2f m  ",CTE_POSE);
  
  move(9,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   r");attroff(COLOR_PAIR(5));printw("      Reset orientation    ");  
  
  move(10,0);clrtoeol();
  printw("                                   ");
  attron(COLOR_PAIR(5));printw(" q");attroff(COLOR_PAIR(5));printw("  Increase altitude %.2f m           ",CTE_ALTITUDE);
  move(11,0);clrtoeol();
  printw(" TELEOPERATION MODE SELECTION     ");
  attron(COLOR_PAIR(5));printw("  a");attroff(COLOR_PAIR(5));printw("  Decrease altitude %.2f m            ",CTE_ALTITUDE);
  move(12,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   1");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(4));printw("      Ground speed mode");attroff(COLOR_PAIR(4));
  
  move(13,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   2");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(2));printw("      Pose mode            ");attroff(COLOR_PAIR(2));  
  attron(COLOR_PAIR(5));printw("     z");attroff(COLOR_PAIR(5));printw("  Turn counter-clockwise %.2f rad      ",CTE_YAW);
  
  move(14,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   3");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(3));printw("      Attitude mode         "); attroff(COLOR_PAIR(3)); 
  attron(COLOR_PAIR(5));printw("    x");attroff(COLOR_PAIR(5));printw("  Turn clockwise %.2f rad        ",CTE_YAW);
  move(15,0);clrtoeol();
  printw("--------------------------------------------------------------------------------");
  refresh();
}

//Ground speed mode controls
void printoutGroundSpeedControls(){
  move(4,0);clrtoeol();
  printw(" BASIC MOTIONS                     GROUND SPEED CONTROL");
  move(5,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   t");attroff(COLOR_PAIR(5)); printw("      Take off              ");  
  attron(COLOR_PAIR(5));printw("    \u2191");attroff(COLOR_PAIR(5));printw("  Increase forward speed %.2f m/s  ",CTE_SPEED);
  
  move(6,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   y");attroff(COLOR_PAIR(5)); printw("      Land                  ");
  attron(COLOR_PAIR(5));printw("    \u2193");attroff(COLOR_PAIR(5));printw("  Increase backward speed %.2f m/s  ",CTE_SPEED);
  
  move(7,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   h");attroff(COLOR_PAIR(5)); printw("      Keep hovering         ");
  attron(COLOR_PAIR(5));printw("    \u2192");attroff(COLOR_PAIR(5));printw("  Increase speed to the right %.2f m/s  ",CTE_SPEED);  

  move(8,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   space ");attroff(COLOR_PAIR(5));printw(" Emergency stop        ");
  attron(COLOR_PAIR(5));printw("    \u2190");attroff(COLOR_PAIR(5));printw("  Increase speed to the left %.2f m/s  ",CTE_SPEED);
  
  move(9,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   r");attroff(COLOR_PAIR(5));printw("      Reset orientation    ");  
  
  move(10,0);clrtoeol();
  printw("                                   POSE CONTROL");
  
  move(11,0);clrtoeol();
  printw(" TELEOPERATION MODE SELECTION      ");
  attron(COLOR_PAIR(5));printw(" q");attroff(COLOR_PAIR(5));printw("  Increase altitude %.2f m           ",CTE_ALTITUDE);
  
  move(12,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   1");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(4));printw("      Ground speed mode");attroff(COLOR_PAIR(4));
  attron(COLOR_PAIR(5));printw("         a");attroff(COLOR_PAIR(5));printw("  Decrease altitude %.2f m            ",CTE_ALTITUDE);
  move(13,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   2");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(2));printw("      Pose mode            ");  attroff(COLOR_PAIR(2));
  attron(COLOR_PAIR(5));printw("     z");attroff(COLOR_PAIR(5));printw("  Turn counter-clockwise %.2f rad      ",CTE_YAW);
  
  move(14,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   3");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(3));printw("      Attitude mode         "); attroff(COLOR_PAIR(3));
  attron(COLOR_PAIR(5));printw("    x");attroff(COLOR_PAIR(5));printw("  Turn clockwise %.2f rad        ",CTE_YAW);
  move(15,0);clrtoeol();
  printw("--------------------------------------------------------------------------------");
  refresh();
}
//Attitude mode controls
void printoutAttitudeControls(){
  move(4,0);clrtoeol();
  printw(" BASIC MOTIONS                     ATTITUDE CONTROL");
  move(5,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   t");attroff(COLOR_PAIR(5)); printw("      Take off              ");  
  attron(COLOR_PAIR(5));printw("    \u2191");attroff(COLOR_PAIR(5));printw("  Pitch %.2f rad during %.2f s  ",CTE_COMMANDS,CTE_COMMANDS_TIME);
  
  move(6,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   y");attroff(COLOR_PAIR(5)); printw("      Land                  ");
  attron(COLOR_PAIR(5));printw("    \u2193");attroff(COLOR_PAIR(5));printw("  Pitch -%.2f rad during %.2f s  ",CTE_COMMANDS,CTE_COMMANDS_TIME);
  
  move(7,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   h");attroff(COLOR_PAIR(5)); printw("      Keep hovering         ");
  attron(COLOR_PAIR(5));printw("    \u2192");attroff(COLOR_PAIR(5));printw("  Roll %.2f rad during %.2f s  ",CTE_COMMANDS,CTE_COMMANDS_TIME);  

  move(8,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   space ");attroff(COLOR_PAIR(5));printw(" Emergency stop        ");
  attron(COLOR_PAIR(5));printw("    \u2190");attroff(COLOR_PAIR(5));printw("  Roll -%.2f rad during %.2f s  ",CTE_COMMANDS,CTE_COMMANDS_TIME);
  
  move(9,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   r");attroff(COLOR_PAIR(5));printw("      Reset orientation    ");  
  
  move(10,0);clrtoeol();
  printw("                                   POSE CONTROL");
  
  move(11,0);clrtoeol();
  printw(" TELEOPERATION MODE SELECTION      ");
  attron(COLOR_PAIR(5));printw(" q");attroff(COLOR_PAIR(5));printw("  Increase altitude %.2f m           ",CTE_ALTITUDE);
  
  move(12,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   1");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(4));printw("      Ground speed mode");attroff(COLOR_PAIR(4));
  attron(COLOR_PAIR(5));printw("         a");attroff(COLOR_PAIR(5));printw("  Decrease altitude %.2f m            ",CTE_ALTITUDE);
  move(13,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   2");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(2));printw("      Pose mode            ");  attroff(COLOR_PAIR(2));
  attron(COLOR_PAIR(5));printw("     z");attroff(COLOR_PAIR(5));printw("  Turn counter-clockwise %.2f rad      ",CTE_YAW);
  
  move(14,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   3");attroff(COLOR_PAIR(5));attron(COLOR_PAIR(3));printw("      Attitude mode         "); attroff(COLOR_PAIR(3));
  attron(COLOR_PAIR(5));printw("    x");attroff(COLOR_PAIR(5));printw("  Turn clockwise %.2f rad        ",CTE_YAW);
  move(15,0);clrtoeol();
  printw("--------------------------------------------------------------------------------");
  refresh();
}

//High level command
void publishCmd(){
  command_publ.publish(command_order);
}

//Take off
void takeOff(){
  command_order.action = aerostack_msgs::FlightActionCommand::TAKE_OFF;
  publishCmd();
}

//Hover
void hover(){
  clearSpeedReferences();
  publishSpeedReference();
  command_order.action = aerostack_msgs::FlightActionCommand::HOVER;
  publishCmd();
}

//Land
void land(){
  clearSpeedReferences();
  publishSpeedReference();
  command_order.action = aerostack_msgs::FlightActionCommand::LAND;
  publishCmd();
}

//Emergency Stop
void emergencyStop(){
  clearSpeedReferences();
  publishSpeedReference();
  command_order.action = aerostack_msgs::FlightActionCommand::HOVER;
  publishCmd();
}

//Move
void move(){
  command_order.action = aerostack_msgs::FlightActionCommand::MOVE;
  publishCmd();
}

//Clear speed references
void clearSpeedReferences(){
  speed_reference_msg.twist.linear.x = 0.0;
  speed_reference_msg.twist.linear.y = 0.0;
  speed_reference_msg.twist.linear.z = 0.0;
  speed_reference_msg.twist.angular.x = 0.0;
  speed_reference_msg.twist.angular.y = 0.0;
  speed_reference_msg.twist.angular.z = 0.0;
}

//Publish Speed Reference
void publishSpeedReference(){
  speed_reference_publ.publish(speed_reference_msg);
}

//Speed reference callback
void speedReferenceCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
  current_speed_ref = (*msg);
}

//Control mode callback
void controlModeCallback(const aerostack_msgs::MotionControlMode::ConstPtr& msg){
  control_mode_msg = (*msg);
}

//Set control mode
bool setControlMode(int new_control_mode){
  // Prepare service message
  aerostack_msgs::SetControlMode setControlModeSrv;
  setControlModeSrv.request.controlMode.mode = new_control_mode;
  // use service
  if (setControlModeClientSrv.call(setControlModeSrv)){
    return setControlModeSrv.response.ack;
  }else{
    return false;
  }
}

//Self localization callback
void selfLocalizationPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  self_localization_pose_msg = (*msg);
}

void selfSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
  self_speed_msg=*msg;
}

void toEulerianAngle(geometry_msgs::PoseStamped q, double *roll, double *pitch, double *yaw){
    if(q.pose.orientation.w == 0 && q.pose.orientation.x == 0 && q.pose.orientation.y == 0 && q.pose.orientation.z == 0){
        *roll   = 0; 
        *pitch = 0;
        *yaw = 0;
    }else{
        *roll  = atan2(2.0 * (q.pose.orientation.z * q.pose.orientation.y + q.pose.orientation.w * q.pose.orientation.x) , 1.0 - 2.0 * (q.pose.orientation.x * q.pose.orientation.x + q.pose.orientation.y * q.pose.orientation.y));
        *pitch = asin(2.0 * (q.pose.orientation.y * q.pose.orientation.w - q.pose.orientation.z * q.pose.orientation.x));
        *yaw   = atan2(2.0 * (q.pose.orientation.z * q.pose.orientation.w + q.pose.orientation.x * q.pose.orientation.y) , - 1.0 + 2.0 * (q.pose.orientation.w * q.pose.orientation.w + q.pose.orientation.x * q.pose.orientation.x));    
    }
}

void attitudeCallback(const std_msgs::Int8::ConstPtr& msg){
  switch(msg->data){
    case LEFT:
    {
          boost::this_thread::sleep(boost::posix_time::milliseconds(miliseconds));
          current_commands_roll = 0;
          current_commands_pitch = current_commands_pitch;
          q_rot.setRPY(current_commands_roll, current_commands_pitch, current_commands_yaw);
          motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
          motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
          motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
          motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
          motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
          pose_reference_publ.publish(motion_reference_pose_msg);   
    }
    break;
    case RIGHT:
    {
          boost::this_thread::sleep(boost::posix_time::milliseconds(miliseconds));
          current_commands_roll = 0;
          current_commands_pitch = current_commands_pitch;
          q_rot.setRPY(current_commands_roll, current_commands_pitch, current_commands_yaw);
          motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
          motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
          motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
          motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
          motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
          pose_reference_publ.publish(motion_reference_pose_msg);   
    }
    break;
    case DOWN:
    {
          boost::this_thread::sleep(boost::posix_time::milliseconds(miliseconds));
          current_commands_roll = current_commands_roll;
          current_commands_pitch = 0;
          q_rot.setRPY(current_commands_roll, current_commands_pitch, current_commands_yaw);
          motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
          motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
          motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
          motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
          motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
          pose_reference_publ.publish(motion_reference_pose_msg);   
    }
    break;
    case UP:
    {     boost::this_thread::sleep(boost::posix_time::milliseconds(miliseconds));
          current_commands_roll = current_commands_roll;
          current_commands_pitch = 0;
          q_rot.setRPY(current_commands_roll, current_commands_pitch, current_commands_yaw);
          motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
          motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
          motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
          motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
          motion_reference_pose_msg.pose.position = motion_reference_pose_msg.pose.position;
          pose_reference_publ.publish(motion_reference_pose_msg);    
    }
    break;    
  }
}
