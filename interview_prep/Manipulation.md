## Prep 
1. Motion Planning (launched as nodelet to save from messaging delays)
    1. moveit_setup_assistant to setup URDF, launch files, etc. 
        - figure out reachable workspace, 
        - gripper we are using consists of only one actuator
    2. single destination (Z pointing down/doesn't matter)
        ```
        class SingleDestinationPlanner{
            public: 
                SingleDestinationPlanner(PLANNING_GROUP){
                    // pre-configured in moveit_setup_assistant
                    static const std::string PLANNING_GROUP = "coffee_bot";
                    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
                    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
                    ...     //such as move_group.setMaxVelocityScalingFactor(0.1); ...
                }

                bool plan(const Pose& target_pose, Plan& my_plan, bool perpendicular){
                    move_group.setPoseTarget(target_pose1);
                    if (perpendicular){
                        moveit_msgs::OrientationConstraint ocm;
                        ...
                        moveit_msgs::Constraints test_constraints;
                        test_constraints.orientation_constraints.push_back(ocm);
                        move_group.setPathConstraints(test_constraints);
                    }
                    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    return success; 
                }

                bool plan(const string& position="home", Plan& my_plan){
                    move_group.setNamedTarget(position); 
                    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    return success; 
                }

                void add_obstacles(const std::vector<moveit_msgs::CollisionObject>& collision_objects;)
                        current_scene.addCollisionObjects(collision_objects);
        }
        ```
    3. show structure, action server and Task scheduling (```main.cpp```)
        - assume the robot is sitting on a table
        - if we have fixed positions, we can even **pre-compute plans**
        - Need to talk to uC over serial over ROSSERIAL(or need our own protocol)
            ```cpp
            // subscribe: /joint_positions from robot, /pause_status from safe_thread
            // publish: /joint_control 
            // pick and place anything looking like coffee in the designated area
            // Assumptions: 
            // 1. no coffees have been removed in the input area
            class PickPlacer{
                void joint_pos_cb(JointStateConstptr& joint_state_msg){
                   
                   if (current_waypoint != null and all_close(joint_states, current_waypoint)){
                       execution_cond_var.notify_one(); 
                   }
                   ...
                   // get velocity here
                }

                // if someone is in the workspace, simply pause. Then resume
                void execution(trajectory){
                    for (const auto& wp:plan.trajectory){
                        if(person_in_work_area){
                            // we are using std::future and std::promise because the /safety node just publish once. 
                            safety_fut.get();
                        }
                        // we use publisher because it's smaller. Else we can just call move_group.move()
                        ... 
                        current_waypoint[0:5] = wp_msg;     
                        control_pub.publish(wp_msg);
                        start_time = system_clock::Now(); 
                        execution_cond_var.wait(); 
                        if (system_clock::Now() - start_time > EXECUTION_TIME){
                            ROS_WARN("execution time out");
                        }
                    }
                    return NORMAL
                }

                // called after executing a trajectory
                void open_gripper(){
                   current_waypoint[6] = 0.5; 
                   execution_cond_var.wait();
                }

                // called after executing a trajectory
                void close_gripper(){
                   current_waypoint[6] = 0.0; 
                   execution_cond_var.wait();
                }

            };
            ```
    4. pick and place action - Moveit has a packaged ```pick_and_place```, but we can't do anything like pausing in between
        ```cpp
        // we always start from home position. 
        bool PickPlacer::pick_and_place(Point3D start, Point3D goal){ 
           current_plans_vec.clear(); 
           // leave
           target_poses = create_target_poses(start, goal);
           // plan 5 segments first
           for (int i = 0; i< 5; ++i){
               if (!single_destination_planner.plan(target_poses[i], current_plans_vec[i])){
                    return false;
               }
           }
           // go 0, open, approach 1, close, retreat 2, move 3, open, retreat 4
           // execute them
           for (int i = 0; i< 5; ++i){
                // open
                if (i == 1 || 4) open_gripper();             
                if (i == 2 ) close_gripper(); 
                execute(current_plans_vec[i]); 
           }
           Plan go_home_plan; 
           single_destination_planner.plan("home", go_home_plan); 
           execute(go_home_plan); 
        }
        ```
    5. main 
        ```cpp
        int main(){
           ros::AsyncSpinner spinner(2); // Use 2 threads to process callbacks concurrently
           spinner.start(); 
           ros::Rate r(20);
           PickPlacer pp; 
           while (ros::ok()){
               std::tie(coffees_start, obstacles) = point_cloud_classification()
               coffees_dests = get_destinations(coffees); 
               // can assume box/cylindrical obstacles
               pp.update_obstacles(obstacles);
               for (int i = 0; i < coffees_start.size(); ++i){
                pp.pick_and_place(coffees_start.at(i), coffees_dests.at(i));
               }
               r.sleep();
           }
           
        }
        ```

2. serving robot 

