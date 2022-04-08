## Prep 
1. Motion Planning (launched as nodelet to save from messaging delays)
    1. moveit_setup_assistant to setup URDF, launch files, etc. 
        - figure out reachable workspace, 
        - gripper we are using consists of only one actuator
    2. single destination (Z pointing down/doesn't matter)
        ```cpp
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
            // service: /pause 
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
                            wait_for_person_to_leave(); 
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

            bool pause_unpause_srv_cb(req, res){
                    if (req.pause){
                        person_in_work_area = true;
                        this->pause_fut.get();
                    }
                    else{
                        // inform execution() that the wait is over
                        this->unpause_promise.set();
                    }
            }

            void wait_for_person_to_leave(){
                this->unpause_promise = std::promise<void>(); 
                this->unpause_future = this->unpause_promise.get_future();
                // inform /safety that we have paused 
                this->pause_promise.set()
                // wait for next unpause srv from /safety 
                this->unpause_future.get(); 
                this->pause_promise=std::promise<void>();
                this->pause_fut=this->pause_promise.get_future();
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
    - [diagram](https://app.diagrams.net/#G1s3RWDtmiwxv2Y_aHYtxH-HjgZVMiYn71) 

2. serving robot 
    - assume we know where to park at each table 
    - ROS NavStack Makes sense
        1. Odometer (wheel encoder, IMU (a bit noisy, unless buy more expensive ones. But that accumulative drift is a problem anyways), Lidar as Odometer through scan matching (icp, gicp...) )
        2. Simple TF tree: ```map->odom->base_link->sensor_link```
        3. AMCL to estimate ```map -> base_link``` by publishing onto ```map->odom```. Particle filter with particles being robot state estimate. The goal is to try to estimate the distribution of states. Scoring is done by scan matching (p(z|x)). Then we resample to "discretize" the distribution. Adaptive is to increase number of particles if max score is too low (e.g., kidnapped robot problem where sensors suddenly stops working), or reduce the number if max score is too high
            <p align="center">
            <img src="https://img-blog.csdnimg.cn/img_convert/91978f9cdf938f57f8ccb79c14281790.png""" height="400" width="width"/>
            </p>
    - Other methods for localization
        1. If possible, use AR tags at corners not attracting attention (so we can potentially do EKF SLAM)
        2. Use radio beacons (<30cm I read), and solve trillateration using Least-Squares method
    - But do we really want a robot arm? IMO not for serving directly for safety reasons. However, it might still be cool (for attracting investors) to have one. So we can have a little slider, 1. the arm puts coffee on the slider 2. the slider slides out for customer to receive. Then it's pretty much 
    - [reference](https://blog.csdn.net/soaring_casia/article/details/119576504)

