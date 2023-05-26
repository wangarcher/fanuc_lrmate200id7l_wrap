// a simple planning demo, initialized in 210813
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iostream>

// #include <mission_mode/mode_switch.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>

struct PoseInfo
{
    double general_dist;
    double start_dist;
    double end_dist;
    geometry_msgs::Pose pose;
};

class GeneralPickPlace
{
public:
    Eigen::Vector3d cam_obj_translation_;
    Eigen::Quaterniond cam_obj_orientation_;
    bool first_reco_flag;    // outwards flag, when the ee reached the desired pre-picking place
    bool second_reco_flag;   // outwards flag, when the ee reached the desired pre-placeing place
    bool pick_flag = false;  // inwards flag, when the flag set as true, the robot is ready for the pick motion sequence.
    bool place_flag = false; // inwards flag, whent the flag set as true, the robot is ready for the place motion sequence.

public:
    void obj_visual_callback(const geometry_msgs::PoseStamped &odom);

    void pseudo_obj_squence_callback(geometry_msgs::PoseArray &obj_poses);

    std::vector<PoseInfo> getKNearest(const geometry_msgs::PoseArray &obj_poses,
                                      const geometry_msgs::PoseStamped &start_pose,
                                      const geometry_msgs::PoseStamped &end_pose,
                                      int k);

    std::vector<PoseInfo> getSequence(std::vector<PoseInfo> &k_nearest_poses_info,
                                      int m, int n, int k);

    std::vector<PoseInfo> getSequence_1(std::vector<PoseInfo> &k_nearest_poses_info,
                                        int k,
                                        const geometry_msgs::PoseStamped &start_pose);
};

void GeneralPickPlace::obj_visual_callback(const geometry_msgs::PoseStamped &odom)
{
    cam_obj_translation_ << odom.pose.position.x,
        odom.pose.position.y,
        odom.pose.position.z;
    cam_obj_orientation_.coeffs() << odom.pose.orientation.x,
        odom.pose.orientation.y,
        odom.pose.orientation.z,
        odom.pose.orientation.w;
    if (odom.header.frame_id == "1st")
    {
        pick_flag = true;
        place_flag = false;
    }
    if (odom.header.frame_id == "2nd")
    {
        place_flag = true;
        pick_flag = false;
    }
}

void GeneralPickPlace::pseudo_obj_squence_callback(geometry_msgs::PoseArray &obj_poses)
{
    srand(static_cast<unsigned>(time(0)));
    for (int i = 0; i < 100; i++)
    {
        Eigen::Vector3f rand_position;
        geometry_msgs::Pose set_pose;
        double lower = -0.37;
        double upper = 0.37;

        rand_position(0) = 0.4 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (0.7 - 0.4)));

        rand_position(1) = -0.5 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (0.5 - (-0.5))));

        set_pose.position.x = rand_position(0);
        set_pose.position.y = rand_position(1);
        set_pose.position.z = 0.13;
        obj_poses.poses.push_back(set_pose);
    }
}

std::vector<PoseInfo> GeneralPickPlace::getKNearest(const geometry_msgs::PoseArray &obj_poses,
                                                    const geometry_msgs::PoseStamped &start_pose,
                                                    const geometry_msgs::PoseStamped &end_pose,
                                                    int k)
{
    int obj_poses_size = obj_poses.poses.size();
    std::vector<PoseInfo> poses_info;
    PoseInfo new_pose_info;
    for (int i = 0; i < obj_poses_size; i++)
    {
        double start_dx = obj_poses.poses[i].position.x - start_pose.pose.position.x;
        double start_dy = obj_poses.poses[i].position.y - start_pose.pose.position.y;
        double start_dz = obj_poses.poses[i].position.z - start_pose.pose.position.z;
        double end_dx = obj_poses.poses[i].position.x - end_pose.pose.position.x;
        double end_dy = obj_poses.poses[i].position.y - end_pose.pose.position.y;
        double end_dz = obj_poses.poses[i].position.z - end_pose.pose.position.z;
        new_pose_info.start_dist = std::sqrt(start_dx * start_dx + start_dy * start_dy + start_dz * start_dz);
        new_pose_info.end_dist = std::sqrt(end_dx * end_dx + end_dy * end_dy + end_dz * end_dz);
        new_pose_info.general_dist = new_pose_info.start_dist + new_pose_info.end_dist;
        new_pose_info.pose = obj_poses.poses[i];
        poses_info.push_back(new_pose_info);
    }
    sort(poses_info.begin(), poses_info.end(), [](PoseInfo a, PoseInfo b)
         { return a.general_dist <= b.general_dist; });

    std::vector<PoseInfo> k_nearest_poses_info(poses_info.begin(), poses_info.begin() + k);
    return k_nearest_poses_info;
}

std::vector<PoseInfo> GeneralPickPlace::getSequence(std::vector<PoseInfo> &k_nearest_poses_info,
                                                    int m, int n, int k)
{
    std::vector<PoseInfo> sequence;
    sort(k_nearest_poses_info.begin(), k_nearest_poses_info.end(), [](PoseInfo a, PoseInfo b)
         { return a.start_dist <= b.start_dist; });
    std::vector<PoseInfo> start_sequence(k_nearest_poses_info.begin(), k_nearest_poses_info.begin() + m);
    std::vector<PoseInfo> end_sequence(k_nearest_poses_info.begin() + m, k_nearest_poses_info.end());

    sort(end_sequence.begin(), end_sequence.begin() + n, [](PoseInfo a, PoseInfo b)
         { return a.end_dist >= b.end_dist; });

    sequence.insert(sequence.end(), start_sequence.begin(), start_sequence.end());
    sequence.insert(sequence.end(), end_sequence.begin(), end_sequence.end());
    return sequence;
}

std::vector<PoseInfo> GeneralPickPlace::getSequence_1(std::vector<PoseInfo> &k_nearest_poses_info,
                                                      int k,
                                                      const geometry_msgs::PoseStamped &start_pose)
{
    std::vector<PoseInfo> sequence;

    for (int i = 0; i < k; i++)
    {
        sort(k_nearest_poses_info.begin(), k_nearest_poses_info.end(), [](PoseInfo a, PoseInfo b)
             { return a.start_dist <= b.start_dist; });

        sequence.push_back(k_nearest_poses_info[0]);
        for (int j = 1; j < k_nearest_poses_info.size(); j++)
        {
            double start_dx = k_nearest_poses_info[j].pose.position.x - k_nearest_poses_info[0].pose.position.x;
            double start_dy = k_nearest_poses_info[j].pose.position.y - k_nearest_poses_info[0].pose.position.y;
            double start_dz = k_nearest_poses_info[j].pose.position.z - k_nearest_poses_info[0].pose.position.z;
            k_nearest_poses_info[j].start_dist = std::sqrt(start_dx * start_dx + start_dy * start_dy + start_dz * start_dz);
        }
        std::vector<PoseInfo>::iterator erase = k_nearest_poses_info.begin();
        k_nearest_poses_info.erase(erase);
    }

    return sequence;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "project_plan_pipeline");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(7);
    spinner.start();

    GeneralPickPlace general_pick_place;

    ros::Publisher sequence_pub_ = n.advertise<nav_msgs::Path>("sequence", 1);
    ros::Publisher marker_pub_ = n.advertise<visualization_msgs::Marker>("teb_markers", 1000);

    // some flag were set here, so as to make sure the sequence flow running in a proper way.
    bool pre_pick_flag = true;
    bool pick_flag = false;
    bool pre_place_flag = false;
    bool place_flag = false;
    bool sequence_flag = true;

    geometry_msgs::PoseArray obj_poses;
    std::vector<PoseInfo> k_nearest_poses_info;
    std::vector<PoseInfo> sequence;
    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::PoseStamped end_pose;
    start_pose.pose.position.x = 0.47;
    start_pose.pose.position.y = 0.0;
    start_pose.pose.position.z = 0.13;
    end_pose.pose.position.x = 0.47;
    end_pose.pose.position.y = 0.37;
    end_pose.pose.position.z = 0.13;

    general_pick_place.pseudo_obj_squence_callback(obj_poses);
    k_nearest_poses_info = general_pick_place.getKNearest(obj_poses, start_pose, end_pose, 12);
    sequence = general_pick_place.getSequence(k_nearest_poses_info, 6, 6, 12);

    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    arm.allowReplanning(true); // Evangelion 3.33: You Can(Not) Redo.
    arm.setGoalJointTolerance(0.001);
    arm.setGoalPositionTolerance(0.001);      // set position torlerance
    arm.setGoalOrientationTolerance(0.01);    // set orientation torlerance
    arm.setMaxAccelerationScalingFactor(1.7); // set max accelerations
    arm.setMaxVelocityScalingFactor(1.7);

    std::string end_effector_link = arm.getEndEffectorLink(); // get the end_effector link
    std::cout << "end_effector_link: " << end_effector_link << std::endl;
    std::string reference_frame = "base_link"; // set the reference frame
    arm.setPoseReferenceFrame(reference_frame);

    ros::Rate loop_rate(10);

    std::cout << "Caution: moving to the pre-pick pose!" << std::endl;
    arm.setNamedTarget("1st"); // first reco state, for the best top-looking view
    arm.move();
    sleep(5);

    geometry_msgs::Pose now_pose = arm.getCurrentPose(end_effector_link).pose;
    std::cout << "now tool position: [x,y,z]: ["
              << now_pose.position.x << "," << now_pose.position.y << "," << now_pose.position.z << "]"
              << std::endl;
    std::cout << "now tool orientation: [x,y,z,w]: ["
              << now_pose.orientation.x << "," << now_pose.orientation.y << "," << now_pose.orientation.z << "," << now_pose.orientation.w << "]"
              << std::endl;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Point";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(2.0);

    nav_msgs::Path gui_path;
    gui_path.poses.resize(sequence.size());
    gui_path.header.frame_id = "base_link";

    for (int i = 0; i < 100; i++)
    {
        geometry_msgs::Point point;
        point.x = obj_poses.poses[i].position.x;
        point.y = obj_poses.poses[i].position.y;
        point.z = obj_poses.poses[i].position.z;
        marker.points.push_back(point);
    }

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.12;
    marker.color.g = 0.56;
    marker.color.b = 1.0;

    for (int i = 0; i < 12; i++)
    {
        std::cout << "sequence num. " << i << ": "
                  << sequence[i].pose.position.x << ", "
                  << sequence[i].pose.position.y << ", "
                  << sequence[i].pose.position.z << std::endl;
        gui_path.poses[i].pose = sequence[i].pose;

        marker_pub_.publish(marker);
        sequence_pub_.publish(gui_path);

        std::vector<geometry_msgs::Pose> waypoints;
        geometry_msgs::Pose pose1;
        pose1.position.x = sequence[i].pose.position.x;
        pose1.position.y = sequence[i].pose.position.y;
        pose1.position.z = sequence[i].pose.position.z + 0.1;
        pose1.orientation.x = 1.0;
        pose1.orientation.y = 0.0;
        pose1.orientation.z = 0.0;
        pose1.orientation.w = 0.0;
        waypoints.push_back(pose1);

        geometry_msgs::Pose pose2;
        pose2.position.x = sequence[i].pose.position.x;
        pose2.position.y = sequence[i].pose.position.y;
        pose2.position.z = sequence[i].pose.position.z;
        pose2.orientation.x = 1.0;
        pose2.orientation.y = 0.0;
        pose2.orientation.z = 0.0;
        pose2.orientation.w = 0.0;
        waypoints.push_back(pose2);

        waypoints.push_back(pose1);

        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.002;
        double fraction = 0.0;
        int maxtries = 100; // max attempts
        int attempts = 0;   // attempts done

        while (fraction < 1.0 && attempts < maxtries)
        {
            fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            attempts++;
            if (attempts % 10 == 0)
                ROS_INFO("Still trying after %d attempts...", attempts);
        }

        if (fraction == 1)
        {
            ROS_INFO("Path computed successfully. Moving the arm.");
            // plan data
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            // execute
            arm.execute(plan);
        }
        else
        {
            ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
        }
    }

    arm.setNamedTarget("1st"); // first reco state, for the best top-looking view
    arm.move();
    sleep(5);

    ros::waitForShutdown();

    return 0;
}
