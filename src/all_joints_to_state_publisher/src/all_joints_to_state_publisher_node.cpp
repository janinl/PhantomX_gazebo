#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "all_joints_to_state_publisher_node");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    //tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(100);

    const double degree = M_PI/180;

    // robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "axis";

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(18);
        joint_state.position.resize(18);
	int i=0;
        joint_state.name[i++] ="j_c1_lf";
        joint_state.name[i++] ="j_c1_lm";
        joint_state.name[i++] ="j_c1_lr";
        joint_state.name[i++] ="j_c1_rf";
        joint_state.name[i++] ="j_c1_rm";
        joint_state.name[i++] ="j_c1_rr";
        joint_state.name[i++] ="j_tibia_lf";
        joint_state.name[i++] ="j_tibia_lm";
        joint_state.name[i++] ="j_tibia_lr";
        joint_state.name[i++] ="j_tibia_rf";
        joint_state.name[i++] ="j_tibia_rm";
        joint_state.name[i++] ="j_tibia_rr";
        joint_state.name[i++] ="j_thigh_lf";
        joint_state.name[i++] ="j_thigh_lm";
        joint_state.name[i++] ="j_thigh_lr";
        joint_state.name[i++] ="j_thigh_rf";
        joint_state.name[i++] ="j_thigh_rm";
        joint_state.name[i++] ="j_thigh_rr";
        i=0;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        joint_state.position[i++] = swivel;
        /*joint_state.name[1] ="thigh_lf";
        joint_state.position[1] = tilt;
        joint_state.name[2] ="c2_lf";
        joint_state.position[2] = height;
        joint_state.name[3] ="tibia_lf";
        joint_state.position[3] = height;
*/

        // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = cos(angle)*2;
        odom_trans.transform.translation.y = sin(angle)*2;
        odom_trans.transform.translation.z = .7;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        //broadcaster.sendTransform(odom_trans);

        // Create new robot state
        tilt += tinc;
        if (tilt<-.5 || tilt>0) tinc *= -1;
        height += hinc;
        if (height>.2 || height<0) hinc *= -1;
        swivel += degree;
        angle += degree/4;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
