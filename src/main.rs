use imu_localization::{create_vector, init_posture};
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    pr_info,
    msg::{common_interfaces::nav_msgs, RosString},
};

use imu_localization::*;

fn main()->Result<(), DynError>
{
    let ctx = Context::new().unwrap();
    let node = ctx.create_node("imu_localization", None, Default::default())?;
    let mut selector = ctx.create_selector()?;
    let log = Logger::new(node.get_name());

    let subscriber = node.create_subscriber::<haya_imu_msgs::msg::ImuData>("/imu_data", None)?;
    let publisher = node.create_publisher::<nav_msgs::msg::Odometry>("/odom", None)?;
    let mut send_msg = nav_msgs::msg::Odometry::new().unwrap();
    send_msg.header.frame_id = RosString::new("map").unwrap();
    send_msg.child_frame_id = RosString::new("odom").unwrap();

    let delta_time = 0.01;

    let mut pos = init_posture();
    let obs = init_posture();
    let mut disp = init_cov();
    let q = predict_noise(0.01);
    let r = observation_noise(0.1);
    pr_info!(log, "Start {}", node.get_name());

    selector.add_subscriber(
        subscriber, 
    Box::new(move |msg|{
        let angular_vel = create_vector(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
        let accel = create_vector(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
        let geo_mag = create_vector(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);

        let predict_posture = predict_posture(pos, angular_vel, delta_time);
        let obs_posture = observation_posture(obs, accel, geo_mag);

        let predict_jacob = predict_jacob(pos, angular_vel, delta_time);
        let obs_jacob = observation_jacob();

        let predict_disp = calc_predict_disp(disp, predict_jacob, q);
        let obs_disp = calc_observation_disp(predict_disp, obs_jacob, r);

        let kalman_gain = calc_kalman_gain(predict_disp, obs_jacob, obs_disp);

        pos = update_posture(predict_posture, kalman_gain, obs_jacob, obs_posture);
        disp = update_disp(kalman_gain, obs_jacob, predict_disp);

        let quat = euler_to_quaternion(pos);

        send_msg.pose.pose.orientation.x = quat.x;
        send_msg.pose.pose.orientation.y = quat.y;
        send_msg.pose.pose.orientation.z = quat.z;
        send_msg.pose.pose.orientation.w = quat.w;

        let _ = publisher.send(&send_msg);
    })
    );

    loop {
        selector.wait()?;
    }
}