use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::{common_interfaces::{sensor_msgs, nav_msgs}, RosString},
    clock::Clock,
    pr_info,
};

use rust_robo_utils::localization::ekf_posture;

fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;

    let node = ctx.create_node("imu_localization", None, Default::default())?;

    let subscriber = node.create_subscriber::<sensor_msgs::msg::Imu>("/imu", None)?;

    let publisher = node.create_publisher::<nav_msgs::msg::Odometry>("/odom", None)?;

    let mut selector = ctx.create_selector()?;

    let log = Logger::new(node.get_name());

    let mut timer = Clock::new()?;

    pr_info!(log, "Start {}", node.get_name());

    let mut old_time = timer.get_now().unwrap() as f64;

    let mut old_x = ekf_posture::init_posture();

    let mut old_p = ekf_posture::p_init(0.0018);

    selector.add_subscriber(
        subscriber, 
        Box::new(move |msg| {

            // setting data
            let now = timer.get_now().unwrap() as f64;
            let delta_t = (now - old_time) * 10e-10;
            let gyro = rust_robo_utils::get_vector3(
                msg.angular_velocity.x, 
                msg.angular_velocity.z, 
                msg.angular_velocity.y
            );
            let accel = rust_robo_utils::get_vector3(
                msg.linear_acceleration.x, 
                msg.linear_acceleration.z, 
                msg.linear_acceleration.y
            );

            // kalman filter
            let u = ekf_posture::calc_input(gyro, delta_t);

            let z = ekf_posture::calc_observe(accel);

            let q = ekf_posture::get_gyro_noise(delta_t);

            let r = ekf_posture::get_accel_noise(delta_t);

            let new_x = ekf_posture::ekf_x(old_x, u, z, old_p, r, q);

            let new_p = ekf_posture::ekf_cov(old_x, u, old_p, r, q);

            let quat = rust_robo_utils::euler_to_quaternion(new_x);

            // generate message
            let mut send_msg = nav_msgs::msg::Odometry::new().unwrap();
            send_msg.header.frame_id = RosString::new("/odom").unwrap();
            send_msg.pose.pose.orientation.x = quat.x;
            send_msg.pose.pose.orientation.y = quat.y;
            send_msg.pose.pose.orientation.z = quat.z;
            send_msg.pose.pose.orientation.w = quat.w;

            let _ = publisher.send(&send_msg);

            old_time = now;
            old_x = new_x;
            old_p = new_p;
        }),
    );

    loop {
        selector.wait()?;
    }
}