extern crate nalgebra as na;

fn init_posture()->na::Vector3<f64>
{
    na::Vector3::<f64>::new(
        0.0,
        0.0,
        0.0,
    )
}

fn init_cov()->na::Matrix3<f64>
{
    na::Matrix3::<f64>::new(
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    )
}

fn predict_posture(posture:na::Vector3<f64>, angular_velocity:na::Vector3<f64>, delta_time:f64)->na::Vector3<f64>
{
    let sin_x = (posture.x).sin();
    let cos_x = (posture.x).cos();
    let cos_y = (posture.y).cos();
    let tan_y = (posture.y).tan();

    let p_x = posture.x + (angular_velocity.x + angular_velocity.y*tan_y*sin_x + angular_velocity.z*tan_y*cos_x)*delta_time;
    let p_y = posture.y + (angular_velocity.y*cos_x - angular_velocity.z*sin_x)*delta_time;
    let p_z = posture.z + (angular_velocity.y*(sin_x/cos_y) + angular_velocity.z*(cos_x/cos_y))*delta_time;

    na::Vector3::<f64>::new(
        p_x,
        p_y,
        p_z
    ) 
}

fn observation_posture(obs:na::Vector3<f64>,acceleration:na::Vector3<f64>, geomagnetism:na::Vector3<f64>)->na::Vector3<f64>
{
    let sin_x = obs.x.sin();
    let sin_y = obs.y.sin();
    let cos_x = obs.x.cos();
    let cos_y = obs.y.cos();

    let o_x = ((-1.0 * acceleration.y)/(-1.0 * acceleration.z)).atan();
    let o_y = (acceleration.x / ((acceleration.y.powi(2) + acceleration.z.powi(2)).sqrt())).atan();
    let o_z = ((geomagnetism.x*cos_y + geomagnetism.y*sin_y*sin_x + geomagnetism.z*sin_y*cos_x) / (geomagnetism.y*cos_x - geomagnetism.z*sin_x)).atan();

    na::Vector3::<f64>::new(
        o_x,
        o_y,
        o_z,
    )
}

fn predict_noise(rate:f64)->na::Matrix3<f64>
{
    let result = na::Matrix3::<f64>::new(
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    );

    result * rate
}

fn observation_noise(rate:f64)->na::Matrix3<f64>
{
    let result = na::Matrix3::<f64>::new(
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    );

    result * rate
}

fn predict_jacob(posture:na::Vector3<f64>, angular_velocity:na::Vector3<f64>, delta_time:f64)->na::Matrix3<f64>
{
    let sin_x = (posture.x).sin();
    let cos_x = (posture.x).cos();
    let cos_y = (posture.y).cos();
    let tan_y = (posture.y).tan();

    let x_x = 1.0 + (angular_velocity.x + angular_velocity.y)
}