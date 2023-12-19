extern crate nalgebra as na;

pub fn init_posture()->na::Vector3<f64>
{
    na::Vector3::<f64>::new(
        0.0,
        0.0,
        0.0,
    )
}

pub fn init_cov()->na::Matrix3<f64>
{
    na::Matrix3::<f64>::new(
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    )
}

pub fn predict_posture(posture:na::Vector3<f64>, angular_velocity:na::Vector3<f64>, delta_time:f64)->na::Vector3<f64>
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

pub fn observation_posture(obs:na::Vector3<f64>,acceleration:na::Vector3<f64>, geomagnetism:na::Vector3<f64>)->na::Vector3<f64>
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

pub fn predict_noise(rate:f64)->na::Matrix3<f64>
{
    let result = na::Matrix3::<f64>::new(
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    );

    result * rate
}

pub fn observation_noise(rate:f64)->na::Matrix3<f64>
{
    let result = na::Matrix3::<f64>::new(
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    );

    result * rate
}

pub fn predict_jacob(posture:na::Vector3<f64>, angular_velocity:na::Vector3<f64>, delta_time:f64)->na::Matrix3<f64>
{
    let sin_x = (posture.x).sin();
    let sin_y = (posture.y).sin();
    let cos_x = (posture.x).cos();
    let cos_y = (posture.y).cos();
    let tan_y = (posture.y).tan();

    let x_x = 1.0 + (angular_velocity.x + angular_velocity.y*tan_y*cos_x + angular_velocity.z*tan_y*sin_x)*delta_time;
    let x_y = (angular_velocity.y*(sin_x/cos_y.powi(2)) + angular_velocity.z)*delta_time;
    let x_z = 0.0;

    let y_x = (-1.0*angular_velocity.y*sin_x - angular_velocity.z*cos_x)*delta_time;
    let y_y = 1.0;
    let y_z = 0.0;

    let z_x = (angular_velocity.y*(cos_x/cos_y) - angular_velocity.z*(sin_x/cos_y))*delta_time;
    let z_y = (angular_velocity.y*sin_x*(sin_y/cos_y.powi(2)) + angular_velocity.z*cos_x*(sin_y/cos_y.powi(2)))*delta_time;
    let z_z = 1.0;

    na::Matrix3::<f64>::new(
        x_x, x_y, x_z,
        y_x, y_y, y_z,
        z_x, z_y, z_z
    )
}

pub fn observation_jacob()->na::Matrix3<f64>
{
    na::Matrix3::<f64>::new(
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    )
}

pub fn calc_predict_disp(disp:na::Matrix3<f64>, jacob:na::Matrix3<f64>, predict_noise:na::Matrix3<f64>)->na::Matrix3<f64>
{
    jacob * disp * jacob.transpose() + predict_noise
}

pub fn calc_observation_disp(predict_disp:na::Matrix3<f64>, obs_jacob:na::Matrix3<f64>, obs_noise:na::Matrix3<f64>)->na::Matrix3<f64>
{
    obs_jacob*predict_disp*obs_jacob.transpose() + obs_noise
}

pub fn calc_kalman_gain(predict_disp:na::Matrix3<f64>, obs_jacob:na::Matrix3<f64>, obs_disp:na::Matrix3<f64>)->na::Matrix3<f64>
{
    let tv_obs_disp = obs_disp.try_inverse().unwrap();

    predict_disp*obs_jacob.transpose()*tv_obs_disp
}

pub fn update_posture(predict_posture:na::Vector3<f64>, kalman_gain:na::Matrix3<f64>, obs_jacob:na::Matrix3<f64>, obs_posture:na::Vector3<f64>)->na::Vector3<f64>
{
    predict_posture + kalman_gain*(obs_posture - obs_jacob*predict_posture)
}

pub fn update_disp(kalman_gain:na::Matrix3<f64>, obs_jacob:na::Matrix3<f64>, predict_disp:na::Matrix3<f64>)->na::Matrix3<f64>
{
    let i = na::Matrix3::<f64>::new(
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    );

    (i - kalman_gain*obs_jacob)*predict_disp
}

pub fn create_vector<T>(x:T, y:T, z:T)->na::Vector3<T>
{
    na::Vector3::<T>::new(
        x,
        y,
        z
    )
}