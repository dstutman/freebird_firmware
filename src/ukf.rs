use libm::sqrtf;
use nalgebra::storage::Owned;
use nalgebra::{
    Matrix, Matrix6, Quaternion, Scalar, UnitQuaternion, Vector, Vector3, Vector6, U1, U6, U9,
};

type Vector9<N> = Matrix<N, U9, U1, Owned<N, U9, U1>>;
pub type Matrix9<N> = Matrix<N, U9, U9, Owned<N, U9, U9>>;
pub type Matrix6x9<N> = Matrix<N, U6, U9, Owned<N, U6, U9>>;

// Can't impl for Vec9 AFAIK because struct is outside crate
//pub fn new_vector9<T: Scalar, Default>(m1: T, m2: T, m3: T, m4: T, m5: T, m6: T, m7: T, m8: T, m9: T) -> Vector9<T> {
//        let mut vec: Vector9<T> = Default::default();
//        vec[0] = m1;
//        vec[1] = m2;
//        vec[2] = m3;
//        vec[3] = m4;
//        vec[4] = m5;
//        vec[5] = m6;
//        vec[6] = m7;
//        vec[7] = m8;
//        vec[8] = m9;
//        return vec;
//}
pub fn new_vector9(
    m1: f32,
    m2: f32,
    m3: f32,
    m4: f32,
    m5: f32,
    m6: f32,
    m7: f32,
    m8: f32,
    m9: f32,
) -> Vector9<f32> {
    let mut vec: Vector9<f32> = Default::default();
    vec[0] = m1;
    vec[1] = m2;
    vec[2] = m3;
    vec[3] = m4;
    vec[4] = m5;
    vec[5] = m6;
    vec[6] = m7;
    vec[7] = m8;
    vec[8] = m9;
    return vec;
}

// Deriving default avoids initialization issues in arrays
// Using unsafe in the arrays may actually be a better choice
#[derive(Copy, Clone, Default)]
pub struct State {
    pub pose: UnitQuaternion<f32>,
    pub wx: f32,
    pub wy: f32,
    pub wz: f32,
}

// The state transition function
// NOTE: Assumes small angles in rotation quaternion construction
fn F(x: State, dt: f32) -> State {
    let State { pose, wx, wy, wz } = x;
    let update_rotation = UnitQuaternion::new(Vector3::new(wx * dt, wy * dt, wz * dt));
    let new_pose = update_rotation * pose;
    return State {
        pose: new_pose,
        wx,
        wy,
        wz,
    };
}
// Deriving default avoids initialization issues in arrays
// Using unsafe in the arrays may actually be a better choice
#[derive(Copy, Clone, Default)]
pub struct Observation {
    ax: f32,
    ay: f32,
    az: f32,
    gx: f32,
    gy: f32,
    gz: f32,
    mx: f32,
    my: f32,
    mz: f32,
}

impl Observation {
    pub fn new(
        ax: f32,
        ay: f32,
        az: f32,
        gx: f32,
        gy: f32,
        gz: f32,
        mx: f32,
        my: f32,
        mz: f32,
    ) -> Observation {
        Observation {
            ax,
            ay,
            az,
            gx,
            gy,
            gz,
            mx,
            my,
            mz,
        }
    }
}

impl From<Observation> for Vector9<f32> {
    fn from(z: Observation) -> Self {
        return new_vector9(z.ax, z.ay, z.az, z.gx, z.gy, z.gz, z.mx, z.my, z.mz);
    }
}

impl From<Vector9<f32>> for Observation {
    fn from(vec: Vector9<f32>) -> Self {
        return Observation {
            ax: vec[0],
            ay: vec[1],
            az: vec[2],
            gx: vec[3],
            gy: vec[4],
            gz: vec[5],
            mx: vec[6],
            my: vec[7],
            mz: vec[8],
        };
    }
}

//impl Into<Vector6<f32>> for Observation {
//    fn into(self) -> Vector6<f32> {
//        return Vector6::new(
//            self.ax,
//            self.ay,
//            self.az,
//            self.gx,
//            self.gy,
//            self.gz
//        )
//    }
//}

fn H(x: State) -> Observation {
    // The Earth frame gravity vector represented as a quaternion
    let grav_earth = Vector3::<f32>::new(0.0, 0.0, 1.0);

    // Translated to the body frame
    let grav_body = x.pose.transform_vector(&grav_earth);

    // The Earth frame magnetic field vector
    let mag_earth = Vector3::<f32>::new(1.0, 0.0, 0.0);
    let mag_body = x.pose.transform_vector(&mag_earth);

    return Observation {
        ax: grav_body.x,
        ay: grav_body.y,
        az: grav_body.z,
        gx: x.wx,
        gy: x.wy,
        gz: x.wz,
        mx: mag_body.x,
        my: mag_body.y,
        mz: mag_body.z,
    };
}

// Deriving default avoids initialization issues in arrays
// Using unsafe in the arrays may actually be a better choice
#[derive(Copy, Clone, Default)]
struct SigmaDeviation {
    rx: f32,
    ry: f32,
    rz: f32,
    wx: f32,
    wy: f32,
    wz: f32,
}

// TODO: For some reason, implementing this from does not
// implement into.
impl From<Vector6<f32>> for SigmaDeviation {
    fn from(vect: Vector6<f32>) -> Self {
        return SigmaDeviation {
            rx: vect[0],
            ry: vect[1],
            rz: vect[2],
            wx: vect[3],
            wy: vect[4],
            wz: vect[5],
        };
    }
}

impl Into<Vector6<f32>> for SigmaDeviation {
    fn into(self) -> Vector6<f32> {
        return Vector6::<f32>::new(self.rx, self.ry, self.rz, self.wx, self.wy, self.wz);
    }
}

fn apply_deviation(dev: SigmaDeviation, x: State) -> State {
    let dev_rotation = UnitQuaternion::new(Vector3::new(dev.rx, dev.ry, dev.rz));
    let new_pose = dev_rotation * x.pose;
    return State {
        pose: new_pose,
        wx: x.wx + dev.wx,
        wy: x.wy + dev.wy,
        wz: x.wz + dev.wz,
    };
}

fn determine_deviation(x: State, x_mean: State) -> SigmaDeviation {
    let rotation = x.pose * x_mean.pose.inverse();
    return SigmaDeviation {
        rx: rotation.vector()[0],
        ry: rotation.vector()[1],
        rz: rotation.vector()[2],
        wx: x.wx - x_mean.wx,
        wy: x.wy - x_mean.wy,
        wz: x.wz - x_mean.wz,
    };
}

// The dimensionality of the state vector
const N: usize = 7;
pub struct UKF {
    current_estimate: State,
    variance: Matrix6<f32>,

    sigma_predictions: [State; 2 * (N - 1)],
    prediction_deviations: [SigmaDeviation; 2 * (N - 1)],
}

impl UKF {
    pub fn new() -> UKF {
        UKF {
            current_estimate: State {
                pose: UnitQuaternion::identity(),
                wx: 0.0,
                wy: 0.0,
                wz: 0.0,
            },
            variance: Matrix6::<f32>::identity(),
            sigma_predictions: Default::default(),
            prediction_deviations: Default::default(),
        }
    }

    pub fn predict(&mut self, Q: Matrix6<f32>, dt: f32) -> State {
        // Create the relative sigma deviations
        let sigma_deviations = sqrtf((N - 1) as f32) * self.variance.cholesky().unwrap().l();
        let mut sigma_states: [State; (N - 1) * 2] = Default::default();

        // Construct the absolute sigma states
        for i in 0..(N - 1) {
            let positive_deviation = SigmaDeviation {
                rx: sigma_deviations[(0, i)],
                ry: sigma_deviations[(1, i)],
                rz: sigma_deviations[(2, i)],
                wx: sigma_deviations[(3, i)],
                wy: sigma_deviations[(4, i)],
                wz: sigma_deviations[(5, i)],
            };
            let negative_deviation = SigmaDeviation {
                rx: -sigma_deviations[(0, i)],
                ry: -sigma_deviations[(1, i)],
                rz: -sigma_deviations[(2, i)],
                wx: -sigma_deviations[(3, i)],
                wy: -sigma_deviations[(4, i)],
                wz: -sigma_deviations[(5, i)],
            };
            sigma_states[i] = apply_deviation(negative_deviation, self.current_estimate);
            sigma_states[(N - 1) + i] = apply_deviation(positive_deviation, self.current_estimate);
        }

        // Map the sigma states into the prediction space
        let mut sigma_predictions: [State; 2 * (N - 1)] = Default::default();
        for (i, &state) in sigma_states.iter().enumerate() {
            sigma_predictions[i] = F(state, dt);
        }

        // Calculate the apriori mean and covariance
        let predicted_state = {
            // FIXME: Check the runtime for this mean_of, might be slow iterative
            let estimated_pose = UnitQuaternion::mean_of(sigma_predictions.iter().map(|y| y.pose));
            let wx = sigma_predictions.iter().map(|y| y.wx).sum::<f32>()
                / (sigma_predictions.len() as f32);
            let wy = sigma_predictions.iter().map(|y| y.wy).sum::<f32>()
                / (sigma_predictions.len() as f32);
            let wz = sigma_predictions.iter().map(|y| y.wz).sum::<f32>()
                / (sigma_predictions.len() as f32);

            State {
                pose: estimated_pose,
                wx,
                wy,
                wz,
            }
        };

        let mut prediction_deviations: [SigmaDeviation; 2 * (N - 1)] = Default::default();
        for (i, &state) in sigma_predictions.iter().enumerate() {
            prediction_deviations[i] = determine_deviation(state, predicted_state);
        }

        // TODO: Next step is to calculate deviations and covariance
        let prediction_variance = {
            let mut tmp_variance: Matrix6<f32> = Default::default();
            for &dev in prediction_deviations.iter() {
                let sigma_mat: Vector6<f32> = dev.into();
                tmp_variance += sigma_mat * sigma_mat.transpose();
            }
            tmp_variance /= prediction_deviations.len() as f32;
            tmp_variance += Q;

            tmp_variance
        };

        self.current_estimate = predicted_state;
        self.variance = prediction_variance;
        self.sigma_predictions = sigma_predictions;
        self.prediction_deviations = prediction_deviations;

        return predicted_state;
    }

    pub fn update(&mut self, z: Observation, R: Matrix9<f32>) -> State {
        // Map the sigma predictions into the observation space
        // and calculate the mean and covariance.
        let mut sigma_observations: [Observation; N * 2] = Default::default();
        for (i, &state) in self.sigma_predictions.iter().enumerate() {
            sigma_observations[i] = H(state);
        }

        let predicted_observation = {
            let ax = sigma_observations.iter().map(|z| z.ax).sum::<f32>()
                / (sigma_observations.len() as f32);
            let ay = sigma_observations.iter().map(|z| z.ay).sum::<f32>()
                / (sigma_observations.len() as f32);
            let az = sigma_observations.iter().map(|z| z.az).sum::<f32>()
                / (sigma_observations.len() as f32);
            let gx = sigma_observations.iter().map(|z| z.gx).sum::<f32>()
                / (sigma_observations.len() as f32);
            let gy = sigma_observations.iter().map(|z| z.gy).sum::<f32>()
                / (sigma_observations.len() as f32);
            let gz = sigma_observations.iter().map(|z| z.gz).sum::<f32>()
                / (sigma_observations.len() as f32);
            let mx = sigma_observations.iter().map(|z| z.mx).sum::<f32>()
                / (sigma_observations.len() as f32);
            let my = sigma_observations.iter().map(|z| z.my).sum::<f32>()
                / (sigma_observations.len() as f32);
            let mz = sigma_observations.iter().map(|z| z.mz).sum::<f32>()
                / (sigma_observations.len() as f32);
            Observation {
                ax,
                ay,
                az,
                gx,
                gy,
                gz,
                mx,
                my,
                mz,
            }
        };

        let (observation_variance, observation_deviations) = {
            let mut tmp_deviations: [Vector9<f32>; 2 * N] = Default::default();
            let mut tmp_variance: Matrix9<f32> = Default::default();
            for (i, &elem) in sigma_observations.iter().enumerate() {
                let sigma_obs: Vector9<f32> = elem.into();
                let pred_obs: Vector9<f32> = predicted_observation.into();
                let zd = sigma_obs - pred_obs;
                tmp_deviations[i] = zd;
                tmp_variance += zd * zd.transpose();
            }
            tmp_variance /= sigma_observations.len() as f32;

            (tmp_variance, tmp_deviations)
        };

        let innovation_variance = observation_variance + R;

        // Calculate the cross correlation
        let cross_variance = {
            let mut tmp_variance: Matrix6x9<f32> = Default::default();
            for (&pred_deviation, &obs_deviation) in self
                .prediction_deviations
                .iter()
                .zip(observation_deviations.iter())
            {
                let pd: Vector6<f32> = pred_deviation.into();
                tmp_variance += pd * obs_deviation.transpose();
            }

            tmp_variance /= observation_deviations.len() as f32;

            tmp_variance
        };

        let kalman_gain = cross_variance * innovation_variance.try_inverse().unwrap();
        let zv: Vector9<f32> = z.into();
        let pov: Vector9<f32> = predicted_observation.into();
        let innovation = zv - pov;
        let delta = kalman_gain * innovation;

        self.current_estimate = apply_deviation(delta.into(), self.current_estimate);
        self.variance = self.variance - kalman_gain * innovation_variance * kalman_gain.transpose();
        return self.current_estimate;
    }
}
