use crate::linalg::Matrix;
use crate::quaternions::Quaternion;
use cortex_m::asm;
use libm::sqrtf;

use rtt_target::rprintln;

// Deriving default avoids initialization issues in arrays
// Using unsafe in the arrays may actually be a better choice
#[derive(Copy, Clone, Default, Debug)]
pub struct State {
    pub pose: Quaternion,
    pub wx: f32,
    pub wy: f32,
    pub wz: f32,
}

// The state transition function
// NOTE: Assumes small angles in rotation quaternion construction
fn F(x: State, dt: f32) -> State {
    let State { pose, wx, wy, wz } = x;
    let update_rotation =
        Quaternion::new(1.0, wx * dt / 2.0, wy * dt / 2.0, wz * dt / 2.0).normalized();
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

impl From<Observation> for Matrix<f32, 9, 1> {
    fn from(z: Observation) -> Self {
        return Matrix::from_array([[z.ax, z.ay, z.az, z.gx, z.gy, z.gz, z.mx, z.my, z.mz]]);
    }
}

impl From<Matrix<f32, 9, 1>> for Observation {
    fn from(mat: Matrix<f32, 9, 1>) -> Self {
        return Observation {
            ax: mat[(0, 0)],
            ay: mat[(1, 0)],
            az: mat[(2, 0)],
            gx: mat[(3, 0)],
            gy: mat[(4, 0)],
            gz: mat[(5, 0)],
            mx: mat[(6, 0)],
            my: mat[(7, 0)],
            mz: mat[(8, 0)],
        };
    }
}

impl Into<Matrix<f32, 6, 1>> for Observation {
    fn into(self) -> Matrix<f32, 6, 1> {
        return Matrix::from_array([[self.ax, self.ay, self.az, self.gx, self.gy, self.gz]]);
    }
}

fn H(x: State) -> Observation {
    // The Earth frame gravity vector
    let grav_earth = Quaternion::new(0.0, 0.0, 0.0, 1.0);
    let grav_body = x.pose * grav_earth * x.pose.conj();

    // The Earth frame magnetic field vector
    let mag_earth = Quaternion::new(0.0, 1.0, 0.0, 0.0);
    let mag_body = x.pose * mag_earth * x.pose.conj();

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
#[derive(Debug, Copy, Clone, Default)]
struct SigmaDeviation {
    rx: f32,
    ry: f32,
    rz: f32,
    wx: f32,
    wy: f32,
    wz: f32,
}

impl From<Matrix<f32, 6, 1>> for SigmaDeviation {
    fn from(mat: Matrix<f32, 6, 1>) -> Self {
        return SigmaDeviation {
            rx: mat[(0, 0)],
            ry: mat[(1, 0)],
            rz: mat[(2, 0)],
            wx: mat[(3, 0)],
            wy: mat[(4, 0)],
            wz: mat[(5, 0)],
        };
    }
}

impl Into<Matrix<f32, 6, 1>> for SigmaDeviation {
    fn into(self) -> Matrix<f32, 6, 1> {
        return Matrix::<f32, 6, 1>::from_array([[
            self.rx, self.ry, self.rz, self.wx, self.wy, self.wz,
        ]]);
    }
}

fn apply_deviation(dev: SigmaDeviation, x: State) -> State {
    let dev_rotation = Quaternion::new(1.0, dev.rx/2.0, dev.ry/2.0, dev.rz/2.0).normalized();
    let new_pose = dev_rotation * x.pose;
    return State {
        pose: new_pose,
        wx: x.wx + dev.wx,
        wy: x.wy + dev.wy,
        wz: x.wz + dev.wz,
    };
}

fn determine_deviation(x: State, x_mean: State) -> SigmaDeviation {
    let rotation = x.pose * x_mean.pose.conj();
    return SigmaDeviation {
        rx: rotation.x,
        ry: rotation.y,
        rz: rotation.z,
        wx: x.wx - x_mean.wx,
        wy: x.wy - x_mean.wy,
        wz: x.wz - x_mean.wz,
    };
}

// The dimensionality of the variance
const N: usize = 6;
pub struct UKF {
    current_estimate: State,
    variance: Matrix<f32, 6, 6>,

    sigma_predictions: [State; 2 * N],
    prediction_deviations: [SigmaDeviation; 2 * N],
}

impl UKF {
    pub fn new() -> UKF {
        UKF {
            current_estimate: State {
                pose: Default::default(),
                wx: 0.0,
                wy: 0.0,
                wz: 0.0,
            },
            variance: Matrix::<f32, 6, 6>::identity(),
            sigma_predictions: Default::default(),
            prediction_deviations: Default::default(),
        }
    }

    pub fn predict(&mut self, Q: Matrix<f32, 6, 6>, dt: f32) -> State {
        // Create the relative sigma deviations
        let sigma_deviations = self.variance.cholesky() * sqrtf(N as f32);
        let mut sigma_states: [State; N * 2] = Default::default();

        // Construct the absolute sigma states
        for i in 0..N {
            let negative_deviation = SigmaDeviation {
                rx: -sigma_deviations[(0, i)],
                ry: -sigma_deviations[(1, i)],
                rz: -sigma_deviations[(2, i)],
                wx: -sigma_deviations[(3, i)],
                wy: -sigma_deviations[(4, i)],
                wz: -sigma_deviations[(5, i)],
            };
            let positive_deviation = SigmaDeviation {
                rx: sigma_deviations[(0, i)],
                ry: sigma_deviations[(1, i)],
                rz: sigma_deviations[(2, i)],
                wx: sigma_deviations[(3, i)],
                wy: sigma_deviations[(4, i)],
                wz: sigma_deviations[(5, i)],
            };
            sigma_states[i] = apply_deviation(negative_deviation, self.current_estimate);
            sigma_states[N + i] = apply_deviation(positive_deviation, self.current_estimate);
        }

        // Map the sigma states into the prediction space
        let mut sigma_predictions: [State; 2 * N] = Default::default();
        for (i, &state) in sigma_states.iter().enumerate() {
            sigma_predictions[i] = F(state, dt);
        }

        // Calculate the apriori mean and covariance
        let predicted_state = {
            // FIXME: Check the runtime for this mean_of, might be slow iterative
            // TODO: Possible error source
            let estimated_pose = Quaternion::mean_of(sigma_predictions.iter().map(|y| y.pose));
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

        let mut prediction_deviations: [SigmaDeviation; 2 * N] = Default::default();
        for (i, &state) in sigma_predictions.iter().enumerate() {
            prediction_deviations[i] = determine_deviation(state, predicted_state);
        }

        // TODO: Next step is to calculate deviations and covariance
        let prediction_variance = {
            let mut tmp_variance: Matrix<f32, 6, 6> = Default::default();
            for &dev in prediction_deviations.iter() {
                let sigma_mat: Matrix<f32, 6, 1> = dev.into();
                tmp_variance += sigma_mat * sigma_mat.transpose();
            }
            tmp_variance /= prediction_deviations.len() as f32;
            tmp_variance += Q * dt;

            tmp_variance
        };

        self.current_estimate = predicted_state;
        self.variance = prediction_variance;
        self.sigma_predictions = sigma_predictions;
        self.prediction_deviations = prediction_deviations;

        return predicted_state;
    }

    pub fn update(&mut self, z: Observation, R: Matrix<f32, 9, 9>) -> State {
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
            let mut tmp_deviations: [Matrix<f32, 9, 1>; 2 * N] = Default::default();
            let mut tmp_variance: Matrix<f32, 9, 9> = Default::default();
            for (i, &elem) in sigma_observations.iter().enumerate() {
                let sigma_obs: Matrix<f32, 9, 1> = elem.into();
                let pred_obs: Matrix<f32, 9, 1> = predicted_observation.into();
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
            let mut tmp_variance: Matrix<f32, 6, 9> = Default::default();
            for (&pred_deviation, &obs_deviation) in self
                .prediction_deviations
                .iter()
                .zip(observation_deviations.iter())
            {
                let pd: Matrix<f32, 6, 1> = pred_deviation.into();
                tmp_variance += pd * obs_deviation.transpose();
            }

            tmp_variance /= observation_deviations.len() as f32;

            tmp_variance
        };

        let kalman_gain = cross_variance * innovation_variance.invert_cholesky();
        let zv: Matrix<f32, 9, 1> = z.into();
        let pov: Matrix<f32, 9, 1> = predicted_observation.into();
        let innovation = zv - pov;
        let delta = kalman_gain * innovation;

        self.current_estimate = apply_deviation(delta.into(), self.current_estimate);
        self.variance = self.variance - kalman_gain * innovation_variance * kalman_gain.transpose();

        return self.current_estimate;
    }
}
