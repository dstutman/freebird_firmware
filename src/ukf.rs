use libm::sqrtf;
use nalgebra::{ArrayStorage, Matrix, Matrix6, Quaternion, UnitQuaternion, Vector3, U1, U7};

//type State = Matrix<f32, U1, U7, ArrayStorage<f32, U1, U7>>;

// Deriving default avoids initialization issues in arrays
// Using unsafe in the arrays may actually be a better choice
#[derive(Copy, Clone, Default)]
struct State {
    pose: UnitQuaternion<f32>,
    wx: f32,
    wy: f32,
    wz: f32,
}

// The state transition function
// NOTE: Assumes small angles in rotation quaternion construction
fn F(x: State, dt: f32) -> State {
    let State { pose, wx, wy, wz } = x;
    let update_rotation = UnitQuaternion::new(Vector3::new(wx * dt, wy * dt, wz * dt));
    let new_pose = update_rotation * pose;
    return State { pose, wx, wy, wz };
}

struct Observation {
    ax: f32,
    ay: f32,
    az: f32,
    gx: f32,
    gy: f32,
    gz: f32,
}

fn H(x: State) -> Observation {
    // The Earth frame gravity vector represented as a quaternion
    let grav_earth = Vector3::<f32>::new(0.0, 0.0, 1.0);

    // Translated to the body frame
    let grav_body = x.pose.transform_vector(&grav_earth);
    return Observation {
        ax: grav_body.x,
        ay: grav_body.y,
        az: grav_body.z,
        gx: x.wx,
        gy: x.wy,
        gz: x.wz,
    };
}

struct UKF {
    // For update and readout
    current_estimate: State,
    covariance: Matrix6<f32>,
    process_noise: Matrix6<f32>, // For sharing from predict -> update
}

struct SigmaDeviation {
    rx: f32,
    ry: f32,
    rz: f32,
    wx: f32,
    wy: f32,
    wz: f32,
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

// The dimensionality of the state vector
// TODO: Check this shouldn't be 6
const n: usize = 7;
impl UKF {
    pub fn predict(&self, Q: Matrix6<f32>, dt: f32) {
        // Create the relative sigma deviations
        let sigma_deviations = sqrtf(n as f32) * self.covariance.cholesky().unwrap().l();
        let mut sigma_states: [State; n * 2] = Default::default();

        // Construct the absolute sigma states
        for i in 0..(n) {
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
            sigma_states[n + i] = apply_deviation(positive_deviation, self.current_estimate);
        }

        // Map the sigma states into the prediction space
        let mut sigma_predictions: [State; n * 2] = Default::default();
        for i in 0..(2 * n) {
            sigma_predictions[i] = F(sigma_states[i], dt);
        }

        // Calculate the apriori mean and covariance
        let apriori_state = {
            // FIXME: Check the runtime for this mean_of, might be slow iterative
            let estimated_pose =
                UnitQuaternion::mean_of(sigma_predictions.into_iter().map(|y| y.pose));
            let mut wx = sigma_predictions.into_iter().map(|y| y.wx).sum::<f32>()
                / (sigma_predictions.len() as f32);
            let mut wy = sigma_predictions.into_iter().map(|y| y.wy).sum::<f32>()
                / (sigma_predictions.len() as f32);
            let mut wz = sigma_predictions.into_iter().map(|y| y.wz).sum::<f32>()
                / (sigma_predictions.len() as f32);

            State {
                pose: estimated_pose,
                wx,
                wy,
                wz,
            }
        };

        // TODO: Next step is to calculate deviations and covariance
    }
}
