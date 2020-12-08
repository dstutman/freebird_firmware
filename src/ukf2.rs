use core::ops::Neg;

use cortex_m::asm;
use rtt_target::rprintln;

use crate::linalg::*;
use crate::quaternions::*;

#[derive(Debug, Copy, Clone, Default)]
pub struct State {
    pub pose: Quaternion,
    pub wx: f32,
    pub wy: f32,
    pub wz: f32,
}

impl State {
    fn mean_of<'a>(states: &[State]) -> State {
        let mp = Quaternion::mean_of(states.iter().map(|x| x.pose));

        // The angular velocity means are best computed in a loop because
        // iterators don't provide reliable length information.
        let mut n = 0.0;
        let mut swx = 0.0;
        let mut swy = 0.0;
        let mut swz = 0.0;
        for x in states {
            n += 1.0;
            swx += x.wx;
            swy += x.wy;
            swz += x.wz;
        }
        return State {
            pose: mp,
            wx: swx / n,
            wy: swy / n,
            wz: swz / n,
        };
    } 
}

#[derive(Debug, Copy, Clone, Default)]
pub struct Observation {
    ax: f32,
    ay: f32,
    az: f32,
    wx: f32,
    wy: f32,
    wz: f32,
//    mx: f32,
//    my: f32,
//    mz: f32
}

impl Observation {
    pub fn new(ax: f32, ay: f32, az: f32, wx: f32, wy: f32, wz: f32) -> Observation {
        return Observation {
            ax,
            ay,
            az,
            wx,
            wy,
            wz,
        };
    }

    fn mean_of(observations: &[Observation]) -> Observation {
        let mut n = 0.0;
        let mut sax = 0.0;
        let mut say = 0.0;
        let mut saz = 0.0;
        let mut swx = 0.0;
        let mut swy = 0.0;
        let mut swz = 0.0;
        for o in observations {
            n += 1.0;
            sax += o.ax;
            say += o.ay;
            saz += o.az;
            swx += o.wx;
            swy += o.wy;
            swz += o.wz;
        }
        return Observation {
            ax: sax / n,
            ay: say / n,
            az: saz / n,
            wx: swx / n,
            wy: swy / n,
            wz: swz / n,
        };
    }
}

impl From<Matrix<f32, 6, 1>> for Observation {
    fn from(mat: Matrix<f32, 6, 1>) -> Self {
        return Observation {
            ax: mat[(0, 0)],
            ay: mat[(1, 0)],
            az: mat[(2, 0)],
            wx: mat[(3, 0)],
            wy: mat[(4, 0)],
            wz: mat[(5, 0)],
        };
    }
}

impl From<Observation> for Matrix<f32, M, 1> {
    fn from(obs: Observation) -> Self {
        return Matrix::from_array([[obs.ax, obs.ay, obs.az, obs.wx, obs.wy, obs.wz]]);
    }
}

pub fn check_notnan<const R: usize, const C: usize>(mat: Matrix<f32, R, C>) {
    for i in 0..R {
        for j in 0..C {
            if mat[(i, j)].is_nan() {
                asm::bkpt()
            }
        }
    }
}

pub fn check_notnanarr(arr: &[State]) {
    for elem in arr {
        if elem.wx.is_nan() || elem.wy.is_nan() || elem.wz.is_nan() {
            asm::bkpt()
        }
    }
}

#[derive(Debug, Copy, Clone, Default)]
struct Sigma {
    drx: f32,
    dry: f32,
    drz: f32,
    dwx: f32,
    dwy: f32,
    dwz: f32,
}

impl Neg for Sigma {
    type Output = Self;
    fn neg(self) -> Self {
        let Sigma {
            drx,
            dry,
            drz,
            dwx,
            dwy,
            dwz,
        } = self;
        return Sigma {
            drx: -drx,
            dry: -dry,
            drz: -drz,
            dwx: -dwx,
            dwy: -dwy,
            dwz: -dwz,
        };
    }
}

impl From<Matrix<f32, 6, 1>> for Sigma {
    fn from(mat: Matrix<f32, 6, 1>) -> Self {
        return Sigma {
            drx: mat[(0, 0)],
            dry: mat[(1, 0)],
            drz: mat[(2, 0)],
            dwx: mat[(3, 0)],
            dwy: mat[(4, 0)],
            dwz: mat[(5, 0)],
        };
    }
}

impl From<Sigma> for Matrix<f32, 6, 1> {
    fn from(sig: Sigma) -> Self {
        return Matrix::from_array([[sig.drx, sig.dry, sig.drz, sig.dwx, sig.dwy, sig.dwz]]);
    }
}

fn f(x: State, dt: f32) -> State {
    let State { pose, wx, wy, wz } = x;
    let qr = Quaternion::new(1.0, wx * dt / 2.0, wy * dt / 2.0, wz * dt / 2.0).normalized();
    return State {
        pose: qr * pose,
        ..x
    };
}

fn h(x: State) -> Observation {
    let State { pose, wx, wy, wz } = x;

    // Represent the Earth frame gravitational acceleration as a 
    // quaternion. Then q * v * q* gives the body frame gravity 
    // vector represented as a quaternion.
    let ae = Quaternion::new(0.0, 0.0, 0.0, 1.0);
    let ab = pose * ae * pose.conj();
    return Observation {
        ax: ab.x(),
        ay: ab.y(),
        az: ab.z(),
        wx,
        wy,
        wz,
    };
}

fn deviate(x: State, d: Sigma) -> State {
    let State { pose, wx, wy, wz } = x;
    let Sigma {
        drx,
        dry,
        drz,
        dwx,
        dwy,
        dwz,
    } = d;
    let qr = Quaternion::new(1.0, drx / 2.0, dry / 2.0, drz / 2.0).normalized();
    return State {
        pose: qr * pose,
        wx: wx + dwx,
        wy: wy + dwy,
        wz: wz + dwz,
    };
}

fn deviation(x: State, m: State) -> Sigma {
    let State { pose, wx, wy, wz } = x;
    let State {
        pose: mpose,
        wx: mwx,
        wy: mwy,
        wz: mwz,
    } = m;
    let qr = pose * mpose.conj();
    return Sigma {
        drx: qr.x(),
        dry: qr.y(),
        drz: qr.z(),
        dwx: wx - mwx,
        dwy: wy - mwy,
        dwz: wz - mwz,
    };
}

// The dimensionality of the state variance of the UKF.
// This cannot be set as an associated const of the impl
// unfortunately.
const N: usize = 6;
// The dimensionality of the observation variance.
const M: usize = 6;

pub struct UKF {
    x: State,
    P: Matrix<f32, N, N>,
    Y: [State; 2 * N + 1],
    WP: [Sigma; 2 * N + 1],
}

impl UKF {
    pub fn new() -> UKF {
        return UKF {
            x: Default::default(),
            P: Matrix::identity() * 1.0,
            Y: Default::default(),
            WP: Default::default(),
        };
    }

    // Internal methods for setting internal states
    // while upholding invariants.
    fn update_estimate(&mut self, x: State) {
        self.x = State {
            // Ensure normalized pose
            pose: x.pose.normalized(),
            ..x
        }
    }

    fn update_variance(&mut self, P: Matrix<f32, N, N>) {
        // Ensure symmetric variance
        self.P = P * 0.5 + P.transpose() * 0.5
    }

    fn update_predictions(&mut self, Y: [State; 2 * N + 1]) {
        self.Y = Y;
    }

    fn update_deltas(&mut self, WP: [Sigma; 2 * N + 1]) {
        self.WP = WP;
    }

    // Public methods for getting state and variance
    pub fn get_state(&self) -> State {
        return self.x;
    }

    pub fn get_variance(&self) -> Matrix<f32, N, N> {
        return self.P;
    }

    // This function predicts the next state and updates
    // the internal prediction state to match.
    // It returns the updated prediction.
    pub fn predict(&mut self, Q: Matrix<f32, N, N>, dt: f32) -> State {
        // Calculate the sigma deviations
        let W = (self.P * dt * N as f32).cholesky();
        //rprintln!("{:?}", self.P * N as f32);
        check_notnan(W);

        // Apply the columns of W as deviations
        // to the current state estimate.
        let x = self.x;
        let mut X: [State; 2 * N + 1] = Default::default();
        for i in 0..N {
            let w = Sigma {
                drx: W[(0, i)],
                dry: W[(1, i)],
                drz: W[(2, i)],
                dwx: W[(3, i)],
                dwy: W[(4, i)],
                dwz: W[(5, i)],
            };
            X[i] = deviate(x, w);
            X[i + N] = deviate(x, -w);
            X[2 * N] = x;
        }

        // Map the states into the prediction space
        let mut Y: [State; 2 * N + 1] = Default::default();
        for i in 0..(2 * N + 1) {
            Y[i] = f(X[i], dt);
        }

        // Use the predictions to calculate the apriori mean,
        // prediction-mean deltas and variance
        let ya = State::mean_of(&Y);
        let mut WP: [Sigma; 2 * N + 1] = Default::default();
        for i in 0..(2 * N + 1) {
            WP[i] = deviation(Y[i], ya);
        }
        let PA = {
            let mut tmp: Matrix<f32, N, N> = Default::default();
            for &wp in WP.iter() {
                let mat: Matrix<f32, N, 1> = wp.into();
                tmp += mat * mat.transpose();
            }
            tmp / WP.len() as f32 + Q * dt
        };

        // Current prediction and variance
        self.update_estimate(ya);
        self.update_variance(PA / dt);

        // Intermediate values for `update`
        self.update_predictions(Y);
        self.update_deltas(WP);

        return self.x;
    }

    // This function updates the current state prediction
    // with a new observation and updates the internal prediction
    // state to match. It returns the updated prediction.
    pub fn update(&mut self, z: Observation, R: Matrix<f32, M, M>) -> State {
        // Map the predictions into the observation space
        let mut Z: [Observation; 2 * N + 1] = Default::default();
        for i in 0..(2 * N + 1) {
            Z[i] = h(self.Y[i]);
            //rprintln!("{:?}", h(self.Y[i]));
        }

        // Use the predicted observations to calculate
        // the expected observation mean and variance
        let za = Observation::mean_of(&Z);
        //rprintln!("{:?}", z);
        //rprintln!("{:?}", za);
        let mut WZ: [Matrix<f32, M, 1>; 2 * N + 1] = Default::default();
        for i in 0..(2 * N + 1) {
            let mean_mat: Matrix<f32, M, 1> = z.into();
            let obs_mat: Matrix<f32, M, 1> = Z[i].into();
            WZ[i] = obs_mat - mean_mat;
        }

        // TODO: Report this bug to rust-analyzer
        // Type inferred as f32, but should be Matrix<f32, N, N>
        let PZ = {
            let mut tmp: Matrix<f32, M, M> = Default::default();
            for i in 0..(2 * N + 1) {
                let mat: Matrix<f32, M, 1> = WZ[i].into();
                tmp += mat * mat.transpose();
            }
            tmp / WZ.len() as f32
        };

        // Calculate the innovation variance
        let PV = PZ + R;

        // Calculate the prediction-observation cross correlation
        let PYZ = {
            let mut tmp: Matrix<f32, N, M> = Default::default();
            for i in 0..(2 * N + 1) {
                let wp_mat: Matrix<f32, N, 1> = self.WP[i].into();
                tmp += wp_mat * WZ[i].transpose();
            }
            tmp / WZ.len() as f32
        };

        // Perform the Kalman update
        let K = PYZ * PV.invert_cholesky();
        let v = Matrix::from(z) - Matrix::from(za);
        let delta = K * v;
        let x = deviate(self.x, delta.into());
        let P = self.P - K * PV * K.transpose();

        // Update the internal state
        self.update_estimate(x);
        self.update_variance(P);

        return self.x;
    }
}
