use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub};
use libm::{powf, sqrtf};
use rtt_target::rprintln;

#[derive(Debug, Copy, Clone)]
pub struct Quaternion {
    w: f32,
    x: f32,
    y: f32,
    z: f32,
}

impl Quaternion {
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        return Quaternion { w, x, y, z };
    }

    pub fn w(&self) -> f32 {
        self.w
    }

    pub fn x(&self) -> f32 {
        self.x
    }

    pub fn y(&self) -> f32 {
        self.y
    }

    pub fn z(&self) -> f32 {
        self.z
    }

    // Dead simple calculation of the mean of quaternions
    // Works for relatively "close" UNIT quaternions.
    pub fn mean_of(quats: impl Iterator<Item = Quaternion>) -> Quaternion {
        let mut tmp_quat = Quaternion::new(0.0, 0.0, 0.0, 0.0);
        let mut n = 0;
        for q in quats {
            tmp_quat += q;
            n += 1;
        }
        tmp_quat /= n as f32;
        return tmp_quat.normalized();
    }

    pub fn norm(&self) -> f32 {
        return sqrtf(powf(self.w, 2.0) + powf(self.x, 2.0) + powf(self.y, 2.0) + powf(self.z, 2.0));
    }

    pub fn conj(&self) -> Self {
        return Quaternion {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        };
    }

    pub fn normalized(&self) -> Self {
        let norm = self.norm();
        return Quaternion {
            w: self.w / norm,
            x: self.x / norm,
            y: self.y / norm,
            z: self.z / norm,
        };
    }

    pub fn scale(&self, factor: f32) -> Self {
        return Quaternion {
            w: self.w * factor,
            x: self.x * factor,
            y: self.y * factor,
            z: self.z * factor,
        };
    }
}

impl Default for Quaternion {
    fn default() -> Self {
        return Quaternion::new(1.0, 0.0, 0.0, 0.0);
    }
}

impl Mul for Quaternion {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self {
        return Quaternion {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        };
    }
}

impl Mul<f32> for Quaternion {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self {
        return Quaternion {
            w: self.w * rhs,
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        };
    }
}

impl MulAssign<f32> for Quaternion {
    fn mul_assign(&mut self, rhs: f32) {
        *self = *self * rhs;
    }
}

impl Div<f32> for Quaternion {
    type Output = Self;
    fn div(self, rhs: f32) -> Self {
        return Quaternion {
            w: self.w / rhs,
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
        };
    }
}

impl DivAssign<f32> for Quaternion {
    fn div_assign(&mut self, rhs: f32) {
        *self = *self / rhs;
    }
}

impl Add for Quaternion {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        return Quaternion {
            w: self.w + rhs.w,
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        };
    }
}

impl AddAssign for Quaternion {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl Sub for Quaternion {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        return Quaternion {
            w: self.w - rhs.w,
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        };
    }
}
