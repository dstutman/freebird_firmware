#![feature(min_const_generics)]
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicUsize, Ordering};

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::{self, asm};
use cortex_m_rt::{entry, exception};
use futures::future::join;
use libm::{powf, sqrtf};
use linalg::Matrix;
use quaternions::Quaternion;
use stm32f3::stm32f303;

use crate::executor::Executor;
use futures::TryFutureExt;

use core::fmt::write;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

mod bmp;
mod executor;
mod i2c;
mod linalg;
mod lsm;
mod quaternions;
//mod ukf;
mod ukf2;
mod usart;

static TICKS: AtomicUsize = AtomicUsize::new(0);

#[inline(always)]
fn get_ticks() -> usize {
    return TICKS.load(Ordering::Relaxed);
}

#[exception]
fn SysTick() {
    let _ = TICKS.fetch_add(1, Ordering::Relaxed);
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let mut cp = stm32f303::CorePeripherals::take().unwrap();
    cp.SYST.set_clock_source(SystClkSource::Core);
    cp.SYST.set_reload(72000000 / 1000 - 1);
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();

    let dp = stm32f303::Peripherals::take().unwrap();
    dp.FLASH.acr.write(|w| w.latency().ws2());
    while !dp.FLASH.acr.read().latency().is_ws2() {}
    dp.RCC.cr.write(|w| w.hsion().on());
    while !dp.RCC.cr.read().hsirdy().bit_is_set() {}
    dp.RCC
        .cfgr
        .write(|w| w.pllsrc().hsi_div_prediv().pllmul().mul9());
    dp.RCC.cr.write(|w| w.pllon().on());
    while !dp.RCC.cr.read().pllrdy().bit_is_set() {}
    dp.RCC.cfgr.write(|w| w.ppre1().div2());
    dp.RCC.cfgr.write(|w| w.sw().pll());
    while !dp.RCC.cfgr.read().sws().is_pll() {}

    dp.RCC.ahbenr.write(|w| w.iopben().enabled());
    let gpiob = dp.GPIOB;
    gpiob
        .moder
        .write(|w| w.moder8().alternate().moder9().alternate());
    gpiob
        .ospeedr
        .write(|w| w.ospeedr8().high_speed().ospeedr9().high_speed());
    gpiob
        .otyper
        .write(|w| w.ot8().push_pull().ot9().push_pull());
    gpiob
        .pupdr
        .write(|w| w.pupdr8().pull_up().pupdr9().pull_up());
    gpiob.afrh.write(|w| w.afrh8().af4().afrh9().af4());
    dp.RCC.apb1enr.write(|w| w.i2c1en().enabled());
    i2c::init(dp.I2C1);

    dp.RCC.ahbenr.write(|w| w.iopaen().enabled());
    let gpioa = dp.GPIOA;
    gpioa
        .moder
        .write(|w| w.moder2().alternate().moder3().alternate());
    gpioa
        .ospeedr
        .write(|w| w.ospeedr2().high_speed().ospeedr3().high_speed());
    gpioa
        .otyper
        .write(|w| w.ot2().push_pull().ot3().push_pull());
    gpioa
        .pupdr
        .write(|w| w.pupdr2().floating().pupdr3().floating());
    gpioa.afrl.write(|w| w.afrl2().af7().afrl3().af7());
    dp.RCC.apb1enr.modify(|_, w| w.usart2en().enabled());
    usart::init(dp.USART2);

    rprintln!("Starting executor...");
    let exec = Executor::new();
    exec.run(test_future());
}

// Just for debug serial
// From https://stackoverflow.com/questions/39488327/how-to-format-output-to-a-byte-array-with-no-std-and-no-allocator
use core::f32::consts::PI;
use core::fmt::{self, Write};

struct Wrapper<'a> {
    buf: &'a mut [u8],
    offset: usize,
}

impl<'a> Wrapper<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Wrapper {
            buf: buf,
            offset: 0,
        }
    }
}

impl<'a> fmt::Write for Wrapper<'a> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();

        // Skip over already-copied data
        let remainder = &mut self.buf[self.offset..];
        // Check if there is space remaining (return error instead of panicking)
        if remainder.len() < bytes.len() {
            return Err(core::fmt::Error);
        }
        // Make the two slices the same length
        let remainder = &mut remainder[..bytes.len()];
        // Copy
        remainder.copy_from_slice(bytes);

        // Update offset to avoid overwriting
        self.offset += bytes.len();

        Ok(())
    }
}

async fn test_future() {
    //let lsm = lsm::Settings::default().init().await.unwrap();
    //loop {
    //    let mut xvg = 0.0;
    //    let mut yvg = 0.0;
    //    let mut zvg = 0.0;
    //    for _ in 0..1000 {
    //        let sample = lsm.acceleration().await.unwrap();
    //        xvg +=  sample.ax / 1000.0;
    //        yvg += sample.ay / 1000.0;
    //        zvg += sample.az / 1000.0;
    //    }
    //    rprintln!("Acc Sample: ({}, {}, {})", xvg, yvg, zvg);
    //}
    let lsm = lsm::Settings::default().init().await.unwrap();
    let bmp = bmp::Settings::default().init().await.unwrap();
    let mut last_ticks = 0;
    //let mut last_quat = Quaternion::new(1., 0., 0., 0.);
    let mut abx = 0.0;
    let mut aby = 0.0;
    let mut abz = 0.0;
    let mut gbx = 0.0;
    let mut gby = 0.0;
    let mut gbz = 0.0;
    let mut n = 0;
    while n < 1000 {
        n += 1;
        let acc_sample = lsm.acceleration().await.unwrap();
        abx += acc_sample.ax;
        aby += acc_sample.ay;
        abz += acc_sample.az;
        let gyro_sample = lsm.angular_rate().await.unwrap();
        gbx += gyro_sample.gx;
        gby += gyro_sample.gy;
        gbz += gyro_sample.gz;
    }
    abx /= n as f32;
    aby /= n as f32;
    abz /= n as f32;

    abz -= 1.0;

    gbx /= n as f32;
    gby /= n as f32;
    gbz /= n as f32;

    // NOTE!!!: Don't put the instantiation inside the loop...
    let mut filter = ukf2::UKF::new();
    loop {
        if get_ticks() - last_ticks > 20 {
            let acc_sample = lsm.acceleration().await.unwrap();
            let rate_sample = lsm.angular_rate().await.unwrap();
            //let mag_sample = lsm.magnetic().await.unwrap();
            //let mut msg = [0 as u8; 100];
            //let mag_mat = Matrix::from_array([[-mag_sample.my, -mag_sample.mx, mag_sample.mz]]);
            //let antibias = Matrix::from_array([[42.65, -35.44, -37.62]]) / 100.0;
            //let antidistortion = Matrix::from_array([
            //    [1.003, 0.031, 0.015],
            //    [0.031, 0.957, 0.003],
            //    [-0.015, 0.003, 1.044],
            //]);
            //let mag_corrected = (mag_mat + antibias / 100.0);
            //write!(
            //    Wrapper::new(&mut msg),
            //    "Uni:{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}\r\n",
            //    ((acc_sample.ax - abx) * 9.80665),     // as i32,
            //    ((acc_sample.ay - aby) * 9.80665),     // as i32,
            //    ((acc_sample.az - abz) * 9.80665),     // as i32,
            //    ((rate_sample.gx - gbx) * PI / 180.0), // as i32,
            //    ((rate_sample.gy - gby) * PI / 180.0), // as i32,
            //    ((rate_sample.gz - gbz) * PI / 180.0), // as i32,
            //    (mag_corrected[(0, 0)] * 100.0),       // as i32,
            //    (mag_corrected[(1, 0)] * 100.0),       // as i32,
            //    (mag_corrected[(2, 0)] * 100.0),       // as i32
            //)
            //.unwrap();
            //rprintln!(
            //    "{}, {}, {}, {}",
            //    mag_corrected[(0, 0)],
            //    mag_corrected[(1, 0)],
            //    mag_corrected[(2, 0)],
            //    libm::sqrtf(
            //        libm::powf(mag_corrected[(0, 0)], 2.0)
            //            + libm::powf(mag_corrected[(1, 0)], 2.0)
            //            + libm::powf(mag_corrected[(2, 0)], 2.0)
            //    )
            //);
            //usart::write_message(&msg).await.unwrap();
            //let bmp_sample = bmp.pressure_temperature().await.unwrap();
            let mut Q = Matrix::identity() * libm::powf(3.0*PI/2.0, 2.0);
            Q[(3, 3)] = libm::powf(6.0*PI/3.0, 2.0);
            Q[(4, 4)] = libm::powf(6.0*PI/3.0, 2.0);
            Q[(5, 5)] = libm::powf(6.0*PI/3.0, 2.0);
            let prediction = filter.predict(
                //Matrix6::<f32>::new(
                //    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                //    0., 1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 1.,
                //)*0.9,
                Q*0.5,
                (get_ticks() - last_ticks) as f32 / 1000.0,
            );

            let mut ax = acc_sample.ax - abx;
            let mut ay = acc_sample.ay - aby;
            let mut az = acc_sample.az - abz;
            let norm_a = sqrtf(powf(ax, 2.0) + powf(ay, 2.0) + powf(az, 2.0));
            ax /= norm_a;
            ay /= norm_a;
            az /= norm_a;
            //rprintln!("{}, {}, {}", ax, ay, az);
            let fake_pose = Quaternion::new(0.0, 1.0, 0.0, 0.0);
            let fake_mag = fake_pose * Quaternion::new(0.0, 1.0, 0.0, 0.0) * fake_pose.conj();
            let estimate = filter.update(
                ukf2::Observation::new(
                    ax,
                    ay,
                    az,
                    (rate_sample.gx - gbx) * PI / 180.0,
                    (rate_sample.gy - gby) * PI / 180.0,
                    (rate_sample.gz - gbz) * PI / 180.0,
                ),
                Matrix::identity() * libm::powf(0.01, 2.0),
            );
            //
            ////// Construct a quaternion rotating from the Earth
            ////// frame to the measured frame.
            ////let gyro_quat = {
            ////    let dt: f32 = (get_ticks() - last_ticks) as f32 / 1000.;
            ////    Quaternion::new(
            ////        1.,
            ////        rate_sample.gx / 180. * PI * dt / 2.,
            ////        rate_sample.gy / 180. * PI * dt / 2.,
            ////        -rate_sample.gz / 180. * PI * dt / 2.,
            ////    )
            ////    .normalized()
            ////        * last_quat
            ////};
            ////let acc_quat = {
            ////    use libm::{powf, sqrtf};
            ////    // Because the expected gravity vector is [0 0 1] in our coordinate system
            ////    // The dot product of the gravity vector and the observation is the measured
            ////    // z value.
            ////    // The sensor z is inverted WRT our coordinate system.
            ////    let dot = acc_sample.az
            ////        / sqrtf(
            ////            powf(acc_sample.ax, 2.) + powf(acc_sample.ay, 2.) + powf(acc_sample.az, 2.),
            ////        );
            ////    let cos_ht = sqrtf((1. + dot) / 2.);
            ////    let sin_ht = sqrtf((1. - dot) / 2.);
            ////    Quaternion::new(
            ////        cos_ht,
            ////        acc_sample.ax * sin_ht,
            ////        acc_sample.ay * sin_ht,
            ////        acc_sample.az * sin_ht,
            ////    )
            ////};
            ////last_quat = (acc_quat.scale(0.05) + gyro_quat.scale(0.95)).normalized();
            let mut msg = [0 as u8; 70];
            ////rprintln!("{:?}", last_quat);
            write!(
                Wrapper::new(&mut msg),
                "w{}wa{}ab{}bc{}c\r\n",
                estimate.pose.w(),
                estimate.pose.x(),
                estimate.pose.y(),
                estimate.pose.z()
            )
            .unwrap();
            usart::write_message(&msg).await.unwrap();
            ////rprintln!(
            ////    "(ax, ay, az): ({:.2}, {:.2}, {:.2}), (gx, gy, gz): ({:.2}, {:.2}, {:.2}), (p, t): ({:.2}, {:.2})",
            ////    acc_sample.ax - abx,
            ////    acc_sample.ay - aby,
            ////    acc_sample.az - abz,
            ////    rate_sample.gx - gbx,
            ////    rate_sample.gy - gby,
            ////    rate_sample.gz - gbz,
            ////    bmp_sample.press,
            ////    bmp_sample.temp
            ////);
            last_ticks = get_ticks();
        }
    }
}
