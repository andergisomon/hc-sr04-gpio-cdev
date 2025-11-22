use gpio_cdev::{Chip, EventRequestFlags, EventType, Line, LineHandle, LineRequestFlags};
use std::{thread::sleep, time::*};
use libc::*;
use std::os::unix::io::AsRawFd;

const DEFAULT_TIMEOUT_MICROSECS: u64 = 8746;

#[derive(Debug)]
pub enum HcSr04Error {
    Io,
    Init,
}

pub enum DistanceUnit {
    Mm(f64),
    Cm(f64),
    Meter(f64),
}
impl DistanceUnit {
    pub fn write_val(&mut self, new_val: f64) {
        match self {
            DistanceUnit::Mm(val) => *val = new_val,
            DistanceUnit::Cm(val) => *val = new_val,
            DistanceUnit::Meter(val) => *val = new_val,
        }
    }

    pub fn to_val(&self) -> f64 {
        match self {
            DistanceUnit::Mm(val) => *val,
            DistanceUnit::Cm(val) => *val,
            DistanceUnit::Meter(val) => *val,
        }
    }
}

pub enum VelocityUnit {
    MetersPerSecs(f64),
    CentimeterPerSecs(f64),
}
impl VelocityUnit {
    pub fn to_val(&self) -> f64 {
        match self {
            VelocityUnit::MetersPerSecs(val) => *val,
            VelocityUnit::CentimeterPerSecs(val) => *val,
        }
    }
}

const SPEED_OF_SOUND: VelocityUnit = VelocityUnit::MetersPerSecs(343.0);

pub struct HcSr04 {
    trig: LineHandle,
    echo: Line,
}

fn poll_with_timeout(fd: i32, timeout: Duration) -> Result<bool, HcSr04Error> {
    let mut pollfd = libc::pollfd {
        fd,
        events: libc::POLLIN | libc::POLLPRI,
        revents: 0,
    };

    let timeout_ms = timeout.as_millis().min(i32::MAX as u128) as i32;

    unsafe {
        match libc::poll(&mut pollfd, 1, timeout_ms) {
            -1 => Err(HcSr04Error::Io),
            0 => Ok(false),  // Timeout
            _ => Ok(true),   // Event available
        }
    }
}

/// YMMV
pub fn range_to_timeout(range: DistanceUnit) -> Result<Duration, String> {
    let res = match range {
        DistanceUnit::Meter(val) => (val / 2.0) / SPEED_OF_SOUND.to_val(),
        DistanceUnit::Cm(val) => (val / 200.0) / SPEED_OF_SOUND.to_val(),
        DistanceUnit::Mm(_) => return Err("range must be in m or cm".to_string())
    };
    Ok(Duration::from_secs_f64(res))
}

impl HcSr04 {
    pub fn new(trig: u32, echo: u32) -> Result<Self, HcSr04Error> {
        let mut chip = Chip::new("/dev/gpiochip4");

        let mut chip = match chip.ok() {
            Some(chip) => chip,
            None => return Err(HcSr04Error::Init)
        };

        let trig_line = match chip.get_line(trig).ok() {
            Some(line) => line,
            None => return Err(HcSr04Error::Init)
        };

        let echo_line = match chip.get_line(echo).ok() {
            Some(line) => line,
            None => return Err(HcSr04Error::Init)
        };

        let trig_handle = match trig_line.request(LineRequestFlags::OUTPUT, 0, "hc-sr04-trigger").ok() {
            Some(pin) => pin,
            None => return Err(HcSr04Error::Init)
        };

        Ok(Self {
            trig: trig_handle,
            echo: echo_line
        })
    }

    /// Returns distance in cm by default.
    fn dist(&mut self, timeout: Option<Duration>) -> Result<f64, HcSr04Error> {
        match self.trig.set_value(0).ok() {
            Some(_) => (),
            None => return Err(HcSr04Error::Io)
        }

        sleep(Duration::from_micros(2));

        match self.trig.set_value(1).ok() {
            Some(_) => (),
            None => return Err(HcSr04Error::Io)
        }

        sleep(Duration::from_micros(10));

        match self.trig.set_value(0).ok() {
            Some(_) => (),
            None => return Err(HcSr04Error::Io)
        }

        let mut dist: DistanceUnit = DistanceUnit::Cm(0.0);
        let start_time = Instant::now();
        let mut tx_time = Instant::now();

        let events_req = self.echo.events(
            LineRequestFlags::INPUT,
            EventRequestFlags::BOTH_EDGES,
            "hc-sr04-echo");

        let mut events = match events_req.ok() {
            Some(events) => events,
            None => return Err(HcSr04Error::Io)
        };
        let fd = events.as_raw_fd();

        let effective_timeout = match timeout {
            Some(val) => 2 * val,
            None => Duration::from_micros(DEFAULT_TIMEOUT_MICROSECS)
        };

        if !poll_with_timeout(fd, effective_timeout).unwrap_or_else(|_| -> bool {false}) {
            return Err(HcSr04Error::Io)
        }
        if let Some(Ok(event)) = events.next() {
            if event.event_type() == EventType::RisingEdge {
                tx_time = Instant::now();
            }
        }

        let remaining = effective_timeout.saturating_sub(start_time.elapsed());
        if !poll_with_timeout(fd, remaining).unwrap_or_else(|_| -> bool {false}) {
            return Err(HcSr04Error::Io)
        }
        if let Some(Ok(event)) = events.next() {
            if event.event_type() == EventType::FallingEdge {
                let tof: Duration = Instant::now() - tx_time;
                dist.write_val(50.0*(SPEED_OF_SOUND.to_val() * tof.as_secs_f64()));

            }
        }
        Ok(dist.to_val())
    }

    /// Returns distance in m. Leaving `timeout` as `None` will give a default timeout of 5.831ms.
    pub fn dist_meter(&mut self, timeout: Option<Duration>) -> Result<DistanceUnit, HcSr04Error> {
        let res = self.dist(timeout)?;
        Ok(DistanceUnit::Meter(res/100.0))
    }

    /// Returns distance in cm. Leaving `timeout` as `None` will give a default timeout of 5.831ms.
    pub fn dist_cm(&mut self, timeout: Option<Duration>) -> Result<DistanceUnit, HcSr04Error> {
        let res = self.dist(timeout)?;
        Ok(DistanceUnit::Cm(res))
    }

    /// Returns distance in mm. Leaving `timeout` as `None` will give a default timeout of 5.831ms.
    pub fn dist_mm(&mut self, timeout: Option<Duration>) -> Result<DistanceUnit, HcSr04Error> {
        let res = self.dist(timeout)?;
        Ok(DistanceUnit::Mm(10.0*res))
    }
}
