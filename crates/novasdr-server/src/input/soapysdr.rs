use crate::state::Retuner;
use anyhow::Context;
use novasdr_core::config::{ReceiverInput, SampleFormat, SignalType, SoapySdrDriver};
use soapysdr::StreamSample;
use std::io::Read;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

/// Retuner backed by a cloned SoapySDR `Device`. SoapySDR `Device` is internally refcounted and
/// thread-safe (the C library guarantees `setFrequency` is safe to call concurrently with stream
/// reads on supported drivers), so we can keep a separate handle here for runtime LO updates.
pub struct SoapyRetuner {
    device: soapysdr::Device,
    channel: usize,
}

impl Retuner for SoapyRetuner {
    fn set_frequency_hz(&self, hz: i64) -> anyhow::Result<i64> {
        self.device
            .set_frequency(soapysdr::Direction::Rx, self.channel, hz as f64, ())
            .with_context(|| format!("retune SoapySDR to {hz} Hz"))?;
        let actual = self
            .device
            .frequency(soapysdr::Direction::Rx, self.channel)
            .context("read back SoapySDR frequency after retune")?;
        Ok(actual.round() as i64)
    }

    fn frequency_range_hz(&self) -> Option<(i64, i64)> {
        let ranges = self
            .device
            .frequency_range(soapysdr::Direction::Rx, self.channel)
            .ok()?;
        let mut lo = i64::MAX;
        let mut hi = i64::MIN;
        for r in ranges {
            // SoapySDRRange has minimum/maximum/step (all f64). Step is informational here; we
            // floor/ceil the bounds so quantization can't push us outside the union.
            let r_min = r.minimum.floor() as i64;
            let r_max = r.maximum.ceil() as i64;
            if r_min < lo {
                lo = r_min;
            }
            if r_max > hi {
                hi = r_max;
            }
        }
        if lo > hi {
            None
        } else {
            Some((lo, hi))
        }
    }
}

fn to_stream_args(driver: &SoapySdrDriver) -> anyhow::Result<soapysdr::Args> {
    let mut args = soapysdr::Args::new();
    for (key, value) in driver.stream_args.iter() {
        anyhow::ensure!(
            !key.contains('\0'),
            "soapysdr stream arg key must not contain NUL"
        );
        anyhow::ensure!(
            !value.contains('\0'),
            "soapysdr stream arg value for {key:?} must not contain NUL"
        );
        args.set(key.as_str(), value.as_str());
    }
    Ok(args)
}

pub fn open(
    driver: &SoapySdrDriver,
    input: &ReceiverInput,
    stop_requested: Arc<AtomicBool>,
    soapy_semaphore: Arc<Mutex<()>>,
) -> anyhow::Result<(Box<dyn Read + Send>, Box<dyn Retuner>)> {
    anyhow::ensure!(
        input.signal == SignalType::Iq,
        "soapysdr input currently requires receiver.input.signal = \"iq\""
    );

    // This lock is used to create one SoapySdr device at a time.
    // Otherwise, multiple soapy devices are created in parallel using multiple threads.
    // This leads to the appearance of errors that are difficult to reproduce and debug.
    let _guard = soapy_semaphore.lock();

    match driver.format {
        SampleFormat::Cs16 => open_fmt::<num_complex::Complex<i16>>(driver, input, stop_requested),
        SampleFormat::Cf32 => open_fmt::<num_complex::Complex<f32>>(driver, input, stop_requested),
        other => anyhow::bail!(
            "soapysdr input only supports format \"cs16\" or \"cf32\" (got {other:?})"
        ),
    }
}

fn apply_gain_and_settings(
    driver: &SoapySdrDriver,
    device: &soapysdr::Device,
) -> anyhow::Result<()> {
    let direction = soapysdr::Direction::Rx;
    let channel = driver.channel;

    if let Some(automatic) = driver.agc {
        let has = device
            .has_gain_mode(direction, channel)
            .context("query SoapySDR gain mode support")?;
        anyhow::ensure!(
            has,
            "soapysdr receiver.input.driver.agc was set, but the device does not support AGC"
        );
        device
            .set_gain_mode(direction, channel, automatic)
            .context("set SoapySDR AGC mode")?;
    }

    if let Some(gain) = driver.gain {
        device
            .set_gain(direction, channel, gain)
            .context("set SoapySDR gain")?;
    }

    if !driver.gains.is_empty() {
        let available = device
            .list_gains(direction, channel)
            .context("list SoapySDR gain elements")?;

        for (name, gain) in driver.gains.iter() {
            anyhow::ensure!(
                !name.contains('\0'),
                "soapysdr gain element name must not contain NUL"
            );
            anyhow::ensure!(
                available.iter().any(|n| n == name),
                "unknown soapysdr gain element {name:?} (available: {available:?})"
            );
            device
                .set_gain_element(direction, channel, name.as_str(), *gain)
                .with_context(|| format!("set SoapySDR gain element {name:?}"))?;
        }
    }

    for (key, value) in driver.settings.iter() {
        anyhow::ensure!(
            !key.contains('\0'),
            "soapysdr setting key must not contain NUL"
        );
        anyhow::ensure!(
            !value.contains('\0'),
            "soapysdr setting value for {key:?} must not contain NUL"
        );
        device
            .write_setting(key.as_str(), value.as_str())
            .with_context(|| format!("write SoapySDR setting {key:?}"))?;
    }

    Ok(())
}

fn open_fmt<E>(
    driver: &SoapySdrDriver,
    input: &ReceiverInput,
    stop_requested: Arc<AtomicBool>,
) -> anyhow::Result<(Box<dyn Read + Send>, Box<dyn Retuner>)>
where
    E: StreamSample + Copy + Default + Send + 'static,
{
    let device = soapysdr::Device::new(driver.device.as_str()).context("open SoapySDR device")?;

    if let Some(ant) = driver.antenna.as_deref() {
        device
            .set_antenna(soapysdr::Direction::Rx, driver.channel, ant)
            .context("set SoapySDR RX antenna")?;
    }

    device
        .set_sample_rate(soapysdr::Direction::Rx, driver.channel, input.sps as f64)
        .context("set SoapySDR sample rate")?;
    device
        .set_frequency(
            soapysdr::Direction::Rx,
            driver.channel,
            input.frequency as f64,
            (),
        )
        .context("set SoapySDR frequency")?;

    apply_gain_and_settings(driver, &device)?;

    let stream_args = to_stream_args(driver).context("build SoapySDR stream args")?;
    let mut stream = device
        .rx_stream_args::<E, _>(&[driver.channel], stream_args)
        .context("create SoapySDR RX stream")?;
    stream
        .activate(None)
        .context("activate SoapySDR RX stream")?;

    // Clone the device handle for the retune side-channel. SoapySDR `Device` is internally
    // refcounted, so this is cheap and the handle stays valid as long as either the stream or the
    // retuner holds it.
    let retuner: Box<dyn Retuner> = Box::new(SoapyRetuner {
        device: device.clone(),
        channel: driver.channel,
    });

    // Use a reasonable internal buffer size (16K complex samples).
    // SoapySDR will fill what it can per read; we accumulate until the caller is satisfied.
    let reader: Box<dyn Read + Send> = Box::new(SoapyRead::new(
        stream,
        driver.rx_buffer_samples,
        stop_requested,
    ));
    Ok((reader, retuner))
}

/// Adapter that turns a SoapySDR RxStream into a blocking `Read` byte-stream,
/// matching the behavior of stdin/pipe: blocks until data is available, never
/// returns 0 (which would signal EOF to `read_exact`).
struct SoapyRead<T: soapysdr::StreamSample> {
    stream: soapysdr::RxStream<T>,
    stop_requested: Arc<AtomicBool>,
    /// Internal sample buffer; we read from SoapySDR into this, then serve bytes to callers.
    buf: Vec<T>,
    /// Current read position in `buf`, measured in bytes.
    read_pos: usize,
    /// Valid data length in `buf`, measured in bytes.
    data_len: usize,
}

impl<T: soapysdr::StreamSample + Copy + Default> SoapyRead<T> {
    fn new(
        stream: soapysdr::RxStream<T>,
        buf_samples: usize,
        stop_requested: Arc<AtomicBool>,
    ) -> Self {
        Self {
            stream,
            stop_requested,
            buf: vec![T::default(); buf_samples.max(1024)],
            read_pos: 0,
            data_len: 0,
        }
    }

    /// Refill internal buffer from SoapySDR. Blocks until at least one sample is available.
    fn refill(&mut self) -> std::io::Result<()> {
        static WARNED_OVERFLOW: AtomicBool = AtomicBool::new(false);
        static WARNED_CORRUPTION: AtomicBool = AtomicBool::new(false);
        static WARNED_TIME_ERROR: AtomicBool = AtomicBool::new(false);

        loop {
            if self.stop_requested.load(Ordering::Relaxed)
                || crate::shutdown::is_shutdown_requested()
            {
                return Err(std::io::Error::new(std::io::ErrorKind::Other, "shutdown"));
            }
            let mut bufs = [self.buf.as_mut_slice()];
            // Long timeout (1 second) to avoid busy-spinning; SoapySDR returns early when data arrives.
            match self.stream.read(&mut bufs, 1_000_000) {
                Ok(n) if n > 0 => {
                    self.data_len = (n as usize) * std::mem::size_of::<T>();
                    self.read_pos = 0;
                    return Ok(());
                }
                Ok(_) => {
                    // Timeout / no samples yet. Brief sleep to avoid busy-loop, then retry.
                    std::thread::sleep(std::time::Duration::from_micros(500));
                }
                Err(e) => match e.code {
                    soapysdr::ErrorCode::Timeout => {
                        std::thread::sleep(std::time::Duration::from_micros(500));
                    }
                    soapysdr::ErrorCode::Overflow => {
                        if !WARNED_OVERFLOW.swap(true, Ordering::Relaxed) {
                            tracing::warn!(error = ?e, "SoapySDR RX overflow (samples dropped)");
                        }
                    }
                    soapysdr::ErrorCode::Corruption => {
                        if !WARNED_CORRUPTION.swap(true, Ordering::Relaxed) {
                            tracing::warn!(error = ?e, "SoapySDR RX corruption (samples dropped)");
                        }
                    }
                    soapysdr::ErrorCode::TimeError => {
                        if !WARNED_TIME_ERROR.swap(true, Ordering::Relaxed) {
                            tracing::warn!(error = ?e, "SoapySDR RX time error (samples dropped)");
                        }
                    }
                    _ => {
                        return Err(std::io::Error::new(
                            std::io::ErrorKind::Other,
                            format!("soapysdr read: {e:?}"),
                        ));
                    }
                },
            }
        }
    }
}

impl<T: soapysdr::StreamSample + Copy + Default> Read for SoapyRead<T> {
    fn read(&mut self, out: &mut [u8]) -> std::io::Result<usize> {
        if out.is_empty() {
            return Ok(0);
        }

        // Refill internal buffer if exhausted.
        if self.read_pos >= self.data_len {
            self.refill()?;
        }

        // Copy as much as we can from internal buffer to caller's buffer.
        let available = self.data_len - self.read_pos;
        let to_copy = available.min(out.len());

        // Safety: `buf` is a contiguous Vec<T>, we're reading `to_copy` bytes starting at `read_pos`.
        let src = unsafe {
            std::slice::from_raw_parts((self.buf.as_ptr() as *const u8).add(self.read_pos), to_copy)
        };
        out[..to_copy].copy_from_slice(src);
        self.read_pos += to_copy;

        Ok(to_copy)
    }
}
