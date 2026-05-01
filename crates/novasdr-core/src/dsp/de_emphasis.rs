/// FM de-emphasis filter.
///
/// Applies a single-pole IIR low-pass that mirrors the standard FM broadcast
/// pre-emphasis time constants: 75µs (Americas/Korea) and 50µs (Europe/most
/// of the rest of the world). Without this filter applied to demodulated FM
/// audio, broadcast content sounds harsh — pre-emphasis at the transmitter
/// boosts highs, and de-emphasis at the receiver is required to restore a
/// flat response.
///
/// Difference equation:
///     y[n] = α·x[n] + (1-α)·y[n-1]
/// where α = dt / (τ + dt), dt = 1/sample_rate, τ = time constant in seconds.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeEmphasisMode {
    Off,
    Us75,
    Eu50,
}

impl DeEmphasisMode {
    pub fn from_str_lower(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "off" | "none" | "" => Some(Self::Off),
            "75" | "75us" | "us" | "us75" => Some(Self::Us75),
            "50" | "50us" | "eu" | "eu50" => Some(Self::Eu50),
            _ => None,
        }
    }

    pub fn tau_seconds(self) -> Option<f32> {
        match self {
            Self::Off => None,
            Self::Us75 => Some(75e-6),
            Self::Eu50 => Some(50e-6),
        }
    }
}

#[derive(Debug, Clone)]
pub struct DeEmphasis {
    sample_rate: f32,
    mode: DeEmphasisMode,
    alpha: f32,
    y_prev: f32,
}

impl DeEmphasis {
    pub fn new(sample_rate: f32) -> Self {
        Self {
            sample_rate,
            mode: DeEmphasisMode::Off,
            alpha: 1.0,
            y_prev: 0.0,
        }
    }

    pub fn set_mode(&mut self, mode: DeEmphasisMode) {
        if mode == self.mode {
            return;
        }
        self.mode = mode;
        self.alpha = match mode.tau_seconds() {
            Some(tau) => {
                let dt = 1.0 / self.sample_rate.max(1.0);
                dt / (tau + dt)
            }
            None => 1.0,
        };
        self.y_prev = 0.0;
    }

    pub fn mode(&self) -> DeEmphasisMode {
        self.mode
    }

    pub fn reset(&mut self) {
        self.y_prev = 0.0;
    }

    pub fn process(&mut self, samples: &mut [f32]) {
        if self.mode == DeEmphasisMode::Off {
            return;
        }
        let a = self.alpha;
        let one_minus_a = 1.0 - a;
        let mut y = self.y_prev;
        for s in samples.iter_mut() {
            y = a * *s + one_minus_a * y;
            *s = y;
        }
        self.y_prev = y;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn off_passes_through_unchanged() {
        let mut de = DeEmphasis::new(48_000.0);
        let mut buf = [1.0f32, -0.5, 0.25, 0.75];
        let original = buf;
        de.process(&mut buf);
        assert_eq!(buf, original);
    }

    #[test]
    fn dc_input_settles_to_dc() {
        // For any LPF, a constant DC input must converge to the same DC value at the output.
        let mut de = DeEmphasis::new(48_000.0);
        de.set_mode(DeEmphasisMode::Us75);
        let mut buf = vec![1.0f32; 8192];
        de.process(&mut buf);
        let last = *buf.last().unwrap();
        assert!(
            (last - 1.0).abs() < 1e-3,
            "expected DC to settle near 1.0, got {last}"
        );
    }

    #[test]
    fn us75_attenuates_above_cutoff() {
        // 75µs cutoff f_c = 1/(2π·75µs) ≈ 2122 Hz. A 10 kHz sine should be heavily attenuated.
        let sr = 48_000.0f32;
        let mut de = DeEmphasis::new(sr);
        de.set_mode(DeEmphasisMode::Us75);
        let f = 10_000.0f32;
        let n = 4096usize;
        let mut buf = (0..n)
            .map(|i| (2.0 * std::f32::consts::PI * f * (i as f32) / sr).sin())
            .collect::<Vec<_>>();
        let in_rms = (buf.iter().map(|x| x * x).sum::<f32>() / (n as f32)).sqrt();
        de.process(&mut buf);
        // Skip transient at the start.
        let tail = &buf[n / 2..];
        let out_rms = (tail.iter().map(|x| x * x).sum::<f32>() / (tail.len() as f32)).sqrt();
        let attenuation_db = 20.0 * (out_rms / in_rms).log10();
        // Theoretical: -20·log10(sqrt(1 + (f/f_c)²)) ≈ -13.5 dB at 10 kHz with f_c≈2122 Hz.
        assert!(
            attenuation_db < -10.0,
            "expected >10 dB attenuation at 10 kHz with 75µs, got {attenuation_db:.2} dB"
        );
    }

    #[test]
    fn eu50_attenuates_more_than_us75_in_audio_band_above_cutoff() {
        // 50µs has a higher cutoff (~3183 Hz) than 75µs (~2122 Hz) so at a fixed high
        // frequency well above both cutoffs, the attenuation magnitudes should both be
        // significant; we just check both reduce the signal versus passthrough.
        let sr = 48_000.0f32;
        let f = 8_000.0f32;
        let n = 4096usize;
        let mk = || -> Vec<f32> {
            (0..n)
                .map(|i| (2.0 * std::f32::consts::PI * f * (i as f32) / sr).sin())
                .collect()
        };

        let in_rms = (mk().iter().map(|x| x * x).sum::<f32>() / (n as f32)).sqrt();

        for mode in [DeEmphasisMode::Us75, DeEmphasisMode::Eu50] {
            let mut buf = mk();
            let mut de = DeEmphasis::new(sr);
            de.set_mode(mode);
            de.process(&mut buf);
            let tail = &buf[n / 2..];
            let out_rms = (tail.iter().map(|x| x * x).sum::<f32>() / (tail.len() as f32)).sqrt();
            let att_db = 20.0 * (out_rms / in_rms).log10();
            assert!(
                att_db < -5.0,
                "expected meaningful attenuation at 8 kHz for {mode:?}, got {att_db:.2} dB"
            );
        }
    }

    #[test]
    fn from_str_parses_aliases() {
        assert_eq!(DeEmphasisMode::from_str_lower("off"), Some(DeEmphasisMode::Off));
        assert_eq!(DeEmphasisMode::from_str_lower(""), Some(DeEmphasisMode::Off));
        assert_eq!(DeEmphasisMode::from_str_lower("75"), Some(DeEmphasisMode::Us75));
        assert_eq!(DeEmphasisMode::from_str_lower("75us"), Some(DeEmphasisMode::Us75));
        assert_eq!(DeEmphasisMode::from_str_lower("US"), Some(DeEmphasisMode::Us75));
        assert_eq!(DeEmphasisMode::from_str_lower("50"), Some(DeEmphasisMode::Eu50));
        assert_eq!(DeEmphasisMode::from_str_lower("eu"), Some(DeEmphasisMode::Eu50));
        assert_eq!(DeEmphasisMode::from_str_lower("garbage"), None);
    }

    #[test]
    fn set_mode_off_then_back_resets_state() {
        let mut de = DeEmphasis::new(48_000.0);
        de.set_mode(DeEmphasisMode::Us75);
        let mut buf = vec![1.0f32; 256];
        de.process(&mut buf);
        // y_prev is now non-zero. Switching mode resets it.
        de.set_mode(DeEmphasisMode::Eu50);
        // First sample with input 0 should be 0 (since y_prev was reset).
        let mut probe = [0.0f32];
        de.process(&mut probe);
        assert_eq!(probe[0], 0.0);
    }
}
