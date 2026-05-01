use crate::{shutdown, state::AppState};
use anyhow::Context;
use reqwest::header::{HeaderMap, HeaderValue, HOST, USER_AGENT};
use serde::Serialize;
use std::{sync::Arc, time::Duration};

const UPDATE_INTERVAL: Duration = Duration::from_secs(60);
const BACKOFF_BASE: Duration = Duration::from_secs(30);
const BACKOFF_MAX: Duration = Duration::from_secs(60 * 60);

#[derive(Debug, Clone, Serialize)]
struct SdrListUpdate {
    id: String,
    name: String,
    antenna: String,
    bandwidth: i64,
    users: usize,
    center_frequency: i64,
    grid_locator: String,
    hostname: String,
    max_users: usize,
    port: u16,
    software: String,
    backend: String,
    version: String,
    receiver_count: usize,
    receiver_id: String,
    range_start_hz: i64,
    range_end_hz: i64,
}

pub fn spawn(state: Arc<AppState>) {
    if !state.cfg.websdr.register_online {
        tracing::info!("SDR list registration disabled (set websdr.register_online=true)");
        return;
    }

    let url = state.cfg.websdr.register_url.clone();
    tracing::info!(%url, "SDR list registration enabled");

    tokio::spawn(async move {
        let id = rand::random::<u32>().to_string();
        let client = match build_client(&url) {
            Ok(c) => c,
            Err(e) => {
                tracing::error!(error = ?e, "SDR list registration client init failed");
                return;
            }
        };

        let mut attempt: u32 = 0;
        while !shutdown::is_shutdown_requested() {
            let payloads = build_payloads(&state, &id);
            match send_all_updates(&client, &url, &payloads).await {
                Ok(()) => {
                    attempt = 0;
                    tokio::time::sleep(UPDATE_INTERVAL).await;
                }
                Err(e) => {
                    attempt = attempt.saturating_add(1);
                    let backoff = compute_backoff(attempt);
                    tracing::warn!(
                        error = ?e,
                        attempt,
                        backoff_secs = backoff.as_secs(),
                        "SDR list registration failed"
                    );
                    tokio::time::sleep(backoff).await;
                }
            }
        }
    });
}

fn build_payloads(state: &AppState, id: &str) -> Vec<SdrListUpdate> {
    let cfg = &state.cfg;
    let receiver_count = state
        .receivers
        .values()
        .filter(|rx| rx.receiver.enabled)
        .count()
        .max(1);

    let mut enabled_receivers = state
        .receivers
        .values()
        .filter(|rx| rx.receiver.enabled)
        .collect::<Vec<_>>();
    enabled_receivers.sort_by(|a, b| a.receiver.id.cmp(&b.receiver.id));

    if enabled_receivers.is_empty() {
        enabled_receivers.push(state.active_receiver_state());
    }

    enabled_receivers
        .into_iter()
        .map(|receiver| {
            let rt = receiver.rt.as_ref();
            let range_start_hz = rt.basefreq;
            let range_end_hz = rt.basefreq.saturating_add(rt.total_bandwidth);
            let bandwidth = range_end_hz.saturating_sub(range_start_hz);
            let center_frequency = range_start_hz.saturating_add(bandwidth / 2);

            SdrListUpdate {
                id: id.to_string(),
                name: cfg.websdr.name.clone(),
                antenna: cfg.websdr.antenna.clone(),
                bandwidth,
                users: receiver.audio_clients.len(),
                center_frequency,
                grid_locator: cfg.websdr.grid_locator.clone(),
                hostname: cfg.websdr.hostname.clone(),
                max_users: cfg.limits.audio,
                port: cfg.websdr.public_port.unwrap_or(cfg.server.port),
                software: "NovaSDR".to_string(),
                backend: "novasdr-server".to_string(),
                version: env!("CARGO_PKG_VERSION").to_string(),
                receiver_count,
                receiver_id: receiver.receiver.id.clone(),
                range_start_hz,
                range_end_hz,
            }
        })
        .collect()
}

fn build_client(url: &str) -> anyhow::Result<reqwest::Client> {
    let mut headers = HeaderMap::new();
    headers.insert(
        USER_AGENT,
        HeaderValue::from_static("NovaSDR/registration (+https://github.com/Brandon4466/NovaSDR)"),
    );

    if let Ok(parsed) = reqwest::Url::parse(url) {
        if let Some(host) = parsed.host_str() {
            if host == "sdr-list.xyz" {
                headers.insert(HOST, HeaderValue::from_static("sdr-list.xyz"));
            }
        }
    }

    reqwest::Client::builder()
        .default_headers(headers)
        .timeout(Duration::from_secs(10))
        .build()
        .context("build reqwest client")
}

async fn send_update(
    client: &reqwest::Client,
    url: &str,
    payload: &SdrListUpdate,
) -> anyhow::Result<()> {
    let res = client
        .post(url)
        .json(payload)
        .send()
        .await
        .context("POST update_websdr")?;

    let status = res.status();
    if !status.is_success() {
        let body = res
            .text()
            .await
            .unwrap_or_else(|e| format!("<failed to read response body: {e}>"));
        anyhow::bail!("HTTP {status}: {body}");
    }
    Ok(())
}

async fn send_all_updates(
    client: &reqwest::Client,
    url: &str,
    payloads: &[SdrListUpdate],
) -> anyhow::Result<()> {
    for payload in payloads {
        send_update(client, url, payload).await?;
    }
    Ok(())
}

fn compute_backoff(attempt: u32) -> Duration {
    let shift = attempt.min(16);
    let mul = 1u64 << shift;
    let secs = BACKOFF_BASE.as_secs().saturating_mul(mul);
    Duration::from_secs(secs.min(BACKOFF_MAX.as_secs()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn backoff_is_monotonic_and_capped() {
        let mut last = Duration::from_secs(0);
        for attempt in 1..64 {
            let d = compute_backoff(attempt);
            assert!(d >= last, "attempt {attempt}: {d:?} < {last:?}");
            assert!(
                d <= BACKOFF_MAX,
                "attempt {attempt}: {d:?} > {BACKOFF_MAX:?}"
            );
            last = d;
        }
    }
}
