#[cfg(feature = "soapysdr")]
mod soapysdr;

use crate::state::Retuner;
use novasdr_core::config::{InputDriver, ReceiverConfig};
use std::io::Read;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

pub struct OpenedInput {
    pub reader: Box<dyn Read + Send>,
    pub retuner: Option<Box<dyn Retuner>>,
    pub driver_name: &'static str,
}

pub fn open(
    receiver: &ReceiverConfig,
    stop_requested: Arc<AtomicBool>,
    soapy_semaphore: Arc<Mutex<()>>,
) -> anyhow::Result<OpenedInput> {
    let driver_name = receiver.input.driver.as_str();
    match &receiver.input.driver {
        InputDriver::Stdin { .. } => Ok(OpenedInput {
            reader: Box::new(std::io::stdin()),
            retuner: None,
            driver_name,
        }),
        InputDriver::Fifo {
            format: _format,
            path,
        } => Ok(OpenedInput {
            reader: Box::new(
                std::fs::File::open(path)
                    .map_err(|e| anyhow::anyhow!("Error open file '{path}': {e}"))?,
            ),
            retuner: None,
            driver_name,
        }),
        InputDriver::SoapySdr(driver) => {
            #[cfg(feature = "soapysdr")]
            {
                let (reader, retuner) =
                    soapysdr::open(driver, &receiver.input, stop_requested, soapy_semaphore)?;
                Ok(OpenedInput {
                    reader,
                    retuner: Some(retuner),
                    driver_name,
                })
            }

            #[cfg(not(feature = "soapysdr"))]
            {
                let _ = (driver, stop_requested, soapy_semaphore);
                anyhow::bail!(
                    "SoapySDR input support is disabled (rebuild with Cargo feature \"soapysdr\")"
                )
            }
        }
    }
}
