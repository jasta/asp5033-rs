use anyhow::anyhow;
use asp5033::asp5033::Asp5033;
use embedded_hal::i2c::{AddressMode, ErrorKind, ErrorType, Operation, SevenBitAddress};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_mock::eh1::delay::StdSleep;
use env_logger::Env;
use futures::channel::mpsc::Receiver as AsyncReceiver;
use futures::channel::mpsc::Sender as AsyncSender;
use futures::StreamExt;
use log::info;
use log::warn;
use std::fmt::Debug;
use std::iter::zip;
use std::sync::mpsc::Receiver as BlockingReceiver;
use std::sync::mpsc::SyncSender as BlockingSender;
use std::sync::{Arc, Mutex};
use std::thread;
use std::thread::JoinHandle;

#[tokio::main(flavor = "current_thread")]
async fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(Env::default().default_filter_or("info")).init();

    info!("Locating FTDI device...");
    let device = ftdi::find_by_vid_pid(0x0403, 0x6014)
            .interface(ftdi::Interface::A)
            .open()?;

    info!("Initializing I2C...");
    let hal = ftdi_embedded_hal::FtHal::init_freq(device, 400_000)?;
    let i2c = hal.i2c()?;

    info!("Starting async I2C bridge...");
    let i2c_send = SendSafeI2c::new(i2c);
    let i2c_async = AsyncI2cDriverBridge::spawn(i2c_send);

    info!("Loading ASP5033 driver...");
    let sensor = Asp5033::new(i2c_async);

    info!("Running sensor read loop...");
    run_sensor_loop(sensor, StdSleep::new()).await?;

    Ok(())
}

async fn run_sensor_loop<I2C: embedded_hal_async::i2c::I2c>(mut sensor: Asp5033<I2C>, mut timer: impl DelayNs) -> anyhow::Result<()> {
    info!("Performing sensor ID check...");
    sensor.perform_sensor_id_check().await
            .map_err(|e| anyhow!("Failed sensor check: {e:?}"))?;

    info!("Starting measurement loop...");
    loop {
        sensor.request_measurement().await
                .map_err(|e| anyhow!("Failed to request measurement: {e:?}"))?;

        timer.delay_ms(100).await;

        let measurement = sensor.read_measurement().await
                .map_err(|e| anyhow!("Failed to read measurement: {e:?}"))?;

        info!(
            "diff pressure={:.2} Pa, temperature={:.2} C",
            measurement.pressure.as_pascals(),
            measurement.temperature.as_celsius());

        timer.delay_ms(900).await;
    }
}

struct SendSafeI2c<I2C>(Arc<Mutex<I2C>>);
unsafe impl<I2C> Send for SendSafeI2c<I2C> {}

impl<I2C> SendSafeI2c<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self(Arc::new(Mutex::new(i2c)))
    }
}

impl<I2C> ErrorType for SendSafeI2c<I2C>
where
        I2C: embedded_hal::i2c::I2c,
{
    type Error = I2C::Error;
}

impl<I2C> embedded_hal::i2c::I2c for SendSafeI2c<I2C>
where
        I2C: embedded_hal::i2c::I2c,
{
    fn transaction(&mut self, address: SevenBitAddress, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        let mut locked = self.0.lock().unwrap();
        locked.transaction(address, operations)
    }
}

// Embarrassingly inefficient async <-> sync I2cDriver bridge.  esp-idf-hal doesn't yet support an
// async I2cDriver and since this is being used just for demos we don't really care about
// making it scale.
pub struct AsyncI2cDriverBridge<I2C: embedded_hal::i2c::I2c<A>, A: AddressMode> {
    handle: Option<JoinHandle<()>>,
    request_tx: Option<BlockingSender<RequestType<A>>>,
    response_rx: AsyncReceiver<ResponseType<I2C>>,
}

impl<I2C, A> AsyncI2cDriverBridge<I2C, A>
where
        I2C: embedded_hal::i2c::I2c<A> + Send + 'static,
        I2C::Error: Send + 'static,
        A: AddressMode + Send + Copy,
{
    pub fn spawn(driver: I2C) -> Self {
        let (request_tx, request_rx) = std::sync::mpsc::sync_channel(1);
        let (response_tx, response_rx) = futures::channel::mpsc::channel(1);
        let handle = thread::spawn(move || SyncI2cDriverRunner::run_loop(driver, request_rx, response_tx));
        Self {
            handle: Some(handle),
            request_tx: Some(request_tx),
            response_rx,
        }
    }
}

impl<I2C: embedded_hal::i2c::I2c<A>, A: AddressMode> AsyncI2cDriverBridge<I2C, A> {
    pub fn shutdown(mut self) {
        if let Some(Err(e)) = self.shutdown_internal() {
            warn!("Failure shutting down spawned thread: {e:?}");
        }
    }

    fn shutdown_internal(&mut self) -> Option<thread::Result<()>> {
        // Dropping request_tx is what lets the spawned thread exit...
        let _ = self.request_tx.take();

        self.handle.take()
                .map(|handle| handle.join())
    }
}

impl<I2C: embedded_hal::i2c::I2c<A>, A: AddressMode> Drop for AsyncI2cDriverBridge<I2C, A> {
    fn drop(&mut self) {
        self.shutdown_internal();
    }
}

impl<I2C, A> ErrorType for AsyncI2cDriverBridge<I2C, A>
where
        I2C: embedded_hal::i2c::I2c<A>,
        A: AddressMode,
{
    type Error = AsyncBridgeError<I2C::Error>;
}

#[derive(Debug, Clone)]
pub enum AsyncBridgeError<E> {
    InternalError,
    BusError(E),
}

impl<E> embedded_hal::i2c::Error for AsyncBridgeError<E>
where
        E: embedded_hal::i2c::Error,
{
    fn kind(&self) -> ErrorKind {
        match self {
            AsyncBridgeError::InternalError => ErrorKind::Other,
            AsyncBridgeError::BusError(e) => e.kind(),
        }
    }
}

impl<I2C, A> embedded_hal_async::i2c::I2c<A> for AsyncI2cDriverBridge<I2C, A>
where
        I2C: embedded_hal::i2c::I2c<A>,
        A: AddressMode,
{
    async fn read(&mut self, address: A, read: &mut [u8]) -> Result<(), Self::Error> {
        let buffers = self.relay_action(I2cAction {
            address,
            function_call: FunctionCall::Read(vec![0; read.len()]),
        }).await?;
        read.copy_from_slice(&buffers[0].0);
        Ok(())
    }

    async fn write(&mut self, address: A, write: &[u8]) -> Result<(), Self::Error> {
        let buffers = self.relay_action(I2cAction {
            address,
            function_call: FunctionCall::Write(write.to_vec()),
        }).await?;
        assert_eq!(buffers.len(), 0);
        Ok(())
    }

    async fn write_read(&mut self, address: A, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        let buffers = self.relay_action(I2cAction {
            address,
            function_call: FunctionCall::WriteRead { write: write.to_vec(), read: vec![0; read.len()] },
        }).await?;
        read.copy_from_slice(&buffers[0].0);
        Ok(())
    }

    async fn transaction(&mut self, address: A, ops_in: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        let ops_copy = ops_in.iter()
                .map(|op| {
                    match op {
                        Operation::Read(buf) => TransactionOperation::Read(vec![0; buf.len()]),
                        Operation::Write(data) => TransactionOperation::Write(data.to_vec()),
                    }
                })
                .collect::<Vec<_>>();

        let buffers = self.relay_action(I2cAction {
            address,
            function_call: FunctionCall::Transaction(ops_copy),
        }).await?;

        let read_ops = ops_in.into_iter()
                .filter_map(|op| {
                    match op {
                        Operation::Read(buf) => Some(buf),
                        Operation::Write(_) => None,
                    }
                });

        for (buffer, read_op) in zip(buffers, read_ops) {
            read_op.copy_from_slice(&buffer.0);
        }

        Ok(())
    }
}

impl<I2C, A> AsyncI2cDriverBridge<I2C, A>
where
        I2C: embedded_hal::i2c::I2c<A>,
        A: AddressMode,
{
    async fn relay_action(&mut self, action: I2cAction<A>) -> Result<Vec<ReadBuffer>, AsyncBridgeError<I2C::Error>> {
        self.request_tx.as_ref()
                .ok_or_else(|| AsyncBridgeError::InternalError)?
                .send(action)
                .map_err(|_| AsyncBridgeError::InternalError)?;

        self.response_rx.next().await
                .ok_or_else(|| AsyncBridgeError::InternalError)?
                .map_err(AsyncBridgeError::BusError)
    }
}

type RequestType<A> = I2cAction<A>;
type ResponseType<I2C> = Result<Vec<ReadBuffer>, <I2C as ErrorType>::Error>;

struct ReadBuffer(Vec<u8>);

struct SyncI2cDriverRunner<I2C: embedded_hal::i2c::I2c<A>, A: AddressMode> {
    i2c: I2C,
    request_rx: BlockingReceiver<RequestType<A>>,
    response_tx: AsyncSender<ResponseType<I2C>>,
}

impl<I2C, A> SyncI2cDriverRunner<I2C, A>
where
        I2C: embedded_hal::i2c::I2c<A>,
        A: AddressMode + Copy,
{
    pub fn run_loop(i2c: I2C, request_rx: BlockingReceiver<RequestType<A>>, response_tx: AsyncSender<ResponseType<I2C>>) {
        let me = Self { i2c, request_rx, response_tx };
        me.run();
    }

    fn run(mut self) {
        while let Ok(mut action) = self.request_rx.recv() {
            let ret = match &mut action.function_call {
                FunctionCall::Read(buf) => {
                    self.i2c.read(action.address, buf)
                }
                FunctionCall::Write(data) => {
                    self.i2c.write(action.address, data)
                }
                FunctionCall::WriteRead { write, read } => {
                    self.i2c.write_read(action.address, write, read)
                }
                FunctionCall::Transaction(ops) => {
                    let mut ops_conv = ops.iter_mut()
                            .map(|op| {
                                match op {
                                    TransactionOperation::Read(buf) => {
                                        Operation::Read(buf.as_mut_slice())
                                    }
                                    TransactionOperation::Write(data) => {
                                        Operation::Write(data.as_slice())
                                    }
                                }
                            })
                            .collect::<Vec<_>>();
                    self.i2c.transaction(action.address, &mut ops_conv)
                }
            };

            let read_buffers = match action.function_call {
                FunctionCall::Read(buf) => vec![ReadBuffer(buf)],
                FunctionCall::Write(_) => vec![],
                FunctionCall::WriteRead { write, read } => {
                    vec![ReadBuffer(read)]
                }
                FunctionCall::Transaction(ops) => {
                    ops.into_iter()
                            .filter_map(|op| {
                                match op {
                                    TransactionOperation::Read(buf) => Some(ReadBuffer(buf)),
                                    TransactionOperation::Write(_) => None,
                                }
                            })
                            .collect::<Vec<_>>()
                }
            };

            let response = ret.map(|_| read_buffers);

            if let Err(_) = self.response_tx.try_send(response) {
                warn!("Response side dropped while transaction was pending!");
            }
        }
    }
}

struct I2cAction<A> {
    address: A,
    function_call: FunctionCall,
}

enum FunctionCall {
    Read(Vec<u8>),
    Write(Vec<u8>),
    WriteRead {
        write: Vec<u8>,
        read: Vec<u8>,
    },
    Transaction(Vec<TransactionOperation>),
}

enum TransactionOperation {
    Read(Vec<u8>),
    Write(Vec<u8>),
}

#[cfg(test)]
mod tests {
    use crate::AsyncI2cDriverBridge;
    use derive_new::new;
    use embedded_hal_async::i2c::{I2c, SevenBitAddress};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::fmt::Debug;

    const TEST_ADDRESS: u8 = 0x12;

    #[tokio::test]
    pub async fn test_with_mock() {
        let write1 = vec![1, 2, 3];
        let read1 = vec![4, 5, 6];
        let write2 = vec![7];
        let write3 = vec![8, 9];
        let read2 = vec![10, 11, 12, 13, 14, 15];
        let read3 = vec![16];
        let expected = vec![
            Transaction::write_read(TEST_ADDRESS, write1.clone(), read1.clone()),
            Transaction::write(TEST_ADDRESS, write2.clone()),
            Transaction::write_read(TEST_ADDRESS, write3.clone(), read2.clone()),
            Transaction::read(TEST_ADDRESS, read3.clone()),
        ];
        let mut i2c = Mock::new(&expected);

        let mut bridge = AsyncI2cDriverBridge::spawn(i2c.clone());

        let mut helper = I2cTestHelper::new(&mut bridge);
        assert_eq!(helper.write_read(&write1, read1.len()).await, Ok(read1));
        assert_eq!(helper.write(&write2).await, Ok(()));
        assert_eq!(helper.write_read(&write3, read2.len()).await, Ok(read2));
        assert_eq!(helper.read(read3.len()).await, Ok(read3));

        i2c.done();
        bridge.shutdown();
    }

    #[derive(new)]
    struct I2cTestHelper<'a, I2C> {
        i2c: &'a mut I2C,
    }

    impl<'a, I2C: I2c> I2cTestHelper<'a, I2C>
    where
            I2C: I2c<SevenBitAddress>,
            I2C::Error: Debug,
    {
        pub async fn write(&mut self, write: &[u8]) -> Result<(), String> {
            self.i2c.write(TEST_ADDRESS, write).await
                    .map_err(|e| format!("{e:?}"))
        }

        pub async fn write_read(&mut self, write: &[u8], read_len: usize) -> Result<Vec<u8>, String> {
            let mut buf = vec![0; read_len];
            self.i2c.write_read(TEST_ADDRESS, write, buf.as_mut_slice()).await
                    .map_err(|e| format!("{e:?}"))?;
            Ok(buf)
        }

        pub async fn read(&mut self, read_len: usize) -> Result<Vec<u8>, String> {
            let mut buf = vec![0; read_len];
            self.i2c.read(TEST_ADDRESS, buf.as_mut_slice()).await
                    .map_err(|e| format!("{e:?}"))?;
            Ok(buf)
        }
    }
}