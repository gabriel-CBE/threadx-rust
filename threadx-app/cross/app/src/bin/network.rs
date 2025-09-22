#![no_main]
#![no_std]

use core::cell::RefCell;
use core::sync::atomic::AtomicU32;
use core::time::Duration;

use alloc::boxed::Box;
use alloc::vec::Vec;
use board::{BoardMxAz3166, DisplayType, I2CBus, LowLevelInit, hts221};

use cortex_m::interrupt;
use embedded_graphics::mono_font::ascii::FONT_7X14;
use heapless::String;
use minimq::broker::IpBroker;
use minimq::embedded_time::rate::Fraction;
use minimq::embedded_time::{self, Clock, Instant};
use minimq::{ConfigBuilder, Minimq};
use netx_sys::ULONG;
use static_cell::StaticCell;
use threadx_app::minimqtransport::MiniMqBasedTransport;
use threadx_app::network::ThreadxTcpWifiNetwork;

use threadx_app::uprotocol_v1::UMessage;
use threadx_app::utransport::LocalUTransport;
use threadx_rs::allocator::ThreadXAllocator;
use threadx_rs::event_flags::GetOption::*;
use threadx_rs::event_flags::{EventFlagsGroup, EventFlagsGroupHandle};

use threadx_rs::WaitOption::*;
use threadx_rs::WaitOption;
use threadx_rs::executor::Executor;
use threadx_rs::mutex::Mutex;
use threadx_rs::queue::{Queue, QueueReceiver, QueueSender};
use threadx_rs::thread::{self, sleep};

use threadx_rs::thread::Thread;
use threadx_rs::timer::Timer;

use core::fmt::Write;

use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

extern crate alloc;

pub type UINT = ::core::ffi::c_uint;
#[derive(Copy, Clone)]
pub enum Event {
    TemperatureMeasurement(i32),
}
impl From<Event> for Vec<u8> {
    fn from(val: Event) -> Self {
        let mut str = String::<32>::new();
        let Event::TemperatureMeasurement(measure) = val;
        let _ = write!(str, "Temp: {} C", measure);
        str.as_bytes().to_vec()
    }
}

pub enum FlagEvents {
    WifiConnected = 1,
    WifiDisconnected = 2,
}

#[global_allocator]
static GLOBAL: ThreadXAllocator = ThreadXAllocator::new();

// Used for Rust heap allocation via global allocator
static HEAP: StaticCell<[u8; 512]> = StaticCell::new();

// Wifi thread globals
static WIFI_THREAD_STACK: StaticCell<[u8; 8192]> = StaticCell::new();
static WIFI_THREAD: StaticCell<Thread> = StaticCell::new();

static MEASURE_THREAD_STACK: StaticCell<[u8; 512]> = StaticCell::new();
static MEASURE_THREAD: StaticCell<Thread> = StaticCell::new();

static BOARD: cortex_m::interrupt::Mutex<RefCell<Option<BoardMxAz3166<I2CBus>>>> =
    cortex_m::interrupt::Mutex::new(RefCell::new(None));
static QUEUE: StaticCell<Queue<Event>> = StaticCell::new();
static QUEUE_MEM: StaticCell<[u8; 128]> = StaticCell::new();

static EVENT_GROUP: StaticCell<EventFlagsGroup> = StaticCell::new();
static DISPLAY: StaticCell<Mutex<Option<DisplayType<I2CBus>>>> = StaticCell::new();

#[cortex_m_rt::entry]
fn main() -> ! {
    let tx = threadx_rs::Builder::new(
        |ticks_per_second| {
            let board = BoardMxAz3166::low_level_init(ticks_per_second);
            // ThreadX mutexes cannot be used here.
            interrupt::free(|cs| BOARD.borrow(cs).borrow_mut().replace(board));
        },
        |mem_start| {
            let stack_start = 0x2002_0000;
            defmt::info!(
                "Define application. Memory starts at: {} free stack space {} byte",
                mem_start,
                stack_start - (mem_start as usize)
            );

            #[cfg(feature = "mqtt_logging")]
            log_to_defmt::setup();

            let heap_mem = HEAP.init_with(|| [0u8; 512]);

            GLOBAL.initialize(heap_mem).unwrap();

            // Get the peripherals
            let display_ref = DISPLAY.init(Mutex::new(None));
            // Create fresh reborrow
            let mut pinned_display = core::pin::Pin::static_mut(display_ref);
            let mut pinned_display_ref = pinned_display.as_mut();
            // Initialize the mutex
            pinned_display_ref
                .as_mut()
                .initialize(c"display_mtx", false)
                .unwrap();
            let (display, btn_a) = interrupt::free(|cs| {
                let mut board = BOARD.borrow(cs).borrow_mut();
                let display = board.as_mut().unwrap().display.take().unwrap();
                let btn_a = board.as_mut().unwrap().btn_a.take();
                (display, btn_a)
            });
            {
                // Temporary scope to hold the lock
                let mut display_guard = pinned_display_ref.lock(WaitForever).unwrap();
                display_guard.replace(display);
            }
            let (hts211, i2c) = interrupt::free(|cs| {
                let mut board = BOARD.borrow(cs).borrow_mut();
                let board = board.as_mut().unwrap();
                (
                    board.temp_sensor.take().unwrap(),
                    board.i2c_bus.take().unwrap(),
                )
            });

            // Create communication queue
            let qm = QUEUE_MEM.init_with(|| [0u8; 128]);
            let queue = QUEUE.init(Queue::new());
            let (sender, receiver) = queue.initialize(c"m_queue", qm).unwrap();

            // create events flag group
            let event_group = EVENT_GROUP.init(EventFlagsGroup::new());
            let evt_handle = event_group.initialize(c"event_flag").unwrap();

            // Static Cell since we need an allocated but uninitialized block of memory
            let wifi_thread_stack = WIFI_THREAD_STACK.init_with(|| [0u8; 8192]);
            let wifi_thread = WIFI_THREAD.init(Thread::new());

            let _ = wifi_thread
                .initialize_with_autostart_box(
                    c"wifi_thread",
                    Box::new(move || do_network(receiver, evt_handle, &pinned_display, btn_a)),
                    wifi_thread_stack,
                    4,
                    4,
                    0,
                )
                .unwrap();
            defmt::info!("WLAN thread started");

            let measure_thread_stack = MEASURE_THREAD_STACK.init_with(|| [0u8; 512]);
            let measure_thread: &'static mut Thread = MEASURE_THREAD.init(Thread::new());

            let _ = measure_thread
                .initialize_with_autostart_box(
                    c"measurement_thread",
                    Box::new(move || do_measurement(sender, evt_handle, hts211, i2c)),
                    measure_thread_stack,
                    4,
                    4,
                    0,
                )
                .unwrap();

            defmt::info!("Measure thread started");
        },
    );

    tx.initialize();
    defmt::info!("Exit");
    threadx_app::exit()
}

fn do_measurement(
    snd: QueueSender<Event>,
    evt_handle: EventFlagsGroupHandle,
    mut hts221: hts221::HTS221<I2CBus, stm32f4xx_hal::i2c::Error>,
    mut i2c: I2CBus,
) {
    let _res = evt_handle
        .get(
            FlagEvents::WifiConnected as u32,
            WaitAllAndClear,
            WaitForever,
        )
        .unwrap();
    defmt::info!("WLAN connected, beginning to measure");
    loop {
        let deg = i32::from(hts221.temperature_x8(&mut i2c).unwrap());
        let _ = snd.send(Event::TemperatureMeasurement(deg), WaitForever);
        defmt::info!("Current temperature: {}", deg);
        let _ = sleep(Duration::from_secs(5));
    }
}

fn start_clock() -> impl Clock {
    static TICKS: AtomicU32 = AtomicU32::new(0);

    // TODO: Hardware Clock implementation
    struct ThreadXSecondClock {}

    impl embedded_time::Clock for ThreadXSecondClock {
        type T = u32;

        const SCALING_FACTOR: embedded_time::rate::Fraction = Fraction::new(1, 1);

        fn try_now(&self) -> Result<embedded_time::Instant<Self>, embedded_time::clock::Error> {
            Ok(Instant::new(
                TICKS.load(core::sync::atomic::Ordering::Relaxed),
            ))
    }
}

    extern "C" fn clock_tick(_arg: ULONG) {
        TICKS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }

    // Start the clock timer --> Should be done in Hardware but we do it via ThreadX for the fun of it

    static CLOCK_TIMER: StaticCell<Timer> = StaticCell::new();
    let clock_timer = CLOCK_TIMER.init(Timer::new());

    clock_timer
        .initialize_with_fn(
            c"clock_timer_mqtt",
            Some(clock_tick),
            0,
            Duration::from_secs(1),
            Duration::from_secs(1),
            true,
        )
        .unwrap();
    ThreadXSecondClock {}
}

fn print_text(text: &str, display: &mut DisplayType<I2CBus>) {
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_7X14)
        .text_color(BinaryColor::On)
        .build();
    display.clear_buffer();
    Text::with_baseline(text, Point::zero(), text_style, Baseline::Top)
        .draw(display)
        .unwrap();

    display.flush().unwrap();
}

/// Initializes the ThreadX TCP WiFi network with the given SSID and password.
///
/// # Arguments
/// * `ssid` - The WiFi SSID to connect to.
/// * `password` - The WiFi password.
///
/// # Returns
/// A connected `ThreadxTcpWifiNetwork` instance. Panics if initialization fails.
fn create_tcp_network(ssid: &str, password: &str) -> ThreadxTcpWifiNetwork {
    ThreadxTcpWifiNetwork::initialize(ssid, password)
        .expect("Failed to initialize TCP network")
}

/// Creates an MQTT configuration for Minimq using the provided buffer.
///
/// # Arguments
/// * `buffer` - A mutable reference to a buffer for MQTT packet storage.
///
/// # Returns
/// A `ConfigBuilder` for the MQTT client using the specified broker and buffer.
fn create_mqtt_config<'a>(buffer: &'a mut [u8; 1024]) -> ConfigBuilder<'a, IpBroker> {
    let remote_addr = core::net::SocketAddr::new(core::net::IpAddr::V4(core::net::Ipv4Addr::new(5, 196, 78, 28)), 1883);
    let broker = IpBroker::new(remote_addr.ip());
    ConfigBuilder::new(broker, buffer)
        .keepalive_interval(60)
        .client_id("mytest")
        .unwrap()
}

/// Creates a Minimq-based transport layer for MQTT communication.
///
/// # Arguments
/// * `network` - The initialized TCP WiFi network.
/// * `clock` - The clock implementation for Minimq timing.
/// * `config` - The MQTT configuration builder.
///
/// # Returns
/// A `MiniMqBasedTransport` instance ready for MQTT operations.
fn create_transport<'a, Clock>(
    network: ThreadxTcpWifiNetwork,
    clock: Clock,
    config: ConfigBuilder<'a, IpBroker>,
) -> MiniMqBasedTransport<'a, ThreadxTcpWifiNetwork, Clock, IpBroker>
where
    Clock: minimq::embedded_time::Clock,
{
    MiniMqBasedTransport::new(Minimq::new(network, clock, config))
}

/// Handles publishing a message to an MQTT topic.
fn handle_publish<'buf, Clock, Broker>(
    transport: &mut MiniMqBasedTransport<'buf, ThreadxTcpWifiNetwork, Clock, Broker>,
    topic: &str,
    message: &[u8],
    executor: &Executor,
)
where
    Clock: minimq::embedded_time::Clock,
    Broker: minimq::Broker,
{
    if transport.is_connected() {
        let mut umessage = UMessage::default();
        umessage.payload.replace(message.to_vec());
        let _res = executor.block_on(transport.send(topic, umessage));
    }
}

/// Handles subscribing to an MQTT topic and processes received messages with a callback.
fn handle_subscribe<'buf, Clock, Broker, F>(
    transport: &mut MiniMqBasedTransport<'buf, ThreadxTcpWifiNetwork, Clock, Broker>,
    topic: &str,
    subscribed: &mut bool,
    mut on_message: F,
)
where
    Clock: minimq::embedded_time::Clock,
    Broker: minimq::Broker,
    F: FnMut(&str, &[u8]),
{
    if transport.is_connected() {
        if !*subscribed {
            if transport.subscribe(topic).is_ok() {
                *subscribed = true;
            }
        }
        transport.poll_with_callback(|recv_topic, payload| {
            if recv_topic == topic {
                on_message(recv_topic, payload);
            }
            ()
        });
    }
}

/// # Panics
///
/// Will panic on nearly any kind of failure:
///     - Not being able to obtain the display lock
///     - Not being able to connect to WiFi or other network initialization issues
pub fn do_network(
    recv: QueueReceiver<Event>,
    evt_handle: EventFlagsGroupHandle,
    display: &Mutex<Option<DisplayType<I2CBus>>>,
    btn_a: Option<board::InputButton<'A', 4>>,
) -> ! {
    let ssid = "__WIFI_SSID__";
    let password = "__WIFI_PASSWORD__";
    let sub_topic = "threadx/A/0/2/8001";
    let pub_topic = "threadx/A/0/2/8001";

    let mut display_guard = display.lock(WaitForever).unwrap();
    if let Some(ref mut actual_display) = *display_guard {
        print_text("Connecting \nto network...", actual_display);
    }
    let network = create_tcp_network(ssid, password);
    let mut buffer = [0u8; 1024];
    if let Some(ref mut actual_display) = *display_guard {
        print_text("Connecting \nto MQTT broker...", actual_display);
    }
    let mqtt_cfg = create_mqtt_config(&mut buffer);
    let clock = start_clock();
    let mut transport = create_transport(network, clock, mqtt_cfg);

    let executor = Executor::new();

    evt_handle
        .publish(FlagEvents::WifiConnected as u32)
        .unwrap();

    let mut display_guard = display.lock(WaitForever).unwrap();
    if let Some(ref mut actual_display) = *display_guard {
        print_text("Connected", actual_display);
    }
    thread::sleep(Duration::from_millis(2000)).unwrap();
    let mut subscribed = false;

    let mut msg_received_counter = 0;
    let mut msg_sent_counter = 0;
    let mut last_msg_received = heapless::String::<64>::new();
    let mut last_msg_sent = heapless::String::<64>::new();
    
    // btn_a is now owned and passed in

    let mut last_engage_value = 0;
    let mut sent_zero = false;
    loop {
        // Lock the display mutex each loop iteration
        let mut display_guard = display.lock(WaitForever).unwrap();
        if let Some(ref mut actual_display) = *display_guard {
            handle_subscribe(
                &mut transport,
                sub_topic,
                &mut subscribed,
                |_, payload| {
                    let msg = core::str::from_utf8(payload).unwrap_or("<invalid>");
                    last_msg_received.clear();
                    let _ = write!(last_msg_received, "{}", msg);
                    // Parse value as integer and only accept 0 or 1
                    match msg.trim().parse::<i32>() {
                        Ok(val) if val == 0 || val == 1 => {
                            last_engage_value = val;
                        }
                        Ok(other) => {
                            last_engage_value = 0;
                            defmt::warn!("Received invalid engage value: {}", other);
                        }
                        Err(_) => {
                            last_engage_value = 0;
                            defmt::warn!("Received non-integer engage value: {}", msg);
                        }
                    }
                    msg_received_counter += 1;
                }
            );
        }
        transport.poll();

        // Check button press and publish if needed
        if let Some(ref btn) = btn_a {
            // Only send 0 when button is pressed (active low)
            if btn.is_low() && last_engage_value == 1 && !sent_zero {
                let msg_vec = b"0";
                last_msg_sent.clear();
                let _ = write!(last_msg_sent, "{}", "0");
                handle_publish(&mut transport, pub_topic, msg_vec, &executor);
                msg_sent_counter += 1;
                sent_zero = true;
            }
            // Reset sent_zero if topic value changes to 0 or button is released
            if sent_zero && btn.is_high() {
                sent_zero = false;
            }
        }

        if let Some(ref mut actual_display) = *display_guard {
            let mut text_buf = heapless::String::<128>::new();
            let _ = write!(text_buf, "Recv {}: \n{}\nSend {}: \n{}", msg_received_counter, last_msg_received, msg_sent_counter, last_msg_sent);
            print_text(&text_buf, actual_display);
        }
        thread::sleep(Duration::from_millis(100)).unwrap();
    }
}
