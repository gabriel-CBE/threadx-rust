#![no_main]
#![no_std]
use crate::alloc::string::ToString;

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


use threadx_rs::allocator::ThreadXAllocator;
use threadx_rs::event_flags::GetOption::*;
use threadx_rs::event_flags::{EventFlagsGroup, EventFlagsGroupHandle};

use threadx_rs::WaitOption::*;

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
            defmt::println!(
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
            defmt::println!("WLAN thread started");

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

            defmt::println!("Measure thread started");
        },
    );

    tx.initialize();
    defmt::println!("Exit");
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
    defmt::println!("WLAN connected, beginning to measure");
    loop {
        let deg = i32::from(hts221.temperature_x8(&mut i2c).unwrap());
        let _ = snd.send(Event::TemperatureMeasurement(deg), WaitForever);
        defmt::println!("Current temperature: {}", deg);
        let _ = sleep(Duration::from_secs(30));
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

fn print_text(text: &str, display: &mut Option<DisplayType<I2CBus>>) {
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_7X14)
        .text_color(BinaryColor::On)
        .build();
    if let Some(actual_display) = display {
        actual_display.clear_buffer();
        Text::with_baseline(text, Point::zero(), text_style, Baseline::Top)
            .draw(actual_display)
            .unwrap();
        actual_display.flush().unwrap();
    }   
}

/// Initializes the ThreadX TCP WiFi network with the given SSID and password.
///
/// # Arguments
/// * `ssid` - The WiFi SSID to connect to.
/// * `password` - The WiFi password.
///
/// # Returns
/// A connected `ThreadxTcpWifiNetwork` instance. Panics if initialization fails.
fn create_tcp_network(ssid: &str, password: &str) -> Result<ThreadxTcpWifiNetwork, ()> {
    match ThreadxTcpWifiNetwork::initialize(ssid, password) {
        Ok(net) => Ok(net),
        Err(_) => Err(()),
    }
}

/// Creates an MQTT configuration for Minimq using the provided buffer.
///
/// # Arguments
/// * `buffer` - A mutable reference to a buffer for MQTT packet storage.
///
/// # Returns
/// A `ConfigBuilder` for the MQTT client using the specified broker and buffer.
fn create_mqtt_config<'a>(buffer: &'a mut [u8; 1024], broker_ip: core::net::Ipv4Addr) -> Result<ConfigBuilder<'a, IpBroker>, minimq::ProtocolError> {
    let remote_addr = core::net::SocketAddr::new(core::net::IpAddr::V4(broker_ip), 1883);
    let broker = IpBroker::new(remote_addr.ip());
    ConfigBuilder::new(broker, buffer)
        .keepalive_interval(60)
        .client_id("mytest")
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
) -> Result<MiniMqBasedTransport<'a, ThreadxTcpWifiNetwork, Clock, IpBroker>, minimq::ProtocolError>
where
    Clock: minimq::embedded_time::Clock,
{
    Ok(MiniMqBasedTransport::new(Minimq::new(network, clock, config)))
}

/// Handles publishing a message to an MQTT topic.
fn handle_mqtt_publish<'buf, Clock, Broker>(
    transport: &mut MiniMqBasedTransport<'buf, ThreadxTcpWifiNetwork, Clock, Broker>,
    topic: &str,
    message: &[u8],
)
where
    Clock: minimq::embedded_time::Clock,
    Broker: minimq::Broker,
{
    if transport.is_connected() {
        // Publish message via MQTT
        match transport.publish_raw(topic, message) {
            Ok(_) => {
                let msg_str = core::str::from_utf8(message).unwrap_or("<invalid>");
                defmt::println!("Published to {}: {}", topic, msg_str);
            }
            Err(e) => {
                defmt::println!("MQTT publish failed: {}", defmt::Debug2Format(&e));
            }
        }
    } else {
        defmt::println!("MQTT not connected");
    }
}

/// Handles subscribing to an MQTT topic and processes received messages with a callback.
fn handle_mqtt_subscribe<'buf, Clock, Broker, F>(
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
    _recv: QueueReceiver<Event>,
    evt_handle: EventFlagsGroupHandle,
    display: &Mutex<Option<DisplayType<I2CBus>>>,
    btn_a: Option<board::InputButton<'A', 4>>,
) -> ! {
    let ssid = "__WIFI_SSID__";
    let password = "__WIFI_PASSWORD__";

    let broker_ip = core::net::Ipv4Addr::new(5, 196, 78, 28);

    let sub_topic = "vehicle/parameters";
    let pub_topic_params = "vehicle/parameters";

    let mut display_guard = display.lock(WaitForever).unwrap();
        
    print_text("Connecting \nto network...", &mut *display_guard);
    defmt::println!("Attempting to connect to network {} ...", ssid);

    let network = match create_tcp_network(ssid, password) {
        Ok(net) => net,
        Err(_) => {
            print_text("TCP connect failed!", &mut *display_guard);
            panic!("Failed to initialize TCP network");
        }
    };
    let mut buffer = [0u8; 1024];
    print_text("Connecting \nto MQTT broker...", &mut *display_guard);
    defmt::println!("Connecting to MQTT broker at {} ...", broker_ip.to_string().as_str());
    
    let mqtt_cfg = match create_mqtt_config(&mut buffer, broker_ip) {
        Ok(cfg) => cfg,
        Err(_) => {
            print_text("MQTT config failed!", &mut *display_guard);
            panic!("Failed to create MQTT config");
        }
    };
    let clock = start_clock();
    let mut transport = match create_transport(network, clock, mqtt_cfg) {
        Ok(t) => t,
        Err(_) => {
            print_text("MQTT transport failed!", &mut *display_guard);
            panic!("Failed to create MQTT transport");
        }
    };

    evt_handle
        .publish(FlagEvents::WifiConnected as u32)
        .unwrap();

    print_text("Connected", &mut *display_guard);

    thread::sleep(Duration::from_millis(2000)).unwrap();
    let mut subscribed = false;

    let mut msg_received_counter = 0;
    let mut msg_sent_counter = 0;
    let mut last_msg_received = heapless::String::<64>::new();
    let mut last_msg_sent = heapless::String::<64>::new();
    
    // Store last received vehicle parameters JSON and cruise control state
    let mut last_vehicle_params = heapless::String::<1024>::new();
    let mut cruise_control_active = false;
    let mut last_button_state = true; // Button is active low
    loop {
        // Lock the display mutex each loop iteration
        let mut display_guard = display.lock(WaitForever).unwrap();
        if let Some(ref mut _actual_display) = *display_guard {
            handle_mqtt_subscribe(
                &mut transport,
                sub_topic,
                &mut subscribed,
                |recv_topic, payload| {
                    let msg = core::str::from_utf8(payload).unwrap_or("<invalid>");
                    defmt::println!("Received from {}: {}", recv_topic, msg);
                    
                    // Store the complete JSON for later use
                    last_vehicle_params.clear();
                    let _ = write!(last_vehicle_params, "{}", msg);
                    
                    // Simple check if CruiseControl is true
                    cruise_control_active = msg.contains("\"CruiseControl\":true");
                    
                    last_msg_received.clear();
                    let _ = write!(last_msg_received, "CC:{}", if cruise_control_active { "ON" } else { "OFF" });
                    
                    msg_received_counter += 1;
                }
            );
        }
        transport.poll();

        // Check button press for cruise control override
        if let Some(ref btn) = btn_a {
            let current_button_state = btn.is_high(); // true = not pressed, false = pressed (active low)
            
            if current_button_state != last_button_state {
                // Button state changed
                if !current_button_state {
                    // Button was just pressed (went from high to low)
                    defmt::println!("Button pressed!");
                    
                    if cruise_control_active && !last_vehicle_params.is_empty() {
                        defmt::println!("Cruise Control Override: Disabling CC and publishing modified parameters");
                        
                        // Create a new string buffer for the modified JSON
                        let mut modified_json = heapless::String::<1024>::new();
                        
                        // Replace "CruiseControl":true with "CruiseControl":false
                        let original_str = last_vehicle_params.as_str();
                        if let Some(pos) = original_str.find("\"CruiseControl\":true") {
                            let before = &original_str[..pos];
                            let after = &original_str[pos + 20..]; // Skip "CruiseControl":true (20 chars)
                            let _ = write!(modified_json, "{}\"CruiseControl\":false{}", before, after);
                        } else {
                            // Fallback: use original if cruise control not found
                            let _ = write!(modified_json, "{}", original_str);
                        }
                        
                        // Publish the modified JSON via direct MQTT
                        handle_mqtt_publish(&mut transport, pub_topic_params, modified_json.as_bytes());
                        
                        msg_sent_counter += 1;
                        last_msg_sent.clear();
                        let _ = write!(last_msg_sent, "CC:OFF");
                    } else {
                        // Button pressed but cruise control not active or no vehicle data
                        last_msg_sent.clear();
                        let _ = write!(last_msg_sent, "BTN");
                    }
                }
                last_button_state = current_button_state;
            }
        }

        let mut text_buf = heapless::String::<128>::new();
        let _ = write!(text_buf, "Recv {}: \n{}\nSend {}: \n{}", msg_received_counter, last_msg_received, msg_sent_counter, last_msg_sent);
        print_text(&text_buf, &mut *display_guard);
        thread::sleep(Duration::from_millis(100)).unwrap();
    }
}
