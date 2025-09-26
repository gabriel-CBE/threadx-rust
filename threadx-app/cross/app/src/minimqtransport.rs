use embedded_nal::TcpClientStack;
use minimq::{Minimq, Property, Publication, embedded_time, types::{Utf8String, TopicFilter}};

use crate::{
    uprotocol_v1::{UMessage, UStatus},
    utransport::LocalUTransport,
};
const KEY_UPROTOCOL_VERSION: &str = "uP";
const KEY_MESSAGE_ID: &str = "1";
const KEY_TYPE: &str = "2";
const KEY_SOURCE: &str = "3";

const _KEY_SINK: &str = "4";
const _KEY_PRIORITY: &str = "5";
const _KEY_PERMISSION_LEVEL: &str = "7";
const _KEY_COMMSTATUS: &str = "8";
const _KEY_TOKEN: &str = "10";
const _KEY_TRACEPARENT: &str = "11";

pub struct MiniMqBasedTransport<
    'buf,
    TcpStack: TcpClientStack,
    Clock: embedded_time::Clock,
    Broker: minimq::Broker,
> {
    mqtt_client: Minimq<'buf, TcpStack, Clock, Broker>,
}

impl<'buf, TcpStack: TcpClientStack, Clock: embedded_time::Clock, Broker: minimq::Broker>
    MiniMqBasedTransport<'buf, TcpStack, Clock, Broker>
{
    pub const fn new(client: Minimq<'buf, TcpStack, Clock, Broker>) -> Self {
        MiniMqBasedTransport {
            mqtt_client: client,
        }
    }
    /// # Panics
    ///
    /// Will panic on poll error different then Network or SessionReset
    pub fn poll(&mut self) {
        match self
            .mqtt_client
            .poll(|_client, _topic, _payload, _properties| 1)
        {
            Ok(_) => (),
            Err(minimq::Error::Network(_e)) => {
                defmt::warn!("Network disconnect, trying to reconnect.");
            }
            Err(minimq::Error::SessionReset) => {
                defmt::info!("Session reset.");
            }
            _ => panic!("Error during poll, giving up."),
        }
    }

    pub fn is_connected(&mut self) -> bool {
        self.mqtt_client.client().is_connected()
    }

    pub fn poll_with_callback<F>(&mut self, mut f: F)
    where
        F: FnMut(&str, &[u8]),
    {
        match self
            .mqtt_client
            .poll(|_client, topic, payload, _properties| {
                f(topic, payload);
                1
            })
        {
            Ok(_) => (),
            Err(minimq::Error::Network(_e)) => {
                defmt::warn!("Network disconnect, trying to reconnect.");
            }
            Err(minimq::Error::SessionReset) => {
                defmt::info!("Session reset.");
            }
            _ => panic!("Error during poll, giving up."),
        }
    }

    pub fn subscribe(&mut self, topic: &str) -> Result<(), minimq::Error<<TcpStack as TcpClientStack>::Error>> {
        let filters = [TopicFilter::new(topic)];
        self.mqtt_client.client().subscribe(&filters, &[])
    }

    /// Direct MQTT publish bypassing uProtocol entirely
    pub fn publish_raw(&mut self, topic: &str, payload: &[u8]) -> Result<(), minimq::PubError<<TcpStack as TcpClientStack>::Error, ()>> {
        self.mqtt_client
            .client()
            .publish(Publication::new(topic, payload))
    }
}

impl<TcpStack, Clock, Broker> LocalUTransport for MiniMqBasedTransport<'_, TcpStack, Clock, Broker>
where
    TcpStack: TcpClientStack,
    Clock: embedded_time::Clock,
    Broker: minimq::Broker,
{
    #[doc = " Sends a message using this transport\'s message exchange mechanism."]
    #[doc = ""]
    #[doc = " # Arguments"]
    #[doc = ""]
    #[doc = " * `message` - The message to send. The `type`, `source` and `sink` properties of the"]
    #[doc = "   [UAttributes](https://github.com/eclipse-uprotocol/up-spec/blob/v1.6.0-alpha.4/basics/uattributes.adoc) contained"]
    #[doc = "   in the message determine the addressing semantics."]
    #[doc = ""]
    #[doc = " # Errors"]
    #[doc = ""]
    #[doc = " Returns an error if the message could not be sent."]
    async fn send(&mut self, topic: &str, message: UMessage) -> Result<(), UStatus> {
        let uuid = uuid::uuid!("01956d55-177b-7556-baf6-040e3127165e");
        let buffer = &mut uuid::Uuid::encode_buffer();
        let uuid_hyp = uuid.as_hyphenated().encode_lower(buffer);

        // Create a user property with '//' + topic
        let mut source_buf = heapless::String::<128>::new();
        source_buf.push_str("//").ok();
        source_buf.push_str(topic).ok();


        let user_properties = [
            Property::UserProperty(Utf8String(KEY_UPROTOCOL_VERSION), Utf8String("1")),
            // UUID handling
            Property::UserProperty(Utf8String(KEY_MESSAGE_ID), Utf8String(uuid_hyp)),
            Property::UserProperty(Utf8String(KEY_TYPE), Utf8String("up-pub.v1")),
            Property::UserProperty(
                Utf8String(KEY_SOURCE),                
                Utf8String("//threadx/000A/2/8001"),
            ),
        ];

        self.mqtt_client
            .client()
            .publish(
                Publication::new(topic, message.payload())
                    .properties(&user_properties),
            )
            .unwrap();
        Ok(())
    }
}
