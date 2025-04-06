use core::cmp::{max, min};

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    async_attribute_server::AttributeServer,
    asynch::Ble,
    gatt,
};

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::Timer;
use esp_hal::{peripherals::BT, time};
use esp_println::println;
use esp_wifi::{ble::controller::BleConnector, EspWifiController};

pub type BlePacket = heapless::Vec<u8, 20>;

#[embassy_executor::task]
pub async fn serial(
    esp_wifi_ctrl: &'static EspWifiController<'static>,
    mut bluetooth: BT,
    signal: &'static Signal<NoopRawMutex, BlePacket>,
) {
    let connector = BleConnector::new(&esp_wifi_ctrl, &mut bluetooth);

    let now = || time::Instant::now().duration_since_epoch().as_millis();
    let mut ble = Ble::new(connector, now);
    println!("Connector created");

    loop {
        println!("{:?}", ble.init().await);
        println!("{:?}", ble.cmd_set_le_advertising_parameters().await);
        println!(
            "{:?}",
            ble.cmd_set_le_advertising_data(
                create_advertising_data(&[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
                    AdStructure::CompleteLocalName(esp_hal::chip!()),
                ])
                .unwrap()
            )
            .await
        );
        println!("{:?}", ble.cmd_set_le_advertise_enable(true).await);

        println!("started advertising");

        let mut rf = |_offset: usize, data: &mut [u8]| {
            data[..3].copy_from_slice(&b"foo"[..]);
            3
        };
        let mut wf = |offset: usize, data: &[u8]| {
            let mut buffer_out = heapless::Vec::<u8, 20>::new();

            let len_min = min(buffer_out.capacity(), data.len());

            buffer_out.extend_from_slice(&data[..len_min]).unwrap();

            buffer_out[..len_min].copy_from_slice(&data[..len_min]);
            signal.signal(buffer_out);
            println!("RECEIVED: Offset {}, data {:?}", offset, data);
        };

        gatt!([service {
            uuid: "937312e0-2354-11eb-9f10-fbc30a62cf38",
            characteristics: [characteristic {
                name: "my_characteristic",
                uuid: "81389efa-51aa-4f24-823a-c107df4e1926",
                notify: true,
                read: rf,
                write: wf,
            },],
        },]);

        let mut rng = bleps::no_rng::NoRng;
        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);

        let mut notifier = || async {
            loop {
                Timer::after_secs(100).await;
            }
        };

        srv.run(&mut notifier).await.unwrap();
    }
}
