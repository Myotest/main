@defgroup ble_stack BLE
@{

# Features

The BLE stack provides a BLE interface, abstracting most of the complexity of BLE.

It provides the APIs covering the central and peripheral roles as well as the
procedures for GAP, SMP, GATT server and GATT client.

Adding a new BLE primary or secondary service in the GATT server database
can easily be done as described in [Add a BLE service](@ref add_ble_service).

A [BLE Application library](@ref ble_app_lib) is provided as an example of
implementation of an interface with the BLE stack, it:
- creates a Component Framework client with a specific message handler
- registers the client to the Component Framework Property service
- subscribe to the Component Framework Battery service
- subscribe to the Component Framework BLE service
- retrieves the device name to apply in the Device Info Service
- retrieves the BD address of the device (from the factory partition or from the device properties)
- sends the Component Framework BLE service enable function
- populates the GATTS database with a default list of BLE primary services
- sets the advertising behavior (timeout, interval and advertising data)

This libary does not:
- offer any central connection related functions
- offer any GATTC related functions

Since the content of this library is very project specific, it is recommended
to an application developper to implement his own version of this library,
eventually copying it entirely and customizing it to suit his needs.

## BLE Services

Currently the following BLE services are implemented and supported:
- @ref ble_gap "BLE GAP service" (UUID: 0x1800)
- @ref ble_bas "BLE DIS service" (UUID: 0x180a): Different device info's (SW version etc) are
  displayed as per BT spec.
- @ref ble_bas "BLE BAS service" (UUID: 0x180f): BLE Battery level service as per BT spec.
  It uses the battery service to update automatically the battery level
  characteristic.
- @ref ble_hrs "BLE HRS service" (UUID: 0x180d): Heart Rate
- @ref ble_rscs "BLE RSCS service" (UUID: 0x1814): only Stride cadence notification is supported
- @ref ble_lns "BLE LNS service" (UUID: 0x1819): only Elevation notification is supported
- BLE ISPP service (UUID: dd97c415-fed9-4766-b18f-ba690d24a06a):
  Intel serial port protocol: A serial port emulation running over BLE

A new BLE primary or secondary service can be easily added. See \ref add_ble_service.

## Sources location

- Include files: `framework/include/lib/ble`
- Source files: `framework/src/lib/ble`

@}

@defgroup add_ble_service Add a BLE service
@{
@ingroup ble_stack
@anchor ble_service


# How-to Add new BLE service

Adding a custom service can be achieved by using the zephyr gatt APIs.
BLE ISPP can be used an example for a proprietary _BLE service and protocol_.

- As a convention, a new BLE service should be implemented in a ble_XXX.[ch] files (e.g. XXX: gap, bas, dis, etc).
- A function ble_XXX_init() should be implemented and called by the application (see an example in the register_services function of the BLE app library).
  It should trigger the initialization of the new BLE service by calling bt_gatt_register()
- Each BLE service is defined by const attribute table using the macros found in gatt.h:
  - BT_GATT_PRIMARY_SERVICE: the UUID of the service
  - BT_GATT_CHARACTERISTIC: the UUID of the characteristics and its properties
  - BT_GATT_DESCRIPTOR: permissions, read & write functions, optionally pointer to data storage
    - read() used init ble controller database
      - if buf pointer is NULL, it needs to return the init/max length if charactristic is readable
      - if buf pointer is not NULL, it needs copy the init data to buf pointer
    - write(): if handler for writeable characteristic
    - data pointer: optional, maybe used for init value or storage for write operation
  - BT_GATT_CCC: for notifiable or indicatable characteristics, must follow above 2 macros
    - the two macro parameters should be NULL as not used. however the macro internal CCC value is updated (see bas) via bt_gatt_attr_write_ccc().
  - additional descriptors for a characteristic maybe defined. however only AFTER above macros
- additional helper functions maybe required to update the ble controller database

A protocol type of service, e.g. ISPP, may also support the protocol events to read/write in addition to ble_update_data()
@}

@defgroup ble_services BLE Services
@{
@ingroup ble_stack
@}
