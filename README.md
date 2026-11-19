# ExVectrLink library
This library allows for use of the ExVectrLink System
## **This project is under initial development. Things will probably break.**
## FHSS runtime parameters (air profiles)

`FHSS` keeps today's FPV link as its member defaults (10 ms slots, 4 slots per hop, role period 15, variable
payload of up to 9 bytes, link version `OTA_VERSION` = 2, hop key 0, 1 ms wakeup lead, push mode). The FPV firmware
never changes them, so its air format and timing are unchanged.

Other uses apply a parameter set with `FHSS::applyProfile()` before the task starts
(`include/ExVectrLink/datalink/AirProfile.hpp`: `kFpvProfile`, `kNetworkProfileV1`), and may switch to pull mode
with `setTxPayloadProvider()` (the payload of each own TX slot is requested right before it is sent). The schedule,
trailer, sync-filter and LoRa time-on-air maths are pure functions in `include/ExVectrLink/datalink/FhssMath.hpp`
(no Arduino, no ExVectrCore). Host tests live in the ExVectrLink firmware repo (`test/test_fhss`, `test/test_air_link`).
