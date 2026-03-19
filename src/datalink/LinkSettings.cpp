#include <cstdint>

#include "ExVectrNetwork/datalink/sx1280/Sx1280_2.hpp"

#include "ExVectrLink/datalink/LinkSettings.hpp"

namespace VCTR::ExVectrLink::datalink {

const ModulationParams modulationPresets[] = {
    {
        VCTR::network::datalink::SX1280_SF::SF_5,       //
        VCTR::network::datalink::SX1280_BW::BW_1600KHz, //
        VCTR::network::datalink::SX1280_CR::LI_4_5,     //
        Core::SECONDS / 250,                            //
    },

    {
        VCTR::network::datalink::SX1280_SF::SF_6,      //
        VCTR::network::datalink::SX1280_BW::BW_800KHz, //
        VCTR::network::datalink::SX1280_CR::LI_4_8,    //
        Core::SECONDS / 250                            //
    },

    {
        VCTR::network::datalink::SX1280_SF::SF_7,      //
        VCTR::network::datalink::SX1280_BW::BW_800KHz, //
        VCTR::network::datalink::SX1280_CR::LI_4_8,    //
        Core::SECONDS / 150                            //
    },

    {
        VCTR::network::datalink::SX1280_SF::SF_8,      //
        VCTR::network::datalink::SX1280_BW::BW_800KHz, //
        VCTR::network::datalink::SX1280_CR::LI_4_8,    //
        Core::SECONDS / 50,                            //
    },
};

// uint8_t getPayloadSizeLimitForInterval(const ModulationParams &params,
//                                        int64_t transmitTimeLimit);

} // namespace VCTR::ExVectrLink::datalink