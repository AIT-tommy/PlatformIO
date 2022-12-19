/*
 * NimBLEAdvertisedDevice.h
 *
 *  Created: on Jan 24 2020
 *      Author H2zero
 *
 * Originally:
 *
 * BLEAdvertisedDevice.h
 *
 *  Created on: Jul 3, 2017
 *      Author: kolban
 */

#ifndef COMPONENTS_NIMBLEADVERTISEDDEVICE_H_
#define COMPONENTS_NIMBLEADVERTISEDDEVICE_H_
#include "sdkconfig.h"
#if defined(CONFIG_BT_ENABLED)

#include "nimconfig.h"
#if defined(CONFIG_BT_NIMBLE_ROLE_OBSERVER)

#include "NimBLEAddress.h"
#include "NimBLEScan.h"
#include "NimBLEUUID.h"

#include "host/ble_hs_adv.h"

#include <map>
#include <vector>


class NimBLEScan;
/**
 * @brief A representation of a %BLE advertised device found by a scan.
 *
 * When we perform a %BLE scan, the result will be a set of devices that are advertising.  This
 * class provides a model of a detected device.
 */
class NimBLEAdvertisedDevice {
public:
    NimBLEAdvertisedDevice();

    NimBLEAddress   getAddress();
    uint8_t         getAdvType();
    uint16_t        getAppearance();
    uint16_t        getAdvInterval();
    uint16_t        getMinInterval();
    uint16_t        getMaxInterval();
    std::string     getManufacturerData();
    std::string     getURI();

    /**
     * @brief A template to convert the service data to <type\>.
     * @tparam T The type to convert the data to.
     * @param [in] skipSizeCheck If true it will skip checking if the data size is less than <tt>sizeof(<type\>)</tt>.
     * @return The data converted to <type\> or NULL if skipSizeCheck is false and the data is
     * less than <tt>sizeof(<type\>)</tt>.
     * @details <b>Use:</b> <tt>getManufacturerData<type>(skipSizeCheck);</tt>
     */
    template<typename T>
    T       getManufacturerData(bool skipSizeCheck = false) {
        std::string data = getManufacturerData();
        if(!skipSizeCheck && data.size() < sizeof(T)) return T();
        const char *pData = data.data();
        return *((T *)pData);
    }

    std::string     getName();
    int             getRSSI();
    NimBLEScan*     getScan();
    size_t          getServiceDataCount();
    std::string     getServiceData(uint8_t index = 0);
    std::string     getServiceData(const NimBLEUUID &uuid);

    /**
     * @brief A template to convert the service data to <tt><type\></tt>.
     * @tparam T The type to convert the data to.
     * @param [in] index The vector index of the service data requested.
     * @param [in] skipSizeCheck If true it will skip checking if the data size is less than <tt>sizeof(<type\>)</tt>.
     * @return The data converted to <type\> or NULL if skipSizeCheck is false and the data is
     * less than <tt>sizeof(<type\>)</tt>.
     * @details <b>Use:</b> <tt>getServiceData<type>(skipSizeCheck);</tt>
     */
    template<typename T>
    T       getServiceData(uint8_t index = 0, bool skipSizeCheck = false) {
        std::string data = getServiceData(index);
        if(!skipSizeCheck && data.size() < sizeof(T)) return T();
        const char *pData = data.data();
        return *((T *)pData);
    }

    /**
     * @brief A template to convert the service data to <tt><type\></tt>.
     * @tparam T The type to convert the data to.
     * @param [in] uuid The uuid of the service data requested.
     * @param [in] skipSizeCheck If true it will skip checking if the data size is less than <tt>sizeof(<type\>)</tt>.
     * @return The data converted to <type\> or NULL if skipSizeCheck is false and the data is
     * less than <tt>sizeof(<type\>)</tt>.
     * @details <b>Use:</b> <tt>getServiceData<type>(skipSizeCheck);</tt>
     */
    template<typename T>
    T       getServiceData(const NimBLEUUID &uuid, bool skipSizeCheck = false) {
        std::string data = getServiceData(uuid);
        if(!skipSizeCheck && data.size() < sizeof(T)) return T();
        const char *pData = data.data();
        return *((T *)pData);
    }

    NimBLEUUID      getServiceDataUUID(uint8_t index = 0);
    NimBLEUUID      getServiceUUID(uint8_t index = 0);
    size_t          getServiceUUIDCount();
    NimBLEAddress   getTargetAddress(uint8_t index = 0);
    size_t          getTargetAddressCount();
    int8_t          getTXPower();
    uint8_t*        getPayload();
    uint8_t         getAdvLength();
    size_t          getPayloadLength();
    uint8_t         getAddressType();
    time_t          getTimestamp();
    bool            isAdvertisingService(const NimBLEUUID &uuid);
    bool            haveAppearance();
    bool            haveManufacturerData();
    bool            haveName();
    bool            haveRSSI();
    bool            haveServiceData();
    bool            haveServiceUUID();
    bool            haveTXPower();
    bool            haveConnParams();
    bool            haveAdvInterval();
    bool            haveTargetAddress();
    bool            haveURI();
    std::string     toString();

private:
    friend class NimBLEScan;

    void    setAddress(NimBLEAddress address);
    void    setAdvType(uint8_t advType);
    void    setPayload(const uint8_t *payload, uint8_t length, bool append);
    void    setRSSI(int rssi);
    uint8_t findAdvField(uint8_t type, uint8_t index = 0, uint8_t *data_loc = nullptr);
    uint8_t findServiceData(uint8_t index, uint8_t* bytes);

    NimBLEAddress   m_address = NimBLEAddress("");
    uint8_t         m_advType;
    int             m_rssi;
    time_t          m_timestamp;
    bool            m_callbackSent;
    uint8_t         m_advLength;

    std::vector<uint8_t>    m_payload;
};

/**
 * @brief A callback handler for callbacks associated device scanning.
 *
 * When we are performing a scan as a %BLE client, we may wish to know when a new device that is advertising
 * has been found.  This class can be sub-classed and registered such that when a scan is performed and
 * a new advertised device has been found, we will be called back to be notified.
 */
class NimBLEAdvertisedDeviceCallbacks {
public:
    virtual ~NimBLEAdvertisedDeviceCallbacks() {}
    /**
     * @brief Called when a new scan result is detected.
     *
     * As we are scanning, we will find new devices.  When found, this call back is invoked with a reference to the
     * device that was found.  During any individual scan, a device will only be detected one time.
     */
    virtual void onResult(NimBLEAdvertisedDevice* advertisedDevice) = 0;
};

#endif // #if defined( CONFIG_BT_NIMBLE_ROLE_CENTRAL)
#endif /* CONFIG_BT_ENABLED */
#endif /* COMPONENTS_NIMBLEADVERTISEDDEVICE_H_ */