#ifndef _BTSerial_h_
#define _BTSerial_h_

#include <Arduino.h>
#include <Stream.h>
#include "esp_gap_ble_api.h"
#include <NimBLEDevice.h>

// ---- UUIDs des Nordic UART Service (NUS) ----
static constexpr const char* NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static constexpr const char* NUS_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static constexpr const char* NUS_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

class BTSerial : public Stream {
public:
    BTSerial() = default;

    bool begin(const char* deviceName, uint16_t mtu = 185) {
        if (_active) return false;

        NimBLEDevice::init(deviceName);
        NimBLEDevice::setMTU(mtu);
        NimBLEDevice::setPower(ESP_PWR_LVL_P9);

        _server = NimBLEDevice::createServer();
        _server->setCallbacks(new ServerCB(*this));

        _service = _server->createService(NUS_SERVICE_UUID);

        _tx = _service->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
        _rx = _service->createCharacteristic(NUS_RX_UUID, NIMBLE_PROPERTY::WRITE);
        _rx->setCallbacks(new RxCB(*this));

        _service->start();

        // >>> WICHTIG: Advertising über NimBLEDevice holen und Daten explizit setzen
        auto adv = NimBLEDevice::getAdvertising();

        NimBLEAdvertisementData advData;
        advData.setName(deviceName);
        advData.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
        advData.setCompleteServices(NimBLEUUID(NUS_SERVICE_UUID));

        adv->setAdvertisementData(advData);
        adv->start();

        _maxPayload    = max(20, (int)NimBLEDevice::getMTU() - 3);
        _connected     = false;
        _notifyEnabled = false;
        _active        = true;

        return true;
    }

    bool stop() {
        if (!_active) return false;

        return true;
    }


    // --- Stream / Print kompatibel ---
    size_t write(uint8_t b) override { return write(&b, 1); }

    size_t write(const uint8_t* buffer, size_t size) override {
        if (!_active || !_tx || !_connected || !_notifyEnabled || size == 0) return 0;

        size_t sent = 0;
        while (sent < size) {
            size_t chunk = min(_maxPayload, size - sent);
            _tx->setValue((uint8_t*)(buffer + sent), chunk);
            _tx->notify();
            sent += chunk;
            delay(1);
        }
        return size;
    }

    size_t printf(const char* fmt, ...) {
        if (!_active) return 0;

        va_list ap;
        va_start(ap, fmt);
        size_t n = vprintf_internal(fmt, ap);
        va_end(ap);
        return n;
    }

    int available() override {
        if (!_active) return 0;
        if (_server->getConnectedCount() == 0) return 0;

        return (_rxHead >= _rxTail)
            ? (_rxHead - _rxTail)
            : (RX_BUF_SIZE - _rxTail + _rxHead);
    }

    int read() override {
        if (_rxHead == _rxTail) return -1;
        uint8_t b = _rxBuf[_rxTail];
        _rxTail = (_rxTail + 1) % RX_BUF_SIZE;
        return b;
    }

    int peek() override {
        if (_rxHead == _rxTail) return -1;
        return _rxBuf[_rxTail];
    }

    void flush() override {
        _rxHead = _rxTail = 0;
    }

    bool connected() const { return _connected; }

private:
    static constexpr size_t RX_BUF_SIZE = 256;

    NimBLEServer*         _server = nullptr;
    NimBLEService*        _service = nullptr;
    NimBLECharacteristic* _rx = nullptr;
    NimBLECharacteristic* _tx = nullptr;

    volatile size_t _rxHead = 0, _rxTail = 0;
    uint8_t  _rxBuf[RX_BUF_SIZE];

    bool     _connected = false;
    bool     _notifyEnabled = false;
    bool     _active = false;
    size_t   _maxPayload = 20;

    size_t vprintf_internal(const char* fmt, va_list ap) {
        char stackBuf[128];
        va_list ap_copy;
        va_copy(ap_copy, ap);
        int needed = vsnprintf(stackBuf, sizeof(stackBuf), fmt, ap_copy);
        va_end(ap_copy);
        if (needed < 0) return 0;

        if ((size_t)needed < sizeof(stackBuf)) {
            write((const uint8_t*)stackBuf, (size_t)needed);
            return (size_t)needed;
        }

        size_t len = (size_t)needed + 1;
        char* heapBuf = (char*)malloc(len);
        if (!heapBuf) return 0;
        vsnprintf(heapBuf, len, fmt, ap);
        write((const uint8_t*)heapBuf, (size_t)needed);
        free(heapBuf);
        return (size_t)needed;
    }

    // --- BLE Server Callback ---
    struct ServerCB : public NimBLEServerCallbacks {
        BTSerial& self;
        explicit ServerCB(BTSerial& s) : self(s) {}
        void onConnect(NimBLEServer*, NimBLEConnInfo& connInfo) override {
            self._connected = true;
            self._notifyEnabled = true;
        }
        void onDisconnect(NimBLEServer* s, NimBLEConnInfo& connInfo, int reason) override {
            self._connected = false;
            self._notifyEnabled = false;
            s->getAdvertising()->start();
        }
    };

    // --- RX Callback (Daten empfangen) ---
    struct RxCB : public NimBLECharacteristicCallbacks {
        BTSerial& self;
        explicit RxCB(BTSerial& s) : self(s) {}
        void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
            auto val = c->getValue();
            for (uint8_t b : val) {
                size_t next = (self._rxHead + 1) % RX_BUF_SIZE;
                if (next != self._rxTail) { // kein Overflow
                    self._rxBuf[self._rxHead] = b;
                    self._rxHead = next;
                }
            }
        }
    };
};

#endif // _BTSerial_h_