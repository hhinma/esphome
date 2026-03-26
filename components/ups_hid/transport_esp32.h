#pragma once

#include "transport_interface.h"

#ifdef USE_ESP32
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <mutex>
#include <set>
#include <atomic>

namespace esphome {
namespace ups_hid {

/**
 * ESP32 USB Transport Implementation
 * 
 * Concrete implementation using ESP-IDF USB Host API
 * Handles all ESP32-specific USB communication details
 */
class Esp32UsbTransport : public IUsbTransport {
public:
    Esp32UsbTransport();
    ~Esp32UsbTransport() override;
    
    // IUsbTransport implementation
    esp_err_t initialize() override;
    esp_err_t deinitialize() override;
    
    bool is_connected() const override;
    uint16_t get_vendor_id() const override;
    uint16_t get_product_id() const override;
    
    esp_err_t hid_get_report(uint8_t report_type, uint8_t report_id,
                           uint8_t* data, size_t* data_len,
                           uint32_t timeout_ms = 1000) override;

    esp_err_t hid_set_report(uint8_t report_type, uint8_t report_id,
                           const uint8_t* data, size_t data_len,
                           uint32_t timeout_ms = 1000) override;

    esp_err_t get_string_descriptor(uint8_t string_index,
                                  std::string& result) override;

    std::string get_last_error() const override;

    // Interrupt IN endpoint support
    esp_err_t start_interrupt_in() override;
    esp_err_t stop_interrupt_in() override;
    bool read_interrupt_report(uint8_t* data, size_t* len, uint32_t timeout_ms) override;

private:
    // USB device structure
    struct UsbDevice {
        usb_host_client_handle_t client_hdl{nullptr};
        usb_device_handle_t dev_hdl{nullptr};
        uint8_t address{0};
        uint8_t interface_num{0};
        uint8_t ep_in{0};
        uint8_t ep_out{0};
        uint16_t vendor_id{0};
        uint16_t product_id{0};
        uint16_t max_packet_size_in{0};
        uint16_t max_packet_size_out{0};
        usb_speed_t speed{USB_SPEED_LOW};
    };
    
    UsbDevice device_;
    mutable std::mutex device_mutex_;
    std::atomic<bool> connected_{false};
    std::atomic<bool> initialized_{false};
    
    // USB Host Library management
    TaskHandle_t usb_lib_task_handle_{nullptr};
    TaskHandle_t usb_client_task_handle_{nullptr};
    std::atomic<bool> usb_tasks_running_{false};
    
    // Error handling
    mutable std::mutex error_mutex_;
    std::string last_error_;

    // Interrupt IN endpoint support
    struct InterruptReportItem {
        uint8_t len;
        uint8_t data[8];
    };
    QueueHandle_t interrupt_queue_{nullptr};
    // Use atomic pointer so the stale-callback guard in interrupt_in_callback()
    // can safely compare it against the arriving transfer without a mutex.
    std::atomic<usb_transfer_t*> interrupt_transfer_{nullptr};
    std::atomic<bool> interrupt_in_active_{false};
    static void interrupt_in_callback(usb_transfer_t* transfer);
    static void fire_and_forget_ctrl_cb(usb_transfer_t* transfer);

    // Private methods
    static void usb_lib_task(void* arg);
    static void usb_client_task(void* arg);
    static void usb_client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg);
    
    void handle_new_device(uint8_t dev_addr);
    void handle_device_gone(usb_device_handle_t dev_hdl);
    
    esp_err_t setup_usb_host();
    esp_err_t teardown_usb_host();
    esp_err_t find_and_open_device();
    esp_err_t claim_interface();
    esp_err_t find_endpoints();
    // HID class initialisation – sent immediately after interface claim so
    // the device starts pushing interrupt IN reports (required by most HID UPS).
    esp_err_t send_hid_set_idle();
    esp_err_t read_hid_report_descriptor();
    
    void set_last_error(const std::string& error);
    esp_err_t submit_control_transfer(uint8_t bmRequestType, uint8_t bRequest,
                                    uint16_t wValue, uint16_t wIndex,
                                    uint8_t* data, size_t data_len,
                                    uint32_t timeout_ms);
};

} // namespace ups_hid
} // namespace esphome

#endif // USE_ESP32