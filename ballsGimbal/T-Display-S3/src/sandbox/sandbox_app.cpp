#include "core/application.h"

#include <Arduino.h>
#include <vector>
#include "BLEDevice.h"

#include "hardware/display.h"
#include "hardware/battery.h"

static const size_t initial_balls = 5;

// The remote service we wish to connect to.
static BLEUUID serviceUUID("0000ffff-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we are interested in.
//static BLEUUID    charUUID("0000ff01-0000-1000-8000-00805f9b34fb");
static BLEUUID    charUUID("0000ff01-0000-1000-8000-00805f9b34fb");
static BLEUUID    charRxUUID("0000ff02-0000-1000-8000-00805f9b34fb");

static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLERemoteCharacteristic* pRemoteCharacteristicRx;
static BLEAdvertisedDevice* myDevice;
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;


struct ball
{
    lv_obj_t *obj_handle;

    struct
    {
        float x;
        float y;
    } position;

    struct
    {
        float x;
        float y;
    } velocity;
};

class sandbox : public application
{
public:
    sandbox() : m_width(hardware::display::get().width()),
                m_height(hardware::display::get().height()),
                m_screen(lv_scr_act())
    {
        lv_group_t *group = lv_group_create();
        lv_indev_t *indev = nullptr;

        while (indev = lv_indev_get_next(indev))
            if (lv_indev_get_type(indev) == LV_INDEV_TYPE_KEYPAD)
                lv_indev_set_group(indev, group);

        lv_group_add_obj(group, m_screen);

        auto on_key = [](lv_event_t *e)
        {
            auto app = static_cast<sandbox *>(lv_event_get_user_data(e));
            auto key = lv_event_get_key(e);

            switch (key)
            {
            case LV_KEY_UP:
                app->add_ball();
                break;
            case LV_KEY_DOWN:
                //app->remove_ball();
                app->scan_ble();
                break;
            case LV_KEY_ENTER:
                app->reset_balls();
                break;
            default:
                break;
            }
        };

        lv_obj_add_event_cb(m_screen, on_key, LV_EVENT_KEY, this);
        Serial.begin(115200);
        Serial.println("Starting Arduino BLE Client application...");
    }

    ~sandbox()
    {
        while (m_balls.size())
            remove_ball();
    }

    void on_create() override
    {
        lv_obj_clear_flag(m_screen, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(m_screen, lv_color_black(), LV_STATE_DEFAULT);

        m_battery_voltage = lv_label_create(lv_layer_top());

        lv_obj_set_style_text_color(m_battery_voltage, lv_color_white(), LV_STATE_DEFAULT);
        lv_obj_align(m_battery_voltage, LV_ALIGN_BOTTOM_LEFT, 4, -22);

        m_ball_count = lv_label_create(lv_layer_top());

        lv_obj_set_style_text_color(m_ball_count, lv_color_white(), LV_STATE_DEFAULT);
        lv_obj_align(m_ball_count, LV_ALIGN_BOTTOM_LEFT, 4, -4);

        m_wifi = lv_label_create(lv_layer_top());

        lv_obj_set_style_text_color(m_wifi, lv_color_white(), LV_STATE_DEFAULT);
        lv_obj_align(m_wifi, LV_ALIGN_BOTTOM_LEFT, 4, -48);

        m_balls.reserve(initial_balls);

        reset_balls();

        for (size_t i = 0; i < 10; i++)
            m_voltage_level = 0.9f * m_voltage_level + 0.1f * hardware::battery::get().voltage_level();

        auto hud_update = [](lv_timer_t *timer)
        {
            static_cast<sandbox *>(timer->user_data)->update_hud();
        };

        lv_timer_create(hud_update, 200, this);
    }

    void on_update(float timestep) override
    {
        const auto max_x = m_width - 32;
        const auto max_y = m_height - 32;

        for (const auto ball : m_balls)
        {
            const auto flip_vx = (ball->position.x < 0 && ball->velocity.x < 0) || (ball->position.x > max_x && ball->velocity.x > 0);
            const auto flip_vy = (ball->position.y < 0 && ball->velocity.y < 0) || (ball->position.y > max_y && ball->velocity.y > 0);

            ball->velocity.x = flip_vx ? -ball->velocity.x : ball->velocity.x;
            ball->velocity.y = flip_vy ? -ball->velocity.y : ball->velocity.y;

            ball->position.x += ball->velocity.x * timestep;
            ball->position.y += ball->velocity.y * timestep;

            lv_obj_set_pos(ball->obj_handle, ball->position.x, ball->position.y);
        }
    }

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks


    void scan_ble()
    {
        BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
        BLEScan* pBLEScan = BLEDevice::getScan();
        pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
        pBLEScan->setInterval(1349);
        pBLEScan->setWindow(449);
        pBLEScan->setActiveScan(true);
        pBLEScan->start(5, false);
    }

    void add_ball()
    {
        Serial.println("Starting Arduino BLE Client application...");


        auto b = static_cast<ball *>(lv_mem_alloc(sizeof(ball)));

        b->obj_handle = lv_obj_create(m_screen);

        b->position.x = (m_width / 2) - 16;
        b->position.y = (m_height / 2) - 16;
        b->velocity.x = lv_rand(50, 150);
        b->velocity.y = lv_rand(50, 150);

        if (lv_rand(0, 1))
            b->velocity.x = -b->velocity.x;

        if (lv_rand(0, 1))
            b->velocity.y = -b->velocity.y;

        lv_obj_set_size(b->obj_handle, 32, 32);
        lv_obj_set_pos(b->obj_handle, b->position.x, b->position.y);

        lv_obj_set_style_radius(b->obj_handle, 16, LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(b->obj_handle, 0, LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(b->obj_handle, lv_color_make(lv_rand(0, 255), lv_rand(0, 255), lv_rand(0, 255)), LV_STATE_DEFAULT);

        m_balls.push_back(b);
    }

    void remove_ball()
    {
        if (!m_balls.size())
            return;

        auto b = m_balls.back();

        m_balls.pop_back();

        lv_obj_del(b->obj_handle);
        lv_mem_free(b);
    }

    void reset_balls()
    {
        if (m_timer)
            return;

        auto timer_cb = [](lv_timer_t *timer)
        {
            auto app = static_cast<sandbox *>(timer->user_data);

            if (app->m_balls.size() == initial_balls)
            {
                lv_timer_del(app->m_timer);

                app->m_timer = nullptr;

                return;
            }

            if (app->m_balls.size() < initial_balls)
                app->add_ball();
            else
                app->remove_ball();
        };

        m_timer = lv_timer_create(timer_cb, 100, this);
    }

    void update_hud()
    {
        m_voltage_level = 0.9f * m_voltage_level + 0.1f * hardware::battery::get().voltage_level();

        lv_label_set_text_fmt(m_battery_voltage, "Battery: %umv", m_voltage_level);
        lv_label_set_text_fmt(m_ball_count, "Ballums: %zu", m_balls.size());
        lv_label_set_text_fmt(m_wifi, "Gollum: %zu", 424);
    }

private:
    const uint16_t m_width;
    const uint16_t m_height;

    lv_obj_t *m_screen;
    lv_obj_t *m_battery_voltage;
    lv_obj_t *m_ball_count;
    lv_obj_t *m_wifi;

    lv_timer_t *m_timer = nullptr;

    std::vector<ball *> m_balls;
    uint32_t m_voltage_level = 0;
};

application *create_application()
{
    return new sandbox();
}