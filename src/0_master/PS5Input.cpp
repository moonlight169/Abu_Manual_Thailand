#include "PS5Input.h"
#include "config_esp32.h"
#include "esp_bt.h"

void PS5Input::begin(){
    ps5.begin(MAC_PS5_WHITE);

    esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9); 

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
}

void PS5Input::update(){
    unsigned long now = millis();

    if ((now - prev_control_time) >= (1000 / COMMAND_RATE)) {
        prev_control_time = now;

        prevButtons = currentButtons;
        currentButtons = ps5.isConnected() ? readRawButtons() : 0;

        if (ps5.isConnected()){
            float x = 0.000;
            float y = 0.000;
            float w = 0.000;

            if (ps5.Up())    x += 1.000;
            if (ps5.Down())  x -= 1.000;
            if (ps5.Right()) y += 1.000;
            if (ps5.Left())  y -= 1.000;

            if (ps5.R1()) w += 1.000;
            if (ps5.L1()) w -= 1.000;

            velocity.valX = x;
            velocity.valY = y;
            velocity.valW = w;
        } else {
            velocity.valX = 0.000;
            velocity.valY = 0.000;
            velocity.valW = 0.000;
        }
    }
}

uint16_t PS5Input::readRawButtons(){
    uint16_t buttons = 0;

    if (ps5.Cross())    buttons |= (1 << Cross);
    if (ps5.Circle())   buttons |= (1 << Circle);
    if (ps5.Triangle()) buttons |= (1 << Triangle);
    if (ps5.Square())   buttons |= (1 << Square);
    if (ps5.L1())       buttons |= (1 << L1);
    if (ps5.R1())       buttons |= (1 << R1);
    if (ps5.Up())       buttons |= (1 << Up);
    if (ps5.Down())     buttons |= (1 << Down);
    if (ps5.Left())     buttons |= (1 << Left);
    if (ps5.Right())    buttons |= (1 << Right);

    return buttons;
}

bool PS5Input::isPressed(PS5Button btn){
    return currentButtons & (1 << btn);
}

bool PS5Input::wasPressed(PS5Button btn){
    return (currentButtons & (1 << btn)) && !(prevButtons & (1 << btn));
}

PS5Input::cmd_vel PS5Input::getVelocity(){
    return velocity;
}

bool PS5Input::isConnected(){
    return ps5.isConnected();
}

void PS5Input::printLog(HardwareSerial &serialPort) {
    serialPort.print("X: "); serialPort.print(velocity.valX, 3);
    serialPort.print(" | Y: "); serialPort.print(velocity.valY, 3);
    serialPort.print(" | W: "); serialPort.println(velocity.valW, 3);
}