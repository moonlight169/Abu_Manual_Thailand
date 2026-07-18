#include "PS5Input.h"
#include "esp_bt.h"

PS5Input::PS5Input(const char* mac) : _mac(mac) {
}

void PS5Input::begin(){
    ps5.begin(_mac);

    esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9);

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
}

void PS5Input::update(){
    prevButtons = currentButtons;
    currentButtons = ps5.isConnected() ? readRawButtons() : 0;

}


uint16_t PS5Input::readRawButtons(){
    uint16_t buttons = 0;

    if (ps5.Cross())    buttons |= (1 << Cross);
    if (ps5.Circle())   buttons |= (1 << Circle);
    if (ps5.Triangle()) buttons |= (1 << Triangle);
    if (ps5.Square())   buttons |= (1 << Square);
    if (ps5.L1())       buttons |= (1 << L1);
    if (ps5.R1())       buttons |= (1 << R1);
    if (ps5.L2())       buttons |= (1 << L2);
    if (ps5.R2())     buttons |= (1 << R2);
    if (ps5.Share())     buttons |= (1 << Share);
    if (ps5.Options())    buttons |= (1 << Options);
    if (ps5.Touchpad())    buttons |= (1 << Touchpad);
    if (ps5.Up())     buttons |= (1 << Up);
    if (ps5.Down())     buttons |= (1 << Down);
    if (ps5.Right())    buttons |= (1 << Right);
    if (ps5.Left())    buttons |= (1 << Left);

    return buttons;
}

bool PS5Input::isPressed(PS5Button btn){
    return currentButtons & (1 << btn);
}

bool PS5Input::wasPressed(PS5Button btn){
    return (currentButtons & (1 << btn)) && !(prevButtons & (1 << btn));
}

bool PS5Input::isConnected(){
    return ps5.isConnected();
}

void PS5Input::printLog(HardwareSerial &serialPort) {
    serialPort.print("buttons: 0x"); serialPort.println(currentButtons, HEX);
}