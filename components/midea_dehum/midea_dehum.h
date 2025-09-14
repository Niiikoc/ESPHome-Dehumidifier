#pragma once


#include "esphome.h"

class MideaDehumComponent : public Component, public UARTDevice {
public:
explicit MideaDehumComponent(UARTComponent *parent) : UARTDevice(parent) {}


void setup() override;
void loop() override;


// Called from YAML lambdas
void set_power(bool on);
void set_swing(bool on);
void set_ion(bool on);
void set_mode(const char *mode);
void set_fan(const char *fan);
void set_target_humidity(int humidity);


// Helpers for publishing to entities defined in YAML (ids)
void publish_current_humidity(float v);
void publish_error(const std::string &err);
void publish_tank_full(bool f);


protected:
// Parsing helpers -- implement protocol specifics in .cpp
void parse_frame(const std::vector<uint8_t> &frame);


// Build commands (protocol-dependent)
std::vector<uint8_t> build_power_command(bool on);
std::vector<uint8_t> build_swing_command(bool on);
std::vector<uint8_t> build_ion_command(bool on);
std::vector<uint8_t> build_mode_command(const std::string &mode);
std::vector<uint8_t> build_fan_command(const std::string &fan);
std::vector<uint8_t> build_target_humidity_command(int humidity);


// Buffer for incoming bytes
std::vector<uint8_t> rx_buffer_;
};


// Create an id that can be referenced in YAML lambdas
extern MideaDehumComponent *midea_dehum_comp;