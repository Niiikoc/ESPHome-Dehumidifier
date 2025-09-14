#include "midea_dehum.h"

// ============ Publishing helpers ============
void MideaDehumComponent::publish_current_humidity(float v) {
// ID in YAML: dehum_current_humidity
id(dehum_current_humidity).publish_state((float) v);
}


void MideaDehumComponent::publish_error(const std::string &err) {
id(dehum_error_code).publish_state(err);
}


void MideaDehumComponent::publish_tank_full(bool f) {
id(dehum_tank_full).publish_state(f);
}


// ============ Protocol-specific parsing & building ============


void MideaDehumComponent::parse_frame(const std::vector<uint8_t> &frame) {
// TODO: Implement the frame decoding exactly as in Hypfer's repo.
// Example pseudocode that you should replace with the real parser:
// - find header
// - verify checksum
// - extract fields: power, mode, fan, setpoint, humidity, ion, swing, tank, error
// - call publish_current_humidity(), publish_tank_full(), publish_error(), and update local state


ESP_LOGD("midea_dehum", "parse_frame called, %d bytes", frame.size());


// --- Example of publishing a dummy reading (remove when implementing real parser)
// publish_current_humidity(45.0);
// publish_tank_full(false);
// publish_error("ok");


// Replace the above with actual parsing and publication.
}


std::vector<uint8_t> MideaDehumComponent::build_power_command(bool on) {
// TODO: implement according to protocol
std::vector<uint8_t> cmd;
// Example placeholder (NOT the real protocol):
// cmd = {0xAA, 0x01, (uint8_t)(on ? 1 : 0), checksum};
return cmd;
}


std::vector<uint8_t> MideaDehumComponent::build_swing_command(bool on) {
std::vector<uint8_t> cmd;
// TODO
return cmd;
}


std::vector<uint8_t> MideaDehumComponent::build_ion_command(bool on) {
std::vector<uint8_t> cmd;
// TODO
return cmd;
}


std::vector<uint8_t> MideaDehumComponent::build_mode_command(const std::string &mode) {
std::vector<uint8_t> cmd;
// TODO: map mode string to protocol values
return cmd;
}


std::vector<uint8_t> MideaDehumComponent::build_fan_command(const std::string &fan) {
std::vector<uint8_t> cmd;
// TODO: map fan string to protocol values
return cmd;
}


std::vector<uint8_t> MideaDehumComponent::build_target_humidity_command(int humidity) {
std::vector<uint8