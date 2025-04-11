/*
 * OpenSCAD Model for ESP32 to DHT11 Connection with Pull-up Resistor (v3)
 *
 * - Corrected resistor orientation.
 * - Increased label Z-offset further to combat Z-fighting.
 * - Pull-up resistor connects DHT11 DATA pin to ESP32 3V3 pin.
 * - Note: Final render (F6) colors might differ from preview (F5).
 *         Export PNG from preview if colors are preferred.
 */

$fn = 50;

// --- Dimensions & Parameters ---
esp32_width = 25;
esp32_depth = 50;
esp32_height = 5;
dht11_width = 15;
dht11_depth = 30;
dht11_height = 5;
pin_radius = 0.8;
pin_height = 3;
wire_radius = 0.5;
resistor_radius = 1.5;
resistor_length = 8;
resistor_lead_length = 3;
spacing = 20;
label_size = 2;
label_height = 0.2; // Keep labels thin
label_z_clearance = 1.5; // How high labels float above pin tops

// Define colors explicitly
color_esp32 = "DarkGreen";
color_dht11 = "LightBlue";
color_pin = "Silver";
color_wire_vcc = "Red";
color_wire_gnd = "Black";
color_wire_data = "Yellow";
color_wire_pullup = "Orange";
color_resistor_body = "Tan";
color_resistor_lead = "DimGray";
color_label = "Black";
color_white = "Green";


// --- Pin Locations (relative to component center [0,0]) ---
esp_pin_x = esp32_width / 2;
esp_vcc_pos = [esp_pin_x, 10, esp32_height];
esp_gpio4_pos = [esp_pin_x, 0, esp32_height];
esp_gnd_pos = [esp_pin_x, -10, esp32_height];
esp_label_pos = [esp_pin_x, -20, esp32_height];

dht_pin_x = -dht11_width / 2;
dht_vcc_pos = [dht_pin_x, 10, dht11_height];
dht_data_pos = [dht_pin_x, 0, dht11_height];
dht_gnd_pos = [dht_pin_x, -10, dht11_height];
dht_label_pos = [dht_pin_x, -20, dht11_height];


// --- Component Modules ---
module esp32_board() {
    color(color_esp32) cube([esp32_width, esp32_depth, esp32_height], center=true);
    color(color_pin) translate(esp_vcc_pos) cylinder(h=pin_height, r=pin_radius);
    color(color_pin) translate(esp_gpio4_pos) cylinder(h=pin_height, r=pin_radius);
    color(color_pin) translate(esp_gnd_pos) cylinder(h=pin_height, r=pin_radius);
}

module dht11_module() {
    color(color_dht11) cube([dht11_width, dht11_depth, dht11_height], center=true);
    color(color_pin) translate(dht_vcc_pos) cylinder(h=pin_height, r=pin_radius);
    color(color_pin) translate(dht_data_pos) cylinder(h=pin_height, r=pin_radius);
    color(color_pin) translate(dht_gnd_pos) cylinder(h=pin_height, r=pin_radius);
}

// --- Resistor Module (Corrected Rotation) ---
module resistor(orientation=[0,0,0]) { // Default no rotation
     rotate(orientation) {
        // Body aligned with Z axis *before* rotation
        color(color_resistor_body) cylinder(h = resistor_length, r=resistor_radius, center=true);
        // Leads aligned with Z axis *before* rotation
        color(color_resistor_lead) translate([0, 0, resistor_length/2])
            cylinder(h=resistor_lead_length, r=wire_radius, center=true);
        color(color_resistor_lead) translate([0, 0, -resistor_length/2])
            cylinder(h=resistor_lead_length, r=wire_radius, center=true);
    }
}

// --- Wire Module ---
module wire(p1, p2, wire_color="Gray") {
    color(wire_color) hull() {
        translate(p1) sphere(r=wire_radius);
        translate(p2) sphere(r=wire_radius);
    }
}

// --- Label Module (Increased Z Offset) ---
module label(pos, txt, label_color="Black") {
     // Calculate position floating above the base pin position
     translate(pos + [0,0, label_z_clearance])
         linear_extrude(height = label_height)
            color(label_color) text(txt, size=label_size, halign="center", valign="center", font="Liberation Sans:style=Bold");
}

// --- Assembly ---
esp32_offset_x = -(spacing/2 + esp32_width/2);
dht11_offset_x = spacing/2 + dht11_width/2;

// Place ESP32
translate([esp32_offset_x, 0, esp32_height/2]) esp32_board();

// Place DHT11
translate([dht11_offset_x, 0, dht11_height/2]) dht11_module();

// Absolute Pin Positions (Top of pin cylinder)
base_z = esp32_height; // Use board height as reference
abs_esp_vcc_pin_top = [esp32_offset_x + esp_vcc_pos[0], esp_vcc_pos[1], base_z + pin_height];
abs_esp_gpio4_pin_top = [esp32_offset_x + esp_gpio4_pos[0], esp_gpio4_pos[1], base_z + pin_height];
abs_esp_gnd_pin_top = [esp32_offset_x + esp_gnd_pos[0], esp_gnd_pos[1], base_z + pin_height];

abs_dht_vcc_pin_top = [dht11_offset_x + dht_vcc_pos[0], dht_vcc_pos[1], base_z + pin_height];
abs_dht_data_pin_top = [dht11_offset_x + dht_data_pos[0], dht_data_pos[1], base_z + pin_height];
abs_dht_gnd_pin_top = [dht11_offset_x + dht_gnd_pos[0], dht_gnd_pos[1], base_z + pin_height];

// --- Place Resistor (Rotated to be roughly horizontal) ---
resistor_placement_z = base_z + pin_height + 0;
resistor_center_x = 0; // Centered between modules
resistor_center_y = abs_dht_data_pin_top[1] + 5; // Slightly above data pin line
resistor_orientation = [0, 90, 90]; // Rotate 90 deg around Y to make it horizontal along X

translate([resistor_center_x, resistor_center_y, resistor_placement_z])
    resistor(orientation=resistor_orientation);

// Calculate resistor lead connection points *after* considering rotation
// Body is now along X axis, leads extend along X from the ends.
resistor_p1_connect = [resistor_center_x - resistor_length/2 - resistor_lead_length, resistor_center_y, resistor_placement_z]; // Left lead end
resistor_p2_connect = [resistor_center_x + resistor_length/2 + resistor_lead_length, resistor_center_y, resistor_placement_z]; // Right lead end

// --- Draw Wires ---
wire(abs_esp_vcc_pin_top, abs_dht_vcc_pin_top, color_wire_vcc);
wire(abs_esp_gnd_pin_top, abs_dht_gnd_pin_top, color_wire_gnd);
wire(abs_esp_gpio4_pin_top, abs_dht_data_pin_top, color_wire_data);

// Pull-up Resistor Wires
//wire(abs_dht_data_pin_top, resistor_p1_connect, color_wire_pullup); // Data pin to left lead
//wire(resistor_p2_connect, abs_esp_vcc_pin_top, color_wire_pullup); // Right lead to ESP32 VCC pin

// --- Add Labels (Using pin top Z + clearance) ---
label_base_z = base_z + pin_height; // Z level of the top of the pins

label([esp32_offset_x + esp_vcc_pos[0] - 5, esp_vcc_pos[1], label_base_z], "3V3", color_label);
label([esp32_offset_x + esp_gpio4_pos[0] - 5, esp_gpio4_pos[1], label_base_z], "GPIO4", color_label);
label([esp32_offset_x + esp_gnd_pos[0] - 5, esp_gnd_pos[1], label_base_z], "GND", color_label);
label([esp32_offset_x + esp_label_pos[0] - 10, esp_label_pos[1], label_base_z], "ESP32", color_label);

label([dht11_offset_x + dht_vcc_pos[0] + 5, dht_vcc_pos[1], label_base_z], "VCC", color_label);
label([dht11_offset_x + dht_data_pos[0] + 5.5, dht_data_pos[1], label_base_z], "DATA", color_label);
label([dht11_offset_x + dht_gnd_pos[0] + 5, dht_gnd_pos[1], label_base_z], "GND", color_label);
label([dht11_offset_x + dht_label_pos[0] + 9, esp_label_pos[1] + 4, label_base_z], "DHT11", color_label);

color(color_white) cube([esp32_width * 4, esp32_depth * 2, esp32_height],center=true);
label([resistor_center_x + 6, resistor_center_y + 14, resistor_placement_z], "R_pullup\n(4.7k-10k)", color_label);