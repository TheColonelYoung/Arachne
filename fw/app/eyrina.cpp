#include "eyrina.hpp"

Eyrina::Eyrina() :
    Application_thread("eyrina", RTOS::Priority::High, 4096)
{ }

int Eyrina::Thread(vector<string> &args){
    Log_line(Log_levels::Debug, "Eyrina Run");
    if (args.size() > 0) {
        args.erase(args.begin());
    }
    Parse(args);
    return 0;
}

int Eyrina::Init(){
    Log_line(Log_levels::Debug, "Eyrina Init");
    Init_motion();
    Init_light();
    return 0;
}

int Eyrina::Input_load(string input){
    command_buffer.push(input);
    return command_buffer.size();
}

int Eyrina::Load_command(){
    if (blocked_queue) {
        return -1;
    }

    string command = command_buffer.front();

    size_t position;
    vector<string> gcode;

    while ((position = command.find(" ")) != string::npos) {
        string cmd = command.substr(0, position);
        if (cmd.find_first_not_of(' ') != std::string::npos) {
            gcode.emplace_back(cmd);
        }
        command.erase(0, position + 1);
    }
    gcode.emplace_back(command);

    Parse(gcode);

    command_buffer.pop();
    return command_buffer.size();
}

int Eyrina::Parse(vector<string> &gcode){
    string command = "";

    for (auto &section:gcode) {
        command += section + " ";
    }
    Log_line(Log_levels::Debug, "Command: " + command);

    // Save command name and removes it from arguments
    string command_tag = gcode[0];
    if (gcode.size() > 0) {
        gcode.erase(gcode.begin());
    }

    // Create parameters and flags from command arguments
    vector<char> flags;
    map<char, double> params;

    for (auto &param:gcode) {
        if (param.length() == 1) {
            // Flags F
            flags.emplace_back(param[0]);
            Log_line(Log_levels::Debug, param.substr(0, 1));
        } else {
            // Params X-1.2
            double param_value = stod(param.substr(1, param.length() - 1));
            params.emplace(param[0], param_value);
            Log_line(Log_levels::Debug, param.substr(0, 1) + ":" + to_string(stod(param.substr(1, param.length() - 1))));
        }
    }

    // Execute validation and receive pointer to g-code execution method
    const gcode_settings *g_code_method = nullptr;

    int ret_code = Validation(command_tag, params, flags, g_code_method);

    if (ret_code) {
        // Invalid command
        return ret_code;
    } else {
        // Execute method of g-code from command settings
        return (*this.*(*g_code_method).method)(params, flags);
    }
}  // Eyrina::Parse

int Eyrina::Validation(string &command, map<char, double> &params, vector<char> &flags, const gcode_settings *&g_code_method){
    // Checks if commands tag exists in defined commands
    auto iterator = find_if(g_code_commands.begin(), g_code_commands.end(),
        [&command] (const pair<string, gcode_settings> &g_code_command){
        return g_code_command.first == command;
    }
    );

    if (iterator == g_code_commands.end()) {
        // Command does not exists
        Log_line(Log_levels::Debug, "Invalid command");
        return -1;
    }

    const gcode_settings gcode_structure = (*iterator).second;

    // Check if command has valid parameters
    for (auto &param:params) {
        auto position = find(gcode_structure.params.begin(), gcode_structure.params.end(), param.first);
        if (position == gcode_structure.params.end()) {
            // Param does not exist
            Log_line(Log_levels::Debug, "Invalid parameter");
            return -2;
        }
    }

    // Check if command has valid flags
    for (auto &flag:flags) {
        auto position = find(gcode_structure.flags.begin(), gcode_structure.flags.end(), flag);
        if (position == gcode_structure.flags.end()) {
            // Flag does not exist
            Log_line(Log_levels::Debug, "Invalid flag");
            return -3;
        }
    }

    Log_line(Log_levels::Debug, "Command is valid");

    g_code_method = &gcode_structure;

    return 0;
}  // Eyrina::Validation

int Eyrina::Add_axis(char axis_name, Motion_axis_L6470 *new_axis){
    if (axis.count(axis_name)) {
        Log_line(Log_levels::Error, "Axis with this identifier already exists");
        return -1;
    }
    axis.insert({ axis_name, new_axis });
    return 0;
}

int Eyrina::Diagnostic(){
    return 0;
}

int Eyrina::G_code_G0(map<char, double> &params, vector<char> &flags){
    int moving_axis = 0;
    for (auto &axis_command:params) {
        // First is tag of axis, second is distance to move
        if (axis.find(axis_command.first) != axis.end()) {
            Log_line(Log_levels::Notice, string("Moving axis ") + axis_command.first);
            axis[axis_command.first]->GoTo(axis_command.second);
            moving_axis += 1;
        }
    }
    return moving_axis;
}

int Eyrina::G_code_G1(map<char, double> &params, vector<char> &flags){
    int moving_axis = 0;
    for (auto &axis_command:params) {
        // First is tag of axis, second is distance to move
        if (axis.find(axis_command.first) != axis.end()) {
            Log_line(Log_levels::Notice, string("Shifting axis ") + axis_command.first);
            axis[axis_command.first]->Move(axis_command.second);
            moving_axis += 1;
        }
    }
    return moving_axis;
}

int Eyrina::G_code_G2(map<char, double> &params, vector<char> &flags){
    int moving_axis = 0;
    for (auto &axis_command:params) {
        // First is tag of axis, second is speed of movement
        if (axis.find(axis_command.first) != axis.end()) {
            Log_line(Log_levels::Notice, string("Running axis ") + axis_command.first + " at speed: " + to_string(axis_command.second));
            axis[axis_command.first]->Run(axis_command.second);
            moving_axis += 1;
        }
    }
    return moving_axis;
}

int Eyrina::G_code_G28(map<char, double> &params, vector<char> &flags){
    if (flags.size() == 0) {
        Log_line(Log_levels::Notice, "Homing all axis");
        for (auto &elem:axis) {
            elem.second->Home();
        }
        return axis.size();
    } else {
        int moving_axis = 0;
        for (auto &axis_label:flags) {
            if (axis.find(axis_label) != axis.end()) {
                Log_line(Log_levels::Notice, "Homing axis " + axis_label);
                axis[axis_label]->Home();
                moving_axis += 1;
            }
        }
        return moving_axis;
    }
}

int Eyrina::G_code_M0(map<char, double> &params, vector<char> &flags){
    if (flags.size() == 0) {
        Log_line(Log_levels::Notice, "Soft stoping all axis");
        for (auto &elem:axis) {
            elem.second->Soft_stop();
        }
        return axis.size();
    } else {
        int moving_axis = 0;
        for (auto &axis_label:flags) {
            if (axis.find(axis_label) != axis.end()) {
                Log_line(Log_levels::Notice, string("Soft stoping axis ") + axis_label);
                axis[axis_label]->Soft_stop();
                moving_axis += 1;
            }
        }
        return moving_axis;
    }
}

int Eyrina::G_code_M10(map<char, double> &params, vector<char> &flags){
    if (flags.size() == 0) {
        Log_line(Log_levels::Notice, "Disabling all axis");
        for (auto &elem:axis) {
            elem.second->Release();
        }
        return axis.size();
    } else {
        int moving_axis = 0;
        for (auto &axis_label:flags) {
            if (axis.find(axis_label) != axis.end()) {
                Log_line(Log_levels::Notice, string("Disabling axis ") + axis_label);
                axis[axis_label]->Release();
                moving_axis += 1;
            }
        }
        return moving_axis;
    }
}

int Eyrina::G_code_E1(map<char, double> &params, vector<char> &flags){
    double intensity = params.at('I');
    double index_d   = params.at('C');
    int index        = static_cast<int>(params.at('C'));
    index = static_cast<int>(index_d);

    if ((index > 3) | (index < 1)) {
        return -1;
    }

    if ((intensity > 100) | (intensity < 0)) {
        return -1;
    }

    intensity = led_channels[index - 1]->Power(intensity);

    unsigned int current = led_channels[index - 1]->Current() / 1000;

    Log_line(Log_levels::Debug, "Setting LED channel " + to_string(index) + " to intensity of " + to_string(static_cast<int>(intensity)) + "% (" + to_string(current) + " mA)");
    return 0;
}

int Eyrina::G_code_E2(map<char, double> &params, vector<char> &flags){
    if (display_status) {
        display_status = false;
        display->Off();
    } else {
        display_status = true;
        display->On();
    }
    return 0;
}

int Eyrina::G_code_E3(map<char, double> &params, vector<char> &flags){
    if ((params.at('X') > 5) | (params.at('X') < 1)) {
        return -1;
    }

    if ((params.at('Y') > 5) | (params.at('Y') < 1)) {
        return -1;
    }

    const int base_position_x = 33;
    const int base_position_y = 1;

    const int step = 15;

    int position_x = params.at('X') - 1 * step + base_position_x;
    int position_y = params.at('Y') - 1 * step + base_position_y;

    display->Put(position_x, position_y);
    display->Put(position_x, position_y + 1);
    display->Put(position_x + 1, position_y);
    display->Put(position_x + 1, position_y + 1);

    return params.at('Y') * 5 + params.at('X');
}

int Eyrina::G_code_R0(map<char, double> &params, vector<char> &flags){
    Log_line(Log_levels::Notice, "Emergency stop");
    for (auto &elem:axis) {
        elem.second->Hard_stop();
    }
    return axis.size();
}

int Eyrina::G_code_R10(map<char, double> &params, vector<char> &flags){
    blocked_queue = false;
    return 0;
}

void Eyrina::Init_light(){
    Log_line(Log_levels::Debug, "Eyrina light initialization");
}

void Eyrina::Init_motion(){
    Log_line(Log_levels::Debug, "Eyrina motor initialization");

    auto chip_selects = Setup_GPIO_expanders();

    Setup_motors(device()->mcu->SPI_2, chip_selects, Eyrina_config::motor_configurations);

    Setup_axis(Eyrina_config::axis_configurations);

    for (auto &motor:stepper_drivers) {
        motor->Status();
        motor->Init();
    }

}  // Eyrina::Init_motors

void Eyrina::Setup_motors(SPI_master *spi, vector<Pin *> &chip_select, const std::array<Eyrina_config::motor_config, 8> &motor_settings){
    for (unsigned int i = 0; i < chip_select.size(); i++) {
        auto &config = motor_settings[i];
        auto driver  = new L6470(*spi, chip_select[i]);
        stepper_drivers[i] = driver;

        driver->Reset();
        driver->Autotune(config.supply_voltage, config.target_current, config.coil_impedance,
          config.coil_inductance, config.electric_constant, 0.4, 1.2);
        driver->Microsteps(config.microsteps);
        driver->Acceleration(config.acceleration);
        driver->Deceleration(config.deceleration);
        driver->Min_speed(config.min_speed, config.low_speed_optimization);
        driver->Max_speed(config.max_speed);
        driver->Overcurrent(config.overcurrent);
        driver->Stall(config.stall_current);
        driver->Full_step_optimization(config.fullstep_optimization);
    }
}

void Eyrina::Setup_axis(const std::array<Eyrina_config::axis_config, 8> axis_settings){
    for (unsigned int i = 0; i < 1; i++) {
        Eyrina_config::axis_config config = axis_settings.at(0);
        auto motion_axis = new Motion_axis_L6470(stepper_drivers[i], config.fullstep_movement, config.max_position, config.homing_speed, config.homing_direction, config.min_position);
        Add_axis(config.axis_name, motion_axis);
    }
}

vector<Pin *> Eyrina::Setup_GPIO_expanders(){
    // Configuration of pins on GPIO expander to fit SPI chip select definition
    vector<Pin *> chip_selects{
        new Pin('A', 4),
    };

    return chip_selects;
}  // Eyrina::Setup_GPIO_expanders
