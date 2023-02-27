#pragma once

extern int  parse_cmdline(int, char **, ClGPSParameters *const);
extern void parse_parameter_file(ClGPSParameters *const);
extern void echo_current_parameters(const ClGPSParameters &);
extern void set_default_parameters(ClGPSParameters *const);