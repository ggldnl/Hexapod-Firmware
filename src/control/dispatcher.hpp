#ifndef DISPATCHER_HPP
#define DISPATCHER_HPP

#include "command.hpp"


class Dispatcher {

private:

    static const size_t MAX_COMMANDS = 32;
    
    struct CommandEntry {
        uint8_t opcode;
        Command* command;
    };
    
    CommandEntry commands[MAX_COMMANDS];
    size_t command_count;

public:
    
    Dispatcher() : command_count(0) {}
    
    // Register a command with an opcode
    bool registerCommand(uint8_t opcode, Command* command) {

        if (command_count >= MAX_COMMANDS) {
            return false;
        }
        
        commands[command_count].opcode = opcode;
        commands[command_count].command = command;
        command_count++;
        return true;
    }
    
    // Dispatch a command from opcode, data, data_len
    bool dispatch(uint8_t opcode, const uint8_t* data, uint8_t data_len,
                uint8_t* response, uint8_t* response_len) {

        // Find and execute the command
        for (size_t i = 0; i < command_count; i++) {
            if (commands[i].opcode == opcode) {
                return commands[i].command->execute(data, data_len, 
                                                    response, response_len);
            }
        }
        
        // Error
        return false;
    }
};

#endif  // DISPATCHER_HPP