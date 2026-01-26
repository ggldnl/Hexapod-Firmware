#ifndef COMMAND_HPP
#define COMMAND_HPP

#include <stdint.h>
#include <cstring>


// Each concrete command must implement the execute method
class Command {

public:

    virtual ~Command() {};

    /*
    * Execute the command with given arguments
    * args: pointer to argument buffer
    * args_len: length of arguments
    * response: buffer to write response data
    * response_len: output parameter - length of response written
    *
    * Returns: 0 on success, error code otherwise
    */
    virtual bool execute(const uint8_t* args, uint8_t args_len, 
                       uint8_t* response, uint8_t* response_len) = 0;
};

#endif // COMMAND_HPP
