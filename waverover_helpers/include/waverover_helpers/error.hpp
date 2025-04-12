#pragma once

#include <string>

namespace waverover_helpers
{
    class Error
    {
    public:
        Error()
        {

        }

        Error(std::string msg)
        {
            error_message = msg;
        }

        std::string getErrorMessage() const
        {
            return error_message;
        }
    private:
        std::string error_message = "";
    };

    template <typename T>
    class ErrorOr
    {
    public:
        ErrorOr()
        {

        }

        ErrorOr(T val)
        {
            value = val;
        }

        ErrorOr(Error err)
        {
            error = err;
            is_error = true;
        }

        T getValue() const
        {
            return value;
        }

        Error getError() const
        {
            return error;
        }

        inline bool isError() const
        {
            return is_error;
        }

        inline operator T() const
        {
            return value;
        }

    private:
        Error error;
        bool is_error = false;
        T value;
    };
}

using waverover_helpers::Error;
using waverover_helpers::ErrorOr;
