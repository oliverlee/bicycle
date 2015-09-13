#pragma once

class NotImplementedException : public std::exception {
    public:
        const char* what() const noexcept {
            return "Functionality not yet implemented";
        }

}; // class NotImplementedException
