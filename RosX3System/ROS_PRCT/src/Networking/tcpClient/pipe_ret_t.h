#ifndef PIPE_RET_T_H
#define PIPE_RET_T_H

class pipe_ret_t {

private:

    bool _successFlag = false;
    std::string _msg = "";

public:

    pipe_ret_t() = default;
    pipe_ret_t(bool successFlag, const std::string &msg) :
        _successFlag{successFlag},
        _msg{msg}
    {}

    std::string message() const { return _msg; }
    bool isSuccessful() const { return _successFlag; }

    static pipe_ret_t failure(const std::string & msg);
    static pipe_ret_t success(const std::string &msg = "");
};

#endif // !PIPE_RET_T_H