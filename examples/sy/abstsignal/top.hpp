#include <forsyde.hpp>

using namespace sc_core;
using namespace ForSyDe;


void report_func(float inp1)
{
    std::cout << "output value: " << inp1 << std::endl;
}

SC_MODULE(top)

{   
    SY::signal <float> out;
    std::vector<float> s1 = {36.7, 36.8, 36.7, 36.8, 36.9, 36.9, 37.0, 37.0, 37.1, 37.2, 37.3, 37.2, 37.3, 37.3, 37.4, 37.5, 37.6, 36.6};

    SC_CTOR(top)
    {   
        std::cout<<s1<<std::endl;
        auto abstsig = new SY::signalabst<float> ("abstsig", s1, 3);
        abstsig-> oport(out);
        SY::make_ssink("report1", report_func, out);
    }
};

