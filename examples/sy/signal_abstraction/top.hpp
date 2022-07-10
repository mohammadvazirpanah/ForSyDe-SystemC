#include <forsyde.hpp>

using namespace sc_core;
using namespace ForSyDe;


void report_func(float inp1)
{
    std::cout << "output value: " << inp1 << std::endl;
}

void abst_func(float& out, const unsigned long& take, const std::vector<float>& inp)
{

 
        float sum = 0;
        for (unsigned long i = 0; i < inp.size(); i++)
        {
            sum += inp[i];
        }
    
        out = (sum/take);
}

SC_MODULE(top)

{   
    SY::signal <float> out_source;
    SY::signal <float> out_asbt;

    std::vector<float> s1 = {36.7, 36.8, 36.7, 36.8, 36.9, 36.9, 37.0, 37.0, 37.1, 37.2, 37.3, 37.2, 37.3, 37.3, 37.4, 37.5, 37.6, 36.6};
    std::vector<float> s2 = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};


    SC_CTOR(top)
    {   
        std::cout<<s2<<std::endl;
        SY::make_svsource ("source", s2, out_source) ;
        auto abstsig = new SY::signalabst <float,float> ("abstsig", 4, abst_func);
        abstsig-> iport1 (out_source);
        abstsig-> oport1(out_asbt);


        SY::make_ssink("report1", report_func, out_asbt);
    }
};

