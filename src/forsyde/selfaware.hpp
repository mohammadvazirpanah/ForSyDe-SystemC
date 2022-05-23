#ifndef SELFAWAER_HPP
#define SELFAWAER_HPP
#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <functional>

namespace ForSyDe
{
namespace SY
{

using namespace sc_core;

#define WRITE_MULTIPORT(PORT,VAL) \
    for (int WMPi=0;WMPi<PORT.size();WMPi++) PORT[WMPi]->write(VAL);

template <class IN, class OT>
class selfaware : public sc_module
{

public:

    SY_in<IN>  iport;
    SY_out<OT> oport;

    typedef std::function<abst_ext<OT>(const abst_ext<IN>&)> functype;
    SY_in<functype> obsport;
    SY_in<functype> decport;
    SY_in<functype> actport;

    selfaware(sc_module_name _name) : sc_module(_name)
    {
        SC_THREAD(worker);
    }

private:
    SC_HAS_PROCESS(selfaware);

    void worker()
    {

        abst_ext<IN> in_val;
        abst_ext<OT> out_val;
        functype obs_f;
        functype dec_f;
        functype act_f;
        while (1)
        {
            in_val = iport.read();
            obs_f = unsafe_from_abst_ext(obsport.read());
            dec_f = unsafe_from_abst_ext(decport.read());
            act_f = unsafe_from_abst_ext(actport.read());
            out_val = act_f(in_val);
            WRITE_MULTIPORT(oport,out_val);
        }
        

    }



};



template <class T0, template <class> class OIf,
          class T1, template <class> class I1If,
                    template <class> class FIf>
inline apply<T0,T1>* make_apply(const std::string& pName,
    OIf<T0>& outS,
    I1If<T1>& inp1S,
    FIf<typename apply<T0,T1>::functype>& fS
    )
{
    auto p = new apply<T0,T1>(pName.c_str());
    
    (*p).iport(inp1S);
    (*p).oport(outS);
    (*p).fport(fS);
    
    return p;
}
}
}

#endif