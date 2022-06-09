#include <forsyde.hpp>

using namespace ForSyDe;


    void comb_func (std::vector<float>& out, const std::vector<float>& inp )
    {

        out.push_back( (inp [0] + inp [1] + inp [2] ) /3 );

        // if ( (s1[1] + s1 [2] + s1[3])/3  < 35.5 )
        //     output = 1 ;
        // else if ( 35.5 <= (s1[1] + s1[2] + s1[3])/3  < 37.5 )
        //     output = 2 ;
        // else if ( 37.5 <= (s1[1] + s1[2] + s1[3])/3  < 38.5 )
        //     output = 3 ;
        // else if ( 38.5 <= (s1[1] + s1[2] + s1[3])/3)
        //     output = 4 ;
        
    }
