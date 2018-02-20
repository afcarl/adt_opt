#include <adt/containers/adt_opt_variable.hpp>

ADT_Opt_Variable::ADT_Opt_Variable(){}
ADT_Opt_Variable::ADT_Opt_Variable(std::string _name, double _value){
	name = _name;
	value = _value;	
}

ADT_Opt_Variable::ADT_Opt_Variable(std::string _name, double _value, double _l_bound, double _u_bound){
	name = _name;
	value = _value;	
	l_bound = _l_bound;
	u_bound = _u_bound;	
}

ADT_Opt_Variable::ADT_Opt_Variable(std::string _name, int _knotpoint, double _value, double _l_bound, double _u_bound){
	name = _name;
	value = _value;	
	knotpoint = _knotpoint;
	l_bound = _l_bound;
	u_bound = _u_bound;	

}

ADT_Opt_Variable::ADT_Opt_Variable(std::string _name, int _type, int _knotpoint, double _value, double _l_bound, double _u_bound){
	name = _name;
	type = _type;
	value = _value;	
	knotpoint = _knotpoint;
	l_bound = _l_bound;
	u_bound = _u_bound;		
}

ADT_Opt_Variable::~ADT_Opt_Variable(){}