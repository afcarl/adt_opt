#ifndef WBT_OPT_VARS_H
#define WBT_OPT_VARS_H

#include <string>
#include <adt/optimization_constants.hpp>

class ADT_Opt_Variable{
public:
	int type = VAR_TYPE_NONE;
	std::string name = "undefined_name";
	double 		value = 0.0;
	int			knotpoint = -1;
	int 		index = -1;

	double l_bound = -OPT_INFINITY;
	double u_bound = OPT_INFINITY;	

	// Constructors
	ADT_Opt_Variable();
	ADT_Opt_Variable(std::string _name, double _value);
	ADT_Opt_Variable(std::string _name, double _value, double _l_bound, double _u_bound);	
	ADT_Opt_Variable(std::string _name, int _knotpoint, double _value, double _l_bound, double _u_bound);	
	ADT_Opt_Variable(std::string _name, int _type, int _knotpoint, double _value, double _l_bound, double _u_bound);		

	// Destructors
	~ADT_Opt_Variable();	
};

#endif